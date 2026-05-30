# SPDX-License-Identifier: Apache-2.0
# ----------------------------------------------------------------------------------
# scan_angle_fixer.py  --  cargo_bot_ws / ROS 2 Humble
#
# PURPOSE
#   Fix the off-by-one beam-count mismatch between Isaac Sim's RTX-Lidar /scan and
#   slam_toolbox (Karto). Subscribes to /scan, rewrites angle_max so the
#   angle-metadata span is consistent with the number of ranges, republishes to
#   /scan_fixed. Point slam_toolbox's scan_topic at /scan_fixed.
#
# ROOT CAUSE
#   Isaac's RTX Lidar (Generic Example_Rotary_2D profile, validStartAzimuthDeg=0,
#   validEndAzimuthDeg=360, scanRateBaseHz=30, reportRateBaseHz=32000) produces
#   N = floor(32000/30) = 1066 range readings, but the LaserScan publisher
#   (isaacsim.ros2.bridge.ROS2PublishLaserScan, C++) reports a FULL 360 deg azimuth
#   span for a ROTARY lidar:
#       angle_min = -pi, angle_max = +pi, angle_increment = 2*pi / N (= FoV/numCols)
#   i.e. it divides the full 360 deg FoV by N (numCols), NOT by (N-1). The last
#   beam at 360 deg coincides with the first at 0 deg, so the span is one increment
#   too wide for the number of samples.
#
#   slam_toolbox wraps Karto. On the first scan Karto's LaserRangeFinder registers
#   the device and computes (lib/karto_sdk, LaserRangeFinder::Update):
#       m_NumberOfRangeReadings =
#           round( (GetMaximumAngle() - GetMinimumAngle()) / GetAngularResolution() ) + 1
#   With Isaac's metadata that is round( 2*pi / (2*pi/1066) ) + 1 = 1066 + 1 = 1067.
#   Every incoming scan carries 1066 ranges -> Karto's Validate() rejects ALL of them:
#       "LaserRangeScan contains 1066 range readings, expected 1067"
#   so map->odom is published (identity) but /map is never built.
#
#   The "+1" fencepost is the classic inclusive-endpoint convention: a scan whose
#   first and last beams are NOT coincident needs angle_max = angle_min + (N-1)*inc.
#   Isaac emits angle_max = angle_min + N*inc (coincident endpoints). There is NO
#   slam_toolbox parameter to relax this check (it lives in unconditional Karto
#   Validate()), and the Isaac-side azimuth attrs do NOT help: the publisher forces
#   a [-180, 180] span for ROTARY lidars regardless of validEndAzimuthDeg
#   (documented: ROS2PublishLaserScan inputs:azimuthRange "Always [-180, 180] for
#   rotary lidars"). Hence this republisher owns the angle math deterministically.
#
# THE FIX (per message, robust to any N drift)
#   keep angle_min and angle_increment as published; set
#       angle_max = angle_min + (len(ranges) - 1) * angle_increment
#   Then Karto computes round((N-1)*inc / inc) + 1 = (N-1) + 1 = N -> matches ranges.
#
# VERIFICATION
#   For N=1066, inc=2*pi/1066: new angle_max - angle_min = 1065*inc.
#   Karto expected = round(1065*inc / inc) + 1 = 1065 + 1 = 1066 == len(ranges).  OK
#
# USAGE
#   ros2 run cargo_bot_bringup scan_angle_fixer
#   (slam.launch.py already starts it; slam_toolbox.yaml scan_topic -> /scan_fixed)
#
# SOURCES
#   - slam_toolbox / karto_sdk Karto.h LaserRangeFinder::Update + Validate:
#       https://github.com/SteveMacenski/slam_toolbox  (lib/karto_sdk)
#   - Isaac publisher azimuth note:
#       isaacsim.ros2.bridge OgnROS2PublishLaserScan.rst (inputs:azimuthRange)
# ----------------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanAngleFixer(Node):
    def __init__(self):
        super().__init__('scan_angle_fixer')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_fixed')
        # 'shrink_angle_max' (default): keep increment, pull angle_max in by one step.
        #   -> expected == N, no resampling, range data untouched. Recommended.
        # 'grow_increment': keep angle_max, widen increment to span N-1 steps.
        #   Only useful if a downstream consumer needs the full [-pi, pi] span.
        self.declare_parameter('mode', 'shrink_angle_max')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.mode = self.get_parameter('mode').value

        # BEST_EFFORT matches Isaac's RTX lidar sensor publisher QoS.
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(LaserScan, self.output_topic, pub_qos)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self.on_scan, sub_qos)

        self._warned_once = False
        self.get_logger().info(
            f"scan_angle_fixer up: {self.input_topic} -> {self.output_topic} (mode={self.mode})"
        )

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n < 2 or msg.angle_increment == 0.0:
            # Nothing sensible to recompute; pass through unchanged.
            self.pub.publish(msg)
            return

        out = msg  # in-place is fine; we own the message and republish immediately

        if self.mode == 'grow_increment':
            # Keep angle_min/angle_max; widen increment so (max-min)/inc == N-1.
            out.angle_increment = (msg.angle_max - msg.angle_min) / float(n - 1)
        else:  # 'shrink_angle_max' (default, recommended)
            # Keep angle_min/increment; pull angle_max in by one increment.
            out.angle_max = msg.angle_min + (n - 1) * msg.angle_increment

        if not self._warned_once:
            expected = round((out.angle_max - out.angle_min) / out.angle_increment) + 1
            self.get_logger().info(
                f"first scan fixed: N={n} angle_min={out.angle_min:.6f} "
                f"angle_max={out.angle_max:.6f} inc={out.angle_increment:.8f} "
                f"-> karto_expected={expected} (should equal {n})"
            )
            self._warned_once = True

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanAngleFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
