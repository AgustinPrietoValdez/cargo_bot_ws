# Disconnect helper.renderProductPath from CreateRenderProduct and set it literally
# to point at the existing auto-created Replicator with GenericModelOutput AOV.
# Run from Script Editor with Isaac STOPPED.
import omni.usd
import omni.graph.core as og
from pxr import Sdf

stage = omni.usd.get_context().get_stage()
helper_path = "/cargo_bot/ActionGraph/ros2_rtx_lidar_helper"
target_render_product = "/Render/OmniverseKit/HydraTextures/Replicator"

helper = stage.GetPrimAtPath(helper_path)
if not helper:
    print("ERROR: helper not found at " + helper_path)
else:
    rpp_attr = helper.GetAttribute("inputs:renderProductPath")
    if not rpp_attr:
        print("ERROR: inputs:renderProductPath attribute not found on helper")
    else:
        # 1) Disconnect any connections
        try:
            rpp_attr.ClearConnections()
            print("disconnected renderProductPath connections")
        except Exception as e:
            print("ClearConnections failed: " + str(e))

        # 2) Set literal value
        rpp_attr.Set(target_render_product)
        print("set renderProductPath = " + target_render_product)

        # Verify
        v = rpp_attr.Get()
        print("verify: renderProductPath = " + repr(v))

# Also: same for execIn — keep it connected to isaac_create_render_product OR
# bypass it directly from on_playback_tick. We'll keep current wiring;
# only re-wire if needed.
print("=== done ===")
