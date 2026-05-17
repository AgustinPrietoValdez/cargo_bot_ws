#!/bin/bash
source /opt/ros/humble/setup.bash
/opt/ros/humble/bin/fast-discovery-server -i 0 -p 11811 &
BGPID=$!
sleep 2
if kill -0 $BGPID 2>/dev/null; then
  echo "Discovery Server ALIVE (PID=$BGPID)"
  echo $BGPID > /tmp/cargo_bot_ds.pid
  wait $BGPID
else
  echo "Discovery Server DIED"
  exit 1
fi
