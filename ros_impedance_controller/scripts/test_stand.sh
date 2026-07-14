#!/usr/bin/env bash
#
# test_stand.sh - Quick manual test for the ros_impedance_controller (Go1).
#
# WHY THIS SCRIPT EXISTS
# ----------------------
# In locosim the per-joint PID gains ship as 0.0 in
# go1_description/config/ros_impedance_controller.yaml and are normally set at
# runtime by the Python control layer through the /set_pids service. If you
# only launch the Gazebo + controller stack (no Python), the gains stay 0.0,
# the controller outputs zero torque, and the robot stays limp - sending a
# /command does nothing.
#
# This script:
#   1) Sets reasonable P/D gains on all 12 joints via /set_pids.
#   2) Sends a /command (sensor_msgs/JointState) with the "home" pose, in the
#      SAME joint order as the controller's `joints:` list (LF, LH, RF, RH).
#      The controller maps the command arrays BY INDEX and ignores `name`,
#      so the order below must match the YAML.
#
# Gains are a sane starting point for the Go1 in sim - tune as needed.
#
# Usage:
#   source ~/ros2_ws/install/setup.bash
#   ./test_stand.sh
#
set -euo pipefail

# Joint order MUST match `joints:` in ros_impedance_controller.yaml
JOINTS=(lf_haa_joint lf_hfe_joint lf_kfe_joint \
        lh_haa_joint lh_hfe_joint lh_kfe_joint \
        rf_haa_joint rf_hfe_joint rf_kfe_joint \
        rh_haa_joint rh_hfe_joint rh_kfe_joint)

# Home pose (from the YAML `home:` block), in the joint order above.
HOME_POS=(0.2 0.78 -1.7  0.2 0.78 -1.7  -0.2 0.78 -1.7  -0.2 0.78 -1.7)

# PID gains (starting point). HAA/HFE/KFE per leg.
P_GAIN=100.0
D_GAIN=4.0
I_GAIN=0.0

echo "==> Setting PID gains (p=$P_GAIN i=$I_GAIN d=$D_GAIN) on all 12 joints via /set_pids ..."
PID_DATA=""
for j in "${JOINTS[@]}"; do
  PID_DATA+="{joint_name: '$j', p_value: $P_GAIN, i_value: $I_GAIN, d_value: $D_GAIN}, "
done
PID_DATA="[${PID_DATA%, }]"

ros2 service call /set_pids ros_impedance_controller/srv/SetPids "{data: $PID_DATA}"

echo "==> Sending home-pose /command ..."
POS="[$(IFS=,; echo "${HOME_POS[*]}")]"
ZEROS="[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"
NAMES="[$(IFS=,; echo "${JOINTS[*]}")]"

ros2 topic pub --once /command sensor_msgs/msg/JointState \
  "{name: $NAMES, position: $POS, velocity: $ZEROS, effort: $ZEROS}"

echo "==> Done. The Go1 should now hold the home pose in Gazebo."
