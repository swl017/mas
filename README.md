# MAS
Multi-Agent Antidrone System

## Dependencies

## Operation
- Create `robot.env` file from `robot.env.template`, put appropriate values to the variables
  ```bash
  # Namespace
  ROBOT_NAME="px4_1"
  # PX4
  FCU_URL="/dev/ttyTHS0:115200"
  TGT_SYSTEM=1
  # Gimbal
  GIMBAL_IP="192.168.144.26"
  ```