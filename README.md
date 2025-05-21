# MAS
Multi-Agent Antidrone System

## Installation
### Network Setting
Needed for controlling gimbal in different local IP (192.168.144.x)
  1. Install `netplan`
      ```bash
      sudo apt update
      sudo apt install netplan.io
      sudo systemctl start systemd-networkd
      sudo systemctl enable systemd-networkd
      ```
  2. Confirm IP settings in [drone_config/network/01-netcfg.yaml](drone_config/network/01-netcfg.yaml)
  3. Apply config
      ```bash
      sudo cp drone_config/network/01-netcfg.yaml /etc/netplan/
      sudo chmod 600 /etc/netplan/01-netcfg.yaml
      sudo netplan apply
      ```

### `udev`
Serial/USB device permission, alias setting.
- Install rule file:
  ```bash
  sudo cp drone_config/udev/99-pixhawk.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
  ```

### Environment Variables
- Create `robot.env` file from `robot.env.template`, put appropriate values to the variables
  ```bash
  # Namespace
  ROBOT_NAME="px4_1"
  # PX4
  FCU_URL="/dev/pixhawk:115200"
  TGT_SYSTEM=1
  # Gimbal
  GIMBAL_IP="192.168.144.26"
  # ROS distro
  ROS2_INSTALL_PATH=/opt/ros/humble
  ```

## Operation
