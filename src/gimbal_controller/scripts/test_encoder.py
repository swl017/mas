#!/usr/bin/env python3
"""Quick diagnostic: test if SIYI gimbal supports 0x26 encoder angle command."""

import sys
import time
sys.path.insert(0, '/home/usrg/mas/src/gimbal_controller')

from gimbal_controller.siyi_sdk.siyi_sdk import SIYISDK

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.144.26"

cam = SIYISDK(server_ip=IP, port=37260)
if not cam.connect():
    print(f"FAILED to connect to {IP}")
    sys.exit(1)

print(f"Connected to {IP}")
time.sleep(0.5)

# Test 1: One-shot 0x26
print("\n--- Test 1: One-shot requestGimbalEncoderAngle() ---")
result = cam.requestGimbalEncoderAngle()
print(f"  sendMsg returned: {result}")
time.sleep(0.5)
yaw, pitch, roll = cam.getGimbalEncoderAngles()
print(f"  Encoder angles: yaw={yaw}, pitch={pitch}, roll={roll}")

# Test 2: Request stream via 0x25 with data_type=3
print("\n--- Test 2: requestDataStreamEncoderAngle(freq=10) ---")
result = cam.requestDataStreamEncoderAngle(10)
print(f"  sendMsg returned: {result}")
time.sleep(2)
yaw, pitch, roll = cam.getGimbalEncoderAngles()
print(f"  Encoder angles after 2s: yaw={yaw}, pitch={pitch}, roll={roll}")

# Test 3: Also check IMU attitude for comparison
print("\n--- Test 3: IMU attitude (0x0D) for comparison ---")
att = cam.getAttitude()
print(f"  IMU attitude: yaw={att[0]}, pitch={att[1]}, roll={att[2]}")

# Test 4: Check stream at higher freq
print("\n--- Test 4: requestDataStreamEncoderAngle(freq=50) ---")
result = cam.requestDataStreamEncoderAngle(50)
print(f"  sendMsg returned: {result}")
time.sleep(2)
for i in range(5):
    yaw, pitch, roll = cam.getGimbalEncoderAngles()
    print(f"  Sample {i}: yaw={yaw}, pitch={pitch}, roll={roll}")
    time.sleep(0.1)

cam.disconnect()
print("\nDone.")
