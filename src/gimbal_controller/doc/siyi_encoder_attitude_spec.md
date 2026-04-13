# SIYI SDK: Encoder Angles & Aircraft Attitude Injection

## Motivation

The current SIYI SDK only uses command `0x0D` (Request Gimbal Attitude Data), which returns IMU-stabilized world-frame angles. During aggressive drone maneuvers, the gimbal's internal IMU drifts due to centrifugal force — corrupting both the reported world-frame angles and the derived body-frame joint angles.

Two protocol commands solve this:
- **`0x26`** — raw magnetic encoder angles (exact body-frame joint angles, immune to IMU drift)
- **`0x22`** — send aircraft EKF attitude to gimbal (improves its internal stabilization)

See `ARCHITECTURE_SEMANTIC_LEVEL.md` § "Gimbal state/command ambiguity" for the full investigation.

## Protocol Reference

Source: `SIYI_Gimbal_Camera_External_SDK_Protocol_Update_Log V0.1.1.pdf`, pages 27-28 and 32-33.

---

### `0x22` — Send Aircraft Attitude Data to Gimbal

Sends the drone's EKF-fused attitude to the gimbal so it can use it for better stabilization instead of relying solely on its internal IMU.

**Send Data Format (28 bytes):**

| No. | Type | Name | Description |
|-----|------|------|-------------|
| 1 | uint32_t | time_boot_ms | Timestamp (ms since system boot) |
| 2 | float | Roll | Roll angle [rad] (-pi..+pi) |
| 3 | float | Pitch | Pitch angle [rad] (-pi/2..+pi/2) |
| 4 | float | Yaw | Yaw angle [rad] (-pi..+pi) |
| 5 | float | Rollspeed | Roll angular speed [rad/s] |
| 6 | float | Pitchspeed | Pitch angular speed [rad/s] |
| 7 | float | Yawspeed | Yaw angular speed [rad/s] |

**ACK:** Empty (no response).

**Notes:**
- Coordinate system: NED (North-East-Down)
- Rotation order: yaw → pitch → roll
- Recommended send frequency: 20-50 Hz
- Uses **float** (4 bytes, IEEE 754), unlike most other commands which use int16

**Data source:** Subscribe to MAVROS `local_position/odom` in the ROS node, convert ENU→NED, pack into this message.

---

### `0x26` — Request Gimbal Magnetic Encoder Angle Data

Returns raw angles from the magnetic rotary encoders on the gimbal motor shafts. These are true mechanical joint angles — not fused with IMU, not affected by centrifugal force.

**Send Data Format:** Empty (no payload).

**ACK Data Format (6 bytes):**

| No. | Type | Name | Description |
|-----|------|------|-------------|
| 1 | int16_t | yaw_angle | Yaw angle (÷10 for degrees, 0.1° precision) |
| 2 | int16_t | pitch_angle | Pitch angle (÷10 for degrees, 0.1° precision) |
| 3 | int16_t | roll_angle | Roll angle (÷10 for degrees, 0.1° precision) |

**Notes:**
- Same encoding as `0x0D`: little-endian int16, divide by 10 for actual degrees
- Can be polled one-shot, but recommended to use `0x25` (data stream) for continuous delivery
- These are body-frame joint angles — they match what `mas_policy` expects

---

### `0x25` update — Request Gimbal to Send Data Stream

The existing data stream command gains new `data_type` values:

| data_type | Name | Notes |
|-----------|------|-------|
| 1 | Attitude Data | Existing (IMU-stabilized, `0x0D` format) |
| 2 | Laser Range Data | Existing |
| **3** | **Magnetic Encoder Angle Data** | **New — streams `0x26` response format** |
| 4 | Motor Voltage Data | New (not needed for MAS) |

**Send Data Format (2 bytes):**

| No. | Type | Name | Description |
|-----|------|------|-------------|
| 1 | uint8_t | data_type | 3 for encoder angles |
| 2 | uint8_t | data_freq | 0: off, 1: 2Hz, 2: 4Hz, 3: 5Hz, 4: 10Hz, 5: 20Hz, 6: 50Hz, 7: 100Hz |

**ACK:** uint8_t `data_type` echoed back.

---

## Implementation Scope

### SDK layer (`siyi_sdk/`)

**`siyi_message.py`:**
- Add `SEND_AIRCRAFT_ATTITUDE = '22'` and `REQUEST_GIMBAL_ENCODER = '26'` to `COMMAND`
- Add `MAGNETIC_ENCODER_ANGLE_DATA = '03'` to `RequestDataStreamMsg`
- Add `GimbalEncoderAngleMsg` data class (yaw_angle, pitch_angle, roll_angle)
- Add `sendAircraftAttitudeMsg(time_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)` — pack as little-endian uint32 + 6× float
- Add `requestGimbalEncoderAngleMsg()` — empty payload

**`siyi_sdk.py`:**
- Add `_gimbal_encoder_msg` instance variable
- Add `requestSendAircraftAttitude(time_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)`
- Add `requestGimbalEncoderAngle()` — one-shot request
- Add `requestDataStreamEncoderAngle(freq)` — continuous via `0x25` with `data_type=3`
- Add `parseGimbalEncoderMsg(msg, seq)` — parse 3× int16, ÷10
- Add `getGimbalEncoderAngles()` → `(yaw_deg, pitch_deg, roll_deg)`
- Update `bufferCallback()` dispatch for `0x26`

### ROS node layer (`siyi_ros_node.py`)

- Add publisher: `gimbal_encoder_rpy_deg` (`geometry_msgs/Vector3`) — raw encoder joint angles
- Add subscriber: `common_frame/odom` (`nav_msgs/Odometry`) — source for `0x22` data
- On odom callback: convert ENU→NED, call `requestSendAircraftAttitude()` at 20-50 Hz
- On timer: also publish encoder angles from `getGimbalEncoderAngles()`
- Initialize encoder stream via `requestDataStreamEncoderAngle(50)` on connect

### Config

- Add parameter `enable_encoder_stream` (bool, default: true) — enable `0x26` streaming
- Add parameter `enable_aircraft_attitude` (bool, default: true) — enable `0x22` injection
- Add parameter `encoder_stream_freq` (int, default: 50) — Hz for encoder stream
