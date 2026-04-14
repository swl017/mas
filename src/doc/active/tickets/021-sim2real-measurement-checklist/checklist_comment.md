# Comments on the checklist with regards to the test script to be written.

## Category A: Flight Dynamics (PX4 Logs)
### A1. Thrust-to-Weight Characterization
- PX4 position mode is enough.
- Flight sequence: Put position mode -> Takeoff -> hover 30 sec(min) -> land or continue to other experiments.


### A2. Velocity Step Response
- We need offboard test script for setting velocity commands.
- Flight sequence: Start the test script(waits for offboard mode change) -> Put position -> Takeoff -> hover 10 sec to stabilize -> put offboard mode -> the script executes speed commands -> land or continue to other experiments.
- The test script should
    - Wait for the flight mode to change into offboard mode
    - 0 -> 3 -> 0 -> -3 (kind of return to origin to save space)


### A3. Attutude Step Response
- We need offboard test script for setting attitude commands. Prerequisite: Thrust-to-weight, thrust curve.
- Is stabilized mode and stick commands from the radio controller sufficient for this for safetly reasons, though not perfectly reproducable?
- With test script: Start the test script(waits for offboard mode change) -> Position mode -> takeoff -> hover 10 sec to stabilize -> put offboard mode -> test script -> land or continue to other experiments.
- Without test script: Position mode -> takeoff -> put stabilized mode -> stick input(be as step as possible) -> hold for 1~2 sec -> put position mode to hover -> repeat -> land or continue to other experiments.


### A4. Yaw Rate Response
- We need offboard test script for setting yaw rate commands.
- Flight sequence: Start the test script(waits for offboard mode change) -> Put position -> Takeoff -> hover 10 sec to stabilize -> put offboard mode -> test script -> land or continue to other experiments.


### A5. Motor RPM Characterization (Optional, High Value)
- So this is automatically collected from A1~A4?
- What are the 27 parameters the interception project identified?

### A6. Model fitting and validation
- How does the interception project(`/home/usrg/source/Aerial_To_Aerial_Interception/`) do model fitting? Does this project use the same model as ours? If different, is it better to fit their model to our's, or our's to their's?

---

## Category B: Drag and Aerodynamics
### B1. Steady-State Drag Force
- Collect this with A2(velocity step response)

### B2. Wind Disturbance Characterization
- We might need multiple days for this

---

## Category C: Sensor Latency Pipeline

### C1. IMU-to-Policy Latency (Proprioceptive)
- Straight forward.

### C2. Detection Inference Latency
- To repeatedly feed representative images, we need a test script that feeds the image at a certain rate.
- We would need to run the triangulation node for maximum load.
- So collect data from outdoors in two locations, run triangulation at maximum rate

### C4. Detection FPS and Dropout Rate
- This is much complicated than others, because we need to fly the target drone in the camera frame, with their location logged.
- Target's relative distance to the camera, camera zoom, apparent bbox size and center location, background clutter would all correlate.
- For background clutter: Collect data once on clean sky, once on cluttered background.

### C5. Raw Image transport Latency(new)
- Measure the latency between the real world timer to captured image by comparing the time shown
- But this measures the pipeline latency for "image processing by camera -> image made into ROS2 topic -> RQT or RViz receives it -> Shown on the monitor", which is different from our "image processing by camera -> image made into ROS2 topic -> downstream nodes receives it"
- We could only tell after we do actual test if image processing latency dominates or others interfere.


### C6. Datalink latency, dropout rate(new)
- Depends heavily on the environmental condition
- Pick multiple places. Each would have different characteristics.
- Collect agent-to-agent, agent-to-gcs.
- WiFi? Radio? LTE? WiFi had range issue and radio(SIYI HM30) had throughput issue. Let me first test with bigger WiFi router then move on to LTE, if not satisfying.(Need a seperate ticket for this)

---

## Category D: Camera and Gimbal

### D1. Camera Intrinsic Calibration
- Straight forward
- Use `mrcal` to measure the uncertainty of the calibration, too.

### D2. Gimbal Joint Calibration
- We need first to check if the raw gimbal encoder reading works. `/home/usrg/mas/src/doc/active/tickets/005-gimbal-encoder-hwtest/ticket.md`
- So write a test script that starts from the hwtest to calibration test.

### D3. Gimbal Dynamic Response
- This can be automatically collected with A1~A4.
- We also need to check if the gimbal compensates for the drone body acceleration that could interfere LOS stabilization in the gimbal. But how to we know if acceleration compensation works? Is there any methods other and visually inspecting?

## Category E: Mass and Payload

### E1. Drone Mass Budget
- Straighforward.

### E2. Center of Gravity with Gimbal
- Good idea.

---

(New. Need review)
## Category G: Uncertainty Modeling

### C1. GPS Covariance Under Motion
- Does it change under motion?
- What's the commonly obverved covariance value in real world in GPS float, fix, RTK?

---

## Data Collection Protocol

### Equipment Needed
- Probably less accurate to gauge the angle offset in the gimbal by hand. Any other ideas?
- Gimbal joint angles and camera image would be collected only in ROS bag. For their calibration/modeling purposes, drone data should be colleted in ROS bag, too. For category A purposes, `.ulg` can stay the same.
- Test scripts should trigger ROS bag record with the test name automatically, so that no experiments are left unrecorded or mixed