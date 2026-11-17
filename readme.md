# Autonomous Hanger Pickup and Rack Placement on Hello Robot Stretch 3

A **Hello Robot Stretch 3** finds an orange clothes hanger on a table with its **Intel RealSense D405** wrist camera, computes the hanger's 3D position from depth, aligns its base, arm and lift to it, lowers onto it with **motor-current contact detection**, grasps it, and hangs it on a garment rack.

Built in the Neuro-symbolic Intelligent Cyber-physical Systems (NICE) Lab, Arizona State University (Prof. Zhe Xu).

| Detection: contour, bounding box and grasp point | Result: hanger placed on the rack |
|---|---|
| ![Wrist camera view with the hanger contour, bounding box and grasp target drawn](docs/hanger_detection.png) | ![Wrist camera view of the hanger hooked on the rack bar](docs/hanger_on_rack.png) |

---

## How it works

### Phase 1: Autonomous pickup

1. **Observation pose.** The lift moves to 0.75 m and the wrist pitches down to −1.30 rad so the D405 looks at the table. The gripper opens.
2. **Hanger detection.** The color image is thresholded in HSV for orange plastic (upper 75% of the frame only, which excludes the gripper fingers), cleaned with morphological close/open, and filtered by contour area and aspect ratio. The largest remaining contour is the hanger. The grasp point sits 30% in from the left edge of its bounding box, at mid-height.
3. **3D localization.** Depth is the median of valid pixels in a 7 × 7 window around the grasp point. The pixel is deprojected to a 3D point in the camera frame with the D405's intrinsics.
4. **Kinematic alignment.** The camera-frame point is rotated by the wrist pitch to get horizontal reach and vertical drop:
   - **Base** translates to cancel the sideways offset (capped at ±0.25 m).
   - **Arm** extends by the horizontal reach minus the camera-to-gripper offset (capped at 0.48 m total).
   - **Lift** drops quickly to 3 cm above the computed grasp height.
5. **Guarded descent.** The lift lowers in 3 mm steps while the lift motor current is monitored. When it passes 650 mA the gripper is touching the surface, so the lift stops and backs off 3 mm. A floor limit at 0.10 m prevents over-travel.
6. **Grasp.** The gripper closes fully and the wrist returns to level.

### Phase 2: Rack placement (scripted)

7. Retract the arm and raise the lift to 0.90 m.
8. Rotate the base −90° (−1.57 rad) to face the rack.
9. Raise the lift to 1.08 m to clear the rack bar.
10. Extend the arm 0.315 m so the hanger hook is over the bar.
11. Lower up to 8 cm with the same guarded descent until the hook seats on the bar.
12. Open the gripper, retract the arm, and save a final camera frame.

```
observe ─► detect (HSV) ─► depth + deproject ─► align base / arm / lift ─► guarded descent ─► grasp
                                                                                                │
        save final frame ◄─ retract ◄─ release ◄─ guarded descent ◄─ extend ◄─ rotate −90° ◄─ lift
```

---

## Requirements

**Hardware**
- Hello Robot Stretch 3 with the dexterous wrist and Stretch gripper
- Intel RealSense D405 wrist camera (found automatically by name)
- An orange plastic hanger and a garment rack with a horizontal bar

**Software**
- Python 3.8+
- [`stretch_body`](https://github.com/hello-robot/stretch_body) (preinstalled on Stretch)
- `pyrealsense2`, `opencv-python`, `numpy`

```bash
pip install pyrealsense2 opencv-python numpy
```

---

## Setup and usage

1. **Home the robot** if it hasn't been homed since power-on:
   ```bash
   stretch_robot_home.py
   ```
2. **Place the hanger** flat on a table within the wrist camera's view from the observation pose.
3. **Place the rack** so that after the robot turns 90° clockwise, the bar is within 0.315 m of arm reach at about 1.08 m lift height. The placement phase uses fixed positions, so the rack must be where the script expects it.
4. Clear the area and keep the run-stop button within reach, then run:
   ```bash
   python3 adm_place5.py
   ```
5. Press `Ctrl+C` at any time to stop. The camera and robot shut down cleanly.

**Outputs**
- `adm_place5_initial_frame.png`: the detection frame with the contour (magenta), bounding box (green) and grasp point (red)
- `adm_place5_final_frame.png`: the wrist camera view after placement

---

## Configuration

All tuning values are at the top of the script.

| Parameter | Default | Purpose |
|---|---|---|
| `PICKUP_OBSERVATION_LIFT_M` | 0.75 m | Lift height for the observation pose |
| `PICKUP_OBSERVATION_PITCH_RAD` | −1.30 rad | Wrist pitch so the camera looks down at the table |
| `HANGER_X_BOX_RATIO`, `HANGER_Y_BOX_RATIO` | 0.30, 0.50 | Grasp point inside the hanger's bounding box |
| HSV range | H 4–24, S 110–255, V 100–255 | Orange threshold in `detect_actual_hanger()` |
| `MAX_BASE_CORRECTION_M` | 0.25 m | Largest allowed base correction |
| `MAX_ARM_EXTENSION_M` | 0.48 m | Largest allowed arm extension |
| `CENTER_X/Y/Z_OFFSET_M` | 0.0 | Calibration offsets added to the 3D target |
| `LIFT_CONTACT_THRESHOLD_MA` | 650 mA | Lift current that counts as contact |
| `POST_PICK_LIFT_SAFE_M` | 0.90 m | Safe lift height after pickup |
| `RACK_BASE_ROTATE_RAD` | −1.57 rad | Base turn to face the rack |
| `RACK_BAR_CLEARANCE_LIFT_M` | 1.08 m | Lift height above the rack bar |
| `RACK_ARM_EXTENSION_M` | 0.315 m | Arm reach over the bar |
| `RACK_HOOK_DESCENT_M` | 0.08 m | Maximum descent onto the bar |

---

## Limitations and next steps

- Detection relies on the hanger's orange color, so lighting changes and other orange objects can cause misses or false detections.
- The rack placement uses fixed positions, so the rack must be set up the same way every run. Next step: detect the rack bar (for example with an ArUco marker) and place relative to it.
- The contact threshold and camera-to-gripper offsets were tuned for this robot and table.
- There is no check that the hanger is actually in the gripper before placement.

---

## Author

**Yashwanth Gowda**, MS Robotics and Autonomous Systems, Arizona State University
[LinkedIn](https://www.linkedin.com/in/nmyashwanthgowda) · [GitHub](https://github.com/yashwanthgowdanm)
