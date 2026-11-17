import signal
import sys
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import stretch_body.robot

# ==============================================================================
# HARD-CODED RACK PLACEMENT TUNING PARAMETERS
# ==============================================================================
POST_PICK_LIFT_SAFE_M = 0.90
RACK_BASE_ROTATE_RAD = -1.57
RACK_BAR_CLEARANCE_LIFT_M = 1.08
RACK_ARM_EXTENSION_M = 0.315
RACK_HOOK_DESCENT_M = 0.08

# ADMITTANCE & FORCE CONTROL PARAMETERS
LIFT_CONTACT_THRESHOLD_MA = 650.0

# ==============================================================================
# PICKUP TUNING
# ==============================================================================
PICKUP_OBSERVATION_LIFT_M = 0.75
PICKUP_OBSERVATION_PITCH_RAD = -1.30
POST_PICK_PITCH_RAD = 0.0

# Target the left side / shoulder of the orange hanger bounding box
HANGER_X_BOX_RATIO = 0.30  # 30% from the left edge of the hanger
HANGER_Y_BOX_RATIO = 0.50  # Center vertically along the hanger body

# Calibration Offsets
CENTER_X_OFFSET_M = 0.0
CENTER_Y_OFFSET_M = 0.0
CENTER_Z_OFFSET_M = 0.0

# Safety limits for calculated motion
MAX_BASE_CORRECTION_M = 0.25
MAX_ARM_EXTENSION_M = 0.48

# Output Filenames
INITIAL_FRAME_PATH = "adm_place5_initial_frame.png"
FINAL_FRAME_PATH = "adm_place5_final_frame.png"

robot = None
pipeline = None


def cleanup_and_exit(signum=None, frame=None):
    """Safely shuts down hardware and streams on Ctrl+C."""
    print("\n[EMERGENCY STOP] Interrupted by user. Shutting down safely...")

    global pipeline, robot

    if pipeline:
        try:
            pipeline.stop()
        except Exception:
            pass

    if robot:
        try:
            robot.stop()
        except Exception:
            pass

    sys.exit(0)


signal.signal(signal.SIGINT, cleanup_and_exit)


def get_motor_current_ma(status_dict):
    """Safely extracts motor current in mA across Stretch driver versions."""
    if "motor" in status_dict:
        m_dict = status_dict["motor"]
        raw_val = m_dict.get("current", m_dict.get("effort", 0.0))
    else:
        raw_val = status_dict.get("current", status_dict.get("effort", 0.0))

    val = abs(float(raw_val))
    if val < 10.0:
        return val * 1000.0

    return val


def get_robust_depth_m(depth_frame, cx, cy, window_size=7):
    """Extract median depth in meters around a pixel."""
    depth_image = np.asanyarray(depth_frame.get_data())

    h, w = depth_image.shape
    half = window_size // 2

    y_min = max(0, cy - half)
    y_max = min(h, cy + half + 1)
    x_min = max(0, cx - half)
    x_max = min(w, cx + half + 1)

    roi = depth_image[y_min:y_max, x_min:x_max]
    valid_depths = roi[roi > 0]

    if len(valid_depths) > 0:
        units = depth_frame.get_units()
        return float(np.median(valid_depths)) * units

    return 0.0


def find_d405_serial():
    ctx = rs.context()
    for dev in ctx.query_devices():
        if "D405" in dev.get_info(rs.camera_info.name):
            return dev.get_info(rs.camera_info.serial_number)
    return None


def detect_actual_hanger(color_image):
    """
    Isolates orange plastic hanger using HSV color thresholding.
    Returns tight bounding box, left-side target point, and exact contour.
    """
    img_h, img_w, _ = color_image.shape

    # Focus on workspace upper area
    roi_mask = np.zeros((img_h, img_w), dtype=np.uint8)
    roi_mask[0 : int(img_h * 0.75), :] = 255

    # HSV Color Segmentation for Orange Plastic Hanger
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([4, 110, 100])
    upper_orange = np.array([24, 255, 255])
    orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Restrict to ROI
    orange_mask = cv2.bitwise_and(orange_mask, roi_mask)

    # Clean mask noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None, None, None

    candidate_contours = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 300 or area > 35000:
            continue

        x, y, w, h = cv2.boundingRect(c)
        aspect_ratio = float(w) / float(h)

        if not (0.3 <= aspect_ratio <= 4.0):
            continue

        candidate_contours.append((area, c, (x, y, w, h)))

    if not candidate_contours:
        return None, None, None

    # Select largest orange structure contour
    candidate_contours.sort(key=lambda item: item[0], reverse=True)
    best_area, best_contour, (x, y, w, h) = candidate_contours[0]

    # Target pixel placed towards the LEFT side of the hanger
    target_x = int(x + (w * HANGER_X_BOX_RATIO))
    target_y = int(y + (h * HANGER_Y_BOX_RATIO))

    return ((x, y, x + w, y + h), (target_x, target_y), best_contour)


def admittance_guarded_descent(
    robot,
    max_descent_m=0.12,
    contact_threshold_ma=LIFT_CONTACT_THRESHOLD_MA,
    step_m=0.003,
):
    """Lowers arm using current-based contact detection until soft touchdown."""
    print(
        f"[ADMITTANCE DESCENT] Lowering arm "
        f"(Max: {max_descent_m:.3f}m, Threshold: {contact_threshold_ma:.1f} mA)..."
    )

    steps = max(1, int(max_descent_m / step_m))
    contact_detected = False

    for step in range(steps):
        if robot.lift.status["pos"] - step_m < 0.10:
            print("[SAFETY] Floor limit safety stop (0.10m). Halting drop.")
            break

        robot.lift.move_by(-step_m)
        robot.push_command()
        time.sleep(0.02)

        lift_current_ma = get_motor_current_ma(robot.lift.status)

        if lift_current_ma > contact_threshold_ma:
            print(
                f"[ADMITTANCE CONTACT] Surface detected at step {step + 1}! "
                f"(Current: {lift_current_ma:.1f} mA)"
            )

            robot.lift.move_by(0.0)
            robot.push_command()
            time.sleep(0.05)

            robot.lift.move_by(0.003)
            robot.push_command()
            robot.wait_command()

            contact_detected = True
            break

    return contact_detected


def full_squeeze_gripper(robot):
    """Commands full gripper closure."""
    print("[FULL GRIP] Squeezing gripper on hanger...")
    robot.end_of_arm.move_to("stretch_gripper", -90.0)
    robot.push_command()
    robot.wait_command()


def open_gripper(robot, pos=40.0):
    """Opens gripper to release object."""
    print(f"[RELEASE] Opening gripper to position {pos}...")
    robot.end_of_arm.move_to("stretch_gripper", pos)
    robot.push_command()
    robot.wait_command()


def main():
    global pipeline, robot

    # ==========================================================================
    # 1. HARDWARE INITIALIZATION
    # ==========================================================================
    d405_serial = find_d405_serial()
    if not d405_serial:
        print("[ERROR] D405 RealSense wrist camera not found.")
        return

    print("\n[1/5] Connecting to Stretch Hardware...")
    robot = stretch_body.robot.Robot()
    if not robot.startup():
        print("[ERROR] Hardware initialization failed.")
        return

    try:
        # ======================================================================
        # PHASE 1: AUTONOMOUS HANGER PICKUP
        # ======================================================================
        target_lift = PICKUP_OBSERVATION_LIFT_M
        target_pitch = PICKUP_OBSERVATION_PITCH_RAD

        print(
            f"[2/5] Staging pickup observation pose "
            f"(Lift: {target_lift:.3f}m, Pitch: {target_pitch:.2f} rad)..."
        )

        robot.lift.move_to(target_lift)
        robot.end_of_arm.move_to("wrist_pitch", target_pitch)
        robot.end_of_arm.move_to("wrist_roll", 0.0)
        robot.end_of_arm.move_to("wrist_yaw", 0.0)
        open_gripper(robot, 40.0)
        time.sleep(1.5)

        # ======================================================================
        # START REALSENSE
        # ======================================================================
        pipeline = rs.pipeline()
        config = rs.config()

        config.enable_device(d405_serial)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        profile = pipeline.start(config)
        align = rs.align(rs.stream.color)
        intrinsics = rs.video_stream_profile(
            profile.get_stream(rs.stream.color)
        ).get_intrinsics()

        for _ in range(15):
            pipeline.wait_for_frames(5000)

        frames = pipeline.wait_for_frames(5000)
        aligned = align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            print("[ERROR] Could not acquire RGB/depth frames.")
            return

        color_image = np.asanyarray(color_frame.get_data())

        # ======================================================================
        # DETECTION & INITIAL FRAME SAVE
        # ======================================================================
        print("[3/5] Running Autonomous Hanger Detection...")
        box, center_pixel, contour = detect_actual_hanger(color_image)

        if box is None:
            print("[ERROR] Orange hanger contour could not be resolved.")
            return

        cx, cy = center_pixel
        x1, y1, x2, y2 = box

        print(f"[DETECTION] Bounding box: ({x1}, {y1}) -> ({x2}, {y2})")
        print(f"[DETECTION] Left-side hanger target pixel: ({cx}, {cy})")

        # Save annotated Initial Frame with explicit contour visualization
        initial_annotated = color_image.copy()

        # 1. Draw Hanger Contour in Magenta
        cv2.drawContours(initial_annotated, [contour], -1, (255, 0, 255), 2)

        # 2. Draw Hanger Bounding Box in Green
        cv2.rectangle(initial_annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 3. Draw Left-Side Target Spot in Red
        cv2.circle(initial_annotated, (cx, cy), 6, (0, 0, 255), -1)
        cv2.putText(
            initial_annotated,
            f"Hanger Left Target ({cx},{cy})",
            (cx + 10, cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )
        cv2.imwrite(INITIAL_FRAME_PATH, initial_annotated)
        print(f"[IMAGE SAVED] Initial frame saved to '{INITIAL_FRAME_PATH}'")

        # ======================================================================
        # DEPTH AT TARGET PIXEL
        # ======================================================================
        depth_m = get_robust_depth_m(depth_frame, cx, cy, window_size=7)

        if depth_m == 0.0:
            print("[WARNING] No valid depth at target pixel. Fallback: 0.45m.")
            depth_m = 0.45

        print(f"[DEPTH] Target depth: {depth_m:.3f}m")

        # ======================================================================
        # 3D DEPROJECTION
        # ======================================================================
        hanger_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth_m)
        x_cam, y_cam, z_cam = hanger_3d

        print(f"[3D TARGET] X={x_cam:.3f}m, Y={y_cam:.3f}m, Z={z_cam:.3f}m")

        x_cam += CENTER_X_OFFSET_M
        y_cam += CENTER_Y_OFFSET_M
        z_cam += CENTER_Z_OFFSET_M

        # ======================================================================
        # KINEMATIC ALIGNMENT CALCULATION
        # ======================================================================
        pitch_rad = abs(PICKUP_OBSERVATION_PITCH_RAD)

        delta_x_base = -np.clip(
            x_cam, -MAX_BASE_CORRECTION_M, MAX_BASE_CORRECTION_M
        )
        delta_y_arm = z_cam * np.cos(pitch_rad) - y_cam * np.sin(pitch_rad)
        delta_z_lift = z_cam * np.sin(pitch_rad) + y_cam * np.cos(pitch_rad)

        print(
            f"[TARGET OFFSET] Base X correction={delta_x_base:.3f}m, "
            f"Arm Y target={delta_y_arm:.3f}m, Lift Z target={delta_z_lift:.3f}m"
        )

        # Base Translation
        if abs(delta_x_base) > 0.01:
            print(f"[ALIGN] Translating base by {delta_x_base:.3f}m...")
            robot.base.translate_by(delta_x_base)
            robot.push_command()
            robot.wait_command()

        # Arm Extension
        current_arm_pos = robot.arm.status["pos"]
        target_arm_ext = np.clip(
            delta_y_arm - 0.11, 0.0, MAX_ARM_EXTENSION_M - current_arm_pos
        )

        if abs(target_arm_ext) > 0.01:
            print(f"[ALIGN] Extending arm by {target_arm_ext:.3f}m...")
            robot.arm.move_by(target_arm_ext)
            robot.push_command()
            robot.wait_command()

        # Dynamic Z Descent
        GRIPPER_Z_OFFSET = 0.106
        SAFETY_BUFFER_M = 0.03

        final_z_descent = delta_z_lift - GRIPPER_Z_OFFSET
        calculated_fast_descent = max(0.0, final_z_descent - SAFETY_BUFFER_M)

        print(f"[VERTICAL ALIGNMENT] Calculated descent: {calculated_fast_descent:.3f}m")

        if calculated_fast_descent > 0.01:
            robot.lift.move_by(-calculated_fast_descent)
            robot.push_command()
            robot.wait_command()

        # Guarded Descent
        admittance_guarded_descent(
            robot,
            max_descent_m=(SAFETY_BUFFER_M + 0.06),
            contact_threshold_ma=LIFT_CONTACT_THRESHOLD_MA,
        )

        print("[PICKUP] Hanger target aligned. Closing gripper...")
        robot.end_of_arm.move_to("wrist_roll", 0.0)
        robot.push_command()
        robot.wait_command()

        full_squeeze_gripper(robot)
        time.sleep(0.5)

        print("[PICKUP SUCCESS] Hanger secured in gripper.")

        # Pitch to neutral
        print("[POST-PICK] Returning wrist pitch to neutral (0.0 rad)...")
        robot.end_of_arm.move_to("wrist_pitch", POST_PICK_PITCH_RAD)
        robot.push_command()
        robot.wait_command()
        time.sleep(0.3)

        # ======================================================================
        # PHASE 2: RACK PLACEMENT
        # ======================================================================
        print("\n[4/5] Initiating Hardcoded Rack Placement Trajectory...")

        print(f"-> Retracting arm & raising lift to safe height ({POST_PICK_LIFT_SAFE_M}m)...")
        robot.arm.move_to(0.0)
        robot.lift.move_to(POST_PICK_LIFT_SAFE_M)
        robot.push_command()
        robot.wait_command()
        time.sleep(0.5)

        print(f"-> Rotating base by {RACK_BASE_ROTATE_RAD:.2f} rad...")
        robot.base.rotate_by(RACK_BASE_ROTATE_RAD)
        robot.push_command()
        robot.wait_command()
        time.sleep(1.0)

        print(f"-> Adjusting lift above rail height ({RACK_BAR_CLEARANCE_LIFT_M}m)...")
        robot.lift.move_to(RACK_BAR_CLEARANCE_LIFT_M)
        robot.end_of_arm.move_to("wrist_pitch", 0.0)
        robot.end_of_arm.move_to("wrist_yaw", 0.0)
        robot.push_command()
        robot.wait_command()
        time.sleep(0.5)

        print(f"-> Extending arm forward +{RACK_ARM_EXTENSION_M}m over rack bar...")
        robot.arm.move_to(RACK_ARM_EXTENSION_M)
        robot.push_command()
        robot.wait_command()
        time.sleep(0.5)

        print(f"-> Lowering hanger onto bar by -{RACK_HOOK_DESCENT_M}m...")
        admittance_guarded_descent(
            robot,
            max_descent_m=RACK_HOOK_DESCENT_M,
            contact_threshold_ma=LIFT_CONTACT_THRESHOLD_MA,
        )

        print("-> Releasing hanger onto garment rack...")
        open_gripper(robot, 40.0)
        time.sleep(0.5)

        print("[5/5] Retracting arm away from rack...")
        robot.arm.move_to(0.0)
        robot.push_command()
        robot.wait_command()

        # Save Final Frame
        time.sleep(0.5)
        final_frames = pipeline.wait_for_frames(5000)
        final_aligned = align.process(final_frames)
        final_color_frame = final_aligned.get_color_frame()

        if final_color_frame:
            final_color_image = np.asanyarray(final_color_frame.get_data())
            cv2.imwrite(FINAL_FRAME_PATH, final_color_image)
            print(f"[IMAGE SAVED] Final frame saved to '{FINAL_FRAME_PATH}'")

        print("\n[SUCCESS] Hanger placed successfully on rack via trajectory!")

    except KeyboardInterrupt:
        cleanup_and_exit()
    except Exception as e:
        print(f"[ERROR] Execution failed: {e}")
    finally:
        if pipeline:
            try:
                pipeline.stop()
            except Exception:
                pass

        if robot:
            try:
                robot.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()