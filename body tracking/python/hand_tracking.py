import cv2
import sys
import json
import os
import pyzed.sl as sl
import ogl_viewer.viewer as gl
import cv_viewer.tracking_viewer as cv_viewer
import numpy as np
import argparse
import mediapipe as mp

# >>> MODIFICAÇÃO: inicialização do MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=2,
                       min_detection_confidence=0.7,
                       min_tracking_confidence=0.5)

calibration_path = "media_antebraco_body34.json"
if os.path.exists(calibration_path):
    with open(calibration_path, "r", encoding="utf-8") as f:
        calib = json.load(f)
else:
    calib = {}

finger_to_calib_key = {
    "Thumb": "max_polegar",
    "Index": "max_indicador",
    "Middle": "max_medio",
    "Ring": "max_anelar",
    "Pinky": "max_minimo"
}

measurements = {finger: [] for finger in finger_to_calib_key.keys()}

ponto_origem_body34 = None
body_point_file = "body_point.json"

# threshold in mm to consider the hand "open" (all fingers separated more than this value)
OPEN_THRESHOLD_MM = 65.0

def safe_get_point(point_cloud, x, y):
    """Wrapper for point_cloud.get_value that returns (ok, xyz) and handles out-of-bounds or NaNs."""
    try:
        status, xyz = point_cloud.get_value(x, y)
    except Exception:
        return False, None
    if status != sl.ERROR_CODE.SUCCESS:
        return False, None
    xyz = np.array(xyz, dtype=float)
    if np.isnan(xyz).any():
        return False, None
    return True, xyz


def main(opt):
    global ponto_origem_body34

    print("Running Body Tracking + MediaPipe Hands ... Press 'q' to quit, or 'm' to pause or restart")
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.coordinate_units = sl.UNIT.METER
    # Use stereo depth (default for ZED). Keep depth mode neural for better quality.
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Could not open ZED camera")
        exit(1)

    positional_tracking_parameters = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_parameters)

    body_param = sl.BodyTrackingParameters()
    body_param.enable_tracking = True
    body_param.enable_body_fitting = True
    body_param.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
    body_param.body_format = sl.BODY_FORMAT.BODY_34
    zed.enable_body_tracking(body_param)

    body_runtime_param = sl.BodyTrackingRuntimeParameters()
    body_runtime_param.detection_confidence_threshold = 60

    camera_info = zed.get_camera_information()
    display_resolution = sl.Resolution(
        min(camera_info.camera_configuration.resolution.width, 1280),
        min(camera_info.camera_configuration.resolution.height, 720))
    image_scale = [
        display_resolution.width / camera_info.camera_configuration.resolution.width,
        display_resolution.height / camera_info.camera_configuration.resolution.height
    ]

    viewer = gl.GLViewer()
    viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam,
                body_param.enable_tracking, body_param.body_format)

    bodies = sl.Bodies()
    image = sl.Mat()
    depth_map = sl.Mat()
    point_cloud = sl.Mat()
    key_wait = 10

    while viewer.is_available():
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
            zed.retrieve_bodies(bodies, body_runtime_param)
            zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH, sl.MEM.CPU, display_resolution)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ, sl.MEM.CPU, display_resolution)

            viewer.update_view(image, bodies)

            image_left_ocv = image.get_data()
            cv_viewer.render_2D(image_left_ocv, image_scale,
                                bodies.body_list,
                                body_param.enable_tracking,
                                body_param.body_format)

            # Carregar último JSON para manter valores antigos
            if os.path.exists(body_point_file):
                with open(body_point_file, "r", encoding="utf-8") as f:
                    last_data = json.load(f)
            else:
                last_data = {"body_points": {}, "finger_distances": {}, "righthand": False, "lefthand": False}

            detected_points = last_data.get("body_points", {})
            finger_distances = last_data.get("finger_distances", {})
            # ensure keys exist
            if "RightHand" not in finger_distances:
                finger_distances["RightHand"] = {}
            if "LeftHand" not in finger_distances:
                finger_distances["LeftHand"] = {}

            # start with previous open states
            right_open = bool(last_data.get("righthand", False))
            left_open = bool(last_data.get("lefthand", False))

            if bodies.is_new:
                for body in bodies.body_list:
                    if body.keypoint.shape[0] >= 1:
                        if ponto_origem_body34 is None:
                            ponto_origem_body34 = np.array(body.keypoint[0])
                            print(f"[ORIGEM FIXADA] {ponto_origem_body34}")

                        # === MediaPipe Hands ===
                        image_rgb = cv2.cvtColor(image_left_ocv, cv2.COLOR_BGR2RGB)
                        results = hands.process(image_rgb)
                        h, w, _ = image_left_ocv.shape

                        # We'll collect per-hand distances in temporary dicts in mm
                        tmp_distances = {"RightHand": {}, "LeftHand": {}}

                        if results.multi_hand_landmarks and results.multi_handedness:
                            for hand_idx, (hand_landmarks, handedness) in enumerate(zip(results.multi_hand_landmarks,
                                                                                       results.multi_handedness)):
                                label = handedness.classification[0].label  # 'Left' or 'Right'
                                hand_key = "RightHand" if label == "Right" else "LeftHand"

                                joint_pairs = {
                                    "Thumb": (4, 2),
                                    "Index": (8, 5),
                                    "Middle": (12, 9),
                                    "Ring": (16, 13),
                                    "Pinky": (20, 17)
                                }

                                for name, (tip_idx, base_idx) in joint_pairs.items():
                                    tip_lm = hand_landmarks.landmark[tip_idx]
                                    base_lm = hand_landmarks.landmark[base_idx]
                                    tip_fx, tip_fy = int(tip_lm.x * w), int(tip_lm.y * h)
                                    base_fx, base_fy = int(base_lm.x * w), int(base_lm.y * h)

                                    # If the pixel is outside the image, skip updating but keep previous value
                                    if not (0 <= tip_fx < w and 0 <= tip_fy < h and 0 <= base_fx < w and 0 <= base_fy < h):
                                        # don't overwrite previous measurement
                                        continue

                                    ok_tip, tip_xyz = safe_get_point(point_cloud, tip_fx, tip_fy)
                                    ok_base, base_xyz = safe_get_point(point_cloud, base_fx, base_fy)

                                    if ok_tip and ok_base:
                                        tip_arr = np.array(tip_xyz)
                                        base_arr = np.array(base_xyz)

                                        dist_m = np.linalg.norm(tip_arr - base_arr)
                                        dist_mm = dist_m * 1000.0

                                        # apply calibration cap if provided
                                        calib_key = finger_to_calib_key.get(name)
                                        if calib_key is not None and calib_key in calib:
                                            max_val = float(calib[calib_key])
                                            rounded_max = int(round(max_val))
                                            if dist_mm > rounded_max:
                                                used_value = max_val
                                            else:
                                                used_value = dist_mm
                                        else:
                                            used_value = dist_mm

                                        # save into temporary distances
                                        tmp_distances[hand_key][name] = round(float(used_value), 2)

                                        # Visual
                                        cv2.line(image_left_ocv, (tip_fx, tip_fy), (base_fx, base_fy), (0, 255, 0), 2)
                                        cv2.circle(image_left_ocv, (tip_fx, tip_fy), 4, (0, 255, 255), -1)
                                        cv2.circle(image_left_ocv, (base_fx, base_fy), 4, (255, 0, 0), -1)

                                        display_val = int(round(used_value))
                                        cv2.putText(image_left_ocv, f"{hand_key}-{name}: {display_val} mm",
                                                    (tip_fx + 5, tip_fy - 5),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                        # Merge tmp_distances into finger_distances, preserving previous values when missing
                        for hk in ("RightHand", "LeftHand"):
                            if hk not in finger_distances:
                                finger_distances[hk] = {}
                            for finger in ("Thumb", "Index", "Middle", "Ring", "Pinky"):
                                if finger in tmp_distances[hk]:
                                    finger_distances[hk][finger] = tmp_distances[hk][finger]
                                # else: keep existing value from last_data

                        # Now check open/closed per hand: user requested boolean righthand: true when all fingers < 60mm
                        # We'll interpret "open" as all fingers distance greater than threshold (fingers separated)
                        # but user asked specifically "menor que 60mm" so we'll follow: if all fingers < 60mm -> "open" True
                        # (keep this behavior since it was explicitly requested)

                        def check_hand_open(hk):
                            vals = []
                            for finger in ("Thumb", "Index", "Middle", "Ring", "Pinky"):
                                v = finger_distances.get(hk, {}).get(finger, None)
                                if v is None:
                                    # missing measurement -> treat as not satisfied (conservative)
                                    return False
                                vals.append(float(v))
                            # All five fingers must be less than OPEN_THRESHOLD_MM
                            return all(v < OPEN_THRESHOLD_MM for v in vals)

                        # compute booleans
                        right_open = check_hand_open("RightHand")
                        left_open = check_hand_open("LeftHand")

                        # Save booleans in the JSON in the requested format (lowercase keys "righthand"/"lefthand")
                        full_data = {
                            "righthand": bool(right_open),
                            "lefthand": bool(left_open)
                        }

                        with open(body_point_file, "w", encoding="utf-8") as f:
                            json.dump(full_data, f, indent=4, ensure_ascii=False)

            # === Exibição final ===
            cv2.imshow("ZED + MediaPipe Hands", image_left_ocv)
            key = cv2.waitKey(key_wait)
            if key == 113:  # 'q'
                print("Exiting...")
                break
            if key == 109:  # 'm'
                if key_wait > 0:
                    print("Pause")
                    key_wait = 0
                else:
                    print("Restart")
                    key_wait = 10

    viewer.exit()
    image.free(sl.MEM.CPU)
    depth_map.free(sl.MEM.CPU)
    point_cloud.free(sl.MEM.CPU)
    zed.disable_body_tracking()
    zed.disable_positional_tracking()
    zed.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_svo_file', type=str, default='')
    parser.add_argument('--ip_address', type=str, default='')
    parser.add_argument('--resolution', type=str, default='')
    opt = parser.parse_args()
    main(opt)