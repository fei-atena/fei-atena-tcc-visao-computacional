import cv2
import sys
import pyzed.sl as sl
import ogl_viewer.viewer as gl
import cv_viewer.tracking_viewer as cv_viewer
import numpy as np
import argparse
import csv
import json
import os
from datetime import datetime

def parse_args(init, opt):
    if len(opt.input_svo_file)>0 and opt.input_svo_file.endswith((".svo", ".svo2")):
        init.set_from_svo_file(opt.input_svo_file)
        print("[Sample] Using SVO File input: {0}".format(opt.input_svo_file))
    elif len(opt.ip_address)>0 :
        ip_str = opt.ip_address
        if ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4 and len(ip_str.split(':'))==2:
            init.set_from_stream(ip_str.split(':')[0],int(ip_str.split(':')[1]))
            print("[Sample] Using Stream input, IP : ",ip_str)
        elif ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4:
            init.set_from_stream(ip_str)
            print("[Sample] Using Stream input, IP : ",ip_str)
        else :
            print("Invalid IP format. Using live stream")
    if ("HD2K" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD2K
    elif ("HD1200" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1200
    elif ("HD1080" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1080
    elif ("HD720" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD720
    elif ("SVGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.SVGA
    elif ("VGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.VGA

def main(opt):
    print("Running Body Tracking with CSV/JSON logging ... Press 'q' to quit")

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    parse_args(init_params, opt)

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera open failed:", err)
        exit(1)

    positional_tracking_parameters = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_parameters)

    body_param = sl.BodyTrackingParameters()
    body_param.enable_tracking = True
    body_param.enable_body_fitting = False
    body_param.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
    body_param.body_format = sl.BODY_FORMAT.BODY_34
    zed.enable_body_tracking(body_param)

    body_runtime_param = sl.BodyTrackingRuntimeParameters()
    body_runtime_param.detection_confidence_threshold = 40

    camera_info = zed.get_camera_information()
    display_resolution = sl.Resolution(min(camera_info.camera_configuration.resolution.width, 1280),
                                       min(camera_info.camera_configuration.resolution.height, 720))
    image_scale = [display_resolution.width / camera_info.camera_configuration.resolution.width,
                   display_resolution.height / camera_info.camera_configuration.resolution.height]

    viewer = gl.GLViewer()
    viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam,
                body_param.enable_tracking, body_param.body_format)

    bodies = sl.Bodies()
    image = sl.Mat()
    key_wait = 10

    # === Arquivos de Saída ===
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"body_tracking_data_{timestamp}.csv"
    json_filename = f"body_tracking_data_{timestamp}.json"
    json_frames = []  # lista para armazenar dados de todos os frames

    with open(csv_filename, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["Frame", "BodyID", "PointID", "X_mm", "Y_mm", "Z_mm"])

        frame_count = 0
        while viewer.is_available():
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
                zed.retrieve_bodies(bodies, body_runtime_param)

                viewer.update_view(image, bodies)
                image_left_ocv = image.get_data()
                cv_viewer.render_2D(image_left_ocv, image_scale,
                                    bodies.body_list, body_param.enable_tracking, body_param.body_format)

                frame_data = {
                    "frame": frame_count,
                    "bodies": []
                }

                for body in bodies.body_list:
                    if body.keypoint.size > 0:
                        keypoints_m = body.keypoint
                        pelvis = keypoints_m[0]
                        keypoints_rel_mm = (keypoints_m - pelvis) * 1000.0  # relativo em mm

                        # Coleta dados deste corpo
                        body_points = []
                        for idx, p in enumerate(keypoints_rel_mm):
                            if not np.isnan(p[0]):  # verifica se é válido
                                csv_writer.writerow([frame_count, body.id, idx,
                                                     round(float(p[0]), 2),
                                                     round(float(p[1]), 2),
                                                     round(float(p[2]), 2)])
                                body_points.append({
                                    "point_id": int(idx),
                                    "x_mm": round(float(p[0]), 2),
                                    "y_mm": round(float(p[1]), 2),
                                    "z_mm": round(float(p[2]), 2)
                                })

                        frame_data["bodies"].append({
                            "body_id": int(body.id),
                            "points": body_points
                        })

                        # Visualização do ponto 0
                        x2d, y2d = body.keypoint_2d[0]
                        x2d = int(x2d * image_scale[0])
                        y2d = int(y2d * image_scale[1])
                        cv2.circle(image_left_ocv, (x2d, y2d), 8, (0, 0, 255), -1)
                        cv2.putText(image_left_ocv, "Origin (0,0,0) mm",
                                    (x2d + 10, y2d - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 0, 255), 1)

                json_frames.append(frame_data)
                frame_count += 1

                cv2.imshow("ZED | Relative Coordinates", image_left_ocv)
                key = cv2.waitKey(key_wait)
                if key == ord('q'):
                    print("Exiting...")
                    break

    # === Salva JSON ao final ===
    with open(json_filename, 'w') as jf:
        json.dump(json_frames, jf, indent=4)

    print(f"\n✅ Dados salvos em:\n   CSV  -> {os.path.abspath(csv_filename)}\n   JSON -> {os.path.abspath(json_filename)}")

    viewer.exit()
    image.free(sl.MEM.CPU)
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
    if len(opt.input_svo_file) > 0 and len(opt.ip_address) > 0:
        print("Specify only input_svo_file or ip_address, not both.")
        exit()
    main(opt)
