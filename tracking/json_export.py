import cv2
import pyzed.sl as sl
import ogl_viewer.viewer as gl
import numpy as np
import json
import math
import mediapipe as mp

# ==== Mapeamento dos índices do BODY_34 ====
BODY_34_NAMES = [
    "Pelvis", "SpineNaval", "SpineChest", "Neck", "ClavicleLeft", "ShoulderLeft",
    "ElbowLeft", "WristLeft", "HandLeft", "HandTipLeft", "ThumbLeft",
    "ClavicleRight", "ShoulderRight", "ElbowRight", "WristRight", "HandRight",
    "HandTipRight", "ThumbRight", "HipLeft", "KneeLeft", "AnkleLeft", "FootLeft",
    "HeelLeft", "ToeLeft", "HipRight", "KneeRight", "AnkleRight", "FootRight",
    "HeelRight", "ToeRight", "Head", "Nose", "EyeLeft", "EyeRight"
]

# ==== Lista de juntas que queremos salvar ====
JOINTS_TO_KEEP = {"ShoulderLeft", "ElbowLeft", "ShoulderRight", "ElbowRight"}

# ==== Funções auxiliares ====
def normalize_quat(ox, oy, oz, ow):
    n = math.sqrt(ox*ox + oy*oy + oz*oz + ow*ow)
    if n == 0:
        return 0.0, 0.0, 0.0, 1.0
    return ox/n, oy/n, oz/n, ow/n

def quat_to_euler_deg(ox, oy, oz, ow):
    qx, qy, qz, qw = normalize_quat(ox, oy, oz, ow)

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def serializeBodyData(body_data):
    out = {}
    out["id"] = body_data.id
    out["confidence"] = body_data.confidence

    orientations = {}
    for i, q in enumerate(body_data.local_orientation_per_joint):
        joint_name = BODY_34_NAMES[i] if i < len(BODY_34_NAMES) else f"Joint_{i}"
        if joint_name not in JOINTS_TO_KEEP:
            continue

        ox, oy, oz, ow = q
        if [ox, oy, oz, ow] == [0.0, 0.0, 0.0, 1.0]:
            orientations[joint_name] = None
        else:
            roll, pitch, yaw = quat_to_euler_deg(ox, oy, oz, ow)
            orientations[joint_name] = {"roll": roll, "pitch": pitch, "yaw": yaw}

    out["local_orientation_euler_deg"] = orientations
    return out

def serializeBodies(bodies):
    out = {}
    out["is_tracked"] = bodies.is_tracked
    out["timestamp"] = bodies.timestamp.data_ns
    out["body_list"] = []
    for sk in bodies.body_list:
        out["body_list"].append(serializeBodyData(sk))
    return out

# ==== Função para verificar se a mão está aberta ou fechada ====
def hand_open_or_closed(hand_landmarks):
    # Índices das pontas dos dedos no MediaPipe
    tips = [4, 8, 12, 16, 20]
    closed_count = 0
    for tip in tips[1:]:  # ignorar polegar por simplicidade
        if hand_landmarks.landmark[tip].y > hand_landmarks.landmark[tip - 2].y:
            closed_count += 1
    return closed_count <= 2  # True se mão aberta, False se fechada

# ==== Inicialização da câmera ZED ====
init_params = sl.InitParameters()
init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
init_params.coordinate_units = sl.UNIT.METER
init_params.depth_mode = sl.DEPTH_MODE.NEURAL

zed = sl.Camera()
if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Não foi possível abrir a câmera.")
    exit(1)

pos_tracking_params = sl.PositionalTrackingParameters()
if zed.enable_positional_tracking(pos_tracking_params) != sl.ERROR_CODE.SUCCESS:
    print("Não foi possível habilitar o rastreamento posicional.")
    exit(1)

body_tracking_params = sl.BodyTrackingParameters()
body_tracking_params.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
body_tracking_params.body_format = sl.BODY_FORMAT.BODY_34
body_tracking_params.enable_body_fitting = True
body_tracking_params.enable_tracking = True

if zed.enable_body_tracking(body_tracking_params) != sl.ERROR_CODE.SUCCESS:
    print("Não foi possível habilitar o rastreamento de corpos.")
    exit(1)

# ==== Inicializa visualizador OpenGL ====
viewer = gl.GLViewer()
viewer.init()

skeleton_file_data = {}
bodies = sl.Bodies()

# ==== Inicialização do MediaPipe Hands ====
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5)

runtime_params = sl.RuntimeParameters()
image_zed = sl.Mat()

# ==== Loop principal ====
while viewer.is_available():
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Captura corpo (ZED)
        zed.retrieve_bodies(bodies)
        frame_data = {}
        if bodies.is_tracked:
            frame_data["body"] = serializeBodies(bodies)

        # Captura imagem RGB para MediaPipe
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        frame = image_zed.get_data()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

        results = hands.process(frame_rgb)

        # Anotar mãos
        hands_status = {"left_hand_open": True, "right_hand_open": True}
        if results.multi_handedness:
            for i, hand_handedness in enumerate(results.multi_handedness):
                label = hand_handedness.classification[0].label  # "Left" ou "Right"
                hand_landmarks = results.multi_hand_landmarks[i]
                is_open = hand_open_or_closed(hand_landmarks)
                if label == "Left":
                    hands_status["left_hand_open"] = is_open
                else:
                    hands_status["right_hand_open"] = is_open

        frame_data["hands"] = hands_status

        # Atualiza visualização
        viewer.update_bodies(bodies)

        # Salva no JSON
        skeleton_file_data["current_frame"] = frame_data
        with open("bodies.json", 'w') as f:
            json.dump(skeleton_file_data, f, indent=4)

with open("bodies.json", 'w') as f:
    json.dump(skeleton_file_data, f, indent=4)

viewer.exit()
zed.close()
