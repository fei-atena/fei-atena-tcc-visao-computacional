import cv2
import pyzed.sl as sl
import ogl_viewer.viewer as gl
import cv_viewer.tracking_viewer as cv_viewer
import numpy as np
import argparse
import time
import json

def parse_args(init, opt):
    # Configura a fonte do vídeo conforme os argumentos passados (SVO file, IP stream ou câmera ao vivo)
    if len(opt.input_svo_file) > 0 and opt.input_svo_file.endswith((".svo", ".svo2")):
        init.set_from_svo_file(opt.input_svo_file)
        print("[Sample] Using SVO File input: {0}".format(opt.input_svo_file))
    elif len(opt.ip_address) > 0:
        ip_str = opt.ip_address
        if ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.')) == 4 and len(ip_str.split(':')) == 2:
            init.set_from_stream(ip_str.split(':')[0], int(ip_str.split(':')[1]))
            print("[Sample] Using Stream input, IP : ", ip_str)
        elif ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.')) == 4:
            init.set_from_stream(ip_str)
            print("[Sample] Using Stream input, IP : ", ip_str)
        else:
            print("Unvalid IP format. Using live stream")
    # Configura resolução da câmera conforme entrada, ou usa padrão
    if ("HD2K" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD2K
        print("[Sample] Using Camera in resolution HD2K")
    elif ("HD1200" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1200
        print("[Sample] Using Camera in resolution HD1200")
    elif ("HD1080" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1080
        print("[Sample] Using Camera in resolution HD1080")
    elif ("HD720" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD720
        print("[Sample] Using Camera in resolution HD720")
    elif ("SVGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.SVGA
        print("[Sample] Using Camera in resolution SVGA")
    elif ("VGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.VGA
        print("[Sample] Using Camera in resolution VGA")
    elif len(opt.resolution) > 0:
        print("[Sample] No valid resolution entered. Using default")
    else:
        print("[Sample] Using default resolution")

def distance_3d(p1, p2):
    # Calcula a distância Euclidiana 3D entre dois pontos p1 e p2
    return np.linalg.norm(np.array(p1) - np.array(p2))

def main(opt):
    print("Running Body Tracking (BODY_34) sample ... Press 'q' to quit")

    zed = sl.Camera()

    # Parâmetros iniciais da câmera
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.coordinate_units = sl.UNIT.METER  # Unidade em metros
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.camera_fps = 15  # 15 FPS conforme pedido
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

    # Configura fonte de vídeo/resolução conforme argumentos
    parse_args(init_params, opt)

    # Abre a câmera com os parâmetros configurados
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera")
        exit(1)

    # Habilita o rastreamento posicional (necessário para o rastreamento corporal)
    positional_tracking_parameters = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_parameters)

    # Configura parâmetros do rastreamento corporal
    body_param = sl.BodyTrackingParameters()
    body_param.enable_tracking = True  # Habilita rastreamento contínuo
    body_param.enable_body_fitting = False  # Desabilita suavização do esqueleto
    body_param.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
    body_param.body_format = sl.BODY_FORMAT.BODY_34  # Usa modelo BODY_34

    # Ativa o rastreamento corporal com os parâmetros acima
    zed.enable_body_tracking(body_param)

    body_runtime_param = sl.BodyTrackingRuntimeParameters()
    body_runtime_param.detection_confidence_threshold = 40  # Limite de confiança para considerar detecção válida

    # Informação da câmera para configuração de visualização
    camera_info = zed.get_camera_information()
    display_resolution = sl.Resolution(min(camera_info.camera_configuration.resolution.width, 1280),
                                       min(camera_info.camera_configuration.resolution.height, 720))
    image_scale = [display_resolution.width / camera_info.camera_configuration.resolution.width,
                   display_resolution.height / camera_info.camera_configuration.resolution.height]

    # Inicializa o visualizador OpenGL para exibir esqueleto 3D
    viewer = gl.GLViewer()
    viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam,
                body_param.enable_tracking,
                body_param.body_format)

    bodies = sl.Bodies()
    image = sl.Mat()

    # === Variáveis para acumular soma das distâncias e contar número de medições ===
    soma_antebraco = 0.0    # Acumula soma dos comprimentos do antebraço em metros
    num_medidas = 0         # Contador de quantas medições válidas foram feitas

    start_time = time.time()
    duration = 5.0          # Tempo de captura: 5 segundos
    key_wait = 1            # Tempo de espera para cv2.waitKey()

    def ponto_valido(p):
        # Considera um ponto válido se sua distância do zero for maior que 1mm (0.001m)
        return np.linalg.norm(p) > 0.001

    # Loop principal roda por 5 segundos ou até a janela OpenGL fechar
    while time.time() - start_time < duration and viewer.is_available():
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Recupera imagem RGB para visualização
            zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
            # Recupera dados dos corpos detectados
            zed.retrieve_bodies(bodies, body_runtime_param)

            # Atualiza visualização 3D (esqueleto) na janela OpenGL
            viewer.update_view(image, bodies)

            # Prepara imagem para visualização 2D com keypoints e caixas delimitadoras
            image_ocv = image.get_data()
            cv_viewer.render_2D(image_ocv, image_scale, bodies.body_list,
                                body_param.enable_tracking,
                                body_param.body_format)
            cv2.imshow("ZED | 2D View", image_ocv)

            # Se há pelo menos um corpo detectado
            if len(bodies.body_list) > 0:
                body = bodies.body_list[0]  # Pega o primeiro corpo detectado
                keypoints = body.keypoint  # Lista dos keypoints 3D

                # Percorre os dois braços (direito e esquerdo)
                for side in ['right', 'left']:
                    if side == 'right':
                        idx_elbow, idx_wrist = 6, 7  # Índices dos keypoints para cotovelo e punho direito
                    else:
                        idx_elbow, idx_wrist = 13,14  # Índices para cotovelo e punho esquerdo

                    p_elbow = keypoints[idx_elbow]  # Posição 3D cotovelo
                    p_wrist = keypoints[idx_wrist]  # Posição 3D punho

                    # Confirma que os pontos são válidos (não zero)
                    if ponto_valido(p_elbow) and ponto_valido(p_wrist):
                        dist = distance_3d(p_elbow, p_wrist)  # Calcula distância entre cotovelo e punho

                        # === Acumula distância na soma total e incrementa contador de medidas ===
                        soma_antebraco += dist
                        num_medidas += 1

            # Aguarda tecla; 'q' para sair antes dos 5 segundos
            key = cv2.waitKey(key_wait)
            if key == ord('q'):
                print("User requested exit")
                break

    # Após o loop, calcula a média em milímetros se houver medições
    if num_medidas > 0:
        media_antebraco_mm = (soma_antebraco / num_medidas) * 1000  # Converte metros para milímetros
        print(f"Média do comprimento do antebraço (mm): {media_antebraco_mm:.2f}")
        media_palma = media_antebraco_mm * 0.7
        max_polegar = 0.4 * media_palma
        max_indicador = 0.4 * media_palma
        max_medio = 0.44 * media_palma
        max_anelar = 0.42 * media_palma
        max_minimo = 0.34 * media_palma

        # Salva resultado em arquivo JSON
        resultado = {
            "media_antebraco_milimetros": media_antebraco_mm,
            "max_polegar": max_polegar,
            "max_indicador": max_indicador,
            "max_medio": max_medio,
            "max_anelar": max_anelar,
            "max_minimo": max_minimo,
            "num_medidas": num_medidas
        }
        with open("media_antebraco_body34.json", "w") as f:
            json.dump(resultado, f, indent=4)
            print("Média salva no arquivo media_antebraco_body34.json")
    else:
        print("Nenhum dado válido foi coletado.")

    # Limpeza e encerramento dos recursos
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
        print("Specify only input_svo_file or ip_address, or none to use wired camera, not both. Exit program")
        exit()
    main(opt)
