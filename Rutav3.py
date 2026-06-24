import cv2
import cv2.aruco as aruco
import numpy as np
import networkx as nx
import math
import paho.mqtt.client as mqtt
import os
from topicos import mqtt_topics

# ======================================================
# CONFIGURACION Y CONSTANTES DEL LABERINTO
# ======================================================
CELL_SIZE = 30                  # cm entre nodos fijos
DIST_CHECKPOINT_THRESHOLD = 8.0   # Umbral en cm para marcar arribo (Flag)
marker_length = 0.04            # 4 cm medidos en la realidad

# Detectar de forma automatica si hay un entorno grafico activo (Monitor/X11)
HAVE_DISPLAY = "DISPLAY" in os.environ

# Cargar calibraciones
try:
    camera_matrix = np.load("camera_matrix.npy")
    dist_coeffs = np.load("dist_coeffs.npy")
except FileNotFoundError:
    print("Error: No se encontraron los archivos 'camera_matrix.npy' o 'dist_coeffs.npy'.")
    exit()

# ======================================================
# DICCIONARIO DE POSICIONES
# ======================================================
positions = {
    "00": (0*CELL_SIZE, 0*CELL_SIZE), "01": (0*CELL_SIZE, 1*CELL_SIZE), "02": (0*CELL_SIZE, 2*CELL_SIZE), "03": (0*CELL_SIZE, 3*CELL_SIZE), "04": (0*CELL_SIZE, 4*CELL_SIZE),
    "10": (1*CELL_SIZE, 0*CELL_SIZE), "11": (1*CELL_SIZE, 1*CELL_SIZE), "12": (1*CELL_SIZE, 2*CELL_SIZE), "13": (1*CELL_SIZE, 3*CELL_SIZE), "14": (1*CELL_SIZE, 4*CELL_SIZE),
    "20": (2*CELL_SIZE, 0*CELL_SIZE), "21": (2*CELL_SIZE, 1*CELL_SIZE), "22": (2*CELL_SIZE, 2*CELL_SIZE), "23": (2*CELL_SIZE, 3*CELL_SIZE), "24": (2*CELL_SIZE, 4*CELL_SIZE),
    "30": (3*CELL_SIZE, 0*CELL_SIZE), "31": (3*CELL_SIZE, 1*CELL_SIZE), "32": (3*CELL_SIZE, 2*CELL_SIZE), "33": (3*CELL_SIZE, 3*CELL_SIZE), "34": (3*CELL_SIZE, 4*CELL_SIZE),
    "40": (4*CELL_SIZE, 0*CELL_SIZE), "41": (4*CELL_SIZE, 1*CELL_SIZE), "42": (4*CELL_SIZE, 2*CELL_SIZE), "43": (4*CELL_SIZE, 3*CELL_SIZE), "44": (4*CELL_SIZE, 4*CELL_SIZE),
    "50": (5*CELL_SIZE, 0*CELL_SIZE), "51": (5*CELL_SIZE, 1*CELL_SIZE), "52": (5*CELL_SIZE, 2*CELL_SIZE), "53": (5*CELL_SIZE, 3*CELL_SIZE), "54": (5*CELL_SIZE, 4*CELL_SIZE),
}

# ======================================================
# FORMATO DE ARUCOS (SISTEMA UNIFICADO DE ANGULOS)
# ======================================================
ARUCO_MAP = {
    0: {"nodo": "02", "eje": "y", "giro": 90},
    1: {"nodo": "02", "eje": "x", "giro": 0},
    2: {"nodo": "00", "eje": "y", "giro": 270},
    3: {"nodo": "30", "eje": "y", "giro": 270},
    4: {"nodo": "30", "eje": "x", "giro": 0},
    5: {"nodo": "32", "eje": "x", "giro": 0},
    6: {"nodo": "32", "eje": "y", "giro": 90},
    7: {"nodo": "12", "eje": "x", "giro": 180},
    8: {"nodo": "11", "eje": "x", "giro": 180},
    9: {"nodo": "11", "eje": "y", "giro": 270},
    10: {"nodo": "21", "eje": "y", "giro": 270},
    11: {"nodo": "21", "eje": "x", "giro": 0},
    12: {"nodo": "13", "eje": "x", "giro": 0},
    13: {"nodo": "13", "eje": "y", "giro": 90},
    14: {"nodo": "03", "eje": "y", "giro": 270},
    15: {"nodo": "03", "eje": "x", "giro": 180},
    16: {"nodo": "04", "eje": "x", "giro": 180},
    17: {"nodo": "04", "eje": "y", "giro": 90},
    18: {"nodo": "23", "eje": "x", "giro": 180},
    19: {"nodo": "34", "eje": "x", "giro": 0},
    20: {"nodo": "43", "eje": "y", "giro": 270},
    21: {"nodo": "43", "eje": "x", "giro": 0},
    22: {"nodo": "44", "eje": "y", "giro": 90},
    23: {"nodo": "54", "eje": "y", "giro": 90},
    24: {"nodo": "54", "eje": "x", "giro": 0},
    25: {"nodo": "50", "eje": "x", "giro": 0},
    26: {"nodo": "50", "eje": "y", "giro": 270},
    27: {"nodo": "40", "eje": "y", "giro": 270},
    28: {"nodo": "40", "eje": "x", "giro": 180},
    29: {"nodo": "42", "eje": "y", "giro": 90},
    30: {"nodo": "00", "eje": "x", "giro": 180},
    31: {"nodo": "24", "eje": "y", "giro": 90},
}

# ======================================================
# CONFIGURACION DEL GRAFO BASE PARA A*
# ======================================================
graph_connections = {
    "00": ["10", "01"], "01": ["00", "02"], "02": ["01"], "03": ["04", "13"], "04": ["03", "14"],
    "10": ["00", "20"], "11": ["12", "21", "22"], "12": ["11", "21", "13", "22"], "13": ["03", "12"], "14": ["04", "24"],
    "20": ["10", "30"], "21": ["22", "11", "12"], "22": ["21", "32", "12", "11", "23"], "23": ["24", "33", "22"], "24": ["14", "23", "34"],
    "30": ["31", "20"], "31": ["30", "32"], "32": ["22", "31"], "33": ["23", "43"], "34": ["24"],
    "40": ["50", "41"], "41": ["40", "42"], "42": ["41"], "43": ["33", "44"], "44": ["43", "54"],
    "50": ["40", "51"], "51": ["50", "52"], "52": ["51", "53"], "53": ["52", "54"], "54": ["53", "44"],
}

G = nx.Graph()
for node, neighbors in graph_connections.items():
    for neighbor in neighbors:
        G.add_edge(node, neighbor)

# ======================================================
# FUNCIONES AUXILIARES
# ======================================================
def heuristic(a, b):
    x1, y1 = positions[a]
    x2, y2 = positions[b]
    return abs(x1 - x2) + abs(y1 - y2)

def nodo_mas_cercano(x_cm, y_cm):
    mejor_nodo = None
    mejor_distancia = float("inf")
    for nodo, (nx_val, ny_val) in positions.items():
        distancia = math.sqrt((x_cm - nx_val)**2 + (y_cm - ny_val)**2)
        if distancia < mejor_distancia:
            mejor_distancia = distancia
            mejor_nodo = nodo
    return mejor_nodo

def obtener_angulo_rvec(rvec):
    R, _ = cv2.Rodrigues(rvec)
    yaw = math.atan2(R[1, 0], R[0, 0])
    return math.degrees(yaw)

# ======================================================
# CONFIGURACION CLIENTE MQTT
# ======================================================
try:
    mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
except AttributeError:
    mqtt_client = mqtt.Client() 

mqtt_client.connect("localhost", 1883)
mqtt_client.loop_start()

destino_final = input("Ingrese el nodo de destino final: ").strip()
if destino_final not in positions:
    print("Nodo invalido, utilizando '54' por defecto.")
    destino_final = "54"

# ======================================================
# INICIALIZACION EXCLUSIVA PARA CAMARA USB
# ======================================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Error Critico: No se pudo abrir la camara USB")
    exit()

# --- SISTEMA DE DETECCION MULTIVERSION ---
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector_moderno = None
legacy_parameters = None

if hasattr(aruco, 'ArucoDetector'):
    legacy_parameters = aruco.DetectorParameters()
    detector_moderno = aruco.ArucoDetector(dictionary, legacy_parameters)
else:
    legacy_parameters = aruco.DetectorParameters_create() if hasattr(aruco, 'DetectorParameters_create') else aruco.DetectorParameters()

obj_points = np.array([
    [-marker_length/2,  marker_length/2, 0],
    [ marker_length/2,  marker_length/2, 0],
    [ marker_length/2, -marker_length/2, 0],
    [-marker_length/2, -marker_length/2, 0]
], dtype=np.float32)

# Variables globales de tracking del robot
x_estimada_cm, y_estimada_cm = 0.0, 0.0
angulo_robot_grados = 0.0
offset_x, offset_y = 0.0, 0.0 

if HAVE_DISPLAY:
    print("Entorno grafico detectado. Mostrando ventanas...")
else:
    print("Corriendo en modo remoto/headless (Sin ventana grafica). Procesando datos...")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if detector_moderno is not None:
            corners, ids, rejected = detector_moderno.detectMarkers(frame)
        else:
            corners, ids, rejected = aruco.detectMarkers(frame, dictionary, parameters=legacy_parameters)

        if ids is not None:
            if HAVE_DISPLAY:
                aruco.drawDetectedMarkers(frame, corners, ids)
                
            ids_flatten = ids.flatten()

            for i, marker_id in enumerate(ids_flatten):
                if marker_id in ARUCO_MAP:
                    config_aruco = ARUCO_MAP[marker_id]
                    nodo_baliza = config_aruco["nodo"]
                    giro_baliza = config_aruco["giro"]

                    x_baliza_cm, y_baliza_cm = positions[nodo_baliza]

                    marker_corners = corners[i][0]
                    _, rvec, tvec = cv2.solvePnP(
                        obj_points, marker_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
                    )

                    desviacion_lateral_cm = tvec[0][0] * 100.0
                    profundidad_cm = tvec[2][0] * 100.0

                    yaw_relativo = obtener_angulo_rvec(rvec)
                    angulo_robot_grados = (giro_baliza + yaw_relativo) % 360

                    # --- LOCALIZACION CON ANGULOS ABSOLUTOS ---
                    if giro_baliza == 0:        
                        x_estimada_cm = x_baliza_cm - profundidad_cm
                        y_estimada_cm = y_baliza_cm - desviacion_lateral_cm
                    elif giro_baliza == 90:     
                        x_estimada_cm = x_baliza_cm + desviacion_lateral_cm
                        y_estimada_cm = y_baliza_cm - profundidad_cm
                    elif giro_baliza == 180:    
                        x_estimada_cm = x_baliza_cm + profundidad_cm
                        y_estimada_cm = y_baliza_cm + desviacion_lateral_cm
                    elif giro_baliza == 270:    
                        x_estimada_cm = x_baliza_cm - desviacion_lateral_cm
                        y_estimada_cm = y_baliza_cm + profundidad_cm

                    if HAVE_DISPLAY:
                        if hasattr(cv2, 'drawFrameAxes'):
                            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.04)
                        elif hasattr(aruco, 'drawAxis'):
                            aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.04)
                    break 

        x_final_cm = x_estimada_cm + offset_x
        y_final_cm = y_estimada_cm + offset_y

        # ======================================================
        # RESOLVEDOR DE TRAYECTORIA CON A*
        # ======================================================
        nodo_actual = nodo_mas_cercano(x_final_cm, y_final_cm)
        
        try:
            ruta = nx.astar_path(G, nodo_actual, destino_final, heuristic=heuristic)
            siguiente_nodo = ruta[1] if len(ruta) > 1 else ruta[0]
            x_ref_cm, y_ref_cm = positions[siguiente_nodo]

            distancia_al_nodo = math.sqrt((x_ref_cm - x_final_cm)**2 + (y_ref_cm - y_final_cm)**2)
            flag_pos_checkpoint = 1 if distancia_al_nodo <= DIST_CHECKPOINT_THRESHOLD else 0
            dist_pared_frente_teorica = distancia_al_nodo + (CELL_SIZE / 2.0)

            # ======================================================
            # ENVIO DE DATOS EN BASE A LOS TÓPICOS PLANOS IMPORTADOS
            # ======================================================
            
            # 1. Envío de Comandos Individuales
            mqtt_client.publish(mqtt_topics["comandos"]["x_ref"], int(x_ref_cm))
            mqtt_client.publish(mqtt_topics["comandos"]["y_ref"], int(y_ref_cm))
            
            # 2. Envío de Flags / Estados Individuales
            mqtt_client.publish(mqtt_topics["estados"]["flag_pos"], flag_pos_checkpoint)

            # 3. Envío Individual a los Tópicos de la Categoría Cámara
            mqtt_client.publish(mqtt_topics["camara"]["x_cam"], int(round(x_final_cm)))
            mqtt_client.publish(mqtt_topics["camara"]["y_cam"], int(round(y_final_cm)))
            mqtt_client.publish(mqtt_topics["camara"]["theta_cam"], float(round(angulo_robot_grados, 2)))
            mqtt_client.publish(mqtt_topics["camara"]["d_pared_cam"], float(round(dist_pared_frente_teorica, 2)))
            mqtt_client.publish(mqtt_topics["camara"]["nodo_actual"], nodo_actual)
            mqtt_client.publish(mqtt_topics["camara"]["siguiente_nodo"], siguiente_nodo)

            # --- PRINTS DE MONITOREO MQTT POR CONSOLA ---
            print(f"[MQTT] Refs -> x_ref: {int(x_ref_cm)} | y_ref: {int(y_ref_cm)} | flag_pos: {flag_pos_checkpoint}")
            print(f"[MQTT] Camara -> Pos: ({int(x_final_cm)},{int(y_final_cm)}) | Ang: {round(angulo_robot_grados, 1)}° | Siguiente: {siguiente_nodo}")
            
            if flag_pos_checkpoint == 1:
                print(f"      >>> [CHECKPOINT] ¡Nodo {nodo_actual} alcanzado exitosamente! <<<")

            if HAVE_DISPLAY:
                info_corredor = f"Nodo: {nodo_actual} | Pos: ({int(x_final_cm)},{int(y_final_cm)}) cm | Ang: {int(angulo_robot_grados)} deg"
                info_target = f"A* -> Siguiente Target: {siguiente_nodo}"
                info_sensores = f"Offset X: {offset_x} | Offset Y: {offset_y}"
                
                cv2.putText(frame, info_corredor, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, info_target, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, info_sensores, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 150, 0), 1)
                
                if flag_pos_checkpoint:
                    cv2.putText(frame, "[CHECKPOINT ALCANZADO]", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                cv2.imshow("Camara Integrada Robot - Lector ArUco Map & A*", frame)

        except nx.NetworkXNoPath:
            if HAVE_DISPLAY:
                cv2.putText(frame, "ALERTA: RUTA BLOQUEADA EN EL GRAFO", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("Camara Integrada Robot - Lector ArUco Map & A*", frame)
            else:
                print(f"[ALERTA GRAFO] No existe ruta disponible desde {nodo_actual} hasta {destino_final}")

        # ======================================================
        # CAPTURA DE TECLADO
        # ======================================================
        if HAVE_DISPLAY:
            key = cv2.waitKey(1) & 0xFF
            if key == 27: 
                break
            elif key == 82 or key == ord('w'):   
                offset_y -= 2.0
            elif key == 84 or key == ord('s'): 
                offset_y += 2.0
            elif key == 81 or key == ord('a'): 
                offset_x -= 2.0
            elif key == 83 or key == ord('d'): 
                offset_x += 2.0
        else:
            import time
            time.sleep(0.01)

finally:
    cap.release()
    if HAVE_DISPLAY:
        cv2.destroyAllWindows()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    print("Modulo optico desconectado correctamente.")
