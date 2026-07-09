import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
import cv2
import math
import numpy as np
import networkx as nx
import time
import torch
import paho.mqtt.client as mqtt

# Importamos el diccionario de topicos desde tu archivo local topicos.py
from topicos import mqtt_topics


class SistemaNavegacionHeadlessTorch:
    def __init__(self, tiempo_espera_obstaculo=30, broker_address="localhost", broker_port=1883):
        # --------------------------------------------------
        # CONFIGURACION Y CONSTANTES DEL LABERINTO
        # --------------------------------------------------
        self.CELL_SIZE = 30  # cm por celda
        self.nodo_destino = None  # Se inicializa vacio, llegara via MQTT
        self.UMBRAL_TOLERANCIA = 2.0  # Umbral unico de 2 cm para todos los nodos

        # Nodos especiales para activar el flag de sensorizacion
        self.NODOS_FLAG_SEN = {"01", "10", "20", "31", "22", "14", "33", "53", "52", "51", "40"}

        # Telemetria: se actualiza dinamicamente via MQTT callbacks
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.nodo_actual_fijo = None

        # Parametros de calibracion de camara
        self.CAM_HEIGHT_CM = 15.0
        self.CAM_PITCH_RAD = math.radians(20.0)
        self.FOCAL_LENGTH_PX = 910.0
        self.PRINCIPAL_POINT_Y = 360.0
        self.PRINCIPAL_POINT_X = 640.0  # Centro optico en X para calcular desviacion lateral

        # Clases COCO mapeadas para YOLOv5
        self.CLASES_OBSTACULO = {0, 24, 26, 28, 32, 39, 41, 67, 73}
        self.clases_nombres = {0: "persona", 24: "mochila", 26: "paraguas", 28: "bolso", 32: "pelota", 67: "mesa", 73: "libro"}

        # --------------------------------------------------
        # ESTRUCTURACION DEL GRAFO
        # --------------------------------------------------
        self.graph_connections = {
            "00": ["10", "01"], "01": ["00", "02"], "02": ["01"], "03": ["04", "13"], "04": ["03", "14"],
            "10": ["00", "20"], "11": ["12", "21"], "12": ["11", "13", "22"], "13": ["03", "12"], "14": ["04", "24"],
            "20": ["10", "30"], "21": ["22", "11"], "22": ["21", "32", "12", "23"], "23": ["24", "33", "22"], "24": ["14", "23", "34"],
            "30": ["31", "20"], "31": ["30", "32"], "32": ["22", "31"], "33": ["23", "43"], "34": ["24"],
            "40": ["50", "41"], "41": ["40", "42"], "42": ["41"], "43": ["33", "44"], "44": ["43", "54"],
            "50": ["40", "51"], "51": ["50", "52"], "52": ["51", "53"], "53": ["52", "54"], "54": ["53", "44"],
        }

        self.positions = {
            "00": (0*self.CELL_SIZE, 0*self.CELL_SIZE), "01": (0*self.CELL_SIZE, 1*self.CELL_SIZE), "02": (0*self.CELL_SIZE, 2*self.CELL_SIZE), "03": (0*self.CELL_SIZE, 3*self.CELL_SIZE), "04": (0*self.CELL_SIZE, 4*self.CELL_SIZE),
            "10": (1*self.CELL_SIZE, 0*self.CELL_SIZE), "11": (1*self.CELL_SIZE, 1*self.CELL_SIZE), "12": (1*self.CELL_SIZE, 2*self.CELL_SIZE), "13": (1*self.CELL_SIZE, 3*self.CELL_SIZE), "14": (1*self.CELL_SIZE, 4*self.CELL_SIZE),
            "20": (2*self.CELL_SIZE, 0*self.CELL_SIZE), "21": (2*self.CELL_SIZE, 1*self.CELL_SIZE), "22": (2*self.CELL_SIZE, 2*self.CELL_SIZE), "23": (2*self.CELL_SIZE, 3*self.CELL_SIZE), "24": (2*self.CELL_SIZE, 4*self.CELL_SIZE),
            "30": (3*self.CELL_SIZE, 0*self.CELL_SIZE), "31": (3*self.CELL_SIZE, 1*self.CELL_SIZE), "32": (3*self.CELL_SIZE, 2*self.CELL_SIZE), "33": (3*self.CELL_SIZE, 3*self.CELL_SIZE), "34": (3*self.CELL_SIZE, 4*self.CELL_SIZE),
            "40": (4*self.CELL_SIZE, 0*self.CELL_SIZE), "41": (4*self.CELL_SIZE, 1*self.CELL_SIZE), "42": (4*self.CELL_SIZE, 2*self.CELL_SIZE), "43": (4*self.CELL_SIZE, 3*self.CELL_SIZE), "44": (4*self.CELL_SIZE, 4*self.CELL_SIZE),
            "50": (5*self.CELL_SIZE, 0*self.CELL_SIZE), "51": (5*self.CELL_SIZE, 1*self.CELL_SIZE), "52": (5*self.CELL_SIZE, 2*self.CELL_SIZE), "53": (5*self.CELL_SIZE, 3*self.CELL_SIZE), "54": (5*self.CELL_SIZE, 4*self.CELL_SIZE),
        }

        self.G = nx.Graph()
        self.construir_grafo_base()

        # --------------------------------------------------
        # MAQUINA DE ESTADOS PARA OBSTACULOS DINAMICOS
        # --------------------------------------------------
        self.tiempo_espera_obstaculo = float(tiempo_espera_obstaculo)
        self.estado_obstaculo = "LIBRE"
        self.nodo_en_espera = None
        self.timestamp_inicio_espera = None

        # --------------------------------------------------
        # CONFIGURACION CLIENTE MQTT
        # --------------------------------------------------
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(broker_address, broker_port, 60)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] Conectado al broker con codigo de resultado: {rc}")
        client.subscribe(mqtt_topics["telemetria"]["x"])
        client.subscribe(mqtt_topics["telemetria"]["y"])
        client.subscribe(mqtt_topics["telemetria"]["teta"])
        # Suscripcion al nuevo topico de nodo destino dinamico
        client.subscribe("robot/comandos/nodo_des")

    def _on_message(self, client, userdata, msg):
        try:
            payload_str = msg.payload.decode().strip()
            
            if msg.topic == "robot/comandos/nodo_des":
                if payload_str in self.positions:
                    self.nodo_destino = payload_str
                    print(f"[MQTT] Nuevo nodo destino seleccionado: {self.nodo_destino}")
                else:
                    print(f"[MQTT AVISO] El nodo recibido '{payload_str}' no existe en el mapa.")
                return

            payload = float(payload_str)
            if msg.topic == mqtt_topics["telemetria"]["x"]:
                self.robot_x = payload
            elif msg.topic == mqtt_topics["telemetria"]["y"]:
                self.robot_y = payload
            elif msg.topic == mqtt_topics["telemetria"]["teta"]:
                self.robot_theta = payload
        except Exception as e:
            print(f"[MQTT ERROR] Error decodificando mensaje en {msg.topic}: {e}")

    def construir_grafo_base(self):
        self.G.clear()
        for node in self.positions:
            self.G.add_node(node)
        for node, neighbors in self.graph_connections.items():
            for neighbor in neighbors:
                if neighbor in self.graph_connections and node in self.graph_connections[neighbor]:
                    self.G.add_edge(node, neighbor)

    def heuristic(self, a, b):
        x1, y1 = self.positions[a]
        x2, y2 = self.positions[b]
        return abs(x1 - x2) + abs(y1 - y2)

    def nodo_mas_cercano(self, x_cm, y_cm):
        mejor_nodo = None
        mejor_distancia = float("inf")
        for nodo, (nx_val, ny_val) in self.positions.items():
            distancia = math.sqrt((x_cm - nx_val)**2 + (y_cm - ny_val)**2)
            if distancia < mejor_distancia:
                mejor_distancia = distancia
                mejor_nodo = nodo
        return mejor_nodo

    def calcular_nodo_actual_hibrido(self, x_robot, y_robot):
        for nodo, (x_teorico, y_teorico) in self.positions.items():
            if (abs(x_robot - x_teorico) <= self.UMBRAL_TOLERANCIA) and (abs(y_robot - y_teorico) <= self.UMBRAL_TOLERANCIA):
                return nodo
        return self.nodo_actual_fijo if self.nodo_actual_fijo else self.nodo_mas_cercano(x_robot, y_robot)

    def ejecutar_ruteo(self, nodo_bloqueado=None):
        self.nodo_actual_fijo = self.calcular_nodo_actual_hibrido(self.robot_x, self.robot_y)
        self.construir_grafo_base()

        # LOGICA DE CONTROL PARA EL FLAG EN NODOS ESPECIFICOS
        topico_flag_sen = mqtt_topics["estados"].get("flag_sen", "robot/estados/flag_sen")
        if self.nodo_actual_fijo in self.NODOS_FLAG_SEN:
            self.client.publish(topico_flag_sen, "1")
        else:
            self.client.publish(topico_flag_sen, "0")

        self.client.publish(mqtt_topics["camara"]["nodo_actual"], str(self.nodo_actual_fijo))

        # Si aun no se recibe un nodo destino por MQTT, pausamos el ruteo
        if not self.nodo_destino:
            print("[A*] Esperando asignacion de nodo_destino via MQTT...")
            return

        x_meta, y_meta = self.positions[self.nodo_destino]
        if (abs(self.robot_x - x_meta) <= self.UMBRAL_TOLERANCIA) and (abs(self.robot_y - y_meta) <= self.UMBRAL_TOLERANCIA):
            print("[A*] Meta del laberinto alcanzada con exito")
            self.client.publish(mqtt_topics["estados"]["flag_pos"], "1")
            self.client.publish(mqtt_topics["camara"]["siguiente_nodo"], str(self.nodo_destino))
            return
        else:
            self.client.publish(mqtt_topics["estados"]["flag_pos"], "0")

        if nodo_bloqueado and nodo_bloqueado in self.G:
            if nodo_bloqueado != self.nodo_actual_fijo and nodo_bloqueado != self.nodo_destino:
                self.G.remove_node(nodo_bloqueado)
                print(f"[A*] Modificando mapa: Evitando Nodo bloqueado {nodo_bloqueado}")

        try:
            ruta = nx.astar_path(self.G, self.nodo_actual_fijo, self.nodo_destino, heuristic=self.heuristic)
            nodo_siguiente = ruta[1] if len(ruta) > 1 else self.nodo_actual_fijo
            x_ref, y_ref = self.positions[nodo_siguiente]

            self.client.publish(mqtt_topics["comandos"]["x_ref"], str(x_ref))
            self.client.publish(mqtt_topics["comandos"]["y_ref"], str(y_ref))
            self.client.publish(mqtt_topics["camara"]["siguiente_nodo"], str(nodo_siguiente))

            print(f"[DATOS] Nodo Actual: {self.nodo_actual_fijo} -> Siguiente Nodo: {nodo_siguiente} | Refs: ({x_ref}, {y_ref}) cm")
        except nx.NetworkXNoPath:
            print(f"[AVISO CRITICO] No existe ruta viable a {self.nodo_destino}. Mapa bloqueado.")

    def gestionar_deteccion_obstaculo(self, objeto_detectado, distancia_cm=None, nodo_objeto=None, clase_nombre=None):
        ahora = time.time()

        topico_nodo_obs = mqtt_topics["camara"].get("nodo_obs", "robot/camara/nodo_obs")
        topico_dist_obs = mqtt_topics["camara"].get("dist_obs", "robot/camara/dist_obs")
        topico_nodo_bloq = mqtt_topics["camara"].get("nodo_bloq", "robot/camara/nodo_bloq")

        if objeto_detectado and nodo_objeto:
            self.client.publish(topico_nodo_obs, str(nodo_objeto))
            self.client.publish(topico_dist_obs, f"{distancia_cm:.1f}")

            if self.estado_obstaculo == "LIBRE":
                self.estado_obstaculo = "ESPERANDO"
                self.nodo_en_espera = nodo_objeto
                self.timestamp_inicio_espera = ahora
                self.client.publish(mqtt_topics["estados"]["flag_obs"], "0")
                self.client.publish(topico_nodo_bloq, "ninguno")
                print(f"[OBSTACULO] Objeto ({clase_nombre}) en nodo {nodo_objeto}. Esperando {self.tiempo_espera_obstaculo}s...")
                return None

            elif self.estado_obstaculo in ("ESPERANDO", "BLOQUEADO_CONFIRMADO") and self.nodo_en_espera == nodo_objeto:
                transcurrido = ahora - self.timestamp_inicio_espera
                if transcurrido >= self.tiempo_espera_obstaculo:
                    self.estado_obstaculo = "BLOQUEADO_CONFIRMADO"
                    self.client.publish(mqtt_topics["estados"]["flag_obs"], "1")
                    self.client.publish(topico_nodo_bloq, str(nodo_objeto))
                    return nodo_objeto
                else:
                    self.client.publish(mqtt_topics["estados"]["flag_obs"], "0")
                    self.client.publish(topico_nodo_bloq, "ninguno")
                    return None
            else:
                self.estado_obstaculo = "ESPERANDO"
                self.nodo_en_espera = nodo_objeto
                self.timestamp_inicio_espera = ahora
                self.client.publish(mqtt_topics["estados"]["flag_obs"], "0")
                self.client.publish(topico_nodo_bloq, "ninguno")
                return None
        else:
            self.estado_obstaculo = "LIBRE"
            self.nodo_en_espera = None
            self.timestamp_inicio_espera = None
            self.client.publish(mqtt_topics["estados"]["flag_obs"], "0")
            self.client.publish(topico_nodo_obs, "ninguno")
            self.client.publish(topico_dist_obs, "-1")
            self.client.publish(topico_nodo_bloq, "ninguno")
            return None


if __name__ == "__main__":
    print("=== MODO HEADLESS SEGURO - CONEXION MQTT ACTIVA ===")
    tiempo_espera_input = input("Tiempo de espera ante un obstaculo en segundos (Default 10.0): ").strip()
    tiempo_espera_obstaculo = float(tiempo_espera_input) if tiempo_espera_input else 10.0

    # Inicializamos sin nodo_destino (se obtendra dinamicamente)
    navegador = SistemaNavegacionHeadlessTorch(
        tiempo_espera_obstaculo=tiempo_espera_obstaculo,
        broker_address="localhost" 
    )

    dir_actual = os.path.dirname(os.path.abspath(__file__))
    ruta_pesos = os.path.join(dir_actual, "yolov5s.pt")
    ruta_repo_local = os.path.join(dir_actual, "yolov5") 

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    try:
        if os.path.isdir(ruta_repo_local) and os.path.exists(os.path.join(ruta_repo_local, "hubconf.py")):
            model = torch.hub.load(ruta_repo_local, 'custom', path=ruta_pesos, source='local', force_reload=False)
        else:
            model = torch.hub.load('ultralytics/yolov5:v6.2', 'custom', path=ruta_pesos, force_reload=False)

        model.to(device)
        model.conf = 0.40
        model.eval()
        if device.type == 'cuda':
            model.half()
        print("[OK] Modelo YOLOv5 inicializado exitosamente.")
    except Exception as e:
        print(f"\n[ERROR CRITICO] Fallo al instanciar el modelo: {e}")
        exit()

    print("[INFO] Inicializando camara USB...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 30)

    if not cap.isOpened():
        print("[ERROR CRITICO] No se pudo acceder a la camara.")
        exit()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] Error de lectura de camara.")
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            with torch.no_grad():
                results = model(frame_rgb)

            pred_tensor = results.xyxy[0]
            predictions = pred_tensor.cpu().numpy() if pred_tensor is not None and len(pred_tensor) else np.empty((0, 6))

            hay_bloqueo_dinamico = 0
            menor_distancia_obstaculo = 999.0
            nodo_bloqueado_actual = None
            clase_detectada_actual = None

            for pred in predictions:
                x1, y1, x2, y2, conf, clase_id = pred[0], pred[1], pred[2], pred[3], pred[4], int(pred[5])

                if clase_id in navegador.CLASES_OBSTACULO:
                    base_y = y2
                    alpha = math.atan2((base_y - navegador.PRINCIPAL_POINT_Y), navegador.FOCAL_LENGTH_PX)
                    denom = math.tan(navegador.CAM_PITCH_RAD + alpha)

                    if denom <= 0:
                        continue

                    distancia_horizontal_cm = navegador.CAM_HEIGHT_CM / denom

                    if 7.0 <= distancia_horizontal_cm <= 120.0:
                        centro_x = (x1 + x2) / 2.0
                        beta_desviacion_rad = math.atan2((centro_x - navegador.PRINCIPAL_POINT_X), navegador.FOCAL_LENGTH_PX)

                        angulo_robot_rad = math.radians(navegador.robot_theta)
                        angulo_total_obstaculo_rad = angulo_robot_rad - beta_desviacion_rad

                        obs_global_x = navegador.robot_x + distancia_horizontal_cm * math.cos(angulo_total_obstaculo_rad)
                        obs_global_y = navegador.robot_y + distancia_horizontal_cm * math.sin(angulo_total_obstaculo_rad)

                        nodo_afectado = navegador.nodo_mas_cercano(obs_global_x, obs_global_y)
                        es_pared_existente = False
                        
                        if nodo_afectado in navegador.graph_connections:
                            nx_val, ny_val = navegador.positions[nodo_afectado]
                            if math.sqrt((obs_global_x - nx_val)**2 + (obs_global_y - ny_val)**2) > (navegador.CELL_SIZE / 2.0) - 4.0:
                                es_pared_existente = True

                        if not es_pared_existente:
                            hay_bloqueo_dinamico = 1
                            if distancia_horizontal_cm < menor_distancia_obstaculo:
                                menor_distancia_obstaculo = distancia_horizontal_cm
                                nodo_bloqueado_actual = nodo_afectado
                                clase_detectada_actual = navegador.clases_nombres.get(clase_id, f"clase_{clase_id}")

            nodo_a_bloquear = navegador.gestionar_deteccion_obstaculo(
                objeto_detectado=bool(hay_bloqueo_dinamico),
                distancia_cm=menor_distancia_obstaculo if hay_bloqueo_dinamico else None,
                nodo_objeto=nodo_bloqueado_actual,
                clase_nombre=clase_detectada_actual,
            )

            if not hay_bloqueo_dinamico:
                print("[MONITOREO] Escaneando... Todo despejado.")

            navegador.ejecutar_ruteo(nodo_bloqueado=nodo_a_bloquear)
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("\nPrueba finalizada de manera conforme.")
    finally:
        cap.release()
        navegador.client.loop_stop()
        navegador.client.disconnect()
