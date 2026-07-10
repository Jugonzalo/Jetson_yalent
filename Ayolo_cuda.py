import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
import cv2
import math
import json
import numpy as np
import networkx as nx
import time
import torch
import paho.mqtt.client as mqtt

from topicos import mqtt_topics


class SistemaNavegacionHeadlessTorch:
    def __init__(self, nodo_destino, tiempo_espera_obstaculo=15, broker_address="localhost", broker_port=1883):
        self.CELL_SIZE = 30  
        self.nodo_destino = str(nodo_destino).strip()
        self.UMBRAL_TOLERANCIA = 2.0  

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.nodo_actual_fijo = None

        self.CAM_HEIGHT_CM = 15.0
        self.CAM_PITCH_RAD = math.radians(20.0)
        self.FOCAL_LENGTH_PX = 910.0
        self.PRINCIPAL_POINT_Y = 360.0
        self.PRINCIPAL_POINT_X = 640.0  

        self.CLASES_OBSTACULO = {0, 24, 26, 28, 32, 39, 41, 67, 73}
        self.clases_nombres = {0: "persona", 24: "mochila", 26: "paraguas", 28: "bolso", 32: "pelota", 67: "mesa", 73: "libro"}

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
        # NUEVA MAQUINA DE ESTADOS COMPENSADA PARA JETSON (LATCH)
        # --------------------------------------------------
        self.tiempo_espera_obstaculo = float(tiempo_espera_obstaculo)
        self.estado_obstaculo = "LIBRE"  # "LIBRE", "RETENIDO_ESPERANDO", "CONFIRMADO_BLOQUEADO"
        self.nodo_detectado = None
        self.timestamp_deteccion = None

        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(broker_address, broker_port, 60)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] Conectado al broker.")
        client.subscribe(mqtt_topics["telemetria"]["x"])
        client.subscribe(mqtt_topics["telemetria"]["y"])
        client.subscribe(mqtt_topics["telemetria"]["teta"])
        client.subscribe(mqtt_topics["comandos"]["nodo_des"])

        # Publicar la estructura del grafo (retenido) para que la web dibuje el
        # laberinto desde una unica fuente de verdad. Se reenvia en cada reconexion.
        self.publicar_grafo()

    def publicar_grafo(self):
        payload_grafo = {
            "cell_size": self.CELL_SIZE,
            "nodes": list(self.positions.keys()),
            "graph_connections": self.graph_connections,
            "positions": {k: list(v) for k, v in self.positions.items()},
        }
        self.client.publish(
            mqtt_topics["planificador"]["grafo"], json.dumps(payload_grafo), retain=True
        )
        print("[MQTT] Grafo del laberinto publicado (retenido).")

    def _on_message(self, client, userdata, msg):
        try:
            # Cambio de nodo destino en caliente (payload de texto, no numerico)
            if msg.topic == mqtt_topics["comandos"]["nodo_des"]:
                nuevo = msg.payload.decode().strip()
                if nuevo in self.positions:
                    self.nodo_destino = nuevo
                    print(f"[MQTT] Nuevo nodo destino recibido: {nuevo}")
                else:
                    print(f"[MQTT ERROR] Nodo destino invalido: '{nuevo}'")
                return

            payload = float(msg.payload.decode().strip())
            if msg.topic == mqtt_topics["telemetria"]["x"]:
                self.robot_x = payload
            elif msg.topic == mqtt_topics["telemetria"]["y"]:
                self.robot_y = payload
            elif msg.topic == mqtt_topics["telemetria"]["teta"]:
                self.robot_theta = payload
        except Exception as e:
            print(f"[MQTT ERROR] {e}")

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

    def publicar_ruta(self, nodo_siguiente, ruta_completa, nodo_bloqueado, x_ref, y_ref):
        """Publica la ruta A* completa como JSON, solo cuando cambia (evita saturar el bucle ~10 Hz)."""
        ruta_payload = {
            "nodo_actual": self.nodo_actual_fijo,
            "nodo_siguiente": nodo_siguiente,
            "nodo_destino": self.nodo_destino,
            "ruta_completa": ruta_completa,
            "nodo_bloqueado": nodo_bloqueado,
            "x_ref": int(x_ref),
            "y_ref": int(y_ref),
            "timestamp": time.time(),
        }
        # El timestamp cambia siempre; comparamos ignorandolo para deduplicar de verdad.
        clave = json.dumps({k: v for k, v in ruta_payload.items() if k != "timestamp"})
        if clave != getattr(self, "_ultima_ruta_json", None):
            self.client.publish(
                mqtt_topics["planificador"]["ruta"], json.dumps(ruta_payload), retain=True
            )
            self._ultima_ruta_json = clave

    def ejecutar_ruteo(self, nodo_bloqueado=None):
        self.nodo_actual_fijo = self.calcular_nodo_actual_hibrido(self.robot_x, self.robot_y)
        self.construir_grafo_base()

        self.client.publish(mqtt_topics["camara"]["nodo_actual"], str(self.nodo_actual_fijo))

        x_meta, y_meta = self.positions[self.nodo_destino]
        if (abs(self.robot_x - x_meta) <= self.UMBRAL_TOLERANCIA) and (abs(self.robot_y - y_meta) <= self.UMBRAL_TOLERANCIA):
            print("[A*] Meta del laberinto alcanzada con exito!")
            self.client.publish(mqtt_topics["estados"]["flag_pos"], "1")
            self.client.publish(mqtt_topics["camara"]["siguiente_nodo"], str(self.nodo_destino))
            self.publicar_ruta(self.nodo_destino, [self.nodo_actual_fijo], nodo_bloqueado, x_meta, y_meta)
            return
        else:
            self.client.publish(mqtt_topics["estados"]["flag_pos"], "0")

        if nodo_bloqueado and nodo_bloqueado in self.G:
            if nodo_bloqueado != self.nodo_actual_fijo and nodo_bloqueado != self.nodo_destino:
                self.G.remove_node(nodo_bloqueado)
                print(f"[A*] Modificando mapa: Evitando Nodo bloqueado dinamicamente {nodo_bloqueado}")

        try:
            ruta = nx.astar_path(self.G, self.nodo_actual_fijo, self.nodo_destino, heuristic=self.heuristic)
            nodo_siguiente = ruta[1] if len(ruta) > 1 else self.nodo_actual_fijo
            x_ref, y_ref = self.positions[nodo_siguiente]

            self.client.publish(mqtt_topics["comandos"]["x_ref"], str(x_ref))
            self.client.publish(mqtt_topics["comandos"]["y_ref"], str(y_ref))
            self.client.publish(mqtt_topics["camara"]["siguiente_nodo"], str(nodo_siguiente))
            self.publicar_ruta(nodo_siguiente, ruta, nodo_bloqueado, x_ref, y_ref)
        except nx.NetworkXNoPath:
            print(f"[AVISO CRITICO] No existe ruta viable a {self.nodo_destino}.")
            x_act, y_act = self.positions[self.nodo_actual_fijo]
            self.publicar_ruta(self.nodo_actual_fijo, [], nodo_bloqueado, x_act, y_act)


if __name__ == "__main__":
    meta_usuario = input("Ingrese el nodo de destino final (ej. 32): ").strip()
    navegador = SistemaNavegacionHeadlessTorch(nodo_destino=meta_usuario, tiempo_espera_obstaculo=15.0)

    dir_actual = os.path.dirname(os.path.abspath(__file__))
    ruta_pesos = os.path.join(dir_actual, "yolov5s.pt")
    ruta_repo_local = os.path.join(dir_actual, "yolov5") 

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"[INFO] Corriendo en: {device}")

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
    except Exception as e:
        print(f"[ERROR MODELO] {e}")
        exit()

    # CORRECCION DE ERROR ANTERIOR: cv2.CAP_V4L2 explicito y seguro para la Jetson Nano
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 30)
    

    try:
        while True:
            ahora = time.time()
            nodo_bloqueado_enviar_a_star = None

            # =========================================================================
            # LOGICA COOPERATIVA DE RETENCION DE MEMORIA (EVITA CONGELAMIENTO EN JETSON)
            # =========================================================================
            if navegador.estado_obstaculo in ("RETENIDO_ESPERANDO", "CONFIRMADO_BLOQUEADO"):
                tiempo_transcurrido = ahora - navegador.timestamp_deteccion
                
                if tiempo_transcurrido < navegador.tiempo_espera_obstaculo:
                    # AHORRO DE RECURSOS: No procesamos la camara en este ciclo. El objeto queda retenido virtualmente
                    nodo_bloqueado_enviar_a_star = navegador.nodo_detectado
                    
                    if navegador.estado_obstaculo == "RETENIDO_ESPERANDO":
                        # El profesor sugirio confirmar tras un tiempo si sigue ahi
                        if tiempo_transcurrido >= (navegador.tiempo_espera_obstaculo / 2.0):
                            navegador.estado_obstaculo = "CONFIRMADO_BLOQUEADO"
                            navegador.client.publish(mqtt_topics["estados"]["flag_obstaculo"], "1")
                            print(f"[LATCH] Obstaculo confirmado en Nodo {navegador.nodo_detectado}. Forzando desvio A*.")
                    
                    navegador.ejecutar_ruteo(nodo_bloqueado=nodo_bloqueado_enviar_a_star)
                    time.sleep(0.5) # Dormimos el script un poco para vaciar la CPU de la Jetson
                    continue # Saltamos la inferencia de YOLO
                else:
                    # El tiempo expiro. Limpiamos estados y obligamos a YOLO a re-escanear en el proximo frame
                    print("[LATCH] El temporizador expiro. Volviendo a escanear con la camara para verificar si se movio.")
                    navegador.estado_obstaculo = "LIBRE"
                    navegador.client.publish(mqtt_topics["estados"]["flag_obstaculo"], "0")

            # =========================================================================
            # INFERENCIA ACTIVA DE YOLO (SOLO CORRE SI EL ESTADO ES "LIBRE")
            # =========================================================================
            ret, frame = cap.read()
            if not ret:
                continue

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            with torch.no_grad():
                results = model(frame_rgb)

            predictions = results.xyxy[0].cpu().numpy() if results.xyxy[0] is not None else np.empty((0, 6))
            hay_objeto_en_este_frame = False

            for pred in predictions:
                x1, y1, x2, y2, conf, clase_id = pred[0], pred[1], pred[2], pred[3], pred[4], int(pred[5])

                if clase_id in navegador.CLASES_OBSTACULO:
                    alpha = math.atan2((y2 - navegador.PRINCIPAL_POINT_Y), navegador.FOCAL_LENGTH_PX)
                    denom = math.tan(navegador.CAM_PITCH_RAD + alpha)
                    if denom <= 0: continue

                    distancia_horizontal_cm = navegador.CAM_HEIGHT_CM / denom

                    if 7.0 <= distancia_horizontal_cm <= 120.0:
                        # Trigonometria unificada con la orientacion del Robot
                        centro_x = (x1 + x2) / 2.0
                        beta_desviacion_rad = math.atan2((centro_x - navegador.PRINCIPAL_POINT_X), navegador.FOCAL_LENGTH_PX)
                        
                        # Correccion de signo para sentido horario (tipico de brujulas/odometrias)
                        angulo_robot_rad = math.radians(-navegador.robot_theta)
                        angulo_total_obstaculo_rad = angulo_robot_rad - beta_desviacion_rad

                        obs_global_x = navegador.robot_x + distancia_horizontal_cm * math.cos(angulo_total_obstaculo_rad)
                        obs_global_y = navegador.robot_y + distancia_horizontal_cm * math.sin(angulo_total_obstaculo_rad)

                        nodo_afectado = navegador.nodo_mas_cercano(obs_global_x, obs_global_y)
                        
                        # Validar si no es una pared fija del laberinto
                        es_pared = False
                        if nodo_afectado in navegador.graph_connections:
                            nx_val, ny_val = navegador.positions[nodo_afectado]
                            if math.sqrt((obs_global_x - nx_val)**2 + (obs_global_y - ny_val)**2) > (navegador.CELL_SIZE / 2.0) - 4.0:
                                es_pared = True

                        if not es_pared:
                            # Encontramos un objeto dinamico. Activamos la retencion de memoria.
                            navegador.estado_obstaculo = "RETENIDO_ESPERANDO"
                            navegador.nodo_detectado = nodo_afectado
                            navegador.timestamp_deteccion = ahora
                            nodo_bloqueado_enviar_a_star = nodo_afectado
                            hay_objeto_en_este_frame = True
                            
                            # Publicar reportes inmediatos por MQTT
                            navegador.client.publish(mqtt_topics["camara"].get("nodo_obs", "robot/camara/nodo_obs"), str(nodo_afectado))
                            navegador.client.publish(mqtt_topics["camara"].get("dist_obs", "robot/camara/dist_obs"), f"{distancia_horizontal_cm:.1f}")
                            print(f"[ALERTA NANO] Visto objeto en nodo {nodo_afectado} a {distancia_horizontal_cm:.1f}cm. Entrando en modo retencion.")
                            break # No necesitamos procesar mas cajas en este frame

            if not hay_objeto_en_este_frame:
                # Si el estado era libre y no se vio nada, limpiamos reportes MQTT
                navegador.client.publish(mqtt_topics["camara"].get("nodo_obs", "robot/camara/nodo_obs"), "ninguno")
                navegador.client.publish(mqtt_topics["camara"].get("dist_obs", "robot/camara/dist_obs"), "-1")

            navegador.ejecutar_ruteo(nodo_bloqueado=nodo_bloqueado_enviar_a_star)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nFinalizado.")
    finally:
        cap.release()
        navegador.client.loop_stop()
        navegador.client.disconnect()