import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
import cv2
import math
import json
import numpy as np
import networkx as nx
import time
import threading
import torch
import paho.mqtt.client as mqtt

from topicos import mqtt_topics


class SistemaNavegacionHeadlessTorch:
    def __init__(self, nodo_destino, tiempo_espera_obstaculo=15, broker_address="localhost", broker_port=1883):
        self.CELL_SIZE = 30  
        self.nodo_destino = str(nodo_destino).strip()
        self.UMBRAL_TOLERANCIA = 2.0  

        # Pose cruda que reporta la ESP32, en SU propio marco: tras un reset_0 su
        # origen es donde este el robot y su eje X apunta hacia donde mira.
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.nodo_actual_fijo = None

        # Pose inicial declarada desde la web: origen del marco de la ESP dentro
        # del laberinto. Identidad por defecto (nodo "00" mirando al Este), asi
        # que sin configurar nada el comportamiento es el de siempre.
        self.ORIENTACIONES_VALIDAS = (0, 90, 180, 270)
        self.nodo_inicial = "00"
        self.theta_inicial = 0        # grados, 0 = +X (Este), creciente antihorario
        self.pose_x0 = 0.0
        self.pose_y0 = 0.0

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
        client.subscribe(mqtt_topics["comandos"]["grafo"])
        client.subscribe(mqtt_topics["comandos"]["pose_inicial"])

        # Publicar la estructura del grafo (retenido) para que la web dibuje el
        # laberinto desde una unica fuente de verdad. Se reenvia en cada reconexion.
        # Esto ademas pisa el retained de una sesion anterior: como las ediciones
        # del laberinto viven solo en memoria, al reiniciar el proceso la web debe
        # ver el grafo base y no el editado de la corrida pasada.
        self.publicar_grafo()
        # Igual que el grafo, la pose inicial vive solo en memoria: al arrancar
        # pisamos el retenido de la sesion anterior con la pose realmente vigente.
        self.publicar_pose_inicial(True, "Pose inicial por defecto")

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

    # ------------------------------------------------------------------
    # POSE INICIAL: conversion entre el marco de la ESP32 y el del laberinto
    # ------------------------------------------------------------------
    # La ESP arranca su odometria en (0,0,0) y no acepta una pose inicial: su
    # paquete serial tiene formato fijo. Por eso el laberinto y la ESP viven en
    # marcos distintos y la conversion se hace aca.
    #
    #   global = R(theta_inicial) * esp + (pose_x0, pose_y0)
    #
    def esp_a_global(self, x_esp, y_esp):
        rad = math.radians(self.theta_inicial)
        cos_t, sin_t = math.cos(rad), math.sin(rad)
        return (
            self.pose_x0 + x_esp * cos_t - y_esp * sin_t,
            self.pose_y0 + x_esp * sin_t + y_esp * cos_t,
        )

    def global_a_esp(self, x_glob, y_glob):
        """Inversa de esp_a_global. Necesaria porque x_ref/y_ref viajan a la ESP,
        que solo entiende su propio marco."""
        rad = math.radians(self.theta_inicial)
        cos_t, sin_t = math.cos(rad), math.sin(rad)
        dx, dy = x_glob - self.pose_x0, y_glob - self.pose_y0
        return (dx * cos_t + dy * sin_t, -dx * sin_t + dy * cos_t)

    @property
    def pose_global(self):
        return self.esp_a_global(self.robot_x, self.robot_y)

    @property
    def theta_global(self):
        """Rumbo del robot en el laberinto, en grados antihorarios desde el Este.
        La ESP reporta theta en sentido horario (de ahi el signo negativo, que ya
        estaba en el calculo de obstaculos)."""
        return self.theta_inicial - self.robot_theta

    def publicar_pose_inicial(self, ok=True, mensaje=""):
        payload = {
            "nodo": self.nodo_inicial,
            "theta": self.theta_inicial,
            "x0": self.pose_x0,
            "y0": self.pose_y0,
            "ok": ok,
            "mensaje": mensaje,
            "timestamp": time.time(),
        }
        self.client.publish(
            mqtt_topics["planificador"]["pose_inicial"], json.dumps(payload), retain=True
        )

    def aplicar_pose_inicial(self, payload_crudo):
        """Fija donde arranca el robot dentro del laberinto (nodo + orientacion).

        Pone a cero la odometria de la ESP y toma ese instante como origen del
        marco, de modo que ambos marcos coincidan exactamente en el momento cero.
        """
        try:
            payload = json.loads(payload_crudo)
        except json.JSONDecodeError as e:
            self.publicar_pose_inicial(False, f"JSON invalido: {e}")
            return

        if not isinstance(payload, dict):
            self.publicar_pose_inicial(False, "El payload debe ser un objeto")
            return

        nodo = str(payload.get("nodo", "")).strip()
        if nodo not in self.positions:
            self.publicar_pose_inicial(False, f"Nodo inicial invalido: '{nodo}'")
            return

        try:
            theta = int(payload["theta"]) % 360
        except (KeyError, TypeError, ValueError):
            self.publicar_pose_inicial(False, "Falta 'theta' o no es un numero")
            return

        if theta not in self.ORIENTACIONES_VALIDAS:
            self.publicar_pose_inicial(False, f"Orientacion invalida: {theta} (usa 0, 90, 180 o 270)")
            return

        self.nodo_inicial = nodo
        self.theta_inicial = theta
        self.pose_x0, self.pose_y0 = (float(v) for v in self.positions[nodo])

        # La ESP debe reiniciar su odometria: su (0,0,0) pasa a ser esta pose.
        # Anticipamos la lectura a cero para que ningun ciclo de A* use la pose
        # vieja con el offset nuevo (saltaria a un nodo equivocado por un instante).
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.nodo_actual_fijo = nodo

        self.client.publish(mqtt_topics["comandos"]["reset_0"], "1")
        # El flag viaja a la ESP en cada ciclo del enlace serial; hay que bajarlo
        # despues, y sin bloquear el hilo de red de paho.
        threading.Timer(0.5, lambda: self.client.publish(mqtt_topics["comandos"]["reset_0"], "0")).start()

        self._ultima_ruta_json = None   # forzar republicacion de la ruta
        print(f"[POSE] Pose inicial fijada: nodo {nodo}, theta {theta} grados.")
        self.publicar_pose_inicial(True, f"Pose inicial: nodo {nodo}, {theta} grados")

    def son_adyacentes(self, a, b):
        """Dos nodos son conectables solo si son celdas contiguas de la grilla.
        El robot no puede saltar de 00 a 44: se mueve entre celdas vecinas."""
        (xa, ya), (xb, yb) = self.positions[a], self.positions[b]
        return abs(xa - xb) + abs(ya - yb) == self.CELL_SIZE

    def validar_grafo(self, propuesto):
        """Valida y normaliza un grafo recibido desde la web.

        Devuelve (grafo_normalizado, None) o (None, motivo_del_rechazo). Toda
        arista se normaliza a bidireccional, igual que hace construir_grafo_base:
        una conexion declarada en un solo sentido no existiria para A*.
        """
        if not isinstance(propuesto, dict):
            return None, "El grafo debe ser un objeto"

        normalizado = {nodo: set() for nodo in self.positions}

        for nodo, vecinos in propuesto.items():
            if nodo not in self.positions:
                return None, f"Nodo desconocido: '{nodo}'"
            if not isinstance(vecinos, list):
                return None, f"Los vecinos de '{nodo}' deben ser una lista"

            for vecino in vecinos:
                if vecino not in self.positions:
                    return None, f"Vecino desconocido: '{vecino}'"
                if vecino == nodo:
                    return None, f"Arista invalida: '{nodo}' consigo mismo"
                if not self.son_adyacentes(nodo, vecino):
                    return None, f"Nodos no contiguos: '{nodo}'-'{vecino}'"
                normalizado[nodo].add(vecino)
                normalizado[vecino].add(nodo)

        return {n: sorted(v) for n, v in normalizado.items()}, None

    def aplicar_grafo_editado(self, payload_crudo):
        """Aplica un laberinto editado desde la web (robot/comandos/grafo).

        Solo vive en memoria: al reiniciar el proceso se vuelve al grafo base.
        No hace falta tocar self.G aqui, porque ejecutar_ruteo() llama a
        construir_grafo_base() en cada ciclo y reconstruye desde graph_connections.
        """
        try:
            payload = json.loads(payload_crudo)
        except json.JSONDecodeError as e:
            self.publicar_ack_edicion(False, f"JSON invalido: {e}")
            return

        propuesto = payload.get("graph_connections") if isinstance(payload, dict) else None
        if propuesto is None:
            self.publicar_ack_edicion(False, "Falta la clave 'graph_connections'")
            return

        normalizado, motivo = self.validar_grafo(propuesto)
        if motivo:
            print(f"[EDICION] Grafo rechazado: {motivo}")
            self.publicar_ack_edicion(False, motivo)
            # Reemitimos el grafo vigente para que la web revierta su borrador.
            self.publicar_grafo()
            return

        # Swap de la referencia completa en vez de mutar el dict en sitio: este
        # callback corre en el hilo de red de paho mientras el bucle principal
        # lee graph_connections, y asi nunca observa un grafo a medio escribir.
        self.graph_connections = normalizado
        aristas = sum(len(v) for v in normalizado.values()) // 2
        print(f"[EDICION] Laberinto actualizado: {aristas} aristas.")

        # Forzar la republicacion de la ruta aunque el A* devuelva lo mismo.
        self._ultima_ruta_json = None
        self.publicar_grafo()
        self.publicar_ack_edicion(True, f"Laberinto actualizado ({aristas} aristas)")

    def publicar_ack_edicion(self, ok, mensaje):
        self.client.publish(
            mqtt_topics["planificador"]["edicion"],
            json.dumps({"ok": ok, "mensaje": mensaje, "timestamp": time.time()}),
        )

    def _on_message(self, client, userdata, msg):
        try:
            # Laberinto editado desde la web (payload JSON)
            if msg.topic == mqtt_topics["comandos"]["grafo"]:
                self.aplicar_grafo_editado(msg.payload.decode())
                return

            # Pose inicial fijada desde la web (payload JSON)
            if msg.topic == mqtt_topics["comandos"]["pose_inicial"]:
                self.aplicar_pose_inicial(msg.payload.decode())
                return

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

    def publicar_referencia(self, x_glob, y_glob):
        """Traduce una meta del laberinto al marco de la ESP antes de mandarsela."""
        x_esp, y_esp = self.global_a_esp(x_glob, y_glob)
        self.client.publish(mqtt_topics["comandos"]["x_ref"], str(round(x_esp, 2)))
        self.client.publish(mqtt_topics["comandos"]["y_ref"], str(round(y_esp, 2)))

    def ejecutar_ruteo(self, nodo_bloqueado=None):
        # Toda la logica de A* razona en coordenadas del laberinto, no de la ESP.
        robot_x_glob, robot_y_glob = self.pose_global
        self.nodo_actual_fijo = self.calcular_nodo_actual_hibrido(robot_x_glob, robot_y_glob)
        self.construir_grafo_base()

        self.client.publish(mqtt_topics["camara"]["nodo_actual"], str(self.nodo_actual_fijo))

        x_meta, y_meta = self.positions[self.nodo_destino]
        if (abs(robot_x_glob - x_meta) <= self.UMBRAL_TOLERANCIA) and (abs(robot_y_glob - y_meta) <= self.UMBRAL_TOLERANCIA):
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

            self.publicar_referencia(x_ref, y_ref)
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
                        
                        # theta_global ya incorpora la orientacion inicial declarada y
                        # la correccion de signo (la ESP reporta theta en horario).
                        angulo_robot_rad = math.radians(navegador.theta_global)
                        angulo_total_obstaculo_rad = angulo_robot_rad - beta_desviacion_rad

                        robot_x_glob, robot_y_glob = navegador.pose_global
                        obs_global_x = robot_x_glob + distancia_horizontal_cm * math.cos(angulo_total_obstaculo_rad)
                        obs_global_y = robot_y_glob + distancia_horizontal_cm * math.sin(angulo_total_obstaculo_rad)

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