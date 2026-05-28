import firbase_funciones as firebase
import paho.mqtt.client as mqtt
import time
from topicos import mqtt_topics, firebase_topics

# ---- Buffer local (se actualiza con cada mensaje MQTT) ----
telemetria_buffer = {
    "v_der": 0.0, "v_izq": 0.0, "v_total": 0.0,
    "teta": 0.0, "omega": 0.0, "x": 0, "y": 0,
    "d_pared_der": 0.0, "d_pared_izq": 0.0,
    "d_pared_trasera": 0.0, "pilas": 0.0
}

TIPO_CAMPO = {
    "v_der": float, "v_izq": float, "v_total": float,
    "teta": float, "omega": float,
    "x": lambda v: int(round(float(v))),
    "y": lambda v: int(round(float(v))),
    "d_pared_der": float, "d_pared_izq": float,
    "d_pared_trasera": float, "pilas": float
}

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    for campo, topic in mqtt_topics["telemetria"].items():
        if msg.topic == topic and campo in TIPO_CAMPO:
            try:
                telemetria_buffer[campo] = TIPO_CAMPO[campo](payload)
            except ValueError:
                print(f"Error parseando {campo}: {payload}")
            break  # ya encontró el topic, sale del loop

# ---- Setup MQTT ----
client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883)

for topic in mqtt_topics["telemetria"].values():
    client.subscribe(topic)
client.subscribe(mqtt_topics["estados"]["conexion_esp"])
client.loop_start()

# ---- Setup Firebase ----
firebase.iniciar_firebase()
firebase.actualizar_estado(firebase_topics["estados"]["conexion_firebase"], True)
time.sleep(1)

# ---- Variables ----
estado_esp   = False
grabando     = 0
duty_der     = 0
duty_izq     = 0
v_der_ref    = 0.0
v_izq_ref    = 0.0

INTERVALO_TELE    = 0.05   # 10Hz Firebase telemetría
INTERVALO_CMDS    = 0.05  # 20Hz comandos MQTT
INTERVALO_ESTADO  = 1.0   # 1Hz estado conexión

t_tele   = t_cmds = t_estado = time.time()

# ---- Loop principal ----
while True:
    ahora = time.time()

    # --- Telemetría → Firebase (una sola escritura) ---
    if ahora - t_tele >= INTERVALO_TELE:
        firebase.actualizar_estado("Telemetria", {
            "v_derecha":          telemetria_buffer["v_der"],
            "v_izquierda":        telemetria_buffer["v_izq"],
            "v_total":            telemetria_buffer["v_total"],
            "teta":               telemetria_buffer["teta"],
            "v_angular":          telemetria_buffer["omega"],
            "x_pos":              telemetria_buffer["x"],
            "y_pos":              telemetria_buffer["y"],
            "d_pared_derecha":    telemetria_buffer["d_pared_der"],
            "d_pared_izquierda":  telemetria_buffer["d_pared_izq"],
            "d_pared_trasera":    telemetria_buffer["d_pared_trasera"],
            "pilas":              telemetria_buffer["pilas"],
        })
        t_tele = ahora

    # --- Comandos MQTT ---
    if ahora - t_cmds >= INTERVALO_CMDS:
        try:
            duty_der  = int(round(float(firebase.leer_estado(firebase_topics["comandos"]["duty_der"]) or 0)))
            duty_izq  = int(round(float(firebase.leer_estado(firebase_topics["comandos"]["duty_izq"]) or 0)))
            v_der_ref = float(firebase.leer_estado(firebase_topics["comandos"]["v_der_ref"]) or 0)
            v_izq_ref = float(firebase.leer_estado(firebase_topics["comandos"]["v_izq_ref"]) or 0)
        except Exception as e:
            print(f"Error leyendo comandos: {e}")

        client.publish(mqtt_topics["comandos"]["duty_der"],  duty_der)
        client.publish(mqtt_topics["comandos"]["duty_izq"],  duty_izq)
        client.publish(mqtt_topics["comandos"]["v_der_ref"], v_der_ref)
        client.publish(mqtt_topics["comandos"]["v_izq_ref"], v_izq_ref)
        client.publish(mqtt_topics["estados"]["grabar"],     grabando)
        t_cmds = ahora

    # --- Estado conexión (baja frecuencia) ---
    if ahora - t_estado >= INTERVALO_ESTADO:
        try:
            firebase.actualizar_estado(firebase_topics["estados"]["conexion_firebase"], True)
            estado_esp = bool(firebase.leer_estado(firebase_topics["estados"]["conexion_esp"]))
            grabando   = int(firebase.leer_estado(firebase_topics["estados"]["grabar"]) or 0)
            client.publish(mqtt_topics["estados"]["conexion_esp"], estado_esp)
        except Exception as e:
            print(f"Error estado: {e}")
        t_estado = ahora

    time.sleep(0.01)