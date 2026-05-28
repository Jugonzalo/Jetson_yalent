"""
Simulador - Robot Diferencial 2D
Controles: MQTT (entrada/salida), Pausa, Retroceso
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from collections import deque
import threading

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    print("[MQTT] paho-mqtt no instalado. Correr: pip install paho-mqtt")


# ════════════════════════════════════════════════════════════════
#  1. CONFIGURACION  <- Editar aqui
# ════════════════════════════════════════════════════════════════

CM = 0.01
RADIO_RUEDA        = 92.84 * CM
LARGO_ENTRE_RUEDAS = 190   * CM
VEL_ANGULAR_MAX    = 10.0

INIT_POS   = np.array([0.0, 0.0])
INIT_TETA  = 0.0
INIT_W_DER = 0.0
INIT_W_IZQ = 0.0

DT            = 0.05
INTERVAL_MS   = 30
MAX_HISTORIAL = 500

GRILLA_ANCHO = 20
GRILLA_ALTO  = 20

MQTT_BROKER    = "localhost"
MQTT_PORT      = 1883
MQTT_CLIENT_ID = "robot_simulador"

mqtt_topics = {
    "telemetria": {
        "v_der":               "robot/telemetria/v_der",
        "v_izq":               "robot/telemetria/v_izq",
        "v_total":             "robot/telemetria/v_total",
        "teta":                "robot/telemetria/teta",
        "omega":               "robot/telemetria/omega",
        "x":                   "robot/telemetria/x",
        "y":                   "robot/telemetria/y",
        "d_pared_der":         "robot/telemetria/d_pared_der",
        "d_pared_izq":         "robot/telemetria/d_pared_izq",
        "d_pared_trasera":     "robot/telemetria/d_pared_trasera",
        "distancia_recorrida": "robot/telemetria/distancia_recorrida",
        "pilas":               "robot/telemetria/pilas",
    },
    "estados": {
        "conexion_esp":      "robot/estados/conectado_esp",
        "conexion_firebase": "robot/estados/conectado_firebase",
        "modo_control":      "robot/estados/modo_control",
        "flag_pos":          "robot/estados/flag_pos",
        "flag_obstaculo":    "robot/estados/flag_pos",
        "ejecutando":        "robot/estados/ejecutando",
        "grabar":            "robot/estados/grabar",
        "reinicio":          "robot/estados/reinicio",
    },
    "comandos": {
        "duty_der":    "robot/comandos/duty_der",
        "duty_izq":    "robot/comandos/duty_izq",
        "teta_ref":    "robot/comandos/teta_ref",
        "v_der_ref":   "robot/comandos/v_der_ref",
        "v_izq_ref":   "robot/comandos/v_izq_ref",
        "v_total_ref": "robot/comandos/v_total_ref",
        "x_ref":       "robot/comandos/x_ref",
        "y_ref":       "robot/comandos/y_ref",
    },
}


# ════════════════════════════════════════════════════════════════
#  2. ESTADO DEL ROBOT
# ════════════════════════════════════════════════════════════════

class RobotState:
    def __init__(self):
        self.reset()

    def reset(self):
        self.pos            = INIT_POS.copy()
        self.teta           = INIT_TETA
        self.w_der          = INIT_W_DER
        self.w_izq          = INIT_W_IZQ
        self.vel            = 0.0
        self.vel_angular    = 0.0
        self.tiempo         = 0.0
        self.dist_recorrida = 0.0

    def step(self):
        v_der = self.w_der * RADIO_RUEDA
        v_izq = self.w_izq * RADIO_RUEDA

        self.vel         = (v_der + v_izq) / 2.0
        self.vel_angular = (v_der - v_izq) / LARGO_ENTRE_RUEDAS

        self.pos[0] += self.vel * np.cos(self.teta) * DT
        self.pos[1] += self.vel * np.sin(self.teta) * DT
        self.teta   += self.vel_angular * DT
        self.tiempo += DT
        self.dist_recorrida += abs(self.vel) * DT

        # TODO: colision con murallas

    def snapshot(self):
        return {
            "pos":            self.pos.copy(),
            "teta":           self.teta,
            "w_der":          self.w_der,
            "w_izq":          self.w_izq,
            "vel":            self.vel,
            "vel_angular":    self.vel_angular,
            "tiempo":         self.tiempo,
            "dist_recorrida": self.dist_recorrida,
        }

    def load_snapshot(self, snap):
        self.pos            = snap["pos"].copy()
        self.teta           = snap["teta"]
        self.w_der          = snap["w_der"]
        self.w_izq          = snap["w_izq"]
        self.vel            = snap["vel"]
        self.vel_angular    = snap["vel_angular"]
        self.tiempo         = snap["tiempo"]
        self.dist_recorrida = snap["dist_recorrida"]

    def telemetria(self):
        """Devuelve dict con todos los campos de telemetria listos para publicar."""
        return {
            "v_der":               round(float(self.w_der * RADIO_RUEDA), 4),
            "v_izq":               round(float(self.w_izq * RADIO_RUEDA), 4),
            "v_total":             round(float(self.vel), 4),
            "teta":                round(float(np.degrees(self.teta % (2 * np.pi))), 2),
            "omega":               round(float(self.vel_angular), 4),
            "x":                   round(float(self.pos[0]), 4),
            "y":                   round(float(self.pos[1]), 4),
            "d_pared_der":         0.0,    # TODO: calcular con murallas
            "d_pared_izq":         0.0,    # TODO: calcular con murallas
            "d_pared_trasera":     0.0,    # TODO: calcular con murallas
            "distancia_recorrida": round(float(self.dist_recorrida), 4),
            "pilas":               100.0,  # TODO: modelo de bateria
        }


# ════════════════════════════════════════════════════════════════
#  3. MQTT
# ════════════════════════════════════════════════════════════════

class MQTTHandler:
    """
    Hilo separado para no bloquear la animacion.

    Suscripciones (recibe float en payload):
        robot/comandos/v_der_ref  -> robot.w_der (convertido a rad/s)
        robot/comandos/v_izq_ref  -> robot.w_izq (convertido a rad/s)
        robot/estados/reinicio    -> resetea la simulacion si payload == "1"

    Publicaciones (un topic por campo):
        robot/telemetria/*        -> cada campo de telemetria por separado
    """
    def __init__(self, robot: RobotState):
        self.robot     = robot
        self.client    = None
        self.connected = False

        if not MQTT_AVAILABLE:
            return

        try:
            self.client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2, client_id=MQTT_CLIENT_ID
            )
        except AttributeError:
            self.client = mqtt.Client(client_id=MQTT_CLIENT_ID)

        self.client.on_connect    = self._on_connect
        self.client.on_message    = self._on_message
        self.client.on_disconnect = self._on_disconnect

        thread = threading.Thread(target=self._connect_loop, daemon=True)
        thread.start()

    def _connect_loop(self):
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
            self.client.loop_forever()
        except Exception as e:
            print(f"[MQTT] No se pudo conectar a {MQTT_BROKER}:{MQTT_PORT} -> {e}")

    def _on_connect(self, client, userdata, flags, rc, *args):
        if rc == 0:
            self.connected = True
            client.subscribe(mqtt_topics["comandos"]["v_der_ref"])
            client.subscribe(mqtt_topics["comandos"]["v_izq_ref"])
            client.subscribe(mqtt_topics["estados"]["reinicio"])
            print("[MQTT] Conectado. Suscripto a comandos.")
        else:
            print(f"[MQTT] Error de conexion, codigo {rc}")

    def _on_disconnect(self, client, userdata, rc, *args):
        self.connected = False
        print("[MQTT] Desconectado")

    def _on_message(self, client, userdata, msg):
        topic   = msg.topic
        payload = msg.payload.decode().strip()
        T       = mqtt_topics

        try:
            if topic == T["comandos"]["v_der_ref"]:
                v = float(payload)
                self.robot.w_der = float(np.clip(v / RADIO_RUEDA, -VEL_ANGULAR_MAX, VEL_ANGULAR_MAX))

            elif topic == T["comandos"]["v_izq_ref"]:
                v = float(payload)
                self.robot.w_izq = float(np.clip(v / RADIO_RUEDA, -VEL_ANGULAR_MAX, VEL_ANGULAR_MAX))

            elif topic == T["estados"]["reinicio"] and payload == "1":
                self.robot.reset()
                print("[MQTT] Reinicio recibido por MQTT")

        except (ValueError, KeyError) as e:
            print(f"[MQTT] Mensaje invalido en {topic}: {e}")

    def publish_telemetria(self):
        """Publica cada campo de telemetria en su propio topic."""
        if not (self.connected and self.client):
            return
        data = self.robot.telemetria()
        for key, value in data.items():
            topic = mqtt_topics["telemetria"].get(key)
            if topic:
                self.client.publish(topic, str(value))

    def publish(self, categoria, campo, valor):
        """Publicacion manual: mqtt_handler.publish("estados", "ejecutando", "1")"""
        if not (self.connected and self.client):
            return
        topic = mqtt_topics.get(categoria, {}).get(campo)
        if topic:
            self.client.publish(topic, str(valor))


# ════════════════════════════════════════════════════════════════
#  4. FIGURA Y ELEMENTOS GRAFICOS
# ════════════════════════════════════════════════════════════════

def build_figure():
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.patch.set_facecolor("#0d1117")
    ax.set_facecolor("#0d1117")
    fig.subplots_adjust(left=0.08, right=0.98, top=0.95, bottom=0.12)

    ax.set_xlim(-GRILLA_ANCHO / 2, GRILLA_ANCHO / 2)
    ax.set_ylim(-GRILLA_ALTO  / 2, GRILLA_ALTO  / 2)
    ax.set_aspect("equal")
    ax.set_xticks(np.arange(-GRILLA_ANCHO / 2, GRILLA_ANCHO / 2 + 1, 1))
    ax.set_yticks(np.arange(-GRILLA_ALTO  / 2, GRILLA_ALTO  / 2 + 1, 1))
    ax.tick_params(colors="#4a5568", labelsize=7)
    ax.grid(True, linestyle="--", alpha=0.25, color="#4a5568")
    for spine in ax.spines.values():
        spine.set_edgecolor("#4a5568")
    ax.set_xlabel("X [m]", color="#8892a4", fontsize=9)
    ax.set_ylabel("Y [m]", color="#8892a4", fontsize=9)
    ax.set_title("Simulador Robot Diferencial", color="#e2e8f0", fontsize=13, pad=10)
    ax.plot(0, 0, "w+", markersize=10, zorder=5)

    px_por_metro = fig.get_size_inches()[0] * fig.dpi / GRILLA_ANCHO
    ms_robot = RADIO_RUEDA * px_por_metro * 0.95

    cuerpo,  = ax.plot([], [], "o", color="#e53e3e", markersize=ms_robot,  zorder=10)
    heading, = ax.plot([], [], "o", color="#1a202c", markersize=ms_robot / 6, zorder=11)
    estela,  = ax.plot([], [], "-", color="#e53e3e", alpha=0.35, linewidth=1, zorder=8)

    hud_base = dict(transform=ax.transAxes, fontsize=9,
                    verticalalignment="top", color="#a0aec0", fontfamily="monospace")
    texts = {
        "tiempo": ax.text(0.02, 0.98, "", **hud_base),
        "pos_x":  ax.text(0.02, 0.94, "", **hud_base),
        "pos_y":  ax.text(0.02, 0.90, "", **hud_base),
        "teta":   ax.text(0.02, 0.86, "", **hud_base),
        "vel":    ax.text(0.02, 0.82, "", **hud_base),
        "omega":  ax.text(0.02, 0.78, "", **hud_base),
        "w_der":  ax.text(0.02, 0.74, "", **hud_base),
        "w_izq":  ax.text(0.02, 0.70, "", **hud_base),
        "dist":   ax.text(0.02, 0.66, "", **hud_base),
        "mqtt":   ax.text(0.02, 0.60, "", **{**hud_base, "color": "#68d391"}),
        "estado": ax.text(0.50, 0.98, "CORRIENDO", transform=ax.transAxes,
                          fontsize=10, color="#68d391", ha="center",
                          verticalalignment="top", fontfamily="monospace"),
    }

    return fig, ax, cuerpo, heading, estela, texts


def add_buttons(fig):
    btn_style = dict(color="#1a202c", hovercolor="#2d3748")
    ax_pause  = fig.add_axes([0.35, 0.03, 0.12, 0.045])
    ax_rewind = fig.add_axes([0.50, 0.03, 0.12, 0.045])
    ax_reset  = fig.add_axes([0.65, 0.03, 0.12, 0.045])

    btn_pause  = Button(ax_pause,  "Pausa",      **btn_style)
    btn_rewind = Button(ax_rewind, "Retroceder", **btn_style)
    btn_reset  = Button(ax_reset,  "Reset",      **btn_style)

    for btn in (btn_pause, btn_rewind, btn_reset):
        btn.label.set_color("#e2e8f0")
        btn.label.set_fontsize(9)

    return btn_pause, btn_rewind, btn_reset


# ════════════════════════════════════════════════════════════════
#  5. CONTROLADOR DE SIMULACION
# ════════════════════════════════════════════════════════════════

class SimulationController:
    def __init__(self):
        self.robot = RobotState()
        self.mqtt  = MQTTHandler(self.robot)

        self.pausado     = False
        self.historial_x = deque(maxlen=MAX_HISTORIAL)
        self.historial_y = deque(maxlen=MAX_HISTORIAL)
        self.snapshots   = deque(maxlen=MAX_HISTORIAL)

        self.fig, self.ax, self.cuerpo, self.heading, self.estela, self.texts = build_figure()
        self.btn_pause, self.btn_rewind, self.btn_reset = add_buttons(self.fig)

        self.btn_pause.on_clicked(self._toggle_pause)
        self.btn_rewind.on_clicked(self._rewind_frame)
        self.btn_reset.on_clicked(self._reset)

        self.ani = FuncAnimation(
            self.fig, self._update_frame,
            interval=INTERVAL_MS, blit=True, cache_frame_data=False
        )

    def _toggle_pause(self, _event):
        self.pausado = not self.pausado
        lbl   = "CORRIENDO" if not self.pausado else "PAUSADO"
        color = "#68d391"   if not self.pausado else "#f6ad55"
        self.texts["estado"].set_text(lbl)
        self.texts["estado"].set_color(color)
        self.btn_pause.label.set_text("Pausa" if not self.pausado else "Continuar")

    def _rewind_frame(self, _event):
        if len(self.snapshots) < 2:
            return
        self.snapshots.pop()
        snap = self.snapshots[-1]
        self.robot.load_snapshot(snap)
        if self.historial_x:
            self.historial_x.pop()
            self.historial_y.pop()
        if not self.pausado:
            self._toggle_pause(None)

    def _reset(self, _event):
        self.robot.reset()
        self.historial_x.clear()
        self.historial_y.clear()
        self.snapshots.clear()
        if self.pausado:
            self._toggle_pause(None)

    def _update_frame(self, _frame):
        if not self.pausado:
            self.snapshots.append(self.robot.snapshot())
            self.robot.step()
            self.historial_x.append(self.robot.pos[0])
            self.historial_y.append(self.robot.pos[1])
            self.mqtt.publish_telemetria()

        self._refresh_visuals()
        return (self.cuerpo, self.heading, self.estela, *self.texts.values())

    def _refresh_visuals(self):
        r  = self.robot
        px = r.pos[0]
        py = r.pos[1]

        self.cuerpo.set_data([px], [py])
        self.heading.set_data(
            [px + RADIO_RUEDA * np.cos(r.teta)],
            [py + RADIO_RUEDA * np.sin(r.teta)]
        )
        self.estela.set_data(list(self.historial_x), list(self.historial_y))

        mqtt_str   = "MQTT conectado"    if self.mqtt.connected else "MQTT desconectado"
        mqtt_color = "#68d391"           if self.mqtt.connected else "#fc8181"
        self.texts["mqtt"].set_text(mqtt_str)
        self.texts["mqtt"].set_color(mqtt_color)

        self.texts["tiempo"].set_text(f"t      : {r.tiempo:7.2f} s")
        self.texts["pos_x"] .set_text(f"x      : {r.pos[0]:7.3f} m")
        self.texts["pos_y"] .set_text(f"y      : {r.pos[1]:7.3f} m")
        self.texts["teta"]  .set_text(f"theta  : {np.degrees(r.teta % (2*np.pi)):7.2f} deg")
        self.texts["vel"]   .set_text(f"v_total: {r.vel:7.3f} m/s")
        self.texts["omega"] .set_text(f"omega  : {r.vel_angular:7.3f} rad/s")
        self.texts["w_der"] .set_text(f"w_der  : {r.w_der:7.3f} rad/s")
        self.texts["w_izq"] .set_text(f"w_izq  : {r.w_izq:7.3f} rad/s")
        self.texts["dist"]  .set_text(f"dist   : {r.dist_recorrida:7.3f} m")

    def run(self):
        plt.show()


# ════════════════════════════════════════════════════════════════
#  6. ENTRADA MANUAL POR CONSOLA (fallback sin MQTT)
# ════════════════════════════════════════════════════════════════

def console_input_thread(robot: RobotState):
    """
    Cambia velocidades desde la terminal mientras corre la simulacion.
    Formato: v_der,v_izq en m/s  (ej: 0.5,0.5)
    """
    print("[Consola] Formato: v_der,v_izq en m/s  (ej: 0.5,-0.5)")
    while True:
        try:
            entrada = input()
            if not entrada.strip():
                continue
            partes = entrada.split(",")
            v_der = float(partes[0])
            v_izq = float(partes[1])
            robot.w_der = float(np.clip(v_der / RADIO_RUEDA, -VEL_ANGULAR_MAX, VEL_ANGULAR_MAX))
            robot.w_izq = float(np.clip(v_izq / RADIO_RUEDA, -VEL_ANGULAR_MAX, VEL_ANGULAR_MAX))
            print(f"[Consola] v_der={v_der:.3f} m/s  v_izq={v_izq:.3f} m/s")
        except (ValueError, IndexError):
            print("[Consola] Formato invalido. Ejemplo: 0.5,-0.5")
        except EOFError:
            break


# ════════════════════════════════════════════════════════════════
#  7. MAIN
# ════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    sim = SimulationController()

    input_thread = threading.Thread(
        target=console_input_thread,
        args=(sim.robot,),
        daemon=True
    )
    input_thread.start()

    sim.run()