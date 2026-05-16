from slider_app import SliderManager
import time

import paho.mqtt.client as mqtt

import random


# Configuración
BROKER = "localhost" # Al estar en tu misma PC
PORT = 1883
TOPIC_D = "robot/v_derecha"
TOPIC_I = "robot/v_izquierda"

client = mqtt.Client()
client.connect(BROKER, PORT)

print(f"Enviando datos al tópico: {TOPIC_D}")
print(f"Enviando datos al tópico: {TOPIC_I}")

manager = SliderManager()


velocidad_derecha = manager.crear_deslizador(-100, 100, "Velocidad derecha")
velocidad_izquierda    = manager.crear_deslizador(-90, 90, "Velocidad izquierda")


tiempo_anterior = time.time()

v_d_actual = 0
v_i_actual = 0
try:
    while manager.activo():
        manager.actualizar()
        if time.time() - tiempo_anterior >= 0.05:
            if v_d_actual != velocidad_derecha.obtener_valor():
                v_d_actual = velocidad_derecha.obtener_valor()
                client.publish(TOPIC_D, str(v_d_actual))
                print(f"Velocidad derecha: {v_d_actual}")
            if v_i_actual != velocidad_izquierda.obtener_valor():
                v_i_actual = velocidad_izquierda.obtener_valor()
                client.publish(TOPIC_I, str(v_i_actual))
                print(f"Velocidad izquierda: {v_i_actual}")
            tiempo_anterior = time.time()
        time.sleep(0.05)
except KeyboardInterrupt:
    client.disconnect()