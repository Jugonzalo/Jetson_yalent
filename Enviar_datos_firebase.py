import firbase_funciones as firebase
import os
import paho.mqtt.client as mqtt


firebase.iniciar_firebase()

def on_message(client, userdata, msg):
    #print(f"Mensaje recibido en {msg.topic}: {msg.payload.decode()}")
    pass
    #if msg.topic == "robot/v_derecha":
     #   firebase.actualizar_estado("/Escritura/Potencia_derecha", int(round(float(msg.payload.decode()))))
    #elif msg.topic == "robot/v_izquierda":
    #    firebase.actualizar_estado("/Escritura/Potencia_izquierda", int(round(float(msg.payload.decode()))))

client = mqtt.Client()
client.on_message = on_message

client.connect("localhost", 1883)
client.subscribe([("robot/v_derecha", 0), ("robot/v_izquierda", 0)])
client.loop_start()








while True:
    v_d_val = firebase.leer_estado("/Escritura/Potencia_derecha")
    v_i_val = firebase.leer_estado("/Escritura/Potencia_izquierda")
    
    if v_d_val is not None and v_i_val is not None:
        v_d = int(round(float(v_d_val)))
        v_i = int(round(float(v_i_val)))
        client.publish("robot/v_derecha", v_d)
        client.publish("robot/v_izquierda", v_i)