import firbase_funciones as firebase
import os
import paho.mqtt.client as mqtt
import time






def recibido_a_float(objeto_firebase):
    if objeto_firebase is not None:
        return float(objeto_firebase)
    else:
        print("error_de_lectura")
        return 0
    
def recibido_a_int(objeto_firebase):
    if objeto_firebase is not None:
        return int(round(float(objeto_firebase)))
    else:
        print("error_de_lectura")
        return 0




######### -----------------TOPICOS MQTT --------------#################


telemetria = {
    "v_der": "robot/telemetria/v_der",
    "v_izq": "robot/telemetria/v_izq",
    "v_total": "robot/telemetria/v_total",
    "teta": "robot/telemetria/teta",
    "omega": "robot/telemetria/omega",
    "x": "robot/telemetria/x",
    "y": "robot/telemetria/y",
    "d_pared_der": "robot/telemetria/d_pared_der",
    "d_pared_izq": "robot/telemetria/d_pared_izq",
    "d_pared_trasera": "robot/telemetria/d_pared_trasera",
    "distancia_recorrida": "robot/telemetria/distancia_recorrida",
    "pilas": "robot/telemetria/pilas",
}

estados = {
    "conexion_esp": "robot/estados/conectado_esp",
    "conexion_firebase": "robot/estados/conectado_firebase",
    "modo_control": "robot/estados/modo_control",
    "flag_pos": "robot/estados/flag_pos",
    "flag_obstaculo": "robot/estados/flag_pos",
    "ejecutando": "robot/estados/ejecutando",
    "grabar": "robot/estados/grabar",
    "reinicio": "robot/estados/reinicio",
}

comandos = {
    "duty_der": "robot/comandos/duty_der",
    "duty_izq": "robot/comandos/duty_izq",
    "teta_ref": "robot/comandos/teta_ref",
    "v_der_ref": "robot/comandos/v_der_ref",
    "v_izq_ref": "robot/comandos/v_izq_ref",
    "v_total_ref": "robot/comandos/v_total_ref",
    "x_ref": "robot/comandos/x_ref",
    "y_ref": "robot/comandos/y_ref",
}
# se usa: telemetria["v_der"], estados["conexion_esp"], comandos["duty_der"]

############ -------- ACCIONES A REALIZAR CUANDO RECIBE DATOS -----------------############



def on_message(client, userdata, msg):
    #print(f"Mensaje recibido en {msg.topic}: {msg.payload.decode()}")
    #if msg.topic == "robot/v_derecha":
     #   firebase.actualizar_estado("/Escritura/Potencia_derecha", int(round(float(msg.payload.decode()))))
    #elif msg.topic == "robot/v_izquierda":
    #    firebase.actualizar_estado("/Escritura/Potencia_izquierda", int(round(float(msg.payload.decode()))))
    pass











#aca llamai al mqtt
client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883)
#client.subscribe([("robot/v_derecha", 0), ("robot/v_izquierda", 0)])



####------------ suscripciones -----------###




# inicio el firebase
firebase.iniciar_firebase()



client.loop_start() #parto el mqtt

ruta1 =comandos["duty_der"]
ruta2 =comandos["duty_izq"]

ruta1_firebase =ruta1.strip("/robot")

ruta2_firebase = ruta2.strip("/robot")
print(ruta1_firebase)
print(ruta2_firebase)

time.sleep(1)

#### --------------- LECTURA DE DATOS FIREBASE ------------ ############

while True:

    # se leen los valores
    obj_der = firebase.leer_estado("/Comandos/duty_der")
    obj_izq = firebase.leer_estado("/Comandos/duty_izq")

    duty_der = recibido_a_int(obj_der)
    duty_izq = recibido_a_int(obj_izq)
    




    client.publish(ruta1, duty_der)
    client.publish(ruta2, duty_izq)