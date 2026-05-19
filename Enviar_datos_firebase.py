import firbase_funciones as firebase
import os
import paho.mqtt.client as mqtt
import time
from topicos import mqtt_topics, firebase_topics






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
    
def recibido_a_bool(objeto_firebase):
    if objeto_firebase is not None:
        return bool(objeto_firebase)
    else:
        print("error_de_lectura")
        return 0




######### -----------------TOPICOS MQTT --------------#################


############ -------- ACCIONES A REALIZAR CUANDO RECIBE DATOS -----------------############



def on_message(client, userdata, msg):
    #print(f"Mensaje recibido en {msg.topic}: {msg.payload.decode()}")
    #if msg.topic == "robot/v_derecha":
     #   firebase.actualizar_estado("/Escritura/Potencia_derecha", int(round(float(msg.payload.decode()))))
    #elif msg.topic == "robot/v_izquierda":
    #    firebase.actualizar_estado("/Escritura/Potencia_izquierda", int(round(float(msg.payload.decode()))))
    if msg.topic == mqtt_topics["estados"]["estado_esp"]:
        texto = msg.payload.decode().strip().lower()
        valor = texto in ("true", "1", "yes", "on")
        estado_esp = valor
        firebase.actualizar_estado(firebase_topics["estados"]["estado_esp"], estado_esp)

    pass











#aca llamai al mqtt
client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883)
#client.subscribe([("robot/v_derecha", 0), ("robot/v_izquierda", 0)])



####------------ suscripciones -----------###

client.subscribe(mqtt_topics["estados"]["conexion_esp"])


# inicio el firebase
firebase.iniciar_firebase()
firebase.actualizar_estado(firebase_topics["estados"]["conexion_firebase"], True)




time.sleep(1)


######---------------- VARIABLES ----------------
estado_esp = False

#### --------------- LECTURA DE DATOS FIREBASE ------------ ############
i = 0 #usare pa ir viendo ejecuciones


while True:
    i+= 1 # variable pa ir contando ciclos
    inicio = time.perf_counter()  #con esto cacho cuanto demora






    # se leen y guardan los valores 
    #----------FIREBASE---------------------------------------------------------------------------
    duty_der = recibido_a_int(firebase.leer_estado(firebase_topics["comandos"]["duty_der"]))
    duty_izq = recibido_a_int(firebase.leer_estado(firebase_topics["comandos"]["duty_izq"]))
    


    if i%10 == 0: # -------DATOS QUE NO QUIERO FULL LATENCIA 
        firebase.actualizar_estado(firebase_topics["estados"]["conexion_firebase"], True) #estado de conexion del firebase
        estado_esp = recibido_a_bool(firebase.leer_estado(firebase_topics["estados"]["conexion_esp"]))
        print(estado_esp)
        client.publish(mqtt_topics["estados"]["conexion_esp"], estado_esp)






    # -------------MQTT----------------------------------------------------------------------------------
    client.publish(mqtt_topics["comandos"]["duty_der"], duty_der)
    client.publish(mqtt_topics["comandos"]["duty_izq"], duty_izq)



    






    #pa ver el tiempo
    duracion = time.perf_counter() - inicio

    time.sleep(0.01) #La esp supongamos que lee y manda a 10hz, por lo que la lectura deberia ir mas o menos al mismo rango