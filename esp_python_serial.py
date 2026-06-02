import struct
import serial
import time
import paho.mqtt.client as mqtt
from topicos import mqtt_topics, firebase_topics
import csv


 ####--------------FUNCIONES-------------------
def leer_serial():

    # reseteo y espero que lleguen los suficientes datos
    #while esp32.in_waiting > packet_size:
        # lee todos los paquetes hasta que solo quede el último completo
        #raw_data = esp32.read(packet_size)
        #print(esp32.in_waiting) 
        # NO se pa que chucha tenia esto xd

    if esp32.in_waiting >= packet_size:
        raw_data = esp32.read(packet_size)
        # Desempaquetamos los bytes
        # El resultado es una tupla con el pack recibido Ej: (header, contador, temperatura, checksum)
        header, duty_izq, duty_der, teta, teta_ref, v_der, v_izq, v_der_ref, v_izq_ref, v_total, v_total_ref, x_pos, y_pos, x_ref, y_ref = struct.unpack('<Biiffffiiiiiiii', raw_data) # asegurate de ajustarlo
        if header == 0xAA:
            return True, header, duty_izq, duty_der, teta, teta_ref, v_der, v_izq, v_der_ref, v_izq_ref, v_total, v_total_ref, x_pos, y_pos, x_ref, y_ref
        else:
            # Si el header no coincide, el buffer está desfasado
            esp32.reset_input_buffer()
            return False, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    else:
        return False, 0, 0  , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0



def enviar_comando(duty_der_ref, duty_izq_ref, teta_ref, v_der_ref, v_izq_ref, v_total_ref, x_ref, y_ref):
    # < : Little-Endian
    # B : unsigned char (Header 0xAA)
    # i : int32 (ID del comando)x
    # f : float (Velocidad deseada)

    ###mmmmmm cual sera la forma mas inteligente de hacer esto.......###
    header = 0xAA

    ## #############--------- ACA DEFINES EL PAQUETE A MANDAR ------------- #########
    paquete = struct.pack('<Biifffiii', header,duty_der_ref, duty_izq_ref, teta_ref, v_der_ref, v_izq_ref, v_total_ref, x_ref, y_ref) 
                                                #duty der #duty izq , tetaref, vderref, vizqref, vtotal, xref, yref,

    esp32.write(paquete)


def setupsincro():
    #conecto la esp
    esp32 = serial.Serial(puerto, baudios, timeout=1)
    #esp32.dtr = False
    #esp32.rts = False
    #esp32.flushInput()
    time.sleep(0.5)
    
    # Forzar reset de la ESP32
    #esp32.dtr = True
    time.sleep(0.1)
    #esp32.dtr = False
    time.sleep(2)  # Esperar que bootee y mande el mensaje
    #esp32.flushInput()
    #la esp deberia mandar un texto diciendo que esta lista
    for i in range(100):
        lectura = esp32.readline()


        try: 
            print(lectura.decode('utf-8').rstrip())        
            if lectura.decode('utf-8').rstrip() == "Serial Jetson listo":
                lectura_final = lectura.decode('utf-8').rstrip()
                i = 31 #31 minutos ??
                break
        except:
            pass
        print(i)




    try:
        if lectura_final != "Serial Jetson listo": #Si el texto no se leyo correctamente
        # puede haber un desfase por lo que se reinicia
            print("no se conecto bien, reinicio")

            esp32.close()
            time.sleep(1)

            #con gracia divina la funcion se ejecuta infinitamente hasta que funcione
            return setupsincro()


        # cuando funciona se ejecuta esto
        print("Sincronización completa. Comenzando a enviar comandos...")
    except:
        esp32.close()
        setupsincro()
    return esp32



def on_message(client, userdata, msg):
    #print(f"Mensaje recibido en {msg.topic}: {msg.payload.decode()}")
    #DUTY DER
    if msg.topic == mqtt_topics["comandos"]["duty_der"]:
        global duty_der_ref
        duty_der_ref = int(round(float(msg.payload.decode())))
    if msg.topic == mqtt_topics["comandos"]["duty_izq"]:
        global duty_izq_ref
        duty_izq_ref = int(round(float(msg.payload.decode())))
    if msg.topic == mqtt_topics["estados"]["conexion_esp"]:
        global esp_conectada
        texto = msg.payload.decode().strip().lower()
        valor = texto in ("true", "1", "yes", "on")
        print(f"lei del mqtt {valor}")
        esp_conectada = valor
    if msg.topic == mqtt_topics["comandos"]["teta_ref"]:
        global teta_ref
        teta_ref = float(msg.payload.decode())
    if msg.topic == mqtt_topics["comandos"]["v_der_ref"]:
        global v_der_ref
        v_der_ref = float(msg.payload.decode())
    if msg.topic == mqtt_topics["comandos"]["v_izq_ref"]:
        global v_izq_ref
        v_izq_ref = float(msg.payload.decode())
    if msg.topic == mqtt_topics["comandos"]["v_total_ref"]:
        global v_total_ref
        v_total_ref = float(msg.payload.decode())
    if msg.topic == mqtt_topics["comandos"]["x_ref"]:
        global x_ref
        x_ref = int(round(float(msg.payload.decode())))
    if msg.topic == mqtt_topics["comandos"]["y_ref"]:
        global y_ref
        y_ref = int(round(float(msg.payload.decode())))
    if msg.topic == mqtt_topics["estados"]["grabar"]:
        global grabando
        grabando = int(round(float(msg.payload.decode())))  # USARE 1 o 0 mas facil




## aca va el codigo final.

#--------------------------------------CONFIG MQTT---------------------------------------

client = mqtt.Client()
client.on_message = on_message  
client.connect("localhost", 1883)



#--------------------------------------Serial----------------------------------------
# CONFIGURA EL PUERTO PARA WINDOWS
puerto = 'COM12' #ACTIVA ACA PA WINOWS
baudios = 115200


#CONFIGURA EL PUERTO PARA JETSON
#puerto = "/dev/ttyUSB0"  # ACTIVA ACA PA JETSON




#--------------------------------------CONFIG CSV----------------------------------------
archivo_csv = open('datos_esp32.csv', 'w', newline='')
writer = csv.writer(archivo_csv, delimiter=';')
#FILAS DEL EXCEL
writer.writerow(['timestamp', 'duty_der', 'duty_izq',  'velocidad_der','velocidad_der_ref', 'velocidad_izq', 'velocidad_izq_ref', 'teta','teta_ref', 'x_pos', 'x_ref', 'y_pos', 'y_ref'])



### -------------------------SUSCRIPCIONES ----------------------------

client.subscribe(mqtt_topics["comandos"]["duty_der"])
client.subscribe(mqtt_topics["comandos"]["duty_izq"])
client.subscribe(mqtt_topics["comandos"]["teta_ref"])
client.subscribe(mqtt_topics["comandos"]["v_der_ref"])
client.subscribe(mqtt_topics["comandos"]["v_izq_ref"])
client.subscribe(mqtt_topics["comandos"]["v_total_ref"])
client.subscribe(mqtt_topics["comandos"]["x_ref"])
client.subscribe(mqtt_topics["comandos"]["y_ref"])
client.subscribe(mqtt_topics["estados"]["grabar"])



### -------------------------Variables ----------------------------

packet_size = 57  #LARGO DEL PAQUETE A LEER
esp_conectada = 0
tiempo_grabando = 0
grabando = 1

#PERIOD (CADA CUANTOS CILCOS QUIERO LEER)
periodo = 2


# INICIO LAS VARIABLES DE COMANDOS EN 0
duty_der_ref = duty_izq_ref = teta_ref = v_der_ref = v_izq_ref = v_total_ref = x_ref = y_ref = 0
# INICIO LAS VARIABLES DE LECTURA EN 0
Header = duty_der_leido = duty_izq_leido = teta_leido = teta_ref_leido = v_der_leido = v_izq_leido = v_der_ref_leido = v_izq_ref_leido = v_total_leido = v_total_ref_leido = x_pos_leido = y_pos_leido = x_ref_leido = y_ref_leido = 0
leyo = False



if True:
    esp32 = setupsincro()
    esp_conectada = 1
    client.publish(mqtt_topics["estados"]["conexion_esp"], True)
    client.loop_start()  # Inicia el loop de MQTT en un hilo separado:      
    t_inicial = time.time()  
    i = 0   


    #INICIO EL BUCLE DE ESPCONECTADA
    while esp_conectada == 1:
        #ENVIO LOS COMANDOS
        enviar_comando(duty_der_ref=duty_der_ref, duty_izq_ref=duty_izq_ref, teta_ref=teta_ref, v_der_ref=v_der_ref, v_izq_ref=v_izq_ref, v_total_ref=v_total_ref, x_ref=x_ref, y_ref=y_ref   )
        #duty der #duty izq , tetaref, vderref, vizqref, vtotal, xref, yref,
        if i%periodo == 0:
            print("-----------------------------------------ENVIO DE COMANDO --------------------------------------------- ")
            print(f"duty_der: {duty_der_ref} | duty_izq: {duty_izq_ref} | teta_ref: {teta_ref} | v_der_ref: {v_der_ref} | v_izq_ref: {v_izq_ref} | v_total_ref: {v_total_ref} | x_ref: {x_ref} |y_ref: {y_ref} ")


        #LEO EL SERIAL1
        leyo, Header, duty_der_leido, duty_izq_leido, teta_leido, teta_ref_leido, v_der_leido, v_izq_leido, v_der_ref_leido, v_izq_ref_leido, v_total_leido, v_total_ref_leido, x_pos_leido, y_pos_leido, x_ref_leido, y_ref_leido = leer_serial()
        if leyo:
            if i%periodo == 0:
                print("--------------------------------------------LECTURA ESP------------------------------------------------------")
                print(f"duty izq: {duty_izq_leido} |duty der: {duty_der_leido} | teta: {teta_leido} | velocidad_izquierda: {v_izq_leido} | velocidad_derecha {v_der_leido}")
                print('\n \n')
        else:
            if i%periodo == 0:
              print("------------------------------------NO SE LEYO----------------------------------------------")
        
        #si el comando grabar esta prendido guarda en el excel
        if grabando:
            tiempo_grabando = time.time() - t_inicial
            try:
                #ESCRIBO EN EL EXCELL
                writer.writerow([
                    round(tiempo_grabando, 3),
                    duty_der_leido,
                    duty_izq_leido,
                    v_der_leido,
                    v_der_ref,
                    v_izq_leido,
                    v_izq_ref,
                    teta_leido,
                    teta_ref,
                    x_pos_leido,
                    x_ref,
                    y_pos_leido,
                    y_ref,
                ])
                #writer.writerow(['timestamp', "duty_der, duty_izq", 'velocidad_der', 'velocidad_izq', 'teta','teta_ref', 'x_pos', 'y_pos'])
                archivo_csv.flush() # claudio dice
            except:
                pass
        elif not grabando:
            t_inicial = time.time()
        i += 1
        time.sleep(0.5)

    while esp_conectada == 0:
        esp32.close()
        pass


