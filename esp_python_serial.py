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


        header, velocidad_der, velocidad_izq, teta, teta_ref, v_der, v_izq, v_der_ref, v_izq_ref, v_total, v_total_ref, x_pos, y_pos, x_ref, y_ref = struct.unpack('<Biiffiiiiiiiiii', raw_data) # asegurate de ajustarlo
        if header == 0xAA:
            print(f"Velocidad derecha: {velocidad_der} | Velocidad izquierda: {velocidad_izq} | teta: {teta}")
            return (header, velocidad_der, velocidad_izq, teta, teta_ref, v_der, v_izq, v_der_ref, v_izq_ref, v_total, v_total_ref, x_pos, y_pos, x_ref, y_ref)
        else:
            # Si el header no coincide, el buffer está desfasado
            esp32.reset_input_buffer()
            return False



def enviar_comando(velocidad_izquierda, velocidad_derecha):
    # < : Little-Endian
    # B : unsigned char (Header 0xAA)
    # i : int32 (ID del comando)x
    # f : float (Velocidad deseada)


    ###mmmmmm cual sera la forma mas inteligente de hacer esto.......###


    header = 0xAA

    ## #############--------- ACA DEFINES EL PAQUETE A MANDAR ------------- #########
    paquete = struct.pack('<Biifiiiii', header, velocidad_izquierda, velocidad_derecha, 0,0,0,0,0,0) 


    ########## 
    esp32.write(paquete)
    print(f"Enviado -> Velocidad izquierda: {velocidad_izquierda} | Velocidad derecha: {velocidad_derecha}")


def setupsincro():
    #conecto la esp
    esp32 = serial.Serial(puerto, baudios, timeout=1)
    esp32.flushInput()
    time.sleep(3)  # Pausa para que el ESP32 se reinicie al conectar

    #la esp deberia mandar un texto diciendo que esta lista
    for i in range(100):
        lectura = esp32.readline()
        print(lectura)

        try: 
            print(lectura)
            print(lectura.decode('utf-8').rstrip())        
            if lectura.decode('utf-8').rstrip() == "Serial Jetson listo":
                lectura_final = lectura.decode('utf-8').rstrip()
                i = 31
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

    if msg.topic == mqtt_topics["comandos"]["duty_der"]:
        global velocidad_derecha
        velocidad_derecha = int(round(float(msg.payload.decode())))
    if msg.topic == mqtt_topics["comandos"]["duty_izq"]:
        global velocidad_izquierda
        velocidad_izquierda = int(round(float(msg.payload.decode())))
    if msg.topic == mqtt_topics["estados"]["conexion_esp"]:
        global esp_conectada
        texto = msg.payload.decode().strip().lower()
        valor = texto in ("true", "1", "yes", "on")
        print(f"lei del mqtt {valor}")
        esp_conectada = valor








## aca va el codigo final.

#--------------------------------------CONFIG MQTT---------------------------------------

client = mqtt.Client()
client.on_message = on_message  
client.connect("localhost", 1883)


#--------------------------------------Serial----------------------------------------
# Configura el puerto: Cambia 'COM12' por el de tu ESP32
puerto = 'COM12'
baudios = 115200



#--------------------------------------CONFIG CSV----------------------------------------
archivo_csv = open('datos_esp32.csv', 'w', newline='')
writer = csv.writer(archivo_csv, delimiter=';')
writer.writerow(['timestamp', 'duty_der', 'duty_izq', 'velocidad_der', 'velocidad_izq', 'teta',
                 'teta_ref', 'x_pos', 'y_pos'])


### -------------------------SUSCRIPCIONES ----------------------------

client.subscribe(mqtt_topics["comandos"]["duty_der"])
client.subscribe(mqtt_topics["comandos"]["duty_izq"])

### -------------------------Variables ----------------------------

packet_size = 57  #LARGO DEL PAQUETE A LEER
esp_conectada = False
grabando =True
tiempo_grabando = 0

lectura = []


velocidad_derecha = 0 
velocidad_izquierda = 0


try:
    esp32 = setupsincro()
    esp_conectada = True
    client.publish(mqtt_topics["estados"]["conexion_esp"], True)
    client.loop_start()  # Inicia el loop de MQTT en un hilo separado:      
    t_inicial = time.time()     
    while esp_conectada:
        enviar_comando(velocidad_izquierda, velocidad_derecha)
        lectura = leer_serial()
        if grabando:
            tiempo_grabando = time.time() - t_inicial
            try:
                writer.writerow([round(tiempo_grabando, 3), lectura[1], lectura[2], 0,   0 , lectura[3], 0, 1, 0])
                #writer.writerow(['timestamp', "duty_der, duty_izq", 'velocidad_der', 'velocidad_izq', 'teta','teta_ref', 'x_pos', 'y_pos'])
                archivo_csv.flush() # claudio dice
            except:
                pass
        elif not grabando:
            t_inicial = time.time()







        time.sleep(0.1)
    while esp_conectada == False:
        pass

except KeyboardInterrupt:
    archivo_csv.close()
    print("Archivo guardado.")

except serial.SerialException as e:
    archivo_csv.close()
    print(f"Error al conectar con el ESP32: {e}")


        































except serial.SerialException as e:
    print(f"Error al conectar con el ESP32: {e}")
        







