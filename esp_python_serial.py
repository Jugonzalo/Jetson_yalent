import struct
import serial
import time
import paho.mqtt.client as mqtt

# Configura el puerto: Cambia 'COM12' por el de tu ESP32
puerto = 'COM12'
baudios = 115200

#define el tamaño del paquete segun toque
# en vola toca moverlo o que dependa del tipo de control
packet_size = 10
 
velocidad_derecha = 0  # defino esto pal inicio
velocidad_izquierda = 0

########### diccionario #############

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


def leer_serial():

    # reseteo y espero que lleguen los suficientes datos
    while esp32.in_waiting > packet_size:
        # lee todos los paquetes hasta que solo quede el último completo
        raw_data = esp32.read(packet_size)
        
    if esp32.in_waiting >= packet_size:
        raw_data = esp32.read(packet_size)
        # Desempaquetamos los bytes
        # El resultado es una tupla con el pack recibido Ej: (header, contador, temperatura, checksum)


        header, velocidad_der, velocidad_izq, check = struct.unpack('<BiiB', raw_data)# asegurate de ajustarlo
        print(header)
        
        if header == 0xAA:
            print(f"Velocidad derecha: {velocidad_der} | Velocidad izquierda: {velocidad_izq} | Check: {check}")
            pass
        else:
            # Si el header no coincide, el buffer está desfasado
            esp32.reset_input_buffer()



def enviar_comando(velocidad_izquierda, velocidad_derecha):
    # < : Little-Endian
    # B : unsigned char (Header 0xAA)
    # i : int32 (ID del comando)x
    # f : float (Velocidad deseada)


    ###mmmmmm cual sera la forma mas inteligente de hacer esto.......###


    header = 0xAA

    ## #############--------- ACA DEFINES EL PAQUETE A MANDAR ------------- #########
    paquete = struct.pack('<Bii', header, velocidad_izquierda, velocidad_derecha) 


    ########## 
    esp32.write(paquete)
    print(f"Enviado -> Velocidad izquierda: {velocidad_izquierda} | Velocidad derecha: {velocidad_derecha}")


def setupsincro():
    #conecto la esp
    esp32 = serial.Serial(puerto, baudios, timeout=1)
    esp32.flushInput()
    time.sleep(3)  # Pausa para que el ESP32 se reinicie al conectar

    #la esp deberia mandar un texto diciendo que esta lista
    for i in range(30):
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




    if lectura_final != "Serial Jetson listo": #Si el texto no se leyo correctamente
        # puede haber un desfase por lo que se reinicia
        print("no se conecto bien, reinicio")

        esp32.close()
        time.sleep(1)

        #con gracia divina la funcion se ejecuta infinitamente hasta que funcione
        return setupsincro()


    # cuando funciona se ejecuta esto
    print("Sincronización completa. Comenzando a enviar comandos...")



    return esp32



def on_message(client, userdata, msg):
    #print(f"Mensaje recibido en {msg.topic}: {msg.payload.decode()}")

    if msg.topic == comandos["duty_der"]:
        global velocidad_derecha
        velocidad_derecha = int(round(float(msg.payload.decode())))
    elif msg.topic == comandos["duty_izq"]:

        global velocidad_izquierda
        velocidad_izquierda = int(round(float(msg.payload.decode())))


## aca va el codigo final.

client = mqtt.Client()
client.on_message = on_message  
client.connect("localhost", 1883)



client.subscribe(comandos["duty_der"])
client.subscribe(comandos["duty_izq"])

packet_size = 10

velocidad_derecha = 0 
velocidad_izquierda = 0


try:
    esp32 = setupsincro()
    client.loop_start()  # Inicia el loop de MQTT en un hilo separado
    velocidad_izquierda = 0
    velocidad_derecha = 0
    if input("presiona enter wacho") != "x":           
        while True:

            enviar_comando(velocidad_izquierda, velocidad_derecha)
            time.sleep(0.1)
            leer_serial()


        































except serial.SerialException as e:
    print(f"Error al conectar con el ESP32: {e}")
        







