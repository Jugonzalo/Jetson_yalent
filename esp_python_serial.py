


import struct
import serial
import time
import paho.mqtt.client as mqtt

def leer_serial():

    # reseteo y espero que lleguen los suficientes datos
    while esp32.in_waiting > packet_size:
        # lee todos los paquetes hasta que solo quede el último completo
        raw_data = esp32.read(packet_size)
        
    if esp32.in_waiting >= packet_size:
        raw_data = esp32.read(packet_size)
        # Desempaquetamos los bytes
        # El resultado es una tupla: (header, contador, temperatura, checksum)
        header, velocidad_der, velocidad_izq, check = struct.unpack('<BiiB', raw_data)
   
        
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
    header = 0xAA
    paquete = struct.pack('<Bii', header, velocidad_izquierda, velocidad_derecha)
    esp32.write(paquete)
    print(f"Enviado -> Velocidad izquierda: {velocidad_izquierda} | Velocidad derecha: {velocidad_derecha}")


def setupsincro():
    esp32 = serial.Serial(puerto, baudios, timeout=1)
    time.sleep(3)  # Pausa para que el ESP32 se reinicie al conectar
    lectura = esp32.readline().decode('utf-8').rstrip()
    if lectura != "Serial Jetson listo":
        print("no se conecto bien, reinicio")
        esp32.close()
        time.sleep(1)
        return setupsincro()

    print("Sincronización completa. Comenzando a enviar comandos...")
    return esp32







# Configura el puerto: Cambia 'COM12' por el de tu ESP32
puerto = 'COM12'
baudios = 115200

def on_message(client, userdata, msg):
    print(f"Mensaje recibido en {msg.topic}: {msg.payload.decode()}")

    if msg.topic == "robot/v_derecha":
        global velocidad_derecha
        velocidad_derecha = int(round(float(msg.payload.decode())))
    elif msg.topic == "robot/v_izquierda":
        global velocidad_izquierda
        velocidad_izquierda = int(round(float(msg.payload.decode())))




client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883)


client.subscribe("robot/v_derecha")
client.subscribe("robot/v_izquierda")

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
        







