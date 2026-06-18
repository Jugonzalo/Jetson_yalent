import struct
import serial
import time
import paho.mqtt.client as mqtt
from topicos import mqtt_topics, firebase_topics
import csv

# ---------------------- CONFIGURACIÓN ----------------------

puerto = input("escribe j si estas en la jetson, cualquier otra letra para windows: ")
if puerto.lower() == 'j':
     puerto = "/dev/ttyUSB0"    # Jetson
else:
    puerto = 'COM5'         # Windows
baudios = 115200
MAX_REINTENTOS_SYNC = 10   # Maximo de intentos para sincronizar con la ESP32 al inicio
HEADER_BYTE = 0xAA         # Byte de inicio del paquete, debe coincidir con el de la ESP32
TIMEOUT_ESP = 10  # segundos
estructura= '<Biiffffffffffff'
periodo_ejecucion = 0.05
# ---------------------- VARIABLES GLOBALES ----------------------
#Referencias
duty_der_ref = duty_izq_ref = 0
teta_ref = v_der_ref = v_izq_ref = v_total_ref = x_ref = y_ref = 0.0
#comandos de estado
esp_conectada = 0    #se ejecuta con sincro/ me falta cachar si es que puedo captar cuando muere
grabando = 0        
tiempo_grabando = 0
ejecutando = 1       #Ejecuta el programa entero
reinicio = 0         #Desconecta la esp espera un rato y vuelve a conectar


#PERIOD (CADA CUAN°TOS CILCOS QUIERO LEER)
periodo = 10
# INICIO LAS VARIABLES DE LECTURA EN 0
Header = duty_der_leido = duty_izq_leido = teta_leido = teta_ref_leido = v_der_leido = v_izq_leido = v_der_ref_leido = v_izq_ref_leido = v_total_leido = v_total_ref_leido = x_pos_leido = y_pos_leido = x_ref_leido = y_ref_leido = 0
leyo = False

#--------------------------------------CONFIG CSV----------------------------------------
archivo_csv = open('datos_esp32.csv', 'w', newline='')
writer = csv.writer(archivo_csv, delimiter=';')
#FILAS DEL EXCEL
writer.writerow(['timestamp', 'duty_der', 'duty_izq',  'velocidad_der','velocidad_der_ref', 'velocidad_izq', 'velocidad_izq_ref', 'teta','teta_ref', 'x_pos', 'x_ref', 'y_pos', 'y_ref'])

#--------------------Struct---------------------

 ####--------------FUNCIONES-------------------
def leer_serial():

    # reseteo y espero que lleguen los suficientes datos
    #while esp32.in_waiting > packet_size:
        # lee todos los paquetes hasta que solo quede el último completo
        #raw_data = esp32.read(packet_size)
        #print(esp32.in_waiting) 
        # NO se pa que chucha tenia esto xd
    packet_size = struct.calcsize(estructura)
    paquetes_disponibles = esp32.in_waiting // packet_size

    if paquetes_disponibles > 1:
        esp32.read((paquetes_disponibles - 1) * packet_size)
    raw_data = esp32.read(packet_size)
        # Desempaquetamos los bytes
        # El resultado es una tupla con el pack recibido Ej: (header, contador, temperatura, checksum)
    header, duty_izq, duty_der, teta, teta_ref, v_der, v_izq, v_der_ref, v_izq_ref, v_total, v_total_ref, x_pos, y_pos, x_ref, y_ref = struct.unpack(estructura, raw_data) # asegurate de ajustarlo

    if header == HEADER_BYTE:
        return True, header, duty_izq, duty_der, teta, teta_ref, v_der, v_izq, v_der_ref, v_izq_ref, v_total, v_total_ref, x_pos, y_pos, x_ref, y_ref
    else:
        esp32.reset_input_buffer() 
        return False, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0



def enviar_comando(duty_der_ref, duty_izq_ref, teta_ref, v_der_ref, v_izq_ref, v_total_ref, x_ref, y_ref):
    # < : Little-Endian
    # B : unsigned char (Header 0xAA)
    # i : int32 (ID del comando)x
    # f : float (Velocidad deseada)

    ###mmmmmm cual sera la forma mas inteligente de hacer esto.......###

    ## #############--------- ACA DEFINES EL PAQUETE A MANDAR ------------- #########
    paquete = struct.pack(
        '<Biiffffff',
        HEADER_BYTE,
        duty_der_ref, duty_izq_ref,
        teta_ref, v_der_ref, v_izq_ref, v_total_ref,
        x_ref, y_ref
    )
    esp32.write(paquete)



def setupsincro(intento = 1):
    #Chequeo si no supere intentos maximos
    if intento > MAX_REINTENTOS_SYNC:
        raise RuntimeError(f"No se pudo sincronizar con la ESP32 tras {MAX_REINTENTOS_SYNC} intentos")
    print(f"[SYNC] Intento {intento}/{MAX_REINTENTOS_SYNC}...")
    try:
            #conecto la esp
            esp32 = serial.Serial(puerto, baudios, timeout=2)
            #espero los beats basura
            esp32.reset_input_buffer()
            #la esp deberia mandar un texto diciendo que esta lista

            deadline = time.time() + 10  # Timeout de 10 segundos para sincronizar
            #Fuerza reset

            esp32.dtr = False
            esp32.rts = False
            time.sleep(0.1)
            esp32.dtr = True  # Este pulso resetea la ESP32
            time.sleep(2)     # Esperar boot
            esp32.reset_input_buffer()

            while time.time() < deadline:
                linea_raw = esp32.readline()
                if not linea_raw:
                    continue
                try:
                    linea = linea_raw.decode('utf-8').strip()
                    print(f"[SYNC] ESP32 dice: '{linea}'")
                    if linea == "Serial Jetson listo":
                        print("[SYNC] Sincronización exitosa")
                        return esp32
                except UnicodeDecodeError:
                    pass  # Puede haber basura binaria al inicio, ignorar

            # Timeout: cerrar e intentar de nuevo
            print("[SYNC] Timeout esperando respuesta. Reintentando...")
            esp32.close()

    except serial.SerialException as e:
        print(f"[SYNC] Error abriendo puerto: {e}")

        time.sleep(1)
        esp32.close()
        return setupsincro(intento + 1)


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

    if msg.topic == mqtt_topics["estados"]["reinicio"]:
        global reinicio
        reinicio = int(round(float(msg.payload.decode())))  # USARE 1 o 0 mas facil
    if msg.topic == mqtt_topics["estados"]["ejecutando"]:
        global ejecutando
        ejecutando = int(round(float(msg.payload.decode())))  # USARE 1 o 0 mas facil




## aca va el codigo final.

#--------------------------------------CONFIG MQTT---------------------------------------

client = mqtt.Client()
client.on_message = on_message  
client.connect("localhost", 1883)


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
client.subscribe(mqtt_topics["estados"]["ejecutando"])
client.subscribe(mqtt_topics["estados"]["reinicio"])




# ---------------BUCLE PRINCIPAL----------------
client.loop_start()  # Inicia el loop de MQTT en un hilo separado:      
while True:
    while ejecutando:
        if  not esp_conectada:
            esp32 = setupsincro()
            esp_conectada = 1
            client.publish(mqtt_topics["estados"]["conexion_esp"], 1)
            t_inicial = time.time()  
            i = 0   
            ultimo_paquete_valido = time.time()  # Para detectar timeout de la ESP32


        #INICIO EL BUCLE DE ESP CONECTADA
        if esp_conectada:
            t_ciclo = time.time()
            #ENVIO LOS COMANDOS
            try: 
                enviar_comando(duty_der_ref=duty_der_ref, duty_izq_ref=duty_izq_ref,
                                teta_ref=teta_ref,  v_total_ref=v_total_ref, 
                                v_der_ref=v_der_ref, v_izq_ref=v_izq_ref,
                                x_ref=x_ref, 
                                y_ref=y_ref )
            except serial.SerialException as e:
                print(f"Error al enviar comando: {e}")
                esp_conectada = 0
                esp32.close()
                client.publish(mqtt_topics["estados"]["conexion_esp"], 0)
            if i%periodo == 0:
                print("-----------------------------------------ENVIO DE COMANDO --------------------------------------------- ")
                print(f"duty_der: {duty_der_ref} | duty_izq: {duty_izq_ref} | teta_ref: {teta_ref} | v_der_ref: {v_der_ref} | v_izq_ref: {v_izq_ref} | v_total_ref: {v_total_ref} | x_ref: {x_ref} |y_ref: {y_ref} ")


            #LEO EL SERIAL1
            try:
                leyo, Header, duty_izq_leido, duty_der_leido, teta_leido, teta_ref_leido, v_der_leido, v_izq_leido, v_der_ref_leido, v_izq_ref_leido, v_total_leido, v_total_ref_leido, x_pos_leido, y_pos_leido, x_ref_leido, y_ref_leido = leer_serial()
            except serial.SerialException as e:
                print(f"Error al leer serial: {e}")
                leyo = False
            if leyo:
                ultimo_paquete_valido = time.time()
                if i%periodo == 0:
                    print("--------------------------------------------LECTURA ESP------------------------------------------------------")
                    sep = "+" + "-"*20 + "+" + "-"*15 + "+"
                    print(sep)
                    print(f"| {'Parameter':<20} | {'Value':<15} |")
                    print(sep)
                    print(f"| {'duty_der':<20} | {duty_der_leido:<15} |")
                    print(f"| {'duty_izq':<20} | {duty_izq_leido:<15} |")
                    print(f"| {'teta':<20} | {teta_leido:<15} |")
                    print(f"| {'teta_ref':<20} | {teta_ref_leido:<15} |")
                    print(sep)
                    print(f"| {'v_der':<20} | {v_der_leido:<15} |")
                    print(f"| {'v_izq':<20} | {v_izq_leido:<15} |")
                    print(f"| {'v_der_ref':<20} | {v_der_ref_leido:<15} |")
                    print(f"| {'v_izq_ref':<20} | {v_izq_ref_leido:<15} |")
                    print(sep)
                    print(f"| {'v_total':<20} | {v_total_leido:<15} |")
                    print(f"| {'v_total_ref':<20} | {v_total_ref_leido:<15} |")
                    print(sep)
                    print(f"| {'x_pos':<20} | {x_pos_leido:<15} |")
                    print(f"| {'y_pos':<20} | {y_pos_leido:<15} |")
                    print(f"| {'x_ref':<20} | {x_ref_leido:<15} |")
                    print(f"| {'y_ref':<20} | {y_ref_leido:<15} |")
                    print(sep)
                    
            else:
                if i%periodo == 0:
                    print("------------------------------------NO SE LEYO----------------------------------------------")
                if time.time() - ultimo_paquete_valido > TIMEOUT_ESP:
                    print(f"ESP32 no responde hace {TIMEOUT_ESP} segundos")
                    esp_conectada = 0
                    esp32.close()
                    client.publish(mqtt_topics["estados"]["conexion_esp"], 0)

            
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
                        v_der_ref_leido,
                        v_izq_leido,
                        v_izq_ref_leido,
                        teta_leido,
                        teta_ref,
                        x_pos_leido,
                        x_ref,
                        y_pos_leido,
                        y_ref,
                    ])
                    archivo_csv.flush() # claudio dice
                    if i%periodo == 0:
                        print(f"Grabando... Tiempo: {round(tiempo_grabando, 2)}s")
                except Exception as e:
                    print(f"ERROR CSV: {e}")

            if not grabando:
                t_inicial = time.time()
                #if i%periodo == 0:
                    #print("No grabado")
            i += 1
            #Sleep adaptativo
            elapsed = time.time() - t_ciclo
            restante = periodo_ejecucion - elapsed
            if restante > 0:
                time.sleep(restante)
            time.sleep(0.05) #  FRECUENCIA A LA QUE SE LEE Y ENVIA


        #Si se desconecta, el tema es que no se excatamente como captar que se desconecto
        if not esp_conectada:
            print("------------------------------------------ESP no conectada------------------------------------------")
            client.publish(mqtt_topics["estados"]["conexion_esp"], 0)
            print(input("Presiona Enter para intentar reconectar..."))
            esp32 = setupsincro()
            esp_conectada = 1
            client.publish(mqtt_topics["estados"]["conexion_esp"], 1)
            t_inicial = time.time()  
            i = 0 
            ultimo_paquete_valido = time.time()  


            

    while not ejecutando:
        print("Programa detenido")
        time.sleep(10)
        pass

