import struct
import serial
import time
import math
import paho.mqtt.client as mqtt
from topicos import mqtt_topics
import csv

# ---------------------- CONFIGURACIÓN ----------------------

puerto = input("escribe j si estas en la jetson, cualquier otra letra para windows: ")
if puerto.lower() == 'j':
     puerto = "/dev/ttyTHS1"    # Jetson
else:
    puerto = 'COM12'         # Windows


print("Escoje el modo de uso")
print("[1] modo duty")
print("[2] modo velocidad y theta")
print("[3] modo coordenadas  (delfault apreta cualquier cosa)")

modo_seleccionado = input("Selecciona modo: ")

if modo_seleccionado.lower() == '1':
    modo_seleccionado = "duty"
elif modo_seleccionado.lower() == '2':
    modo_seleccionado = "velocidad"
else:
    modo_seleccionado = "coordenadas"

baudios = 115200
MAX_REINTENTOS_SYNC = 10   # Maximo de intentos para sincronizar con la ESP32 al inicio
HEADER_BYTE = 0xAA         # Byte de inicio del paquete, debe coincidir con el de la ESP32
TIMEOUT_ESP = 3   # segundos
MAX_FALLOS_SYNC = 30  # 30 ciclos × 50ms = 1.5s sin dato válido → resync suave
estructura= '<Biiffffffffffff'
FRECUENCIA_LECTURA_MS = 50          # igual que FRECUENCIA_LECTURA en tareas.h
periodo_ejecucion = FRECUENCIA_LECTURA_MS / 1000.0

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
reset_pos = 0


#PERIOD (CADA CUAN°TOS CILCOS QUIERO LEER)
periodo = 1
# INICIO LAS VARIABLES DE LECTURA EN 0
Header = duty_der_leido = duty_izq_leido = teta_leido = teta_ref_leido = v_der_leido = v_izq_leido = v_der_ref_leido = v_izq_ref_leido = v_total_leido = v_total_ref_leido = x_pos_leido = y_pos_leido = x_ref_leido = y_ref_leido = 0.0
leyo = False
fallos_sync_consecutivos = 0
esp32 = None  # Guardián: evita NameError si MQTT activa esp_conectada antes del setup

#--------------------------------------CONFIG CSV----------------------------------------
archivo_csv = open('datos_esp32.csv', 'w', newline='')
writer = csv.writer(archivo_csv, delimiter=';')
#FILAS DEL EXCEL
writer.writerow(['timestamp', 'duty_der', 'duty_izq',  'velocidad_der','velocidad_der_ref', 'velocidad_izq', 'velocidad_izq_ref', 'teta','teta_ref', 'x_pos', 'x_ref', 'y_pos', 'y_ref'])

#--------------------Struct---------------------

 ####--------------FUNCIONES-------------------
def leer_serial():
    packet_size = struct.calcsize(estructura)

    # Busca el HEADER_BYTE en el stream para realinearse ante cualquier desincronía.
    # Si no encuentra el header en 2*packet_size bytes, descarta y retorna False.
    MAX_BUSQUEDA = packet_size * 2
    buscados = 0
    try:
        while True:
            if esp32.in_waiting == 0:
                return False, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
            b = esp32.read(1)
            if not b:
                return False, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
            if b[0] == HEADER_BYTE:
                break
            buscados += 1
            if buscados > MAX_BUSQUEDA:
                print(f"[SYNC] Header no encontrado tras {MAX_BUSQUEDA} bytes, descartando buffer")
                esp32.reset_input_buffer()
                return False, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

        # Ya encontramos el HEADER_BYTE, leemos el resto del paquete
        resto = packet_size - 1
        raw_resto = esp32.read(resto)
    except (serial.SerialException, OSError) as e:
        # USB físicamente desconectado — propagar para que el loop principal haga reconnect
        raise serial.SerialException(f"Desconexión durante lectura: {e}")

    if len(raw_resto) < resto:
        # Lectura incompleta (timeout del puerto serial)
        print(f"[SERIAL] Paquete incompleto: esperaba {resto} bytes, recibí {len(raw_resto)}")
        esp32.reset_input_buffer()
        return False, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

    raw_data = b + raw_resto
    try:
        header, duty_izq, duty_der, teta, teta_ref, v_der, v_izq, v_der_ref, v_izq_ref, v_total, v_total_ref, x_pos, y_pos, x_ref, y_ref = struct.unpack(estructura, raw_data)
    except struct.error as e:
        print(f"[SERIAL] Error de desempaque: {e}")
        esp32.reset_input_buffer()
        return False, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

    return True, header, duty_izq, duty_der, teta, teta_ref, v_der, v_izq, v_der_ref, v_izq_ref, v_total, v_total_ref, x_pos, y_pos, x_ref, y_ref



def enviar_comando(duty_der_ref, duty_izq_ref, teta_ref, v_der_ref, v_izq_ref, v_total_ref, x_ref, y_ref, reset_pos):
    # < : Little-Endian
    # B : unsigned char (Header 0xAA)
    # i : int32 (ID del comando)x
    # f : float (Velocidad deseada)

    ###mmmmmm cual sera la forma mas inteligente de hacer esto.......###

    ## #############--------- ACA DEFINES EL PAQUETE A MANDAR ------------- #########
    paquete = struct.pack(
        '<Biifffffff',
        HEADER_BYTE,
        duty_izq_ref, duty_der_ref,
        teta_ref, v_der_ref, v_izq_ref, v_total_ref,
        x_ref, y_ref, reset_pos
    )
    esp32.write(paquete)



def setupsincro(intento = 1, modo_de_uso = "coordenadas"):
    #Chequeo si no supere intentos maximos
    if intento > MAX_REINTENTOS_SYNC:
        raise RuntimeError(f"No se pudo sincronizar con la ESP32 tras {MAX_REINTENTOS_SYNC} intentos")
    print(f"[SYNC] Intento {intento}/{MAX_REINTENTOS_SYNC}...")
    try:
            #conecto la esp
            esp32 = serial.Serial(puerto, baudios, timeout=2)
            #espero los beats basura
            esp32.reset_input_buffer()

            #Fuerza reset al ESP32 via pulso DTR
            #esp32.dtr = False
            #esp32.rts = False
            #time.sleep(0.1)
            #esp32.dtr = True  # Este pulso resetea la ESP32
            #time.sleep(2)     # Esperar boot
            #esp32.reset_input_buffer()

            # Deadline empieza DESPUÉS del reset, para tener los 10s completos
            deadline = time.time() + 10

            while time.time() < deadline:
                linea_raw = esp32.readline()
                if not linea_raw:
                    continue
                try:
                    linea = linea_raw.decode('utf-8').strip()
                    print(f"[SYNC] ESP32 dice: '{linea}'")
                    if linea == "Serial Jetson listo":
                        print("[SYNC] Sincronización exitosa")

                        #================== Asignacion de Modo ==================== 
                        # d= duty /////   v = vel //// c = coord
                        if modo_de_uso == "duty":
                            esp32.write(b'd')
                        elif modo_de_uso == "velocidad":
                            esp32.write(b'v')
                        elif modo_de_uso == "coordenadas":
                            esp32.write(b'c')

                        # Leer confirmación del modo que imprime la ESP

                        #saco un jetson listo
                        esp32.readline().decode()
                        
                        #ahora leo la confirmacion de modo
                        confirmacion_raw = esp32.readline()
                        try:
                            confirmacion = confirmacion_raw.decode('utf-8').strip()
                            print(f"[SYNC] ESP32 confirmó modo: '{confirmacion}'")
                        except UnicodeDecodeError:
                            print("[SYNC] No se pudo decodificar confirmación de modo")

                        return esp32
                    
                    
                except UnicodeDecodeError:
                    pass  # Puede haber basura binaria al inicio, ignorar

            # Timeout: cerrar e intentar de nuevo
            print("[SYNC] Timeout esperando respuesta. Reintentando...")
            esp32.close()
            return setupsincro(intento + 1, modo_de_uso=modo_de_uso)

    except serial.SerialException as e:
        print(f"[SYNC] Error abriendo puerto: {e}")
        time.sleep(1)
        return setupsincro(intento + 1, modo_de_uso= modo_de_uso)


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
    if msg.topic == mqtt_topics["estados"]["flag_pos"]:
        global reset_pos
        reset_pos = int(round(float(msg.payload.decode())))




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
client.subscribe(mqtt_topics["estados"]["flag_pos"])




# ---------------BUCLE PRINCIPAL----------------
client.loop_start()  # Inicia el loop de MQTT en un hilo separado:      
while True:
    while ejecutando:
        if  not esp_conectada:
            try:
                esp32 = setupsincro(modo_de_uso=modo_seleccionado)
                esp_conectada = 1
                fallos_sync_consecutivos = 0
                client.publish(mqtt_topics["estados"]["conexion_esp"], 1)
                t_inicial = time.time()
                i = 0
                ultimo_paquete_valido = time.time()  # Para detectar timeout de la ESP32
            except RuntimeError as e:
                print(f"[RECONNECT] {e}. Reintentando en 5 segundos...")
                time.sleep(5)
                continue

        #INICIO EL BUCLE DE ESP CONECTADA
        if esp_conectada:
            t_ciclo = time.time()
            #ENVIO LOS COMANDOS

            try:
                enviar_comando(duty_der_ref=duty_der_ref, duty_izq_ref=duty_izq_ref,
                                teta_ref=teta_ref,  v_total_ref=v_total_ref,
                                v_der_ref=v_der_ref, v_izq_ref=v_izq_ref,
                                x_ref=x_ref,
                                y_ref=y_ref,
                                reset_pos=reset_pos )
            except (serial.SerialException, OSError) as e:
                print(f"[SERIAL] Puerto desconectado al enviar: {e}")
                esp_conectada = 0
                try: esp32.close()
                except: pass
                client.publish(mqtt_topics["estados"]["conexion_esp"], 0)
                continue  # CRÍTICO: sin esto el código sigue y accede a esp32 cerrado → crash
            if i%periodo == 0:
                print("-----------------------------------------ENVIO DE COMANDO --------------------------------------------- ")
                print(f"duty_der: {duty_der_ref} | duty_izq: {duty_izq_ref} | teta_ref: {teta_ref} | v_der_ref: {v_der_ref} | v_izq_ref: {v_izq_ref} | v_total_ref: {v_total_ref} | x_ref: {x_ref} |y_ref: {y_ref} ")


            #LEO EL SERIAL1
            # Solo leer cuando hay datos suficientes para un paquete completo
            try:
                bytes_disponibles = esp32.in_waiting
            except (serial.SerialException, OSError) as e:
                print(f"[SERIAL] Puerto desconectado en in_waiting: {e}")
                esp_conectada = 0
                try: esp32.close()
                except: pass
                client.publish(mqtt_topics["estados"]["conexion_esp"], 0)
                continue
            if bytes_disponibles >= struct.calcsize(estructura):
                try:
                    leyo, Header, duty_izq_leido, duty_der_leido, teta_leido, teta_ref_leido, v_der_leido, v_izq_leido, v_der_ref_leido, v_izq_ref_leido, v_total_leido, v_total_ref_leido, x_pos_leido, y_pos_leido, x_ref_leido, y_ref_leido = leer_serial()
                except (serial.SerialException, OSError) as e:
                    print(f"[SERIAL] Puerto desconectado al leer: {e}")
                    esp_conectada = 0
                    try: esp32.close()
                    except: pass
                    client.publish(mqtt_topics["estados"]["conexion_esp"], 0)
                    leyo = False
                    continue
            else:
                leyo = False
            # Descartar paquetes basura (falsa sincronización por 0xAA en medio de un paquete)
            if leyo and (abs(duty_der_leido) > 500 or abs(duty_izq_leido) > 500 or
                         not math.isfinite(teta_leido) or not math.isfinite(v_der_leido) or
                         not math.isfinite(x_pos_leido) or not math.isfinite(y_pos_leido)):
                print("[SYNC] Paquete inválido descartado (falsa sincronización)")
                leyo = False

            if leyo:
                fallos_sync_consecutivos = 0
                ultimo_paquete_valido = time.time()


                # =================PUBLICO MQTT ================
                client.publish(mqtt_topics["telemetria"]["v_der"], v_der_leido)
                client.publish(mqtt_topics["telemetria"]["v_der"],   v_der_leido)
                client.publish(mqtt_topics["telemetria"]["v_izq"],   v_izq_leido)
                client.publish(mqtt_topics["telemetria"]["v_total"], v_total_leido)
                client.publish(mqtt_topics["telemetria"]["teta"],    teta_leido)
                client.publish(mqtt_topics["telemetria"]["x"],       x_pos_leido)
                client.publish(mqtt_topics["telemetria"]["y"],       y_pos_leido)

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
                    print(f"| {'v_total':<20} | {((v_der_leido + v_izq_leido)/2):<15} |")
                    print(f"| {'v_total_ref':<20} | {v_total_ref_leido:<15} |")
                    print(sep)
                    print(f"| {'x_pos':<20} | {x_pos_leido:<15} |")
                    print(f"| {'y_pos':<20} | {y_pos_leido:<15} |")
                    print(f"| {'x_ref':<20} | {x_ref:<15} |")
                    print(f"| {'y_ref':<20} | {y_ref:<15} |")
                    print(sep)

            else:
                fallos_sync_consecutivos += 1
                if i%periodo == 0:
                    print("------------------------------------NO SE LEYO----------------------------------------------")
                if fallos_sync_consecutivos >= MAX_FALLOS_SYNC:
                    # El ESP32 NO crasheó — solo se perdió sincronización del buffer.
                    # Cerrar y reabrir el puerto sin resetear el ESP32 (sin pulso DTR).
                    print(f"[SYNC] Buffer desincronizado, reabriendo puerto serial...")
                    try: esp32.close()
                    except: pass
                    time.sleep(0.3)
                    try:
                        esp32 = serial.Serial(puerto, baudios, timeout=2)
                        esp32.reset_input_buffer()
                        time.sleep(0.06)  # Esperar ~1 ciclo de paquete
                        fallos_sync_consecutivos = 0
                        ultimo_paquete_valido = time.time()
                        print("[SYNC] Puerto reabierto, buscando sincronización...")
                    except (serial.SerialException, OSError) as e:
                        print(f"[SYNC] Error al reabrir puerto ({e}), forzando reconexión completa...")
                        esp_conectada = 0
                        client.publish(mqtt_topics["estados"]["conexion_esp"], 0)
                        fallos_sync_consecutivos = 0
                elif time.time() - ultimo_paquete_valido > TIMEOUT_ESP:
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
                        float(v_der_leido),
                        float(v_der_ref_leido),
                        float(v_izq_leido),
                        float(v_izq_ref_leido),
                        teta_leido,
                        teta_ref_leido,
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


        # Si la ESP se desconecta, el loop vuelve arriba y el bloque de reconexión lo maneja


            

    while not ejecutando:
        print("Programa detenido")
        time.sleep(10)
        pass

