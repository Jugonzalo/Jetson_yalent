import struct
import serial
import time
import math
import paho.mqtt.client as mqtt

import csv
esp32 = serial.Serial("/dev/ttyTHS1" , 115200, timeout=2)

print("iniciando prueba")

time.sleep(1)
while True:

    try:
        aviso = esp32.readline().decode('utf-8').strip()
        print(aviso)

        dato = "a".encode()

        print("respuesta:  ")
        esp32.write(dato)
        print(esp32.readline().decode('utf-8').strip())
    except:
        pass

    time.sleep(0.5)