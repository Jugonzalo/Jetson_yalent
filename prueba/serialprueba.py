import serial
import time
from datetime import datetime

PUERTO = "/dev/ttyTHS1" 
BAUDIOS = 115200
INTERVALO = 0.01  # 100 Hz de envio, ajustalo segun lo que quieras simular

esp32 = serial.Serial(PUERTO, BAUDIOS, timeout=1)
time.sleep(2)  # espero que la ESP termine de bootear

contador = 0
ultimo_print = time.time()

print("Enviando contador incremental. Ctrl+C para detener.\n")

with open("log_test_lectura.txt", "a") as log:
    try:
        while True:
            # Mando el contador como un solo byte (0-255, con vuelta)
            esp32.write(bytes([contador]))
            contador = (contador + 1) % 256

            # Leo y muestro cualquier respuesta/reporte que mande la ESP,
            # sin bloquear el envio
            if esp32.in_waiting > 0:
                linea = esp32.readline().decode('utf-8', errors='replace').strip()
                if linea:
                    marca = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    salida = f"[{marca}] {linea}"
                    print(salida)
                    log.write(salida + "\n")
                    log.flush()

            time.sleep(INTERVALO)
    except KeyboardInterrupt:
        print("\nDetenido por el usuario.")