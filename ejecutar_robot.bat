
REM Cambia al directorio donde se encuentra el .bat
cd /d "%~dp0"


start cmd /k "py .\Enviar_datos_firebase.py"


start cmd /k "py .\esp_python_serial.py"


exit