
mqtt_topics = {
    "telemetria": {
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
    },
    "estados": {
        "conexion_esp": "robot/estados/conectado_esp",
        "conexion_firebase": "robot/estados/conectado_firebase",
        "modo_control": "robot/estados/modo_control",
        "flag_pos": "robot/estados/flag_pos",
        "flag_obstaculo": "robot/estados/flag_pos",
        "ejecutando": "robot/estados/ejecutando",
        "grabar": "robot/estados/grabar",
        "reinicio": "robot/estados/reinicio",
    },
    "comandos": {
        "duty_der": "robot/comandos/duty_der",
        "duty_izq": "robot/comandos/duty_izq",
        "teta_ref": "robot/comandos/teta_ref",
        "v_der_ref": "robot/comandos/v_der_ref",
        "v_izq_ref": "robot/comandos/v_izq_ref",
        "v_total_ref": "robot/comandos/v_total_ref",
        "x_ref": "robot/comandos/x_ref",
        "y_ref": "robot/comandos/y_ref",
    },
}
# se usa: mqtt_topics["telemetria"]["v_der"], mqtt_topics["estados"]["conexion_esp"], mqtt_topics["comandos"]["duty_der"]


## ----------------TOPICOS FIREBASE

firebase_topics = {
    "telemetria": {
        "v_der": "Telemetria/v_derecha",
        "v_izq": "Telemetria/v_izquierda",
        "v_total": "Telemetria/v_total",
        "teta": "Telemetria/teta",
        "omega": "Telemetria/v_angular",
        "x": "Telemetria/x_pos",
        "y": "Telemetria/y_pos",
        "d_pared_der": "Telemetria/d_pared_derecha",
        "d_pared_izq": "Telemetria/d_pared_izquierda",
        "d_pared_trasera": "Telemetria/d_pared_trasera",
        "pilas": "Telemetria/pilas",
    },
    "estados": {
        "conexion_esp": "Estados/python_a_esp",
        "conexion_firebase": "Estados/firebase_a_python",
        "modo_control": "Estados/tipo_control",
        "flag_pos": "Estados/flag_pos",
        "flag_obstaculo": "Estados/flag_obstaculo",
        "ejecutando": "Estados/ejecutando",
        "grabar": "Estados/estado_grabacion",
        "reinicio": "Estados/reinicio_esp",
    },
    "comandos": {
        "duty_der": "Comandos/duty_der",
        "duty_izq": "Comandos/duty_izq",
        "teta_ref": "Comandos/teta_ref",
        "v_der_ref": "Comandos/v_der_ref",
        "v_izq_ref": "Comandos/v_izq_ref",
        "v_total_ref": "Comandos/v_total_ref",
        "x_ref": "Comandos/x_ref",
        "y_ref": "Comandos/y_ref",
    },
}