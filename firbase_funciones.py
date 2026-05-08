import firebase_admin
from firebase_admin import credentials, db
import os




def iniciar_firebase():
    archivo = os.path.join(os.path.dirname(__file__), 'project-yalent-firebase-adminsdk-fbsvc-cb87e96646.json')
    cred = credentials.Certificate(archivo)
    firebase_admin.initialize_app(cred, {
        'databaseURL': 'https://project-yalent-default-rtdb.firebaseio.com/' # URL de tu Realtime Database
})




def actualizar_estado(ruta, valor):
    ref = db.reference(ruta)
    ref.set(valor) # Cambia el estado a 1 o 0
    print(f"Estado actualizado a: {valor}")

def leer_estado(ruta):
    ref = db.reference(ruta)
    return ref.get()

