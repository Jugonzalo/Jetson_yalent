import firebase_admin
from firebase_admin import credentials, db
import os




def iniciar_firebase():
    archivo = os.path.join(os.path.dirname(__file__), 'project-yalent-firebase-adminsdk-fbsvc-f8d16650b8.json')
    print(archivo)
    cred = credentials.Certificate(archivo)
    firebase_admin.initialize_app(cred, {
        'databaseURL': 'https://project-yalent-default-rtdb.firebaseio.com/' # URL de tu Realtime Database
})


def _asegurar_firebase_inicializado():
    if not firebase_admin._apps:
        iniciar_firebase()



def actualizar_estado(ruta, valor):
    _asegurar_firebase_inicializado()
    ref = db.reference(ruta)
    ref.set(valor)  # Cambia el estado a 1 o 0
    print(f"Estado actualizado a: {valor}")


def leer_estado(ruta):
    _asegurar_firebase_inicializado()
    ref = db.reference(ruta)
    return ref.get()

