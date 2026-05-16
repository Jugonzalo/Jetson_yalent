import tkinter as tk


class Deslizador:
    """
    Un slider dentro de la ventana del SliderManager.
    No crea ventana propia — vive dentro del Frame que le asigna el manager.
    """

    def __init__(self, parent, min_val, max_val, titulo="Deslizador"):
        """
        Parámetros:
            parent   (tk.Frame): Frame contenedor provisto por SliderManager.
            min_val  (int|float): Valor mínimo.
            max_val  (int|float): Valor máximo.
            titulo   (str):       Etiqueta del slider.
        """
        marco = tk.LabelFrame(parent, text=titulo, padx=10, pady=5)
        marco.pack(fill="x", padx=10, pady=5)

        self._valor = tk.DoubleVar(value=min_val)

        self.slider = tk.Scale(
            marco,
            from_=min_val,
            to=max_val,
            orient='horizontal',
            variable=self._valor,
            length=350,
            tickinterval=(max_val - min_val) / 5,
            resolution=1
        )
        self.slider.pack()

    def obtener_valor(self):
        """Retorna el valor actual del slider."""
        return self._valor.get()


class SliderManager:
    """
    Ventana única que contiene todos los sliders apilados verticalmente.
    No bloquea el programa — usar actualizar() en el bucle principal.

    Uso típico:
        manager = SliderManager()
        s1 = manager.crear_deslizador(0, 100, "Velocidad")
        s2 = manager.crear_deslizador(-90, 90, "Ángulo")

        while manager.activo():
            manager.actualizar()
            v = s1.obtener_valor()
            a = s2.obtener_valor()
            # ... tu lógica aquí
    """

    def __init__(self, titulo_ventana="Panel de control"):
        self._root = tk.Tk()
        self._root.title(titulo_ventana)
        self._root.resizable(False, False)

    def crear_deslizador(self, min_val, max_val, titulo="Deslizador"):
        """Agrega un slider a la ventana y lo retorna."""
        return Deslizador(self._root, min_val, max_val, titulo)

    def actualizar(self):
        """Procesa eventos de tkinter SIN bloquear. Usar en el bucle principal."""
        try:
            self._root.update()
        except tk.TclError:
            pass

    def activo(self):
        """Retorna True mientras la ventana esté abierta."""
        try:
            return self._root.winfo_exists()
        except tk.TclError:
            return False

    def iniciar(self):
        """Loop bloqueante (alternativa si no tienes bucle propio)."""
        self._root.mainloop()

    def cerrar(self):
        """Cierra la ventana."""
        self._root.destroy()


# ===========================================================================
# Ejemplo de uso directo
# ===========================================================================
if __name__ == "__main__":
    import time

    manager = SliderManager("Panel de control del robot")
    velocidad = manager.crear_deslizador(0, 100,  "Velocidad del motor")
    angulo    = manager.crear_deslizador(-90, 90, "Ángulo de giro")

    print("Leyendo sliders... Cierra la ventana para salir.")

    while manager.activo():
        manager.actualizar()
        v = velocidad.obtener_valor()
        a = angulo.obtener_valor()
        print(f"Velocidad: {v:>6.1f}   Ángulo: {a:>6.1f}", end="\r")
        time.sleep(0.1)

    print("\nVentana cerrada. Fin del programa.")