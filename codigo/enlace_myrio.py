"""
enlace_myrio.py  -  Enlace Raspberry Pi 4B <-> NI myRIO

1) UART (RPi TX -> myRIO RX): envia la posicion de la pelota para graficarla en LabVIEW.
   - Puerto: /dev/serial0  (GPIO14 = TXD, pin fisico 8 -> myRIO MXP-A UART.RX, pin 10)
   - 115200 baud, 8 bits, sin paridad, 1 bit de parada (8N1), sin control de flujo
   - Trama ASCII terminada en salto de linea (LF, 0x0A), 20 veces por segundo:
         x;y;e\n        ejemplo:  -1.25;0.40;1\n
     x, y : posicion estimada en cm con 2 decimales (punto decimal)
     e    : 1 = control ON, 0 = control OFF
     Si no hay pelota se envia:  nan;nan;e\n

2) Linea digital (myRIO MXP-A DIO1, pin 13 -> RPi GPIO27, pin fisico 13): habilita el control.
   - Nivel ALTO = control ON, nivel BAJO = control OFF.
   - Se actua solo en los CAMBIOS de nivel (flancos) y con antirrebote, igual que si se
     pulsara la tecla C. Asi, el estado de la linea al arrancar no activa nada por si solo.
   - Recomendado: resistencia de 10 kOhm de GPIO27 a GND (las DIO del myRIO tienen pull-up
     de 40 kOhm a 3.3 V; sin ella la linea queda en alto si el VI no esta corriendo).
     Sin la resistencia: correr primero el VI con el boton en OFF y luego este programa.

Si pyserial o gpiozero no estan disponibles, el enlace se desactiva y el programa sigue
funcionando igual que antes (teclado).
"""

import math
import time

PUERTO_UART = "/dev/serial0"
BAUDIOS = 115200
PERIODO_ENVIO_S = 0.05          # 20 Hz: suficiente para graficar en el panel del myRIO
PIN_HABILITAR = 27              # GPIO27 (BCM) = pin fisico 13
ANTIRREBOTE_S = 0.05


class EnlaceMyRIO:

    def __init__(self):

        self.uart = None
        self.entrada = None
        self.t_envio = 0.0
        self.nivel = None
        self.nivel_candidato = None
        self.t_candidato = 0.0

        try:
            import serial
            self.uart = serial.Serial(PUERTO_UART, BAUDIOS, bytesize=8, parity="N",
                                      stopbits=1, timeout=0, write_timeout=0)
            print(f"UART al myRIO abierta en {PUERTO_UART} a {BAUDIOS} baud")
        except Exception as error:
            print(f"AVISO: UART al myRIO no disponible ({error})")

        try:
            from gpiozero import DigitalInputDevice
            self.entrada = DigitalInputDevice(PIN_HABILITAR, pull_up=None, active_state=True)
            self.nivel = bool(self.entrada.value)
            print(f"Linea de habilitacion en GPIO{PIN_HABILITAR} (nivel inicial {int(self.nivel)})")
        except Exception as error:
            print(f"AVISO: linea de habilitacion no disponible ({error})")

    def enviar(self, pos_est, control_activo):
        """Llamar en cada ciclo; solo transmite cada PERIODO_ENVIO_S."""

        if self.uart is None:
            return

        ahora = time.perf_counter()

        if ahora - self.t_envio < PERIODO_ENVIO_S:
            return

        self.t_envio = ahora

        if pos_est is None or not all(math.isfinite(v) for v in pos_est):
            texto = f"nan;nan;{int(bool(control_activo))}\n"
        else:
            texto = f"{pos_est[0]:.2f};{pos_est[1]:.2f};{int(bool(control_activo))}\n"

        try:
            self.uart.write(texto.encode("ascii"))
        except Exception:
            pass                # nunca detener el control por la UART (solo es visualizacion)

    def pedido(self):
        """Devuelve 'on' u 'off' cuando la linea cambia de nivel (con antirrebote); si no, None."""

        if self.entrada is None:
            return None

        nivel = bool(self.entrada.value)
        ahora = time.perf_counter()

        if nivel == self.nivel:
            self.nivel_candidato = None
            return None

        if nivel != self.nivel_candidato:
            self.nivel_candidato = nivel
            self.t_candidato = ahora
            return None

        if ahora - self.t_candidato < ANTIRREBOTE_S:
            return None

        self.nivel = nivel
        self.nivel_candidato = None
        return "on" if nivel else "off"

    def cerrar(self):

        try:
            if self.uart is not None:
                self.uart.close()
            if self.entrada is not None:
                self.entrada.close()
        except Exception:
            pass
