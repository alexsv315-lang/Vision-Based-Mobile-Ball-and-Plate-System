"""
CONTROL PRO MEJORADO - BALL AND PLATE   (Control10-3 ARREGLADO)

>>> CAMBIOS (el control - LQR, Kalman, ganancias, zona muerta, integral - NO se toco):
>>>  1) El lazo interno con IMU ya NO se apaga nunca. Antes se apagaba al llegar a 5 grados de correccion de servo;
>>>     ahora tiene mas recorrido (para rampas) y si se satura se reinicia y reintenta solo.
>>>  2) El limite de inclinacion del IMU ya no apaga el control: la lectura se ignora un momento y se
>>>     MANTIENE la correccion aprendida.
>>>  3) Lazo interno sin "cazar": compara con la inclinacion que el servo DEBE tener en ese instante (no con la pedida
>>>     hace un rato) y tiene banda muerta. Asi los servos ya no suben y bajan en el centro.
>>>  4) Nunca pide identificar servos ni cero del IMU: si faltan los archivos los hace solo al arrancar.
>>>  5) IMU congelado o con error -> se reinicia solo (y deshace lo que el lazo "aprendio" estando congelado).
>>>  6) Camara en un hilo propio: si se congela se reinicia sola (antes el programa se quedaba pegado).
>>>  7) Deteccion de la pelota SOLO dentro del plato, redonda y del tamano de la pelota (ignora la mano).
>>>  8) Registro de eventos (log_eventos.txt) y de temperatura / bajo voltaje de la Raspberry.
LQR + filtro de Kalman + prediccion del retardo + lazo interno con IMU + integral + zona muerta

Une lo mejor de tus dos versiones SIN cambiar tus parametros: se mantienen tus valores
(nivel X 105 / Y 70, recorrido +/- 50, KI 1.5, RETRASO 0.1, K_TRIM 15, TRIM_MAX 5, THETA_MAX 10,
ANCHO_BANDA 5 ...), tu orientacion de camara, los signos -1 / -1, el color de la pelota y la
tecla B para enmarcar el plato.

MEJORAS agregadas (cada una se puede apagar con su parametro, ver el bloque "MEJORAS"):
  1) ZONA MUERTA suave: no persigue el 0.0 exacto cuando el error ya es aceptable ->
     menos movimiento de los servos y menos consumo de bateria. Teclas 7 / 8.
  2) Servo SUAVE cerca de la referencia y RAPIDO cuando el error es grande o hay una
     perturbacion (VEL_SERVO_CERCA_DPS).
  3) La integral ya NO se borra al salir de la zona: recuerda el nivel real del plato y se
     recupera antes despues de un empujon (INTEGRAL_CONGELAR).
  4) Velocidad de aproximacion limitada (VEL_MAX_CM_S): sin frenazos bruscos.
  5) Kalman que reacciona rapido cuando le dan un empujon a la pelota (KALMAN_BOOST_*).
  6) Cambio minimo al servo (CMD_MIN_CAMBIO_GRADOS) y paralaje opcional (DISTANCIA_CAMARA_CM).
  7) "mov" (grados/s que se mueven los servos) en pantalla y en el log: mide el consumo.

Necesita mpu.py y exportar_resultados.py en la misma carpeta.

Flujo de uso:
  1) Al iniciar te pide la referencia (X, Y en cm). Enter, Enter = centro del plato (0, 0).
  2) Pulsa Z con el plato plano (guarda el cero del IMU) - solo la primera vez.
  3) QUITA LA PELOTA y pulsa G: identifica los servos con el IMU (~15 s) - solo la primera vez.
  4) Pon la pelota y pulsa C para activar el control.

Teclas:
  C  activar / desactivar el control        N  plato al centro
  Z  guardar cero del IMU (plato plano)     G  identificar servos (SIN pelota)
  U  reiniciar la deteccion del plato       F  activar/desactivar lazo interno
  B  marcar A MANO las 4 esquinas del plato con el mouse (se guardan en plato_fijo.json)
  W/A/S/D  mover la referencia 1 cm (arriba/izq/abajo/der)     R  referencia inicial
  Click izquierdo: fija la referencia en ese punto de la imagen
  1 / 2  ancho de banda (mas lento / mas rapido)
  3 / 4  amortiguamiento extra  -/+         5 / 6  Ki -/+ (pasos de 0.05)
  7 / 8  zona muerta -/+ (pasos de 0.1 cm; 0 = sin zona muerta)
  J / L  servo X -/+ 1 grado    I / K  servo Y +/- 1 grado  (control apagado)
  M  ver/ocultar la mascara de la pelota (para ajustar el color)
  E  exportar resultados (graficas, GIF y datos para PPT) en resultados/<fecha_hora>/
  P  imprimir parametros        Q  salir (el plato queda en el centro y se exporta solo)
"""

import csv
import json
import math
import os
import subprocess
import threading
import time
from collections import deque

import cv2
import numpy as np
from picamera2 import Picamera2
from smbus2 import SMBus

from mpu import MPU
from exportar_resultados import exportar_resultados
from enlace_myrio import EnlaceMyRIO   # UART + linea ON/OFF con el myRIO


# ============================================================
# REFERENCIA DONDE SE BALANCEA LA PELOTA (cm, centro del plato = 0, 0)
# ============================================================

REFERENCIA_X_CM = 0.0          # + derecha / - izquierda
REFERENCIA_Y_CM = 0.0          # + arriba  / - abajo
PEDIR_REFERENCIA_AL_INICIO = True


# ============================================================
# DIMENSIONES DEL PLATE Y PELOTA
# ============================================================

PLATE_X_CM = 19.45
PLATE_Y_CM = 19.00


# ============================================================
# ORIENTACION DE LA CAMARA RESPECTO A LOS EJES DEL PLATO
# ============================================================
# Direccion en que AUMENTA cada eje dentro de la imagen, como (derecha, abajo):
#   (1, 0) = hacia la derecha      (-1, 0) = hacia la izquierda
#   (0, 1) = hacia abajo           (0, -1) = hacia arriba
#
# Orientacion ACTUAL: arriba del plato = X-, abajo = X+, derecha = Y+, izquierda = Y-
EJE_X_EN_IMAGEN = (0, 1)     # X aumenta hacia ABAJO de la imagen
EJE_Y_EN_IMAGEN = (1, 0)     # Y aumenta hacia la DERECHA de la imagen
# Orientacion ANTERIOR (X a la derecha, Y hacia arriba): X = (1, 0) e Y = (0, -1)

TIPO_PELOTA = "maciza"         # "maciza" o "hueca" (tipo ping-pong)
K_PELOTA_OVERRIDE = 8.0   # v4: aceleracion real de la pelota (cm/s2 por grado) medida en tu CSV; la teorica 12.2 era demasiado alta  (antes None)

G_GRAV_CM_S2 = 981.0


# ============================================================
# SERVOS (PCA9685)
# ============================================================

BUS_I2C = 1
DIRECCION_PCA = 0x40

CANAL_X = 2
CANAL_Y = 1

FRECUENCIA_HZ = 50
PULSO_MIN_US = 500
PULSO_MAX_US = 2500

CENTRO_X = 105.0
CENTRO_Y = 70.0

# LIMITES DUROS POR SERVO: nunca se manda un angulo fuera de este rango (nivel +/- LIMITE_RECORRIDO)
LIMITE_RECORRIDO_GRADOS = 50.0     # cada servo puede moverse +/- 50 grados desde su nivel
ANGULO_MIN = {"x": CENTRO_X - LIMITE_RECORRIDO_GRADOS, "y": CENTRO_Y - LIMITE_RECORRIDO_GRADOS}
ANGULO_MAX = {"x": CENTRO_X + LIMITE_RECORRIDO_GRADOS, "y": CENTRO_Y + LIMITE_RECORRIDO_GRADOS}

# Signos confirmados en tu prueba con la camara girada (X+ = abajo, Y+ = derecha en la imagen):
#   subir servo X (tecla L) -> la pelota va hacia ARRIBA   (= X negativo)  -> SIGNO -1
#   subir servo Y (tecla I) -> la pelota va hacia la IZQUIERDA (= Y negativo) -> SIGNO -1
# Regla: +1 si SUBIR el servo empuja la pelota hacia el lado POSITIVO de su eje; -1 si al reves.
SIGNO_SERVO_X = -1
SIGNO_SERVO_Y = -1

MAX_VELOCIDAD_SERVO_DPS = 300.0     # grados por segundo (limita los tirones)


# ============================================================
# CONTROL
# ============================================================

ANCHO_BANDA = 5.0           # Control V3
AMORT_EXTRA = 0.1           # Control V3
KI = 1.5                    # Control V3
ZONA_INTEGRAL_CM = 4.0
I_MAX_GRADOS = 3.0          # tope de la integral (grados de plato)
THETA_MAX_GRADOS = 10.0     # Control V3

RETRASO_CAMARA_S = 0.1      # Control V3

KALMAN_SIGMA_MED_CM = 0.15  # ruido de la medicion de la camara
KALMAN_SIGMA_ACC = 30.0     # Control V3
KALMAN_GATING_CM = 6.0      # descarta mediciones que saltan mas que esto

TIEMPO_PELOTA_PERDIDA_S = 0.25   # sin pelota mas que esto -> plato al nivel

# ============================================================
# MEJORAS (ponlas en 0 / None / 1 / False para apagar cada una)
# ============================================================
ZONA_MUERTA_CM = 0.4        # Control V3
VEL_MUERTA_CM_S = 1.0       # Control V3
VEL_MAX_CM_S = 30.0         # Control V3
CMD_MIN_CAMBIO_GRADOS = 0.3 # no se manda al servo un cambio menor a esto (menos micro-movimientos)

# Servo suave cerca de la referencia y rapido lejos (MAX_VELOCIDAD_SERVO_DPS es la velocidad "lejos")
VEL_SERVO_CERCA_DPS = 150.0 # Control V3
ERROR_CERCA_CM = 1.0        # error menor a esto -> velocidad "cerca"
ERROR_LEJOS_CM = 3.0        # error mayor a esto -> velocidad "lejos" (entre ambos se interpola)

INTEGRAL_CONGELAR = True    # True = la integral se congela fuera de la zona; False = se borra (version anterior)

KALMAN_BOOST_FACTOR = 8.0  # Control V3
KALMAN_BOOST_UMBRAL_CM = 1.2    # error de prediccion que se considera empujon
KALMAN_BOOST_TIEMPO_S = 0.3

DISTANCIA_CAMARA_CM = None  # distancia de la camara al centro del plato (cm). Con un numero se activa la
                            # correccion de paralaje cuando el plato esta inclinado (solo con plato fijo)

# Lazo interno con IMU (corrige holgura y relacion de engranajes)
USAR_LAZO_INTERNO = True
K_TRIM = 20.0               # Control V3
TRIM_MAX_GRADOS = 5.0       # correccion minima disponible del lazo interno (grados de servo)
HOLGURA_SERVO_GRADOS = 1.0  # compensacion de holgura cuando NO hay lazo interno
G_DEFECTO = 1.0             # grados de plato por grado de servo si no hay identificacion


# ============================================================
# IMU
# ============================================================

USAR_IMU = True
DIRECCION_IMU = 0x68
TOLERANCIA_INICIO_GRADOS = 3.0
LIMITE_INCLINACION_IMU = 25.0   # lectura del IMU NO confiable por encima de esto. NO apaga el control: se ignora un momento
HISTERESIS_IMU_GRADOS = 2.0
IMU_CONGELADO_S = 0.5           # roll y pitch EXACTAMENTE iguales este tiempo = IMU congelado -> se reinicia solo
IMU_REINICIO_ESPERA_S = 1.0
IMU_REINTENTO_S = 4.0
AUTO_CALIBRAR_SI_FALTA = True   # si no hay calibracion_imu.json, guarda el cero solo (plato plano en el centro)
AUTO_IDENTIFICAR_SI_FALTA = True  # si no hay identificacion_servos.json, identifica los servos solo (SIN pelota en el plato)
AUTO_ESPERA_IDENT_S = 3.0
CAMARA_TIMEOUT_S = 2.0          # sin cuadros este tiempo -> se reinicia la camara
SALUD_CADA_S = 10.0
ARCHIVO_EVENTOS = "log_eventos.txt"
MAX_REINICIOS = 30              # reinicios automaticos maximos tras un error inesperado
ARCHIVO_IMU = "calibracion_imu.json"
ARCHIVO_IDENT = "identificacion_servos.json"
DELTA_IDENT_GRADOS = 6.0    # cuanto se mueve cada servo para identificarlo


# ============================================================
# CAMARA / DETECCION
# ============================================================

FPS_OBJETIVO = 60
ENFOQUE_DIST_M = None        # distancia camara-plato en metros (ej. 0.35) para enfoque manual
                             # SOLO si tu camara tiene autofoco (Camera Module 3); None = no tocar
PLATO_FIJO = True            # fija las esquinas del plato (mas rapido y estable)
AUTOFIJAR_CUADROS = 45       # cuadros estables seguidos para auto-fijar el plato
WARMUP_CAMARA_S = 2.0        # no se auto-fija el plato antes de este tiempo (deja ajustar la exposicion)
ESTABILIDAD_PX = 2.0         # las esquinas deben moverse menos de esto para auto-fijar
ARCHIVO_PLATO = "plato_fijo.json"   # esquinas del plato marcadas a mano (tecla B)
USAR_PLATO_GUARDADO = True   # si existe el archivo, el plato arranca FIJADO con esas esquinas
MARGEN_ROI_PX = 40
MOSTRAR_CADA = 3             # dibuja la ventana cada N cuadros (mas alto = mas fps)

AREA_MIN_PLATO = 5000
AREA_MIN_PELOTA = 40          # pelota pequena (antes 300)

MEDIANA_ESPACIAL = 7
MEDIANA_FRAMES = 5
MAX_SALTO_PX = 120
FRAMES_RETENER = 10
SOLIDEZ_MIN = 0.80
RATIO_MIN = 0.5
RATIO_MAX = 2.0

# Rango HSV de la pelota azul (el que ya te funcionaba). Tambien se ajusta en vivo
# con las barras "Pelota ..." de la ventana.
PELOTA_H_MIN = 100
PELOTA_H_MAX = 130
PELOTA_S_MIN = 120
PELOTA_V_MIN = 70

# Filtros para que SOLO cuente la pelota (no la mano ni cosas de afuera)
PELOTA_DIAM_CM = None        # diametro REAL de tu pelota en cm (ej. 4.0). None = lo aprende solo
DIAM_MIN_CM = 0.8            # antes de aprenderlo
DIAM_MAX_CM = 6.0
TOLERANCIA_DIAM = 0.35       # una vez conocido: +/- 35 %
CIRCULARIDAD_MIN = 0.55      # 1 = circulo perfecto
LLENADO_MIN = 0.55           # area / area del circulo minimo (descarta dedos y formas alargadas)
ZONA_EXTRA_PX = 6            # margen de la zona del plato
SALTO_MAX_CM = 5.0           # no salta a otro objeto a mas de esto de donde estaba
RADIO_BUSQUEDA_CM = 8.0
PELOTA_RETENER = 4           # cuadros que mantiene la ultima posicion si la pierde un instante
DET_ESCALA = 0.5             # la deteccion trabaja a esta escala (mas rapido)

# Umbral del plato blanco (solo se usa si el plato se detecta en vivo)
BLANCO_S_MAX = 60
BLANCO_V_MIN = 170
# Si en tu camara NO la detecta, prueba el rango alterno (picamera2 "RGB888" entrega los
# canales R y B intercambiados, y entonces el azul real cae en el rango del naranja):
#   PELOTA_H_MIN = 5, PELOTA_H_MAX = 22, PELOTA_S_MIN = 120, PELOTA_V_MIN = 100

kernel = np.ones((5, 5), np.uint8)

VENTANA = "Ball and Plate - Control avanzado"

ARCHIVO_LOG = "log_control.csv"
REGISTRAR_SIEMPRE = False    # True = registra aunque el control este apagado


# ============================================================
# ALGORITMOS (se prueban por separado en simulacion)
# ============================================================
# ===== INICIO ALGORITMOS =====

def limitar_valor(v, minimo, maximo):

    return max(minimo, min(maximo, v))


def lqr_ganancias(b, wn, q_vel=0.0):
    """
    LQR del doble integrador  x'' = b * theta  (theta = inclinacion del plato en grados).
    Ley de control: theta = -(k1 * error + k2 * velocidad).
    Se elige el peso de posicion para que la frecuencia natural sea wn; con q_vel = 0
    el amortiguamiento queda en 0.707 (optimo). q_vel > 0 amortigua mas.
    """

    R = 1.0

    q1 = (wn ** 2 / b) ** 2

    p12 = math.sqrt(q1 * R) / b

    k1 = math.sqrt(q1 / R)

    k2 = math.sqrt((2.0 * p12 + q_vel) / R)

    return k1, k2




class KalmanEje:
    """
    Kalman de un eje: estado [posicion, velocidad], entrada = inclinacion del plato.
    Si detecta un empujon (error de prediccion grande) sube el ruido de proceso un momento
    para reaccionar rapido a la nueva velocidad de la pelota.
    """

    def __init__(self, b, sigma_med, sigma_acc, gating,
                 boost_factor=1.0, boost_umbral=1e9, boost_tiempo=0.0):

        self.b = b
        self.sm2 = sigma_med ** 2
        self.sa2 = sigma_acc ** 2
        self.gating = gating
        self.boost_factor = boost_factor
        self.boost_umbral = boost_umbral
        self.boost_tiempo = boost_tiempo
        self.boost_t = 0.0

        self.x = np.zeros(2)
        self.P = np.diag([1.0, 100.0])
        self.iniciado = False
        self.rechazos = 0

    def reiniciar(self):

        self.iniciado = False
        self.rechazos = 0
        self.boost_t = 0.0

    def iniciar(self, pos):

        self.x = np.array([pos, 0.0])
        self.P = np.diag([1.0, 100.0])
        self.iniciado = True
        self.rechazos = 0
        self.boost_t = 0.0

    def predecir(self, dt, theta):

        F = np.array([[1.0, dt], [0.0, 1.0]])

        Bu = np.array([0.5 * dt * dt, dt]) * self.b * theta

        self.x = F @ self.x + Bu

        q = self.sa2 * (self.boost_factor if self.boost_t > 0.0 else 1.0)

        self.boost_t = max(0.0, self.boost_t - dt)

        Q = q * np.array([
            [dt ** 4 / 4.0, dt ** 3 / 2.0],
            [dt ** 3 / 2.0, dt ** 2]
        ])

        self.P = F @ self.P @ F.T + Q

    def actualizar(self, med):

        innovacion = med - self.x[0]

        if abs(innovacion) > self.gating:

            self.rechazos += 1

            if self.rechazos >= 5:
                self.iniciar(med)
                return True

            return False

        if abs(innovacion) > self.boost_umbral:
            self.boost_t = self.boost_tiempo

        S = self.P[0, 0] + self.sm2

        K = self.P[:, 0] / S

        self.x = self.x + K * innovacion

        self.P = self.P - np.outer(K, self.P[0, :])

        self.rechazos = 0

        return True

    def adelantar(self, tau, theta):
        """Estado (posicion, velocidad) predicho tau segundos hacia adelante."""

        pos, vel = self.x

        acc = self.b * theta

        return (
            pos + vel * tau + 0.5 * acc * tau * tau,
            vel + acc * tau
        )


class ControlEje:
    """
    Realimentacion de estados (LQR) con:
      - zona muerta suave: solo actua sobre la parte del error que pasa de la zona,
      - velocidad de aproximacion limitada (sin frenazos bruscos si el error es grande),
      - integral con anti-windup: se congela (no se borra) fuera de la zona o al saturar.
    """

    def __init__(self):

        self.integral = 0.0
        self.saturado = False

    def reiniciar(self):

        self.integral = 0.0
        self.saturado = False

    def calcular(self, pos, vel, ref, dt, k1, k2, ki, zona, i_max_grados, theta_max,
                 zona_muerta=0.0, vel_muerta=0.0, vel_max=None, congelar_integral=True):

        error = pos - ref

        # Zona muerta suave: dentro de la zona no se persigue el error
        if abs(error) <= zona_muerta:
            e_ef = 0.0
        else:
            e_ef = error - math.copysign(zona_muerta, error)

        # Velocidades pequenas (ruido) se ignoran
        if abs(vel) <= vel_muerta:
            v_ef = 0.0
        else:
            v_ef = vel - math.copysign(vel_muerta, vel)

        # Integral: solo cerca de la referencia y sin saturar; fuera de eso se congela
        # (o se borra si congelar_integral = False, como en la version anterior)
        if ki > 0:

            if abs(error) < zona and not self.saturado:

                self.integral += e_ef * dt

                limite = i_max_grados / ki

                self.integral = limitar_valor(self.integral, -limite, limite)

            elif abs(error) >= zona and not congelar_integral:

                self.integral = 0.0

        if vel_max is not None and k2 > 0:

            # Equivale a -(k1*e + k2*v) mientras la velocidad deseada no se satura
            v_des = -limitar_valor((k1 / k2) * e_ef, -vel_max, vel_max)

            theta = -(k2 * (v_ef - v_des) + ki * self.integral)

        else:

            theta = -(k1 * e_ef + k2 * v_ef + ki * self.integral)

        self.saturado = abs(theta) >= theta_max

        return limitar_valor(theta, -theta_max, theta_max)


class LazoInterno:
    """
    Lazo interno: compara la inclinacion deseada con la que mide el IMU y ajusta
    el servo (accion integral) hasta que el plato llegue donde se pidio.
    Absorbe la holgura de los servos y los errores de la relacion de engranajes.
    """

    def __init__(self):

        self.trim = 0.0
        self.saturado_s = 0.0

    def reiniciar(self):

        self.trim = 0.0
        self.saturado_s = 0.0

    def actualizar(self, theta_des, theta_meas, dt, signo, G, k_trim, trim_max):

        error = theta_des - theta_meas

        self.trim += k_trim * dt * signo * error / G

        self.trim = limitar_valor(self.trim, -trim_max, trim_max)

        if abs(self.trim) >= trim_max - 1e-6:
            self.saturado_s += dt
        else:
            self.saturado_s = 0.0

        return self.trim


def corregir_paralaje(xm, ym, th_x_deg, th_y_deg, distancia_cm):
    """
    Con el plato FIJADO en la imagen, cuando el plato se inclina los puntos lejos del centro
    se ven corridos (cambia su altura respecto a la camara). Con la inclinacion medida por
    el IMU y la distancia de la camara al centro del plato se recupera la posicion real.
    th > 0 significa que el lado positivo del eje queda mas bajo (el que empuja a la pelota).
    """

    tx = math.radians(th_x_deg)
    ty = math.radians(th_y_deg)

    cx = math.cos(tx)
    cy = math.cos(ty)

    x, y = xm, ym

    for _ in range(4):

        s = x * math.sin(tx) + y * math.sin(ty)

        x = xm * (distancia_cm + s) / (distancia_cm * cx)
        y = ym * (distancia_cm + s) / (distancia_cm * cy)

    return x, y

def limite_velocidad_servo(error_cm, v_lejos, v_cerca, e_cerca, e_lejos):
    """Velocidad maxima del servo: suave cerca de la referencia y rapida cuando hay error grande."""

    if v_cerca is None or v_cerca >= v_lejos:
        return v_lejos

    if error_cm <= e_cerca:
        return v_cerca

    if error_cm >= e_lejos:
        return v_lejos

    f = (error_cm - e_cerca) / (e_lejos - e_cerca)

    return v_cerca + f * (v_lejos - v_cerca)

# ===== FIN ALGORITMOS =====


def constante_pelota(tipo):
    """cm/s^2 por grado de plato: (factor) * g, con factor 5/7 (maciza) o 3/5 (hueca)."""

    factor = 5.0 / 7.0 if tipo == "maciza" else 3.0 / 5.0

    return factor * G_GRAV_CM_S2 * math.pi / 180.0


CENTRO = {"x": CENTRO_X, "y": CENTRO_Y}
SIGNO = {"x": SIGNO_SERVO_X, "y": SIGNO_SERVO_Y}
CANALES = {"x": CANAL_X, "y": CANAL_Y}


# ============================================================
# DRIVER PCA9685
# ============================================================

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

bus_pca = SMBus(BUS_I2C)

REINICIOS = [0]          # cuantas veces el programa se reinicio solo tras un error
ang = {"x": None, "y": None}
ERRORES_I2C = {"total": 0, "seguidos": 0}


def iniciar_pca():

    prescale = int(round(25000000.0 / (4096 * FRECUENCIA_HZ)) - 1)

    bus_pca.write_byte_data(DIRECCION_PCA, MODE1, 0x10)
    bus_pca.write_byte_data(DIRECCION_PCA, PRESCALE, prescale)
    bus_pca.write_byte_data(DIRECCION_PCA, MODE1, 0x00)
    time.sleep(0.005)
    bus_pca.write_byte_data(DIRECCION_PCA, MODE1, 0xA0)


def angulo_a_pulso(angulo):

    return PULSO_MIN_US + (angulo / 180.0) * (PULSO_MAX_US - PULSO_MIN_US)


def poner_pulso(canal, microsegundos):

    periodo_us = 1000000.0 / FRECUENCIA_HZ

    ticks = int(round(microsegundos * 4096 / periodo_us))
    ticks = max(0, min(4095, ticks))

    base = LED0_ON_L + 4 * canal

    for _ in range(4):

        try:

            bus_pca.write_i2c_block_data(
                DIRECCION_PCA,
                base,
                [0, 0, ticks & 0xFF, (ticks >> 8) & 0x0F]
            )

            ERRORES_I2C["seguidos"] = 0

            return

        except OSError:

            time.sleep(0.002)

    # no se pudo escribir: se cuenta y se sigue (el servo conserva el ultimo pulso); no se cae el programa
    ERRORES_I2C["total"] += 1
    ERRORES_I2C["seguidos"] += 1

    if ERRORES_I2C["seguidos"] % 20 == 1:
        evento(f"I2C PCA9685: error de escritura (total {ERRORES_I2C['total']}); reintentando")

    if ERRORES_I2C["seguidos"] % 50 == 0:
        try:
            iniciar_pca()
        except Exception:
            pass


def limitar(nombre, valor):

    return limitar_valor(valor, ANGULO_MIN[nombre], ANGULO_MAX[nombre])


def enviar(nombre, angulo):

    angulo = limitar(nombre, angulo)

    poner_pulso(CANALES[nombre], angulo_a_pulso(angulo))

    ang[nombre] = angulo


def ir_suave(destino_x, destino_y, paso=1.0, espera=0.02):
    """Mueve los dos servos al destino en pasos pequenos (bloquea hasta llegar)."""

    destinos = {"x": limitar("x", destino_x), "y": limitar("y", destino_y)}

    for nombre in ("x", "y"):
        if ang[nombre] is None:
            enviar(nombre, destinos[nombre])

    while True:

        listo = True

        for nombre in ("x", "y"):

            actual = ang[nombre]
            meta = destinos[nombre]

            if abs(meta - actual) > paso:
                enviar(nombre, actual + math.copysign(paso, meta - actual))
                listo = False
            else:
                enviar(nombre, meta)

        if listo:
            break

        time.sleep(espera)


# ============================================================
# IMU: cero guardado, comprobacion e identificacion de servos
# ============================================================

def cargar_cero_imu(mpu):

    if not os.path.exists(ARCHIVO_IMU):
        return False

    with open(ARCHIVO_IMU, "r") as f:
        datos = json.load(f)

    with mpu._lock:
        mpu.roll_zero = float(datos["roll_zero"])
        mpu.pitch_zero = float(datos["pitch_zero"])

    return True


def guardar_cero_imu(mpu):

    rolls = []
    pitches = []

    for _ in range(30):

        with mpu._lock:
            rolls.append(mpu.roll)
            pitches.append(mpu.pitch)

        time.sleep(0.02)

    roll_zero = sum(rolls) / len(rolls)
    pitch_zero = sum(pitches) / len(pitches)

    with mpu._lock:
        mpu.roll_zero = roll_zero
        mpu.pitch_zero = pitch_zero

    with open(ARCHIVO_IMU, "w") as f:
        json.dump({"roll_zero": roll_zero, "pitch_zero": pitch_zero}, f, indent=4)

    return roll_zero, pitch_zero


def verificar_inicio(mpu, imu_calibrada):

    if mpu is None:
        return True, "IMU no disponible: se omite la comprobacion de inicio."

    if not imu_calibrada:
        return False, "IMU sin cero guardado: con el plato plano en el centro, pulsa Z."

    roll, pitch = mpu.angulos()

    if abs(roll) > TOLERANCIA_INICIO_GRADOS or abs(pitch) > TOLERANCIA_INICIO_GRADOS:
        return False, (
            f"El plato no esta plano en el centro (roll={roll:.1f}, pitch={pitch:.1f}). "
            "Revisa el montaje, o pulsa Z si el plato esta realmente plano."
        )

    return True, f"Inicio OK: roll={roll:.1f}  pitch={pitch:.1f}"


def promedio_imu(mpu, n=12, espera=0.025):

    r = 0.0
    p = 0.0

    for _ in range(n):

        roll, pitch = mpu.angulos()

        r += roll
        p += pitch

        time.sleep(espera)

    return r / n, p / n


def identificar_servo(mpu, nombre):
    """
    Mueve un servo a dos posiciones y mide con el IMU cuanto se inclina el plato.
    Devuelve {"eje_imu", "signo_imu", "G"} o None si la medicion no es confiable.
    G = grados de plato por grado de servo.
    """

    centro = CENTRO[nombre]

    d_dn = min(DELTA_IDENT_GRADOS, centro - ANGULO_MIN[nombre])
    d_up = min(DELTA_IDENT_GRADOS, ANGULO_MAX[nombre] - centro)

    def ir(valor):

        if nombre == "x":
            ir_suave(valor, CENTRO_Y)
        else:
            ir_suave(CENTRO_X, valor)

        time.sleep(1.2)

    ir(centro)
    ir(centro - d_dn)
    m1 = promedio_imu(mpu, n=30)

    ir(centro + d_up)
    m2 = promedio_imu(mpu, n=30)

    ir(centro)

    d_roll = m2[0] - m1[0]
    d_pitch = m2[1] - m1[1]

    if abs(d_roll) >= abs(d_pitch):
        eje, delta, cruz = "roll", d_roll, d_pitch
    else:
        eje, delta, cruz = "pitch", d_pitch, d_roll

    span = d_up + d_dn

    print(f"  Servo {nombre.upper()}: d_roll={d_roll:.2f}  d_pitch={d_pitch:.2f}  (span {span:.1f} grados de servo)")

    if span < 5.0 or abs(delta) < 0.4 or abs(cruz) > 0.5 * abs(delta):
        return None

    return {
        "eje_imu": eje,
        "signo_imu": 1 if delta > 0 else -1,
        "G": abs(delta) / span
    }


def identificar_todos(mpu):

    resultado = {}

    for nombre in ("x", "y"):

        r = None

        for intento in range(3):

            r = identificar_servo(mpu, nombre)

            if r is not None:
                break

            print(f"  Servo {nombre.upper()}: respuesta del IMU poco clara (intento {intento + 1}/3).")

        if r is None:
            print(f"  No se pudo identificar el servo {nombre.upper()} (respuesta del IMU poco clara).")
            return None

        resultado[nombre] = r

    with open(ARCHIVO_IDENT, "w") as f:
        json.dump(resultado, f, indent=4)

    return resultado


def cargar_identificacion():

    if not os.path.exists(ARCHIVO_IDENT):
        return None

    try:
        with open(ARCHIVO_IDENT, "r") as f:
            datos = json.load(f)

        for nombre in ("x", "y"):
            datos[nombre]["G"] = float(datos[nombre]["G"])
            datos[nombre]["signo_imu"] = int(datos[nombre]["signo_imu"])

            if not (0.1 <= datos[nombre]["G"] <= 4.0):
                print(f"Identificacion descartada: G del servo {nombre.upper()} = {datos[nombre]['G']:.2f} no es creible.")
                return None

        return datos

    except Exception:
        return None


def theta_bola(ident, nombre, roll, pitch):
    """Inclinacion del plato (grados) en el sentido en que empuja a la pelota hacia el lado positivo."""

    i = ident[nombre]

    valor = roll if i["eje_imu"] == "roll" else pitch

    return SIGNO[nombre] * i["signo_imu"] * valor


def mostrar_identificacion(ident):

    for nombre in ("x", "y"):

        G = ident[nombre]["G"]

        arriba = (ANGULO_MAX[nombre] - CENTRO[nombre]) * G
        abajo = (CENTRO[nombre] - ANGULO_MIN[nombre]) * G

        print(
            f"  Servo {nombre.upper()}: usa el eje {ident[nombre]['eje_imu']} del IMU, "
            f"G = {G:.2f} grados de plato por grado de servo; "
            f"inclinacion disponible: +{arriba:.1f} / -{abajo:.1f} grados de plato"
        )


# ============================================================
# DETECCION DEL PLATO Y LA PELOTA
# ============================================================

def dims_imagen():
    """Ancho y alto del plato (cm) tal como se ven en la imagen."""

    if abs(EJE_X_EN_IMAGEN[0]) == 1:
        return PLATE_X_CM, PLATE_Y_CM

    return PLATE_Y_CM, PLATE_X_CM


def pixel_a_ejes(matriz, px, py):
    """Pixel de la imagen -> (x, y) en cm sobre los ejes del plato (centro = 0, 0)."""

    ancho, alto = dims_imagen()

    punto = np.array([[[px, py]]], dtype=np.float32)

    u, v = cv2.perspectiveTransform(punto, matriz)[0][0]

    cu = float(u) - ancho / 2.0      # + hacia la derecha de la imagen
    cv_ = float(v) - alto / 2.0      # + hacia abajo de la imagen

    x = cu * EJE_X_EN_IMAGEN[0] + cv_ * EJE_X_EN_IMAGEN[1]
    y = cu * EJE_Y_EN_IMAGEN[0] + cv_ * EJE_Y_EN_IMAGEN[1]

    return x, y


def ejes_a_pixel(inversa, x, y):
    """(x, y) en cm sobre los ejes del plato -> pixel de la imagen."""

    ancho, alto = dims_imagen()

    cu = x * EJE_X_EN_IMAGEN[0] + y * EJE_Y_EN_IMAGEN[0]
    cv_ = x * EJE_X_EN_IMAGEN[1] + y * EJE_Y_EN_IMAGEN[1]

    punto = np.array([[[cu + ancho / 2.0, cv_ + alto / 2.0]]], dtype=np.float32)

    return cv2.perspectiveTransform(punto, inversa)[0][0]


def nada(_):
    pass


def limpiar_mascara(mask):

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.medianBlur(mask, MEDIANA_ESPACIAL)

    return mask


def limpiar_mascara_pelota(mask):
    """Limpieza suave: no borra pelotas pequenas ni desenfocadas."""

    k = np.ones((3, 3), np.uint8)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    mask = cv2.medianBlur(mask, 3)

    return mask


def contorno_mayor(mask, area_min):

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    mayor = None
    area_mayor = 0

    for contour in contours:

        area = cv2.contourArea(contour)

        if area > area_min and area > area_mayor:
            area_mayor = area
            mayor = contour

    return mayor


def ordenar_puntos(puntos):

    puntos = np.array(puntos, dtype=np.float32).reshape(4, 2)

    centro = puntos.mean(axis=0)

    angulos = np.arctan2(
        puntos[:, 1] - centro[1],
        puntos[:, 0] - centro[0]
    )

    puntos = puntos[np.argsort(angulos)]

    inicio = np.argmin(puntos[:, 0] + puntos[:, 1])

    return np.roll(puntos, -inicio, axis=0)


def encontrar_esquinas(contorno):

    hull = cv2.convexHull(contorno)

    perimetro = cv2.arcLength(hull, True)

    for factor in np.arange(0.01, 0.08, 0.005):

        aprox = cv2.approxPolyDP(hull, factor * perimetro, True)

        if len(aprox) == 4:
            return ordenar_puntos(aprox), True

    caja = cv2.boxPoints(cv2.minAreaRect(hull))

    return ordenar_puntos(caja), False


def evaluar_candidato(contorno):

    area = cv2.contourArea(contorno)

    area_hull = cv2.contourArea(cv2.convexHull(contorno))

    if area_hull <= 0:
        return None

    if area / area_hull < SOLIDEZ_MIN:
        return None

    esquinas, exacto = encontrar_esquinas(contorno)

    lados = [
        np.linalg.norm(esquinas[i] - esquinas[(i + 1) % 4])
        for i in range(4)
    ]

    ancho = (lados[0] + lados[2]) / 2.0
    alto = (lados[1] + lados[3]) / 2.0

    if alto <= 0:
        return None

    ratio = ancho / alto

    if ratio < RATIO_MIN or ratio > RATIO_MAX:
        return None

    return esquinas, exacto, esquinas.mean(axis=0), area


def buscar_candidatos(mask, area_min):

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    candidatos = []

    for contour in contours:

        if cv2.contourArea(contour) < area_min:
            continue

        candidato = evaluar_candidato(contour)

        if candidato is not None:
            candidatos.append(candidato)

    return candidatos


class SeguidorPlato:
    """Sigue el plato con mediana temporal y sin saltar a otros objetos blancos."""

    def __init__(self):
        self.historial = deque(maxlen=MEDIANA_FRAMES)
        self.centro_previo = None
        self.ultimas = None
        self.perdido = 0

    def reiniciar(self):
        self.historial.clear()
        self.centro_previo = None
        self.ultimas = None
        self.perdido = 0

    def actualizar(self, mask_white):

        candidatos = buscar_candidatos(mask_white, AREA_MIN_PLATO)

        elegido = None

        if self.centro_previo is not None:

            cercanos = [
                c for c in candidatos
                if np.linalg.norm(c[2] - self.centro_previo) < MAX_SALTO_PX
            ]

            if cercanos:
                elegido = max(cercanos, key=lambda c: c[3])

        elif candidatos:

            elegido = max(candidatos, key=lambda c: c[3])

        puntos = None
        estado = "PLATO NO DETECTADO"
        color = (0, 0, 255)

        if elegido is not None:

            esquinas, exacto, centro, area = elegido

            self.historial.append(esquinas)

            puntos = np.median(
                np.array(self.historial),
                axis=0
            ).astype(np.float32)

            self.centro_previo = puntos.mean(axis=0)
            self.ultimas = puntos
            self.perdido = 0

            if exacto:
                estado = "PLATO DETECTADO"
                color = (0, 255, 0)
            else:
                estado = "PLATO (esquinas aproximadas)"
                color = (0, 255, 255)

        elif self.ultimas is not None:

            self.perdido += 1

            if self.perdido <= FRAMES_RETENER:

                puntos = self.ultimas
                estado = "PLATO (reteniendo ultimo marco)"
                color = (0, 165, 255)

            else:

                self.reiniciar()

        return puntos, estado, color


def calcular_roi(puntos, forma):

    alto, ancho = forma[:2]

    if puntos is None:
        return (0, 0, ancho, alto)

    x0 = int(max(0, puntos[:, 0].min() - MARGEN_ROI_PX))
    y0 = int(max(0, puntos[:, 1].min() - MARGEN_ROI_PX))
    x1 = int(min(ancho, puntos[:, 0].max() + MARGEN_ROI_PX))
    y1 = int(min(alto, puntos[:, 1].max() + MARGEN_ROI_PX))

    return (x0, y0, x1, y1)


def guardar_plato(puntos):

    with open(ARCHIVO_PLATO, "w") as f:
        json.dump({"esquinas": np.asarray(puntos, dtype=float).tolist()}, f, indent=4)


def cargar_plato():

    if not os.path.exists(ARCHIVO_PLATO):
        return None

    try:

        with open(ARCHIVO_PLATO, "r") as f:
            datos = json.load(f)

        return np.array(datos["esquinas"], dtype=np.float32).reshape(4, 2)

    except Exception:

        return None


USAR_BARRAS = True          # True = lee las barras de la ventana; False = usa las constantes


def leer_barra(nombre, defecto):

    if USAR_BARRAS:
        return cv2.getTrackbarPos(nombre, VENTANA)

    return defecto


def detectar_pelota(frame_rgb, roi):
    """Detecta la pelota azul solo dentro del ROI. Devuelve (centro, contorno, mascara)."""

    x0, y0, x1, y1 = roi

    sub = frame_rgb[y0:y1, x0:x1]

    blur = cv2.GaussianBlur(sub, (5, 5), 0)

    hsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)

    h_min = leer_barra("Pelota H min", PELOTA_H_MIN)
    h_max = leer_barra("Pelota H max", PELOTA_H_MAX)
    s_min = leer_barra("Pelota S min", PELOTA_S_MIN)
    v_min = leer_barra("Pelota V min", PELOTA_V_MIN)

    mask = cv2.inRange(
        hsv,
        np.array([h_min, s_min, v_min]),
        np.array([h_max, 255, 255])
    )

    mask = limpiar_mascara_pelota(mask)

    contorno = contorno_mayor(mask, AREA_MIN_PELOTA)

    if contorno is None:
        return None, None, mask

    M = cv2.moments(contorno)

    if M["m00"] == 0:
        return None, None, mask

    centro = (M["m10"] / M["m00"] + x0, M["m01"] / M["m00"] + y0)

    contorno = contorno + np.array([[[x0, y0]]], dtype=np.int32)

    return centro, contorno, mask


# ============================================================
# EVENTOS (se guardan en log_eventos.txt para ver que paso si algo falla)
# ============================================================

T_PROGRAMA = time.monotonic()


def evento(texto):

    linea = f"[{time.monotonic() - T_PROGRAMA:8.2f}s] {texto}"

    print(linea)

    try:
        with open(ARCHIVO_EVENTOS, "a") as f:
            f.write(time.strftime("%Y-%m-%d %H:%M:%S ") + linea + "\n")
    except Exception:
        pass



class DetectorPelota:
    """
    Busca SOLO un objeto REDONDO del tamano de la pelota DENTRO del plato:
      - zona: solo el interior del plato fijado (la mano / cosas de afuera no cuentan)
      - color: el rango HSV de tu codigo original (la camara "ve" la roja como azul)
      - forma: circularidad y relleno del circulo minimo (descarta dedos, brazo, manchas alargadas)
      - tamano: diametro en cm (se aprende solo si PELOTA_DIAM_CM = None)
      - continuidad: no salta de golpe a otro objeto lejano
      - 2 intentos (estricto y relajado cerca de donde estaba) + retencion de unos cuadros
    """

    def __init__(self):

        self.diam = deque(maxlen=60)
        self.clave = None
        self.zona = None
        self.px_cm = 10.0
        self.poly = None
        self.diam_ref_px = None
        self.ultima = None
        self.perdida = 0
        self.estado = None

    def _zona(self, puntos, roi, escala):

        clave = (puntos.tobytes(), tuple(roi), escala)

        if clave != self.clave:

            x0, y0, x1, y1 = roi

            h = max(1, int((y1 - y0) * escala + 0.5))
            w = max(1, int((x1 - x0) * escala + 0.5))

            z = np.zeros((h, w), np.uint8)

            poly = ((puntos - np.array([x0, y0], np.float32)) * escala).astype(np.int32)

            cv2.fillConvexPoly(z, poly, 255)

            k = max(3, int(2 * ZONA_EXTRA_PX * escala) | 1)

            self.zona = cv2.dilate(z, np.ones((k, k), np.uint8))
            self.poly = puntos.astype(np.float32).reshape(-1, 1, 2)
            self.px_cm = math.sqrt(max(cv2.contourArea(self.poly), 1.0) / (PLATE_X_CM * PLATE_Y_CM))
            self.clave = clave

        return self.zona

    def rango_diametro(self):
        """(d_min, d_max) en pixeles de la imagen completa."""

        if PELOTA_DIAM_CM:
            d0 = PELOTA_DIAM_CM * self.px_cm
        elif len(self.diam) >= 20:
            d0 = float(np.median(self.diam))
        else:
            return DIAM_MIN_CM * self.px_cm, DIAM_MAX_CM * self.px_cm

        self.diam_ref_px = d0

        return d0 * (1.0 - TOLERANCIA_DIAM), d0 * (1.0 + TOLERANCIA_DIAM)

    def detectar(self, frame, roi, puntos, ultima, perdida):
        """Devuelve (x, y, estado, contorno, mascara). estado: ok / relajado / retenida / None (pixeles de la imagen completa)."""

        x0, y0, x1, y1 = roi

        sub = frame[y0:y1, x0:x1]

        escala = DET_ESCALA

        if escala < 1.0:
            w = max(1, int((x1 - x0) * escala + 0.5))
            h = max(1, int((y1 - y0) * escala + 0.5))
            sub = cv2.resize(sub, (w, h), interpolation=cv2.INTER_AREA)       # tambien actua de desenfoque
            escala_x = w / float(x1 - x0)
            escala_y = h / float(y1 - y0)
        else:
            sub = cv2.GaussianBlur(sub, (5, 5), 0)
            escala_x = escala_y = 1.0

        zona = self._zona(puntos, roi, escala_x)

        if zona.shape[:2] != sub.shape[:2]:
            zona = cv2.resize(zona, (sub.shape[1], sub.shape[0]), interpolation=cv2.INTER_NEAREST)

        hsv = cv2.cvtColor(sub, cv2.COLOR_RGB2HSV)

        h_min = leer_barra("Pelota H min", PELOTA_H_MIN)
        h_max = leer_barra("Pelota H max", PELOTA_H_MAX)
        s_min = leer_barra("Pelota S min", PELOTA_S_MIN)
        v_min = leer_barra("Pelota V min", PELOTA_V_MIN)

        d_min, d_max = self.rango_diametro()

        mascara_final = None

        for rel, circ_min, cerca in ((1.0, CIRCULARIDAD_MIN, False), (0.7, CIRCULARIDAD_MIN * 0.8, True)):

            if cerca and ultima is None:
                continue

            mask = cv2.inRange(hsv, np.array([h_min, int(s_min * rel), int(v_min * rel)]), np.array([h_max, 255, 255]))

            mask = cv2.bitwise_and(mask, zona)

            if cerca:
                disco = np.zeros_like(mask)
                cx_l = int((ultima[0] - x0) * escala_x)
                cy_l = int((ultima[1] - y0) * escala_y)
                cv2.circle(disco, (cx_l, cy_l), int(RADIO_BUSQUEDA_CM * self.px_cm * escala_x), 255, -1)
                mask = cv2.bitwise_and(mask, disco)

            mask = limpiar_mascara_pelota(mask)

            if mascara_final is None:
                mascara_final = mask

            c = self._mejor(mask, circ_min, d_min, d_max, escala_x, escala_y, x0, y0, ultima, perdida)

            if c is not None:

                cx, cy, d_px, circ, cont = c

                if rel == 1.0 and circ >= 0.75:
                    self.diam.append(d_px)

                return cx, cy, ("ok" if rel == 1.0 else "relajado"), cont, mask

        if ultima is not None and perdida < PELOTA_RETENER:
            return ultima[0], ultima[1], "retenida", None, mascara_final

        return None, None, None, None, mascara_final

    def _mejor(self, mask, circ_min, d_min, d_max, ex, ey, x0, y0, ultima, perdida):

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mejor = None
        mejor_puntaje = 0.0

        area_min = AREA_MIN_PELOTA * ex * ey
        compuerta = (SALTO_MAX_CM + 1.0 * perdida) * self.px_cm if (ultima is not None and perdida <= 4) else None

        for c in contornos:

            area = cv2.contourArea(c)

            if area < area_min:
                continue

            d_eq = math.sqrt(4.0 * area / math.pi) / ex

            if d_eq < d_min or d_eq > d_max:
                continue

            per = cv2.arcLength(c, True)

            if per <= 0:
                continue

            circ = 4.0 * math.pi * area / (per * per)

            if circ < circ_min:
                continue

            (_, _), radio = cv2.minEnclosingCircle(c)

            if radio <= 0 or area / (math.pi * radio * radio) < LLENADO_MIN:
                continue

            M = cv2.moments(c)

            if M["m00"] == 0:
                continue

            cx = (M["m10"] / M["m00"] + 0.5) / ex - 0.5 + x0
            cy = (M["m01"] / M["m00"] + 0.5) / ey - 0.5 + y0

            if cv2.pointPolygonTest(self.poly, (float(cx), float(cy)), True) < -2.0:
                continue

            dist = 0.0

            if compuerta is not None:
                dist = math.hypot(cx - ultima[0], cy - ultima[1])
                if dist > compuerta:
                    continue

            puntaje = circ * area / (1.0 + dist / (2.0 * self.px_cm))

            if puntaje > mejor_puntaje:

                mejor_puntaje = puntaje

                cont = (c.astype(np.float32) + 0.5) / np.array([ex, ey], np.float32) - 0.5 + np.array([x0, y0], np.float32)

                mejor = (cx, cy, d_eq, circ, cont.astype(np.int32))

        return mejor


    def buscar(self, frame, roi, puntos):
        """Como detectar_pelota(), pero con filtros y memoria. Devuelve (centro, contorno, mascara)."""

        px, py, estado, contorno, mask = self.detectar(frame, roi, puntos, self.ultima, self.perdida)

        if estado in ("ok", "relajado"):
            self.perdida = 0
            self.ultima = (px, py)
        else:
            self.perdida += 1

            if estado is None:
                self.ultima = None if self.perdida > PELOTA_RETENER else self.ultima

        self.estado = estado

        if px is None:
            return None, None, mask

        return (px, py), contorno, mask



class GestorIMU:
    """Lee el IMU y dice si la lectura es valida. Si se congela o da error, lo reinicia solo (sin bloquear los servos)."""

    def __init__(self):

        self.mpu = None
        self.calibrada = False
        self.prev = None
        self.igual_desde = None
        self.fuera = False
        self.motivo = ""
        self.motivo_prev = ""
        self.invalido_desde = None
        self.t_reintento = 0.0
        self.t_aviso = 0.0
        self.reiniciando = False
        self.reinicios = 0
        self.ultimo = (0.0, 0.0)
        self.tuvo_imu = False

    def arrancar(self):

        try:

            self.mpu = MPU(addr=DIRECCION_IMU, hz=200, ventana_mediana=3)
            self.mpu.iniciar()

            self.calibrada = cargar_cero_imu(self.mpu)
            self.tuvo_imu = True

            return True

        except Exception as error:

            print(f"IMU no disponible: {error}")
            self.mpu = None

            return False

    def _reiniciar(self):

        try:

            if self.mpu is not None:
                try:
                    self.mpu.detener()
                except Exception:
                    pass

            self.mpu = MPU(addr=DIRECCION_IMU, hz=200, ventana_mediana=3)
            self.mpu.iniciar()

            cargar_cero_imu(self.mpu)
            self.tuvo_imu = True

            self.prev = None
            self.igual_desde = None
            self.invalido_desde = None

            evento("IMU: reiniciado OK")

        except Exception as error:

            self.mpu = None
            evento(f"IMU: no se pudo reiniciar ({error}); se reintenta en {IMU_REINTENTO_S:.0f} s")

        self.reiniciando = False

    def leer(self, ahora):
        """Devuelve (roll, pitch) si la lectura es valida, o None. El motivo queda en self.motivo."""

        motivo = ""
        roll = pitch = None

        if self.mpu is not None and not self.reiniciando:

            try:
                roll, pitch = self.mpu.angulos()
            except Exception:
                roll = pitch = None
                motivo = "error"

            if not motivo and (roll is None or pitch is None):
                roll = pitch = None
                motivo = "sin datos"

            elif roll is not None and not (math.isfinite(roll) and math.isfinite(pitch)):
                roll = pitch = None
                motivo = "error"

            if roll is not None:

                if IMU_CONGELADO_S > 0:
                    if self.prev is not None and roll == self.prev[0] and pitch == self.prev[1]:
                        if self.igual_desde is None:
                            self.igual_desde = ahora
                        elif ahora - self.igual_desde > IMU_CONGELADO_S:
                            motivo = "congelado"
                    else:
                        self.igual_desde = None
                    self.prev = (roll, pitch)

                limite = LIMITE_INCLINACION_IMU - (HISTERESIS_IMU_GRADOS if self.fuera else 0.0)

                if abs(roll) > limite or abs(pitch) > limite:
                    self.fuera = True
                    if not motivo:
                        motivo = "fuera de rango"
                else:
                    self.fuera = False

        elif self.mpu is None and self.tuvo_imu:

            motivo = "no disponible"

        if motivo != self.motivo_prev:

            if motivo:
                if ahora - self.t_aviso > 2.0:
                    evento(f"IMU invalido ({motivo}): se ignora y se mantiene la correccion del lazo interno"
                           + ("" if roll is None else f" [roll={roll:.1f} pitch={pitch:.1f}]"))
                    self.t_aviso = ahora
                if self.invalido_desde is None:
                    self.invalido_desde = ahora
            else:
                evento("IMU valido otra vez")
                self.invalido_desde = None

            self.motivo_prev = motivo

        self.motivo = motivo

        # reinicio automatico (sin teclado) si estuvo congelado / con error / no existe
        if (
            motivo in ("congelado", "error", "no disponible", "sin datos") and not self.reiniciando
            and self.invalido_desde is not None
            and ahora - self.invalido_desde > (3.0 if motivo == "sin datos" else IMU_REINICIO_ESPERA_S)
            and ahora - self.t_reintento > IMU_REINTENTO_S and MPU is not None
        ):
            self.t_reintento = ahora
            self.reiniciando = True
            self.reinicios += 1
            evento(f"IMU: reiniciando (intento {self.reinicios}, motivo: {motivo})")
            threading.Thread(target=self._reiniciar, daemon=True).start()

        if motivo:
            return None

        self.ultimo = (roll, pitch)

        return roll, pitch



class CamaraHilo:
    """
    Un hilo pide cuadros sin parar y deja el ULTIMO disponible: el programa principal nunca procesa un cuadro viejo
    ni espera a la camara mientras calcula. Si no llegan cuadros en CAMARA_TIMEOUT_S, se cierra y se vuelve a abrir.
    """

    def __init__(self):

        self.cam = None
        self.frame = None
        self.n = 0
        self.t = time.monotonic()
        self.cond = threading.Condition()
        self.gen = 0
        self.parar = False
        self.reinicios = 0

    def abrir(self):

        self.cam = iniciar_camara()
        self.gen += 1
        self.t = time.monotonic()

        threading.Thread(target=self._run, args=(self.gen, self.cam), daemon=True).start()

    def _run(self, gen, cam):

        errores = 0

        while not self.parar and gen == self.gen:

            try:
                f = cam.capture_array()
                errores = 0
            except Exception:
                errores += 1
                time.sleep(0.05)
                continue

            with self.cond:
                self.frame = f
                self.n += 1
                self.t = time.monotonic()
                self.cond.notify_all()

    def esperar(self, ultimo_n, timeout):
        """Devuelve (frame, n) cuando hay un cuadro NUEVO, o None si pasa el timeout."""

        with self.cond:

            if self.n == ultimo_n:
                self.cond.wait(timeout)

            if self.n == ultimo_n:
                return None

            return self.frame, self.n

    def reiniciar(self):

        self.reinicios += 1

        viejo = self.cam
        self.gen += 1                      # el hilo viejo termina solo cuando salga de capture_array

        def cerrar():
            try:
                viejo.stop()
            except Exception:
                pass
            try:
                viejo.close()
            except Exception:
                pass

        t = threading.Thread(target=cerrar, daemon=True)
        t.start()
        t.join(2.0)

        for intento in range(3):

            try:
                self.abrir()
                evento(f"Camara reiniciada (intento {intento + 1}, reinicio #{self.reinicios})")
                return True
            except Exception as error:
                evento(f"Camara: no se pudo reabrir ({error})")
                time.sleep(1.0)

        return False

    def cerrar(self):

        self.parar = True
        self.gen += 1

        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass

    def stop(self):

        self.parar = True
        self.gen += 1

        try:
            self.cam.stop()
        except Exception:
            pass

    def close(self):

        try:
            self.cam.close()
        except Exception:
            pass



class Salud(threading.Thread):
    """Mide temperatura y bajo voltaje de la Raspberry y lo deja en log_eventos.txt si algo anda mal."""

    def __init__(self):

        super().__init__(daemon=True)

        self.parar = False

    def run(self):

        thr_prev = 0

        while not self.parar:

            time.sleep(SALUD_CADA_S)

            temp = thr = None

            try:
                t = subprocess.run(["vcgencmd", "measure_temp"], capture_output=True, text=True, timeout=1.0).stdout
                temp = float(t.split("=")[1].split("'")[0])
                t = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=1.0).stdout
                thr = int(t.strip().split("=")[1], 16)
            except Exception:
                continue

            if thr is not None and thr != thr_prev:
                avisos = []
                if thr & 0x1: avisos.append("BAJO VOLTAJE AHORA")
                if thr & 0x4: avisos.append("LIMITADO por voltaje/temperatura")
                if thr & 0x10000: avisos.append("hubo bajo voltaje")
                if thr & 0x40000: avisos.append("hubo limitacion")
                evento(f"SALUD Raspberry: 0x{thr:X} -> " + (", ".join(avisos) or "sin problemas") + f" | {temp:.0f} C")
                thr_prev = thr

            if temp is not None and temp >= 80.0:
                evento(f"AVISO: temperatura alta ({temp:.0f} C): la Raspberry puede congelarse (ventilador / disipador)")



# ============================================================
# MINIMAPA X, Y (plano del plato con la pelota y la referencia)
# ============================================================

def dibujar_minimapa(img, pos_est, ref, trayectoria):

    tam = 150
    margen = 10

    x0 = img.shape[1] - tam - margen
    y0 = img.shape[0] - tam - margen

    cx = x0 + tam // 2
    cy = y0 + tam // 2

    escala = tam / max(PLATE_X_CM, PLATE_Y_CM)

    def a_px(x, y):
        cu = x * EJE_X_EN_IMAGEN[0] + y * EJE_Y_EN_IMAGEN[0]
        cv_ = x * EJE_X_EN_IMAGEN[1] + y * EJE_Y_EN_IMAGEN[1]
        return (int(cx + cu * escala), int(cy + cv_ * escala))

    cv2.rectangle(img, (x0, y0), (x0 + tam, y0 + tam), (255, 255, 255), 1)
    cv2.line(img, (x0, cy), (x0 + tam, cy), (90, 90, 90), 1)
    cv2.line(img, (cx, y0), (cx, y0 + tam), (90, 90, 90), 1)

    cv2.putText(img, "X+", a_px(PLATE_X_CM / 2 - 3.0, 0.0), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.putText(img, "Y+", a_px(0.0, PLATE_Y_CM / 2 - 3.0), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    if len(trayectoria) > 1:

        puntos = np.array([a_px(x, y) for x, y in trayectoria], dtype=np.int32)

        cv2.polylines(img, [puntos], False, (0, 200, 255), 1)

    cv2.drawMarker(img, a_px(ref[0], ref[1]), (255, 255, 0), cv2.MARKER_CROSS, 10, 1)

    if pos_est is not None:

        cv2.circle(img, a_px(pos_est[0], pos_est[1]), 4, (0, 0, 255), -1)

    cv2.putText(
        img, "plano X,Y (cm)", (x0, y0 - 4),
        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1
    )


# ============================================================
# REFERENCIA INTERACTIVA
# ============================================================

def limitar_referencia(ref):

    lx = PLATE_X_CM / 2 - 2.0
    ly = PLATE_Y_CM / 2 - 2.0

    ref[0] = float(max(-lx, min(lx, ref[0])))
    ref[1] = float(max(-ly, min(ly, ref[1])))


def pedir_referencia(ref):

    lx = PLATE_X_CM / 2 - 2.0
    ly = PLATE_Y_CM / 2 - 2.0

    print("======================================")
    print("REFERENCIA: donde quieres que se balancee la pelota")
    print("(centro del plato = 0, 0; Enter = valor por defecto)")
    print("======================================")

    try:

        texto = input(f"  X en cm (de -{lx:.1f} a {lx:.1f}) [{ref[0]}]: ").strip()

        if texto:
            ref[0] = float(texto)

        texto = input(f"  Y en cm (de -{ly:.1f} a {ly:.1f}) [{ref[1]}]: ").strip()

        if texto:
            ref[1] = float(texto)

    except (ValueError, EOFError):

        print("  Valor no valido: se usa la referencia por defecto.")

    limitar_referencia(ref)

    print(f"  Referencia: X = {ref[0]:.1f} cm   Y = {ref[1]:.1f} cm\n")


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

def crear_config(camara, con_enfoque):

    controles = {"FrameDurationLimits": (int(1e6 / FPS_OBJETIVO), int(1e6 / FPS_OBJETIVO))}

    if con_enfoque:
        controles["AfMode"] = 0                              # enfoque manual
        controles["LensPosition"] = 1.0 / ENFOQUE_DIST_M     # dioptrias

    try:

        return camara.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            controls=controles,
            queue=False,
            buffer_count=3
        )

    except TypeError:

        return camara.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            controls=controles
        )


def iniciar_camara():

    con_enfoque = ENFOQUE_DIST_M is not None

    camara = Picamera2()

    try:

        camara.configure(crear_config(camara, con_enfoque))
        camara.start()

        if con_enfoque:
            print(f"Enfoque manual a {ENFOQUE_DIST_M} m")

    except Exception as error:

        if not con_enfoque:
            raise

        print(f"Enfoque manual no disponible en esta camara ({error}); se usa el enfoque normal.")

        try:
            camara.close()
        except Exception:
            pass

        camara = Picamera2()
        camara.configure(crear_config(camara, False))
        camara.start()

    return camara


def main():

    enlace = None

    mpu = None
    picam2 = None
    archivo_log = None
    codigo_salida = 0
    hubo_error = False
    gestor = GestorIMU()

    ref = [REFERENCIA_X_CM, REFERENCIA_Y_CM]

    compartido = {"matriz": None, "modo_esquinas": False, "clics": [], "nuevas": None}

    b = K_PELOTA_OVERRIDE if K_PELOTA_OVERRIDE else constante_pelota(TIPO_PELOTA)

    wn = ANCHO_BANDA
    q_vel = AMORT_EXTRA
    ki = KI
    zona_m = ZONA_MUERTA_CM

    k1, k2 = lqr_ganancias(b, wn, q_vel)

    try:

        # ---------------- Referencia ----------------

        limitar_referencia(ref)

        if PEDIR_REFERENCIA_AL_INICIO and REINICIOS[0] == 0:
            pedir_referencia(ref)

        print(f"Modelo de la pelota ({TIPO_PELOTA}): {b:.1f} cm/s2 por grado de plato")

        # ---------------- IMU (plato quieto) ----------------

        imu_calibrada = False

        if USAR_IMU:

            try:

                print("Iniciando IMU: no muevas el plato...")

                mpu = MPU(addr=DIRECCION_IMU, hz=200, ventana_mediana=3)
                mpu.iniciar()

                imu_calibrada = cargar_cero_imu(mpu)

                time.sleep(0.5)

            except Exception as error:

                print(f"IMU no disponible: {error}")
                mpu = None

        # ---------------- Servos al centro ----------------

        iniciar_pca()

        print(f"Llevando servos al centro (X {CENTRO_X}, Y {CENTRO_Y})...")

        ir_suave(CENTRO_X, CENTRO_Y)

        time.sleep(1.5)

        gestor = GestorIMU()
        gestor.mpu = mpu
        gestor.calibrada = imu_calibrada
        gestor.tuvo_imu = mpu is not None

        if mpu is not None and not imu_calibrada and AUTO_CALIBRAR_SI_FALTA:

            evento("No hay calibracion del IMU -> se guarda el cero solo (plato plano en el centro)")

            time.sleep(AUTO_ESPERA_IDENT_S)

            rz, pz = guardar_cero_imu(mpu)

            imu_calibrada = True
            gestor.calibrada = True

            evento(f"Cero del IMU guardado (roll={rz:.2f}, pitch={pz:.2f})")

        inicio_ok, mensaje = verificar_inicio(mpu, imu_calibrada)

        print(mensaje)

        if not inicio_ok:
            print("AVISO: " + mensaje + "  (el control se puede activar igual)")
            inicio_ok = True

        ident = cargar_identificacion() if mpu is not None else None

        if ident is not None:
            print("Identificacion de servos cargada:")
            mostrar_identificacion(ident)
        elif mpu is not None:

            if AUTO_IDENTIFICAR_SI_FALTA:

                evento("No hay identificacion de servos -> se identifica sola (el plato debe estar SIN pelota)")

                time.sleep(AUTO_ESPERA_IDENT_S)

                ident = identificar_todos(mpu)

                ir_suave(CENTRO_X, CENTRO_Y)

                if ident is not None:
                    evento("Identificacion lista y guardada")
                    mostrar_identificacion(ident)
                else:
                    evento("No se pudo identificar (IMU poco claro): se opera SOLO con la camara y G = " + str(G_DEFECTO))

            else:
                print("Servos sin identificar: se opera solo con la camara (tecla G para identificar).")

        # ---------------- Camara ----------------

        picam2 = CamaraHilo()
        picam2.abrir()

        salud = Salud()
        salud.start()

        # ---------------- Ventana principal ----------------

        cv2.namedWindow(VENTANA)

        cv2.createTrackbar("Blanco S max", VENTANA, 60, 255, nada)
        cv2.createTrackbar("Blanco V min", VENTANA, 170, 255, nada)

        cv2.createTrackbar("Pelota H min", VENTANA, PELOTA_H_MIN, 179, nada)
        cv2.createTrackbar("Pelota H max", VENTANA, PELOTA_H_MAX, 179, nada)
        cv2.createTrackbar("Pelota S min", VENTANA, PELOTA_S_MIN, 255, nada)
        cv2.createTrackbar("Pelota V min", VENTANA, PELOTA_V_MIN, 255, nada)

        def clic(evento, px, py, flags, param):

            if evento != cv2.EVENT_LBUTTONDOWN:
                return

            # Modo "marcar plato": los 4 clics son las esquinas del plato
            if compartido["modo_esquinas"]:

                compartido["clics"].append((px, py))

                if len(compartido["clics"]) == 4:
                    compartido["nuevas"] = ordenar_puntos(compartido["clics"])
                    compartido["modo_esquinas"] = False

                return

            # Modo normal: el clic fija la referencia de la pelota
            if compartido["matriz"] is not None:

                ref[0], ref[1] = pixel_a_ejes(compartido["matriz"], px, py)

                limitar_referencia(ref)

        cv2.setMouseCallback(VENTANA, clic)

        # ---------------- Log ----------------

        archivo_log = open(ARCHIVO_LOG, "a" if REINICIOS[0] else "w", newline="")

        escritor = csv.writer(archivo_log)

        if REINICIOS[0] == 0:
          escritor.writerow([
            "t", "x_med_cm", "y_med_cm", "x_est_cm", "y_est_cm", "vx_cm_s", "vy_cm_s",
            "ref_x_cm", "ref_y_cm", "theta_des_x", "theta_des_y",
            "theta_meas_x", "theta_meas_y", "servo_x_deg", "servo_y_deg",
            "pwm_x_us", "pwm_y_us", "duty_x_pct", "duty_y_pct", "fps", "mov_dps"
        ])

        # ---------------- Estado ----------------

        ancho_img, alto_img = dims_imagen()


        puntos_dst = np.array(

            [[0, 0], [ancho_img, 0], [ancho_img, alto_img], [0, alto_img]],

            dtype=np.float32

        )

        seguidor = SeguidorPlato()
        detector = DetectorPelota()

        puntos_fijos = None
        origen_plato = ""
        consecutivos = 0
        ultimas_esq = deque(maxlen=AUTOFIJAR_CUADROS)
        t_inicio = time.perf_counter()

        if PLATO_FIJO and USAR_PLATO_GUARDADO:

            puntos_fijos = cargar_plato()

            if puntos_fijos is not None:
                origen_plato = "archivo"
                print(f"Plato fijado con las esquinas guardadas en {ARCHIVO_PLATO} (B = marcarlas de nuevo).")

        kalman = {
            "x": KalmanEje(b, KALMAN_SIGMA_MED_CM, KALMAN_SIGMA_ACC, KALMAN_GATING_CM,
                           KALMAN_BOOST_FACTOR, KALMAN_BOOST_UMBRAL_CM, KALMAN_BOOST_TIEMPO_S),
            "y": KalmanEje(b, KALMAN_SIGMA_MED_CM, KALMAN_SIGMA_ACC, KALMAN_GATING_CM,
                           KALMAN_BOOST_FACTOR, KALMAN_BOOST_UMBRAL_CM, KALMAN_BOOST_TIEMPO_S)
        }

        controlador = {"x": ControlEje(), "y": ControlEje()}
        lazo = {"x": LazoInterno(), "y": LazoInterno()}

        theta_cmd = {"x": 0.0, "y": 0.0}
        dir_servo = {"x": 0, "y": 0}

        lazo_activo = USAR_LAZO_INTERNO
        imu_valido_prev = False
        hist_trim = deque(maxlen=400)
        ultimo_n = 0
        t_reinicio_cam = 0.0
        t_vista0 = None
        auto_pausa = False

        ver_mascara = False

        control_activo = False
        t_ultima_pelota = -1e9
        t_control0 = 0.0
        contador_log = 0
        contador_cuadros = 0

        fps = 0.0
        proc_ms = 0.0
        mov_dps = 0.0
        ang_prev = dict(ang)

        trayectoria = deque(maxlen=60)

        t_ant = time.perf_counter()

        enlace = EnlaceMyRIO()

        print("\nListo. Teclas en la cabecera del archivo. Pulsa C (o la linea ON del myRIO) para activar el control.")

        # ====================================================
        # BUCLE
        # ====================================================

        while True:

            # ---------------- captura y tiempo ----------------

            res = picam2.esperar(ultimo_n, 0.25)

            if res is None:

                # sin cuadros nuevos: se sigue atendiendo la ventana y, si pasa el timeout, se reinicia la camara
                cv2.waitKey(1)

                ahora_m = time.monotonic()

                if ahora_m - picam2.t > CAMARA_TIMEOUT_S and ahora_m - t_reinicio_cam > CAMARA_TIMEOUT_S + 3.0:
                    evento(f"CAMARA: sin cuadros hace {ahora_m - picam2.t:.1f} s -> se reinicia")
                    t_reinicio_cam = ahora_m
                    picam2.reiniciar()
                    t_ant = time.perf_counter()

                continue

            frame, ultimo_n = res

            ahora = time.perf_counter()

            dt = ahora - t_ant
            t_ant = ahora

            if dt <= 0:
                dt = 1.0 / FPS_OBJETIVO

            dt = min(dt, 0.1)

            fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else 1.0 / dt

            contador_cuadros += 1

            t_proc0 = time.perf_counter()

            # ---------------- plato ----------------

            if compartido["nuevas"] is not None:

                puntos_fijos = compartido["nuevas"].astype(np.float32)
                compartido["nuevas"] = None
                origen_plato = "manual"
                guardar_plato(puntos_fijos)
                print(f"Plato fijado con tus 4 esquinas y guardado en {ARCHIVO_PLATO}.")

            estado_plato = f"PLATO FIJADO ({origen_plato})"
            color_plato = (0, 255, 0)

            if puntos_fijos is not None:

                puntos_src = puntos_fijos

            else:

                blur_full = cv2.GaussianBlur(frame, (7, 7), 0)

                hsv_full = cv2.cvtColor(blur_full, cv2.COLOR_RGB2HSV)

                w_s_max = leer_barra("Blanco S max", BLANCO_S_MAX)
                w_v_min = leer_barra("Blanco V min", BLANCO_V_MIN)

                mask_white = cv2.inRange(
                    hsv_full,
                    np.array([0, 0, w_v_min]),
                    np.array([180, w_s_max, 255])
                )

                mask_white = limpiar_mascara(mask_white)

                puntos_src, estado_plato, color_plato = seguidor.actualizar(mask_white)

                if puntos_src is not None and seguidor.perdido == 0:
                    consecutivos += 1
                    ultimas_esq.append(puntos_src.copy())
                else:
                    consecutivos = 0
                    ultimas_esq.clear()

                estable = False

                if len(ultimas_esq) >= AUTOFIJAR_CUADROS:
                    arr = np.array(ultimas_esq)
                    estable = float(np.max(arr.max(axis=0) - arr.min(axis=0))) < ESTABILIDAD_PX

                if (
                    PLATO_FIJO
                    and consecutivos >= AUTOFIJAR_CUADROS
                    and estable
                    and estado_plato == "PLATO DETECTADO"
                    and (ahora - t_inicio) > WARMUP_CAMARA_S
                    and not control_activo
                ):
                    puntos_fijos = np.median(np.array(ultimas_esq), axis=0).astype(np.float32)
                    origen_plato = "auto"
                    print("Plato fijado (esquinas estables). Si no encaja, pulsa B y marca las 4 esquinas.")

            matriz = None

            if puntos_src is not None:
                matriz = cv2.getPerspectiveTransform(puntos_src, puntos_dst)

            compartido["matriz"] = matriz

            # ---------------- pelota (solo en el ROI) ----------------

            roi = calcular_roi(puntos_src, frame.shape)

            if puntos_src is not None:
                centro_pelota, contorno_pelota, mascara_pelota = detector.buscar(frame, roi, puntos_src)
            else:
                centro_pelota = contorno_pelota = mascara_pelota = None

            medida = None

            if centro_pelota is not None and matriz is not None:

                medida = pixel_a_ejes(matriz, centro_pelota[0], centro_pelota[1])

                t_ultima_pelota = ahora

            if REINICIOS[0] > 0 and not control_activo and not auto_pausa and medida is not None and puntos_fijos is not None:

                if t_vista0 is None:
                    t_vista0 = ahora

                if ahora - t_vista0 >= 0.6:

                    control_activo = True

                    for eje in ("x", "y"):
                        controlador[eje].reiniciar()
                        lazo[eje].reiniciar()

                    lazo_activo = USAR_LAZO_INTERNO
                    t_control0 = time.perf_counter()
                    t_ant = time.perf_counter()
                    t_vista0 = None

                    evento("Control ACTIVADO (automatico, tras el reinicio por error)")

            elif medida is None:
                t_vista0 = None

            # ---------------- IMU ----------------

            roll = pitch = None

            theta_meas = None

            if USAR_IMU and (gestor.mpu is not None or gestor.tuvo_imu):

                lectura = gestor.leer(time.monotonic())

                mpu = gestor.mpu

                if lectura is not None:

                    roll, pitch = lectura

                    if ident is not None:

                        try:
                            theta_meas = {
                                "x": theta_bola(ident, "x", roll, pitch),
                                "y": theta_bola(ident, "y", roll, pitch)
                            }
                        except Exception:
                            theta_meas = None
                            roll = pitch = None

                imu_valido = theta_meas is not None or (lectura is not None and ident is None)

                # IMU congelado: lo que el lazo "aprendio" mientras estaba congelado es falso -> se deshace
                if imu_valido_prev and not imu_valido and gestor.motivo == "congelado":

                    limite_t = time.monotonic() - IMU_CONGELADO_S - 0.15

                    for t_h, tx, ty in hist_trim:
                        if t_h <= limite_t:
                            lazo["x"].trim, lazo["y"].trim = tx, ty
                        else:
                            break

                    evento("IMU congelado: se deshace la correccion de los ultimos instantes y se sigue con la camara")

                imu_valido_prev = imu_valido

            hist_trim.append((time.monotonic(), lazo["x"].trim, lazo["y"].trim))

            # ---------------- correccion de paralaje (plato fijo e inclinado) ----------------

            if (
                medida is not None
                and DISTANCIA_CAMARA_CM is not None
                and puntos_fijos is not None
                and theta_meas is not None
            ):

                medida = corregir_paralaje(
                    medida[0], medida[1], theta_meas["x"], theta_meas["y"], DISTANCIA_CAMARA_CM
                )

            # ---------------- Kalman (siempre corriendo) ----------------

            perdida_s = ahora - t_ultima_pelota

            for eje, idx in (("x", 0), ("y", 1)):

                kf = kalman[eje]

                theta_in = theta_meas[eje] if theta_meas is not None else theta_cmd[eje]

                if kf.iniciado:

                    kf.predecir(dt, theta_in)

                    if medida is not None:
                        kf.actualizar(medida[idx])

                    if perdida_s > TIEMPO_PELOTA_PERDIDA_S:
                        kf.reiniciar()

                elif medida is not None:

                    kf.iniciar(medida[idx])

            estimado_ok = kalman["x"].iniciado and kalman["y"].iniciado

            pos_est = None
            vel_est = (0.0, 0.0)

            if estimado_ok:

                pos_est = (kalman["x"].x[0], kalman["y"].x[0])
                vel_est = (kalman["x"].x[1], kalman["y"].x[1])

            # posicion al myRIO para graficarla (20 Hz)
            enlace.enviar(pos_est, control_activo)

            # ---------------- control ----------------

            theta_des = {"x": 0.0, "y": 0.0}

            if control_activo:

                d_obj = {"x": 0.0, "y": 0.0}

                usar_lazo = lazo_activo and theta_meas is not None and ident is not None

                for eje, idx in (("x", 0), ("y", 1)):

                    G = ident[eje]["G"] if ident is not None else G_DEFECTO

                    if estimado_ok:

                        theta_in = theta_meas[eje] if theta_meas is not None else theta_cmd[eje]

                        pos_p, vel_p = kalman[eje].adelantar(RETRASO_CAMARA_S, theta_in)

                        theta_des[eje] = controlador[eje].calcular(
                            pos_p, vel_p, ref[idx], dt, k1, k2, ki,
                            ZONA_INTEGRAL_CM, I_MAX_GRADOS, THETA_MAX_GRADOS,
                            zona_m, VEL_MUERTA_CM_S, VEL_MAX_CM_S, INTEGRAL_CONGELAR
                        )

                        d = SIGNO[eje] * theta_des[eje] / G

                        if usar_lazo:

                            d += lazo[eje].actualizar(
                                theta_des[eje], theta_meas[eje], dt,
                                SIGNO[eje], G, K_TRIM, TRIM_MAX_GRADOS
                            )

                            if lazo[eje].saturado_s > 0.7:

                                lazo_activo = False

                                print(
                                    f"Lazo interno desactivado: el servo {eje.upper()} no logra la "
                                    "inclinacion pedida (revisa la identificacion con G)."
                                )

                        d_obj[eje] = d

                    else:

                        # Sin estimacion de la pelota: volver al nivel
                        controlador[eje].reiniciar()
                        lazo[eje].reiniciar()

                    theta_cmd[eje] = theta_des[eje]

                # Velocidad maxima del servo: suave cerca de la referencia, rapida lejos
                if estimado_ok:
                    err_max = max(abs(pos_est[0] - ref[0]), abs(pos_est[1] - ref[1]))
                else:
                    err_max = 1e9

                vmax_servo = limite_velocidad_servo(
                    err_max, MAX_VELOCIDAD_SERVO_DPS, VEL_SERVO_CERCA_DPS, ERROR_CERCA_CM, ERROR_LEJOS_CM
                )

                # Aplicar a los servos con limite de velocidad y limites duros
                for nombre in ("x", "y"):

                    meta0 = CENTRO[nombre] + d_obj[nombre]

                    if not usar_lazo:

                        if meta0 > ang[nombre] + 0.05:
                            dir_servo[nombre] = 1
                        elif meta0 < ang[nombre] - 0.05:
                            dir_servo[nombre] = -1

                        meta0 += dir_servo[nombre] * HOLGURA_SERVO_GRADOS / 2.0

                    meta = limitar(nombre, meta0)

                    maximo = vmax_servo * dt

                    delta = limitar_valor(meta - ang[nombre], -maximo, maximo)

                    # No se manda al servo un cambio minimo (menos micro-movimientos y consumo)
                    if abs(delta) < CMD_MIN_CAMBIO_GRADOS:
                        continue

                    enviar(nombre, ang[nombre] + delta)

            mov_inst = (abs(ang["x"] - ang_prev["x"]) + abs(ang["y"] - ang_prev["y"])) / dt
            mov_dps = 0.98 * mov_dps + 0.02 * mov_inst
            ang_prev = {"x": ang["x"], "y": ang["y"]}

            # ---------------- registro ----------------

            if (control_activo or REGISTRAR_SIEMPRE) and ang["x"] is not None:

                pwm_x = angulo_a_pulso(ang["x"])
                pwm_y = angulo_a_pulso(ang["y"])

                periodo_us = 1000000.0 / FRECUENCIA_HZ

                escritor.writerow([
                    f"{ahora - t_control0:.4f}",
                    "" if medida is None else f"{medida[0]:.3f}",
                    "" if medida is None else f"{medida[1]:.3f}",
                    "" if pos_est is None else f"{pos_est[0]:.3f}",
                    "" if pos_est is None else f"{pos_est[1]:.3f}",
                    f"{vel_est[0]:.3f}", f"{vel_est[1]:.3f}",
                    f"{ref[0]:.2f}", f"{ref[1]:.2f}",
                    f"{theta_des['x']:.3f}", f"{theta_des['y']:.3f}",
                    "" if theta_meas is None else f"{theta_meas['x']:.3f}",
                    "" if theta_meas is None else f"{theta_meas['y']:.3f}",
                    f"{ang['x']:.2f}", f"{ang['y']:.2f}",
                    f"{pwm_x:.1f}", f"{pwm_y:.1f}",
                    f"{pwm_x / periodo_us * 100:.3f}", f"{pwm_y / periodo_us * 100:.3f}",
                    f"{fps:.1f}", f"{mov_dps:.1f}"
                ])

                contador_log += 1

                if contador_log % 60 == 0:
                    archivo_log.flush()

            if pos_est is not None:
                trayectoria.append(pos_est)

            proc_ms = 0.9 * proc_ms + 0.1 * (time.perf_counter() - t_proc0) * 1000.0

            # ---------------- ventana (cada N cuadros) ----------------

            if contador_cuadros % MOSTRAR_CADA == 0:

                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                if puntos_src is not None:

                    cv2.polylines(
                        frame_bgr,
                        [puntos_src.astype(int)],
                        True,
                        color_plato,
                        2
                    )

                if contorno_pelota is not None:

                    cv2.drawContours(frame_bgr, [contorno_pelota], -1, (0, 0, 255), 2)

                    cv2.circle(
                        frame_bgr,
                        (int(centro_pelota[0]), int(centro_pelota[1])),
                        6,
                        (255, 0, 0),
                        -1
                    )

                if matriz is not None:

                    try:

                        inversa = np.linalg.inv(matriz)

                        # Centro del plato (0, 0): circulo verde
                        px_c = ejes_a_pixel(inversa, 0.0, 0.0)
                        cc = (int(px_c[0]), int(px_c[1]))

                        cv2.circle(frame_bgr, cc, 8, (0, 255, 0), 2)
                        cv2.drawMarker(frame_bgr, cc, (0, 255, 0), cv2.MARKER_TILTED_CROSS, 14, 1)
                        cv2.putText(
                            frame_bgr, "CENTRO", (cc[0] + 10, cc[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1
                        )

                        # Referencia (objetivo de la pelota): cruz amarilla
                        px_ref = ejes_a_pixel(inversa, ref[0], ref[1])

                        cv2.drawMarker(
                            frame_bgr,
                            (int(px_ref[0]), int(px_ref[1])),
                            (255, 255, 0),
                            cv2.MARKER_CROSS,
                            18,
                            2
                        )

                    except np.linalg.LinAlgError:
                        pass

                if compartido["modo_esquinas"]:

                    for i, (cx_, cy_) in enumerate(compartido["clics"]):

                        cv2.circle(frame_bgr, (int(cx_), int(cy_)), 6, (0, 255, 255), -1)
                        cv2.putText(
                            frame_bgr, str(i + 1), (int(cx_) + 8, int(cy_) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1
                        )

                    cv2.putText(
                        frame_bgr,
                        f"MARCA LAS 4 ESQUINAS DEL PLATO ({len(compartido['clics'])}/4)   B = cancelar",
                        (10, frame_bgr.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2
                    )

                blanco = (255, 255, 255)
                naranja = (0, 165, 255)
                rojo = (0, 0, 255)
                verde = (0, 255, 0)

                lineas = [(estado_plato, color_plato)]

                lineas.append((
                    f"CONTROL: {'ON' if control_activo else 'OFF'} | {fps:.0f} fps | proc {proc_ms:.0f} ms | "
                    f"lazo interno: {'ON' if (lazo_activo and ident is not None) else 'OFF'}",
                    verde if control_activo else naranja
                ))

                if medida is not None:
                    lineas.append((
                        f"Pelota X={medida[0]:.1f} Y={medida[1]:.1f} cm | "
                        f"Ref X={ref[0]:.1f} Y={ref[1]:.1f}",
                        blanco
                    ))
                elif matriz is None:
                    lineas.append(("Sin plato: no se puede medir la pelota", rojo))
                else:
                    lineas.append(("PELOTA NO DETECTADA", rojo))

                if pos_est is not None:
                    lineas.append((
                        f"Estim. X={pos_est[0]:.1f} Y={pos_est[1]:.1f} cm | "
                        f"V=({vel_est[0]:.0f}, {vel_est[1]:.0f}) cm/s",
                        blanco
                    ))

                lineas.append((
                    f"Servo X={ang['x']:.1f} Y={ang['y']:.1f} | "
                    f"PWM {angulo_a_pulso(ang['x']):.0f}/{angulo_a_pulso(ang['y']):.0f} us",
                    blanco
                ))

                lineas.append((
                    f"LQR wn={wn:.1f} k1={k1:.2f} k2={k2:.2f} Ki={ki:.2f}",
                    blanco
                ))

                lineas.append((
                    f"Zona muerta={zona_m:.1f} cm | vmax={VEL_MAX_CM_S} cm/s | mov={mov_dps:.0f} deg/s"
                    + (" | paralaje ON" if DISTANCIA_CAMARA_CM is not None else ""),
                    blanco
                ))

                if roll is not None:
                    lineas.append((f"IMU roll={roll:.1f} pitch={pitch:.1f}", blanco))
                else:
                    lineas.append((f"IMU: {gestor.motivo or 'no disponible'}", naranja))

                if not inicio_ok:
                    lineas.append(("REVISAR: IMU sin cero (plato plano y pulsa Z)", rojo))

                if mpu is not None and ident is None:
                    lineas.append(("Servos sin identificar: solo camara (G sin pelota para identificar)", naranja))

                for i, (texto, color) in enumerate(lineas):

                    cv2.putText(
                        frame_bgr,
                        texto,
                        (10, 20 + 20 * i),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        1
                    )

                dibujar_minimapa(frame_bgr, pos_est, ref, trayectoria)

                cv2.imshow(VENTANA, frame_bgr)

                if ver_mascara and mascara_pelota is not None:
                    cv2.imshow("Mascara pelota", mascara_pelota)

            # ---------------- teclas ----------------

            tecla = cv2.waitKey(1) & 0xFF

            # La linea digital del myRIO actua como la tecla C (solo en los cambios de nivel)
            pedido = enlace.pedido()
            if tecla == 255 and pedido is not None and (pedido == "on") != control_activo:
                tecla = ord("c")

            if tecla == 255:
                continue

            if tecla == ord("q"):
                break

            elif tecla == ord("c"):

                if not control_activo:

                    if not inicio_ok:
                        print("No se activa el control: " + mensaje)
                    else:

                        if mpu is not None and ident is None:
                            print(
                                "AVISO: servos sin identificar; se usa G = "
                                f"{G_DEFECTO} y sin lazo interno (pulsa G sin pelota para identificar)."
                            )

                        control_activo = True

                        for eje in ("x", "y"):
                            controlador[eje].reiniciar()
                            lazo[eje].reiniciar()

                        lazo_activo = USAR_LAZO_INTERNO
                
                        t_control0 = time.perf_counter()
                        t_ant = time.perf_counter()

                        print("Control ACTIVADO")

                else:

                    control_activo = False
                    auto_pausa = True
                    print("Control DESACTIVADO: volviendo al centro")
                    ir_suave(CENTRO_X, CENTRO_Y)
                    t_ant = time.perf_counter()

            elif tecla == ord("z"):

                if mpu is None:
                    print("No hay IMU.")
                elif control_activo:
                    print("Apaga el control antes de guardar el cero del IMU.")
                else:
                    ir_suave(CENTRO_X, CENTRO_Y)
                    time.sleep(1.0)
                    rz, pz = guardar_cero_imu(mpu)
                    inicio_ok = True
                    print(f"Cero del IMU guardado: roll={rz:.2f}  pitch={pz:.2f}")
                    t_ant = time.perf_counter()

            elif tecla == ord("g"):

                if mpu is None:
                    print("No hay IMU: no se puede identificar.")
                elif control_activo:
                    print("Apaga el control antes de identificar.")
                else:
                    print("Identificando servos: QUITA LA PELOTA del plato. Empieza en 4 s...")
                    time.sleep(4.0)
                    resultado = identificar_todos(mpu)
                    if resultado is not None:
                        ident = resultado
                        print("Identificacion lista:")
                        mostrar_identificacion(ident)
                    else:
                        print("Identificacion fallida: revisa que el IMU este firme en el plato y repite.")
                    t_ant = time.perf_counter()

            elif tecla == ord("u"):

                if control_activo:
                    print("Apaga el control antes de volver a fijar el plato.")
                else:
                    ir_suave(CENTRO_X, CENTRO_Y)
                    puntos_fijos = None
                    seguidor.reiniciar()
                    consecutivos = 0
                    ultimas_esq.clear()
                    print("Re-detectando el plato; se fijara solo en un momento.")
                    t_ant = time.perf_counter()

            elif tecla == ord("f"):

                lazo_activo = not lazo_activo
                lazo["x"].reiniciar()
                lazo["y"].reiniciar()
                print(f"Lazo interno: {'ON' if lazo_activo else 'OFF'}")

            elif tecla == ord("n"):

                if not control_activo:
                    ir_suave(CENTRO_X, CENTRO_Y)
                    t_ant = time.perf_counter()

            elif tecla == ord("r"):

                ref[0] = REFERENCIA_X_CM
                ref[1] = REFERENCIA_Y_CM
                limitar_referencia(ref)

            elif tecla in (ord("w"), ord("a"), ord("s"), ord("d")):

                if tecla == ord("w"):
                    ref[1] += 1.0
                elif tecla == ord("s"):
                    ref[1] -= 1.0
                elif tecla == ord("a"):
                    ref[0] -= 1.0
                elif tecla == ord("d"):
                    ref[0] += 1.0

                limitar_referencia(ref)

                print(f"Referencia: X={ref[0]:.1f}  Y={ref[1]:.1f} cm")

            elif tecla in (ord("1"), ord("2"), ord("3"), ord("4"), ord("5"), ord("6"), ord("7"), ord("8")):

                if tecla == ord("1"):
                    wn = max(1.0, wn - 0.5)
                elif tecla == ord("2"):
                    wn += 0.5
                elif tecla == ord("3"):
                    q_vel = max(0.0, q_vel - 0.05)
                elif tecla == ord("4"):
                    q_vel += 0.05
                elif tecla == ord("5"):
                    ki = max(0.0, ki - 0.05)
                elif tecla == ord("6"):
                    ki += 0.05
                elif tecla == ord("7"):
                    zona_m = max(0.0, zona_m - 0.1)
                elif tecla == ord("8"):
                    zona_m += 0.1

                k1, k2 = lqr_ganancias(b, wn, q_vel)

            elif tecla in (ord("j"), ord("l"), ord("i"), ord("k")):

                if control_activo:
                    print("Apaga el control para mover los servos a mano.")
                else:
                    if tecla == ord("j"):
                        enviar("x", ang["x"] - 1)
                    elif tecla == ord("l"):
                        enviar("x", ang["x"] + 1)
                    elif tecla == ord("i"):
                        enviar("y", ang["y"] + 1)
                    elif tecla == ord("k"):
                        enviar("y", ang["y"] - 1)

                    print(f"Servo X={ang['x']:.1f}  Y={ang['y']:.1f}")

            elif tecla == ord("m"):

                ver_mascara = not ver_mascara

                if not ver_mascara:
                    cv2.destroyWindow("Mascara pelota")

                print(f"Mascara de la pelota: {'visible' if ver_mascara else 'oculta'}")

            elif tecla == ord("b"):

                if control_activo:
                    print("Apaga el control antes de marcar el plato.")
                elif compartido["modo_esquinas"]:
                    compartido["modo_esquinas"] = False
                    compartido["clics"] = []
                    print("Marcado de esquinas cancelado.")
                else:
                    ir_suave(CENTRO_X, CENTRO_Y)
                    compartido["modo_esquinas"] = True
                    compartido["clics"] = []
                    print("MARCAR PLATO: haz clic en las 4 esquinas del plato (en cualquier orden).")
                    t_ant = time.perf_counter()

            elif tecla == ord("e"):

                archivo_log.flush()
                print("Exportando resultados (puede tardar unos segundos)...")
                exportar_resultados(ARCHIVO_LOG, plate_x_cm=PLATE_X_CM, plate_y_cm=PLATE_Y_CM)
                t_ant = time.perf_counter()

            elif tecla == ord("p"):

                print(
                    f"wn={wn:.2f} q_vel={q_vel:.2f} k1={k1:.3f} k2={k2:.3f} Ki={ki:.3f} | "
                    f"zona={zona_m:.1f} | b={b:.2f} | retraso={RETRASO_CAMARA_S}s | Ref=({ref[0]:.1f}, {ref[1]:.1f}) | "
                    f"Pelota H={cv2.getTrackbarPos('Pelota H min', VENTANA)}-"
                    f"{cv2.getTrackbarPos('Pelota H max', VENTANA)} "
                    f"S min={cv2.getTrackbarPos('Pelota S min', VENTANA)} "
                    f"V min={cv2.getTrackbarPos('Pelota V min', VENTANA)}"
                )

    except KeyboardInterrupt:

        print("\nInterrumpido.")

    except Exception:

        import traceback

        evento("ERROR INESPERADO:\n" + traceback.format_exc())
        codigo_salida = 1
        hubo_error = True

    finally:

        try:
            if ang["x"] is not None:
                ir_suave(CENTRO_X, CENTRO_Y)
        except Exception as error:
            print(f"No se pudo llevar al centro: {error}")

        try:
            if gestor.mpu is not None:
                gestor.mpu.detener()
        except Exception:
            pass

        if enlace is not None:
            enlace.cerrar()

        if picam2 is not None:
            picam2.stop()
            picam2.close()

        if archivo_log is not None:
            archivo_log.close()
            if not hubo_error:
                try:
                    exportar_resultados(ARCHIVO_LOG, plate_x_cm=PLATE_X_CM, plate_y_cm=PLATE_Y_CM)
                except Exception as error:
                    print(f"No se pudo exportar: {error}")

        cv2.destroyAllWindows()

        bus_pca.close()

        print("Programa cerrado")

    return codigo_salida


if __name__ == "__main__":

    # Si ocurre un error inesperado (camara, I2C, IMU...) el programa NO se muere: se reinicia solo,
    # sin volver a pedir la referencia, y activa el control en cuanto vuelve a ver la pelota.
    while True:

        codigo = main()

        if codigo == 0:
            break

        REINICIOS[0] += 1

        if REINICIOS[0] > MAX_REINICIOS:
            evento(f"Demasiados reinicios ({MAX_REINICIOS}); se cierra.")
            break

        evento(f"Reiniciando el programa tras el error (reinicio #{REINICIOS[0]})...")

        time.sleep(2.0)
