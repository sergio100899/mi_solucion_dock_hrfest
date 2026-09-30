import math

import numpy as np

RADIO_ROBOT = 0.339 / 2  # m
SALIENTE_CAJAS = 0.08  # m, cuanto sobresalen las cajas de la pared
D_MIN = RADIO_ROBOT + SALIENTE_CAJAS  # m
D_ACOPLE = 0.34  # m
D_RETIRADA = 0.55  # m

# Perfil de velocidad (trapezoidal en lazo cerrado)
V_MAX = 0.30  # m/s
V_ACOPLE = 0.04  # m/s
A_ACEL = 0.4  # m/s^2
A_FRENO = 0.25  # m/s^2
W_MAX = 1.5  # rad/s
ALFA_MAX = 3.0  # rad/s^2

# Seguimiento del eje
K_W = 2.5  # ganancia de giro: w = K_W * error de rumbo
K_L = 0.35  # distancia de mira sobre el eje
L_MIN, L_MAX = 0.05, 0.6
EMBUDO = 0.35
GIRO_EN_SITIO = math.radians(90)
GIRO_ACOPLE = math.radians(15)

EDAD_MAX = 1.5  # s sin detectar: la estimacion vieja se sigue usando


def ang(v):
    return math.atan2(v[1], v[0])


def factor_avance(err, limite=GIRO_EN_SITIO):
    """Cuanto de la velocidad lineal se permite segun el error de rumbo.

    1 mirando al objetivo, baja suave (cos^2) y llega a 0 en limite:
    asi avanza y gira a la vez, en curva, en lugar de girar y luego avanzar.
    """
    if abs(err) >= limite:
        return 0.0
    return math.cos(err * (math.pi / 2) / limite) ** 2


def errores(p, n):
    """(d, lateral, rumbo): distancia a la pared, desplazamiento del eje y
    angulo que hay que girar para mirar la pared de frente."""
    t = np.array([-n[1], n[0]])
    return float(-p @ n), float(-p @ t), ang(-n)


class Controlador:
    def __init__(self):
        self.estado = "BUSQUEDA"
        self.v = 0.0
        self.w = 0.0

    def paso(self, dt, est, edad, centro, acoplado):
        """Un ciclo de control. Devuelve (v, w) ya limitados.

        est: (p, n) del dock en base_link, o None si no hay estimacion confiable.
        edad: segundos desde la ultima deteccion.
        centro: centro de la sala en base_link, o None.
        acoplado: /dock_status.is_docked.
        """
        v, w = self._decidir(est, edad, centro, acoplado)
        return self._limitar(v, w, dt)

    def _decidir(self, est, edad, centro, acoplado):
        if acoplado:
            self.estado = "ACOPLADO"
            return 0.0, 0.0
        if self.estado == "ACOPLADO":  # se solto: volver a intentar
            self.estado = "RETIRADA"

        hay_dock = est is not None and edad < EDAD_MAX
        if not hay_dock:
            if self.estado != "RETIRADA":
                self.estado = "BUSQUEDA"
            return self._buscar(centro)

        p, n = est
        d, lat, _ = errores(p, n)

        if self.estado == "RETIRADA":
            if d < D_RETIRADA:
                return -0.08, 0.0  # marcha atras recta
            self.estado = "APROXIMACION"

        if self.estado in ("BUSQUEDA", "APROXIMACION", "PREPOSICION"):
            fuera = abs(lat) > EMBUDO * max(d - D_MIN, 0.0) + 0.02
            self.estado = "PREPOSICION" if fuera else "APROXIMACION"
            if not fuera and d < D_ACOPLE:
                self.estado = "ACOPLE"

        if self.estado == "PREPOSICION":
            d_q = max(d, D_MIN + abs(lat) / EMBUDO + 0.1)
            return self._ir_a(p + n * d_q, frenar_en=0.0)

        if self.estado == "ACOPLE" and d <= D_MIN:
            self.estado = "RETIRADA"  # llego sin acoplar: retroceder y reintentar
            return 0.0, 0.0
        if self.estado == "ACOPLE" and abs(lat) > 0.03:
            self.estado = "RETIRADA"  # muy descentrado para acoplar
            return 0.0, 0.0

        return self._seguir_eje(p, n, d, lat)

    def _seguir_eje(self, p, n, d, lat):
        """Apuntar a un punto del eje L metros por delante: converge al eje."""
        L = float(np.clip(K_L * (d - D_MIN), L_MIN, L_MAX))
        meta = p + n * (d - L)
        err = ang(meta)
        w = K_W * err
        falta = max(d - D_ACOPLE, 0.0)
        v = min(V_MAX, math.sqrt(V_ACOPLE**2 + 2 * A_FRENO * falta))
        if self.estado == "ACOPLE":
            peor = max(abs(err), abs(ang(-n)))
            return V_ACOPLE * factor_avance(peor, GIRO_ACOPLE), w
        return max(v, V_ACOPLE) * factor_avance(err), w

    def _ir_a(self, q, frenar_en):
        """Girar hacia q y avanzar hasta el, frenando con el perfil."""
        err = ang(q)
        w = K_W * err
        dist = float(np.hypot(*q))
        v = min(V_MAX, math.sqrt(2 * A_FRENO * max(dist - frenar_en, 0.0)))
        return v * factor_avance(err), w

    def _buscar(self, centro):
        # Si no se ve el marcador, ir hacia el centro de la sala; ya ahi, girar.
        if centro is not None and np.hypot(*centro) > 0.3:
            return self._ir_a(centro, frenar_en=0.0)
        return 0.0, 0.6

    def _limitar(self, v, w, dt):
        """Rampas de aceleracion (flancos del trapecio) y topes."""
        v = float(np.clip(v, -V_MAX, V_MAX))
        w = float(np.clip(w, -W_MAX, W_MAX))
        self.v += float(np.clip(v - self.v, -A_ACEL * dt, A_ACEL * dt))
        self.w += float(np.clip(w - self.w, -ALFA_MAX * dt, ALFA_MAX * dt))
        return self.v, self.w
