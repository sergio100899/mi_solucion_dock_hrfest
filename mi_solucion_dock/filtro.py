import math

import numpy as np


def wrap(a):
    """Angulo en (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class Pose2D:
    """Pose (x, y, yaw) de un frame B dentro de otro A: convierte puntos B -> A."""

    def __init__(self, x, y, yaw):
        self.t = np.array([x, y])
        self.yaw = yaw
        c, s = math.cos(yaw), math.sin(yaw)
        self.R = np.array([[c, -s], [s, c]])

    def punto(self, p):  # B -> A
        return self.R @ p + self.t

    def vector(self, v):  # B -> A (solo rota: sirve para normales)
        return self.R @ v

    def punto_inv(self, p):  # A -> B
        return self.R.T @ (p - self.t)

    def vector_inv(self, v):  # A -> B
        return self.R.T @ v


class FiltroDock:
    """Estimacion suavizada del eje del dock en odom: punto p y angulo de la normal."""

    ALPHA = 0.3  # peso de cada deteccion nueva (0 = ignorar, 1 = sin filtro)
    SALTO_P = 0.10  # m: mas lejos que esto de la estimacion es sospechoso
    SALTO_ANG = math.radians(10)
    REPETICIONES = 3  # sospechosas consistentes seguidas para aceptar el cambio
    MIN_DETECCIONES = 3  # para considerar la estimacion confiable

    def __init__(self):
        self.reset()

    def reset(self):
        self.p = None  # punto del eje en odom
        self.ang = None  # angulo de la normal en odom
        self.n_ok = 0  # detecciones aceptadas
        self.t_ultima = None  # tiempo (s) de la ultima deteccion aceptada
        self._cand = []  # detecciones sospechosas seguidas

    @property
    def n(self):
        return np.array([math.cos(self.ang), math.sin(self.ang)])

    def confiable(self):
        return self.p is not None and self.n_ok >= self.MIN_DETECCIONES

    def edad(self, t):
        """Segundos desde la ultima deteccion aceptada (inf si nunca)."""
        return math.inf if self.t_ultima is None else t - self.t_ultima

    def actualizar(self, p, n, t):
        """Mezcla una deteccion (p, n en odom, tiempo t en s). Devuelve si se acepto."""
        ang = math.atan2(n[1], n[0])
        if self.p is None:
            self._aceptar(p, ang, t, reemplazar=True)
            return True

        cerca = (
            np.hypot(*(p - self.p)) < self.SALTO_P
            and abs(wrap(ang - self.ang)) < self.SALTO_ANG
        )
        if cerca:
            self._cand = []
            self._aceptar(p, ang, t)
            return True

        if self._cand and not self._parecidas(self._cand[-1], (p, ang)):
            self._cand = []
        self._cand.append((p, ang))
        if len(self._cand) >= self.REPETICIONES:
            self._aceptar(p, ang, t, reemplazar=True)
            self._cand = []
            return True
        return False

    def _parecidas(self, a, b):
        return (
            np.hypot(*(a[0] - b[0])) < self.SALTO_P
            and abs(wrap(a[1] - b[1])) < self.SALTO_ANG
        )

    def _aceptar(self, p, ang, t, reemplazar=False):
        if reemplazar:
            self.p, self.ang, self.n_ok = np.array(p, float), ang, 1
        else:
            self.p = self.p + self.ALPHA * (p - self.p)
            self.ang = wrap(self.ang + self.ALPHA * wrap(ang - self.ang))
            self.n_ok += 1
        self.t_ultima = t
