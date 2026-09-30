from dataclasses import dataclass

import numpy as np


@dataclass
class Line:
    """Recta n . p = c, con la normal n apuntando hacia el robot."""

    n: np.ndarray  # normal unitaria (hacia el robot)
    c: float
    t: np.ndarray  # direccion unitaria a lo largo de la recta
    s_min: float  # extremos del tramo visto, medidos sobre t
    s_max: float
    inliers: np.ndarray  # indices de los puntos que pertenecen a la recta

    def dist(self, pts):
        """Distancia con signo: positiva = del lado del robot."""
        return pts @ self.n - self.c

    def proj(self, pts):
        """Posicion de cada punto a lo largo de la recta."""
        return pts @ self.t

    def endpoints(self):
        base = self.n * self.c
        return base + self.t * self.s_min, base + self.t * self.s_max


def fit_line(pts):
    """Minimos cuadrados (total) de una recta: devuelve (n, c)."""
    centro = pts.mean(axis=0)
    # La direccion de mayor varianza es la recta; la de menor, su normal
    _, _, vt = np.linalg.svd(pts - centro)
    n = vt[1]
    return n, float(n @ centro)


def ransac_line(pts, rng, iters=150, tol=0.015):
    """Indices de los inliers de la mejor recta encontrada por RANSAC."""
    best = np.array([], dtype=int)
    for _ in range(iters):
        i, j = rng.choice(len(pts), 2, replace=False)
        d = pts[j] - pts[i]
        largo = np.hypot(*d)
        if largo < 0.05:  # dos puntos muy juntos dan una recta mal definida
            continue
        n = np.array([-d[1], d[0]]) / largo
        inl = np.flatnonzero(np.abs((pts - pts[i]) @ n) < tol)
        if len(inl) > len(best):
            best = inl
    if len(best) == 0:
        return best
    # Refinar: ajustar con todos los inliers y recalcularlos con la recta nueva
    n, c = fit_line(pts[best])
    return np.flatnonzero(np.abs(pts @ n - c) < tol)


def find_lines(pts, max_lines=4, min_inliers=25, tol=0.015, seed=0):
    """Encuentra hasta max_lines rectas, de la mas poblada a la menos."""
    rng = np.random.default_rng(seed)
    restantes = np.arange(len(pts))
    lines = []
    while len(lines) < max_lines and len(restantes) >= min_inliers:
        inl = restantes[ransac_line(pts[restantes], rng, tol=tol)]
        if len(inl) < min_inliers:
            break
        n, c = fit_line(pts[inl])
        if c > 0:  # orientar la normal hacia el robot (el origen)
            n, c = -n, -c
        t = np.array([-n[1], n[0]])
        s = pts[inl] @ t
        lines.append(Line(n, c, t, float(s.min()), float(s.max()), inl))
        restantes = np.setdiff1d(restantes, inl)
    return lines


# Medidas del marcador
BANDA_MIN, BANDA_MAX = 0.05, 0.11  # la cara frontal esta a 0.08 de la pared
SALTO_GRUPO = 0.05  # corte minimo entre grupos (el hueco mide 0.095)
ANCHO_CAJA = 0.08
SEP_CENTROS, TOL_SEP = 0.175, 0.015
MIN_PUNTOS_CAJA = 2
PASO_MAX = 0.05  # con rayos mas separados que esto no se distinguen las cajas
INC_LIDAR = 0.0087388  # rad entre muestras (0.5 grados)


@dataclass
class Caja:
    s_lo: float
    s_hi: float
    idx: np.ndarray  # indices de sus puntos en la nube
    h: float  # separacion entre rayos sobre la pared en esta caja

    @property
    def ancho(self):
        return self.s_hi - self.s_lo

    @property
    def centro(self):
        # Punto medio de los bordes
        return (self.s_lo + self.s_hi) / 2

    def ancho_valido(self):
        minimo = max(ANCHO_CAJA - 2 * self.h - 0.005, 0.02)
        return minimo < self.ancho < ANCHO_CAJA + 0.02


@dataclass
class Marcador:
    p: np.ndarray  # eje del dock: centro del hueco, sobre la pared
    n: np.ndarray  # normal de la pared, hacia el robot
    pared: Line
    cajas: tuple
    error: float  # cuanto se aleja de las medidas ideales (menor = mejor)


def paso_rayos(pts, n):
    """Separacion entre rayos consecutivos"""
    r = np.hypot(pts[:, 0], pts[:, 1])
    cos_inc = np.abs(pts @ n) / r
    return r * INC_LIDAR / np.maximum(cos_inc, 0.1)


def agrupar(s, idx, h):
    """Ordena por s y corta donde el salto es mayor que lo esperable entre rayos."""
    orden = np.argsort(s)
    s, idx, h = s[orden], idx[orden], h[orden]
    umbral = np.clip(1.5 * np.maximum(h[:-1], h[1:]), SALTO_GRUPO, 0.09)
    cortes = np.flatnonzero(np.diff(s) > umbral) + 1
    return [
        Caja(float(g_s[0]), float(g_s[-1]), g_i, float(g_h.max()))
        for g_s, g_i, g_h in zip(
            np.split(s, cortes), np.split(idx, cortes), np.split(h, cortes)
        )
    ]


def buscar_cajas(pts, pared):
    """El mejor par de cajas frente a una pared, o None."""
    d = pared.dist(pts)
    s = pared.proj(pts)
    en_banda = (
        (d > BANDA_MIN)
        & (d < BANDA_MAX)
        & (s > pared.s_min - 0.1)
        & (s < pared.s_max + 0.1)
    )
    idx = np.flatnonzero(en_banda)
    if len(idx) < 2 * MIN_PUNTOS_CAJA:
        return None

    h = paso_rayos(pts[idx], pared.n)
    grupos = [
        g
        for g in agrupar(s[idx], idx, h)
        if len(g.idx) >= MIN_PUNTOS_CAJA and g.h < PASO_MAX and g.ancho_valido()
    ]

    mejor = None
    for a, b in zip(grupos, grupos[1:]):  # pares de grupos vecinos
        # Cada centro puede estar corrido hasta h/2: la tolerancia crece con h
        error = abs((b.centro - a.centro) - SEP_CENTROS)
        if error > TOL_SEP + (a.h + b.h) / 2:
            continue
        if mejor is None or error < mejor[2]:
            mejor = (a, b, error)
    return mejor


def detectar(pts, lines):
    """Busca la firma frente a cada pared y devuelve el mejor Marcador o None."""
    mejor = None
    for pared in lines:
        par = buscar_cajas(pts, pared)
        if par is None:
            continue
        a, b, error = par
        s_eje = (a.centro + b.centro) / 2
        p = pared.n * pared.c + pared.t * s_eje
        if mejor is None or error < mejor.error:
            mejor = Marcador(p, pared.n, pared, (a, b), error)
    return mejor


def centro_sala(lines):
    """Punto (en base_link) a medio camino entre cada par de paredes opuestas."""
    objetivo = np.zeros(2)
    ejes = []  # normales ya usadas, para no contar dos veces la misma direccion
    for pared in lines:  # vienen de la mas poblada a la menos
        if any(abs(pared.n @ u) > 0.9 for u in ejes):
            continue
        opuestas = [m for m in lines if pared.n @ m.n < -0.9]
        if not opuestas:
            continue
        otra = max(opuestas, key=lambda m: len(m.inliers))
        # Distancias del robot a cada una: -c. Moverse (d_otra - d_pared)/2
        # a lo largo de la normal de la pared las iguala.
        objetivo += pared.n * (pared.c - otra.c) / 2
        ejes.append(pared.n)
    return objetivo if ejes else None
