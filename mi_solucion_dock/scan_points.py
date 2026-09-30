import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from mi_solucion_dock.detector import centro_sala, detectar, find_lines
from mi_solucion_dock.filtro import FiltroDock, Pose2D
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


def scan_to_points(msg):
    """LaserScan -> array Nx2 de puntos (x, y) en el frame del laser."""
    r = np.asarray(msg.ranges, dtype=float)
    ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
    r, ang = r[ok], ang[ok]
    return np.column_stack((r * np.cos(ang), r * np.sin(ang)))


def yaw_de(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ScanPoints(Node):
    def __init__(self, nombre="scan_points"):
        super().__init__(nombre)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.laser_tf = None  # (tx, ty, yaw) de base_link -> laser_link
        self.filtro = FiltroDock()
        self.centro_odom = None  # centro de la sala en odom (para la busqueda)
        self.pub = self.create_publisher(Marker, "debug/scan_points", 10)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )

    def get_laser_tf(self, frame):
        """La TF base_link -> laser es estatica: se lee una vez y se guarda."""
        try:
            t = self.tf_buffer.lookup_transform("base_link", frame, Time())
        except RuntimeError as e:
            self.get_logger().warn(
                f"Esperando TF base_link -> {frame}: {e}", throttle_duration_sec=2.0
            )
            return None
        yaw = yaw_de(t.transform.rotation)
        tf = (t.transform.translation.x, t.transform.translation.y, yaw)
        self.get_logger().info(
            f"TF laser: x={tf[0]:.4f} y={tf[1]:.4f} yaw={math.degrees(yaw):.1f} deg"
        )
        return tf

    def get_odom_pose(self, stamp=None):
        """Pose del robot en odom en ese instante (None = la ultima), o None."""
        try:
            instante = Time() if stamp is None else Time.from_msg(stamp)
            t = self.tf_buffer.lookup_transform("odom", "base_link", instante)
        except Exception:
            try:  # si ese instante aun no llego al buffer, la ultima disponible
                t = self.tf_buffer.lookup_transform("odom", "base_link", Time())
            except Exception as e:
                self.get_logger().warn(
                    f"Sin TF odom -> base_link: {e}", throttle_duration_sec=2.0
                )
                return None
        tr = t.transform.translation
        return Pose2D(tr.x, tr.y, yaw_de(t.transform.rotation))

    def on_scan(self, msg):
        if self.laser_tf is None:
            self.laser_tf = self.get_laser_tf(msg.header.frame_id)
            if self.laser_tf is None:
                return

        # Rotar y trasladar los puntos del laser al frame del robot
        tx, ty, yaw = self.laser_tf
        c, s = math.cos(yaw), math.sin(yaw)
        pts = scan_to_points(msg) @ np.array([[c, s], [-s, c]]) + (tx, ty)

        self.publish_points(pts, msg.header.stamp)

        # Paso 2: paredes con RANSAC
        lines = find_lines(pts)
        self.publish_lines(lines, msg.header.stamp)

        # Paso 3: buscar las dos cajas frente a alguna pared
        mk = detectar(pts, lines)
        self.publish_marcador(mk, pts, msg.header.stamp)

        # Paso 4: pasar la deteccion a odom y filtrarla
        odom = self.get_odom_pose(msg.header.stamp)
        if odom is None:
            return
        t_s = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if mk is not None:
            aceptada = self.filtro.actualizar(odom.punto(mk.p), odom.vector(mk.n), t_s)
            if not aceptada:
                self.get_logger().warn("Deteccion descartada (salto)")
        centro = centro_sala(lines)
        self.centro_odom = None if centro is None else odom.punto(centro)
        self.log_estado(odom, t_s)
        self.publish_filtrado(msg.header.stamp)

    def log_estado(self, odom, t_s):
        f = self.filtro
        if not f.confiable():
            self.get_logger().warn(
                f"Sin estimacion confiable ({f.n_ok} detecciones)",
                throttle_duration_sec=1.0,
            )
            return
        # De odom de vuelta al robot, para leer los errores que usara el control
        p = odom.punto_inv(f.p)
        n = odom.vector_inv(f.n)
        t = np.array([-n[1], n[0]])
        self.get_logger().info(
            f"DOCK (filtrado): dist={-p @ n:.3f}m lateral={-p @ t:+.3f}m "
            f"rumbo={math.degrees(math.atan2(-n[1], -n[0])):+.1f}deg | "
            f"ultima deteccion hace {f.edad(t_s):.1f}s",
            throttle_duration_sec=1.0,
        )

    def publish_filtrado(self, stamp):
        # Flecha verde: la estimacion filtrada, en odom (deberia quedarse quieta)
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = stamp
        m.ns, m.id = "eje_filtrado", 4
        m.type = Marker.ARROW
        m.scale.x, m.scale.y, m.scale.z = 0.03, 0.07, 0.07
        m.color.g, m.color.a = 1.0, 1.0
        if not self.filtro.confiable():
            m.action = Marker.DELETE
        else:
            m.action = Marker.ADD
            p, punta = self.filtro.p, self.filtro.p + 0.6 * self.filtro.n
            m.points = [
                Point(x=float(p[0]), y=float(p[1]), z=0.05),
                Point(x=float(punta[0]), y=float(punta[1]), z=0.05),
            ]
        self.pub.publish(m)

    def publish_points(self, pts, stamp):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = stamp
        m.ns, m.id = "scan", 0
        m.type, m.action = Marker.POINTS, Marker.ADD
        m.scale.x = m.scale.y = 0.02
        m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0
        m.points = [Point(x=float(x), y=float(y)) for x, y in pts]
        self.pub.publish(m)

    def publish_marcador(self, mk, pts, stamp):
        # Puntos de las cajas (magenta) y flecha del eje del dock (amarilla)
        cajas = Marker()
        cajas.header.frame_id = "base_link"
        cajas.header.stamp = stamp
        cajas.ns, cajas.id = "cajas", 2
        cajas.type = Marker.POINTS
        cajas.scale.x = cajas.scale.y = 0.03
        cajas.color.r, cajas.color.b, cajas.color.a = 1.0, 1.0, 1.0

        eje = Marker()
        eje.header = cajas.header
        eje.ns, eje.id = "eje", 3
        eje.type = Marker.ARROW
        eje.scale.x, eje.scale.y, eje.scale.z = 0.02, 0.05, 0.05
        eje.color.r, eje.color.g, eje.color.a = 1.0, 1.0, 1.0

        if mk is None:
            cajas.action = eje.action = Marker.DELETE
        else:
            cajas.action = eje.action = Marker.ADD
            for caja in mk.cajas:
                for x, y in pts[caja.idx]:
                    cajas.points.append(Point(x=float(x), y=float(y)))
            punta = mk.p + 0.5 * mk.n
            eje.points = [
                Point(x=float(mk.p[0]), y=float(mk.p[1])),
                Point(x=float(punta[0]), y=float(punta[1])),
            ]
        self.pub.publish(cajas)
        self.pub.publish(eje)

    def publish_lines(self, lines, stamp):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = stamp
        m.ns, m.id = "paredes", 1
        m.type, m.action = Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0
        for L in lines:
            for p in L.endpoints():
                m.points.append(Point(x=float(p[0]), y=float(p[1])))
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ScanPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
