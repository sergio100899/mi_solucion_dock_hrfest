import rclpy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import DockStatus
from mi_solucion_dock.control import Controlador
from mi_solucion_dock.scan_points import ScanPoints
from rclpy.qos import qos_profile_sensor_data

DT = 0.05  # s, control a 20 Hz


class DockLidar(ScanPoints):
    def __init__(self):
        super().__init__("dock_lidar")
        self.ctrl = Controlador()
        self.acoplado = False
        self.estado_previo = None
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            DockStatus, "/dock_status", self.on_dock_status, qos_profile_sensor_data
        )
        self.create_timer(DT, self.on_control)

    def on_dock_status(self, msg):
        self.acoplado = msg.is_docked

    def on_control(self):
        odom = self.get_odom_pose()
        if odom is None:
            return
        t_s = self.get_clock().now().nanoseconds * 1e-9

        f = self.filtro
        est = None
        if f.confiable():
            est = (odom.punto_inv(f.p), odom.vector_inv(f.n))
        centro = None
        if self.centro_odom is not None:
            centro = odom.punto_inv(self.centro_odom)

        v, w = self.ctrl.paso(DT, est, f.edad(t_s), centro, self.acoplado)

        if self.ctrl.estado != self.estado_previo:
            self.get_logger().info(f"Estado: {self.ctrl.estado}")
            self.estado_previo = self.ctrl.estado

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)

    def parar(self):
        try:  # con Ctrl-C el contexto de ROS puede estar ya cerrado
            self.cmd_pub.publish(Twist())
        except Exception:
            pass


def main():
    rclpy.init()
    node = DockLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.parar()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
