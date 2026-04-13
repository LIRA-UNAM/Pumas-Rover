import rclpy
from rclpy.node import Node
from xarm_msgs.srv import MoveJoint, SetInt16
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import  Bool
import time

# ESTADOS
SM_MOVING = 0   # Movimiento de cadera (Rápido)
SM_LOOK = 1     # Agachando (Lento/Suave)
SM_RETRACT = 2  # Levantando (Lento/Suave)
SM_WAIT = 3     # Pausa por detección

class SearchNode(Node):
    def __init__(self):
        super().__init__('search_node')
        
        self.joint_client = self.create_client(MoveJoint, '/xarm/set_servo_angle')
        self.state_client = self.create_client(SetInt16, '/xarm/set_state')
        
        self.joint_sub = self.create_subscription(JointState, '/xarm/joint_states', self.initial_pos_callback, 10)
        self.confirmed_sub = self.create_subscription(PointStamped, '/yolo/confirmed_object', self.detection_callback, 10)

        #ros2 topic pub --once /arm_searcher std_msgs/msg/Bool "{data: true}"

        self.subscription_start = self.create_subscription(Bool, '/arm_searcher', self.start_callback, 10)
        self.publisher_stop = self.create_publisher(Bool, '/arm_stop',10)

        self.sm_start = False
        self.loops = 0

        self.state = SM_MOVING
        self.idx = 0
        self.busy = False
        self.initial_check_done = False 
        
        self.j5_normal = -1.0594
        self.j5_look = -0.7461

        # CONFIGURACIÓN DE VELOCIDADES
        self.VEL_CADERA = 0.45  # Rápido para el barrido (aprox 30 deg/s)
        self.VEL_LOOK = 0.1   # Lento para no vibrar la cámara

        # Puntos base
        self.puntos_base = [
            [0.0, -0.28099, -0.10472, -0.00523, self.j5_normal, 0.0017],
            [-1.57079, -0.28099, -0.10472, -0.00523, self.j5_normal, 0.0017],
            [-3.14159, -0.28099, -0.10472, -0.00523, self.j5_normal, 0.0017]
        ]
        
        self.get_logger().info('Iniciando búsqueda con velocidades independientes...')
        self.timer = self.create_timer(0.2, self.fsm_loop)

    def initial_pos_callback(self, msg):
        if self.initial_check_done: return
        current_j1 = msg.position[0]
        distancias = [abs(current_j1 - p[0]) for p in self.puntos_base]
        self.idx = distancias.index(min(distancias))
        self.initial_check_done = True
        self.destroy_subscription(self.joint_sub)

    def detection_callback(self, msg):
        if self.state != SM_WAIT:
            self.get_logger().warn('!!! ROCA DETECTADA !!! Deteniendo...')
            self.state = SM_WAIT
            self.stop_arm()
            self.sm_start = False
            self.loops = 0

    def start_callback (self, msg):
        self.sm_start = True

    def stop_arm(self):
        req = SetInt16.Request()
        req.data = 4
        self.state_client.call_async(req)

    def move_done_callback(self, future):
        # Reducimos el tiempo de asentamiento a 0.2s para que sea más ágil
        time.sleep(0.2)
        self.busy = False

    def send_angles(self, angles, speed):
        """Ahora recibe la velocidad como argumento"""
        self.busy = True
        req = MoveJoint.Request()
        req.angles = angles
        req.speed = speed
        req.acc = 2.0  # Un poco más de aceleración para que no sea tan "chicle"
        req.wait = True 
        
        future = self.joint_client.call_async(req)
        future.add_done_callback(self.move_done_callback)

    def fsm_loop(self):
        if self.state == SM_WAIT or self.busy or not self.initial_check_done or not self.sm_start:
            self.get_logger().info(f'Wait {self.idx}')
            return

        if self.state == SM_MOVING:
            self.get_logger().info(f'-> CADERA (Rápida) al Punto {self.idx}')
            self.send_angles(self.puntos_base[self.idx], self.VEL_CADERA)
            self.loops += 1
            if self.loops > 3:
                self.sm_start = False
                self.publisher_stop.publish(Bool(data=True))
            self.state = SM_LOOK

        elif self.state == SM_LOOK:
            self.get_logger().info('-> LOOK (Lento) Agachando J5')
            pose_look = list(self.puntos_base[self.idx])
            pose_look[4] = self.j5_look
            self.send_angles(pose_look, self.VEL_LOOK)
            self.state = SM_RETRACT

        elif self.state == SM_RETRACT:
            self.get_logger().info('-> RETRACT (Lento) Levantando J5')
            self.send_angles(self.puntos_base[self.idx], self.VEL_LOOK)
            self.idx = (self.idx + 1) % len(self.puntos_base)
            self.state = SM_MOVING

def main(args=None):
    rclpy.init(args=args)
    node = SearchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()