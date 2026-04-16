import rclpy
from rclpy.node import Node
import math
import time
from geometry_msgs.msg import PointStamped
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16, Call, SetInt16ById
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

class FollowerAutonomo(Node):
    def __init__(self):
        super().__init__('Follower')

        # Clientes
        self.cartesian_client = self.create_client(MoveCartesian, '/xarm/set_position')
        self.joint_client = self.create_client(MoveJoint, '/xarm/set_servo_angle')
        self.state_client = self.create_client(SetInt16, '/xarm/set_state')
        self.mode_client = self.create_client(SetInt16, '/xarm/set_mode')
        self.enable_client = self.create_client(SetInt16ById, '/xarm/motion_enable')
        self.error_client = self.create_client(Call, '/xarm/clean_error')
        self.grab_client = self.create_client(Trigger, '/gripper/smart_grab')
        self.open_client = self.create_client(Trigger, '/gripper/open')

        # Suscriptores
        self.create_subscription(Bool, '/follower/start', self.start_mission_cb, 10)
        self.create_subscription(PointStamped, '/stone/target_pose_base', self.target_cb, 10)
        self.resume_pub = self.create_publisher(Bool, '/search/resume', 10)

        # Variables de Misión
        self.live_target = None
        self.mission_target = None
        self.state = "IDLE"
        self.step_idx = 0
        self.busy = False
        self.stable_count = 0
        self.action_queue = [] 
        
        # Banderas de Delay Físico
        self.waiting_for_delay = False
        self.delay_end_time = 0.0
        
        # Parámetros Calibrados Originales
        self.EPSILON = 50.0        
        self.Z_FLOOR = -219.2      
        self.Z_APPROACH = -169.2   
        self.GRIPPER_OFFSET_Y = 16.5  
        self.GRIPPER_OFFSET_X = 8.0   

        self.HOME_CARTESIAN = [1.4, -292.1, 239.5, -3.1352, -0.6782, -1.5780]
        self.HOME_JOINTS = [-1.5747, -0.2777, -0.0971, -0.0056, 0.1502, 0.0002]
        self.DEPOSIT_JOINTS = [-1.5752, -0.2315, -0.1048, -2.2752, -0.2931, 0.0008]

        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info('Follower Final (Tiempos Corregidos) Listo.')

    def target_cb(self, msg):
        if self.state == "WAITING_FOR_STABILITY":
            self.live_target = msg.point

    def reset_arm_services(self):
        self.error_client.call_async(Call.Request())
        req_en = SetInt16ById.Request(); req_en.id = 8; req_en.data = 1
        self.enable_client.call_async(req_en)
        req_mode = SetInt16.Request(); req_mode.data = 0
        self.mode_client.call_async(req_mode)
        req_state = SetInt16.Request(); req_state.data = 0
        self.state_client.call_async(req_state)

    def start_mission_cb(self, msg):
        if msg.data and self.state == "IDLE":
            self.get_logger().info('Confirmado HOME. Iniciando pausa física...')
            self.live_target = None
            self.stable_count = 0
            self.state = "SETTLING_PAUSE"
            self.start_time = time.time()

    def call_service(self, client, req):
        self.busy = True
        future = client.call_async(req)
        future.add_done_callback(self.service_done)

    def service_done(self, future):
        time.sleep(0.1) 
        self.busy = False
        self.step_idx += 1

    def main_loop(self):
        if self.busy: return

        if self.waiting_for_delay:
            if time.time() >= self.delay_end_time:
                self.waiting_for_delay = False
                self.step_idx += 1 
            return 

        if self.state == "SETTLING_PAUSE":
            if time.time() - self.start_time > 2.0:
                self.get_logger().info('Abriendo ojos desde HOME. Esperando lecturas...')
                self.state = "WAITING_FOR_STABILITY"
                self.start_time = time.time()

        elif self.state == "WAITING_FOR_STABILITY":
            if self.live_target:
                self.stable_count += 1
                if self.stable_count >= 10:
                    self.get_logger().info('¡Roca estable! Armando lista de tareas...')
                    self.mission_target = self.live_target
                    self.prepare_action_queue() 
                    self.reset_arm_services()
                    self.decision_delay = time.time()
                    self.state = "DECISION_DELAY"
            else:
                self.stable_count = 0

            if time.time() - self.start_time > 10.0:
                self.get_logger().error('Roca no vista. Retomando búsqueda...')
                self.resume_pub.publish(Bool(data=True))
                self.state = "IDLE"

        elif self.state == "DECISION_DELAY":
            if time.time() - self.decision_delay > 1.5:
                self.state = "EXECUTING"
                self.step_idx = 0

        elif self.state == "EXECUTING":
            self.execute_action()

    def prepare_action_queue(self):
        target_x = self.mission_target.x * 1000.0
        target_y = self.mission_target.y * 1000.0
        grip_r, grip_p, grip_yw = -3.1416, 0.0, -1.5708
        
        self.action_queue = [] 
        
        # --- FASE 1: DESCENSO VISUAL ---
        goal_b = [target_x, target_y, self.Z_APPROACH]
        dx = goal_b[0] - self.HOME_CARTESIAN[0]
        dy = goal_b[1] - self.HOME_CARTESIAN[1]
        dz = goal_b[2] - self.HOME_CARTESIAN[2]
        dist_total = math.sqrt(dx**2 + dy**2 + dz**2)
        num_pasos = math.ceil(dist_total / self.EPSILON)
        
        for i in range(1, num_pasos + 1):
            ratio = i / num_pasos
            w_x = self.HOME_CARTESIAN[0] + (dx * ratio)
            w_y = self.HOME_CARTESIAN[1] + (dy * ratio)
            w_z = self.HOME_CARTESIAN[2] + (dz * ratio)
            self.action_queue.append({
                'type': 'cartesian',
                'pose': [w_x, w_y, w_z, self.HOME_CARTESIAN[3], self.HOME_CARTESIAN[4], self.HOME_CARTESIAN[5]],
                'speed': 50.0, 'acc': 500.0, 'label': f'Orden Waypoint {i}/{num_pasos}'
            })
            self.action_queue.append({'type': 'delay', 'duration': 1.2, 'label': 'Físico: Viajando a Waypoint'})

        # --- FASE 2: AGARRE ---
        final_x = target_x + self.GRIPPER_OFFSET_X
        final_y = target_y + self.GRIPPER_OFFSET_Y 
        
        self.action_queue.append({
            'type': 'cartesian',
            'pose': [final_x, final_y, self.Z_FLOOR, grip_r, grip_p, grip_yw],
            'speed': 40.0, 'acc': 400.0, 'label': 'Orden Punto C (Piso)'
        })
        self.action_queue.append({'type': 'delay', 'duration': 2.5, 'label': 'Físico: Estabilizando en el piso'})
        
        self.action_queue.append({'type': 'service', 'client': self.grab_client, 'label': 'Orden Cerrar Gripper'})
        self.action_queue.append({'type': 'delay', 'duration': 3.5, 'label': 'Físico: Cierre y presión de Gripper'})

        # --- FASE 3: RETORNO Y DEPÓSITO ---
        self.action_queue.append({
            'type': 'cartesian',
            'pose': [final_x, final_y, self.Z_APPROACH, grip_r, grip_p, grip_yw],
            'speed': 50.0, 'acc': 500.0, 'label': 'Orden Escape Vertical'
        })
        self.action_queue.append({'type': 'delay', 'duration': 2.0, 'label': 'Físico: Levantando la roca'})

        self.action_queue.append({
            'type': 'joint', 'angles': self.HOME_JOINTS, 
            'speed': 0.35, 'acc': 2.0, 'label': 'Orden Arco a Home'
        })
        self.action_queue.append({'type': 'delay', 'duration': 5.0, 'label': 'Físico: Llegando a Home'})

        # ==========================================
        # EL FIX DEL DEPÓSITO Y LOS 3 SEGUNDOS
        # ==========================================
        self.action_queue.append({
            'type': 'joint', 'angles': self.DEPOSIT_JOINTS, 
            'speed': 0.35, 'acc': 2.0, 'label': 'Orden Arco a Depósito'
        })
        # 1. Le damos 7.5 segundos al brazo para completar físicamente el trayecto
        self.action_queue.append({'type': 'delay', 'duration': 7.5, 'label': 'Físico: Viaje a Depósito'})
        # 2. Los 3 segundos congelados que solicitaste
        self.action_queue.append({'type': 'delay', 'duration': 3.0, 'label': 'Físico: Pausa de 3s en Depósito'})

        self.action_queue.append({'type': 'service', 'client': self.open_client, 'label': 'Orden Abrir Gripper'})
        self.action_queue.append({'type': 'delay', 'duration': 2.5, 'label': 'Físico: Soltando roca'})

        self.action_queue.append({
            'type': 'joint', 'angles': self.HOME_JOINTS, 
            'speed': 0.35, 'acc': 2.0, 'label': 'Orden Regreso a Operación'
        })
        self.action_queue.append({'type': 'delay', 'duration': 6.5, 'label': 'Físico: Posicionando en Home'})

    def execute_action(self):
        if self.step_idx < len(self.action_queue):
            action = self.action_queue[self.step_idx]
            
            if action['type'] == 'delay':
                self.get_logger().info(f"[{self.step_idx + 1}/{len(self.action_queue)}] {action['label']} ({action['duration']}s)")
                self.waiting_for_delay = True
                self.delay_end_time = time.time() + action['duration']
                
            else:
                self.get_logger().info(f"[{self.step_idx + 1}/{len(self.action_queue)}] {action['label']}")
                if action['type'] == 'cartesian':
                    req = MoveCartesian.Request()
                    req.pose, req.speed, req.acc = action['pose'], action['speed'], action['acc']
                    self.call_service(self.cartesian_client, req)
                    
                elif action['type'] == 'joint':
                    req = MoveJoint.Request()
                    req.angles, req.speed, req.acc = action['angles'], action['speed'], action['acc']
                    self.call_service(self.joint_client, req)
                    
                elif action['type'] == 'service':
                    req = Trigger.Request()
                    self.call_service(action['client'], req)
        else:
            self.get_logger().info('¡Recolección completada! Regresando estafeta al Search.')
            self.resume_pub.publish(Bool(data=True))
            self.state = "IDLE"

def main():
    rclpy.init(); node = FollowerAutonomo(); rclpy.spin(node); rclpy.shutdown()

if __name__ == '__main__':
    main()