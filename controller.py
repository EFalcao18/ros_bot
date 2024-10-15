#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Configura o QoS para usar a política de confiabilidade BEST_EFFORT
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)

        self.stop_distance = 0.1  # Distância mínima para parar o robô (em metros)
        self.forward_speed = 0.15  # Velocidade linear do robô

    def scan_callback(self, msg):
        # Verifica a distância mínima à frente do robô
        min_distance = min(msg.ranges)
        twist = Twist()

        if min_distance < self.stop_distance:
            # Se um obstáculo estiver perto, para o robô
            twist.linear.x = 0.0
            self.get_logger().info('Obstáculo detectado! Parando.')
        else:
            # Se não houver obstáculos, move para frente
            twist.linear.x = self.forward_speed
            self.get_logger().info('Movendo para frente.')

        # Publica a mensagem de velocidade no tópico /cmd_vel
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
