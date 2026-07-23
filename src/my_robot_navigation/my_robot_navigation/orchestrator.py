#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class Orchestrator(Node):

    def __init__(self):
        super().__init__("orchestrator")

        self.current_goal = None

        self.goal_subscription = self.create_subscription(
            PoseStamped,       
            "/goal_pose",      
            self.goal_callback,
            10,                
        )

        self.get_logger().info(
            "Orquestador iniciado. Esperando metas en /goal_pose..."
        )

    def goal_callback(self, message):
        """
        Se ejecuta automáticamente cada vez que
        llega un PoseStamped por /goal_pose.
        """

        # Guardamos el mensaje completo.
        self.current_goal = message

        # Extraemos la posición para imprimirla.
        x = message.pose.position.x
        y = message.pose.position.y
        z = message.pose.position.z

        # Extraemos también la orientación.
        qx = message.pose.orientation.x
        qy = message.pose.orientation.y
        qz = message.pose.orientation.z
        qw = message.pose.orientation.w

        self.get_logger().info(
            "\nMeta recibida:\n"
            f" \tposición:    x={x:.2f}, y={y:.2f}, z={z:.2f}\n"
            f" \torientación: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()