#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

class Orchestrator(Node):

    def __init__(self):
        super().__init__("orchestrator")

        self.current_goal = None
        self.tf_buffer = Buffer() # Guardo las transformaciones
        self.tf_listener = TransformListener(self.tf_buffer, self) #Escucho /tf y /tf_static


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

        transform = self.get_robot_pose()

        if transform is None:
            self.get_logger().warning(
            "No se pudo consultar la posición actual del robot."
            )
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        robot_z = transform.transform.translation.z

        orientation = transform.transform.rotation

        # Chequear la posición actual del robot
        # self.get_logger().info(
        #     "Posición actual del robot:\n"
        #     f"  x={robot_x:.2f}\n"
        #     f"  y={robot_y:.2f}\n"
        #     f"  z={robot_z:.2f}\n"
        #     f"  orientación quaternion: "
        #     f"x={orientation.x:.3f}, "
        #     f"y={orientation.y:.3f}, "
        #     f"z={orientation.z:.3f}, "
        #     f"w={orientation.w:.3f}"
        # )



    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception as error:
            self.get_logger().warning(f"No se pudo obtener map -> baselink {error}")
            return None
        return transform

def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()