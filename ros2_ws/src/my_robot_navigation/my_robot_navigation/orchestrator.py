import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import ComputePathToPose, FollowPath
from action_msgs.msg import GoalStatus


class Orchestrator(Node):
    def __init__(self):
        super().__init__("orchestrator")

        # --- Estado de la navegación ---
        self.current_goal = None        # meta activa (PoseStamped)
        self.current_path = None        # path del planner
        self.follow_goal_handle = None  # handle del FollowPath activo
        self.pending_goal = None        # meta en espera (llegó durante una navegación)
        self.navigating = False         # True mientras hay navegación en curso
        self.replan_timer = None        # timer de replanificación
        self.session_id = 0             # generación de meta: invalida callbacks al reemplazar (Fase 06)
        self.follow_goal_id = 0         # id de envío al controller: invalida el goal reemplazado (Fase 04)
        self._awaiting_plan = False     # hay un plan en curso
        self._canceling = False         # estamos cancelando para arrancar una meta nueva
        self._last_feedback_log = None  # throttle del feedback a 1 Hz

        # --- Parámetros configurables (deben matchear los nombres de la Parte 3) ---
        self.declare_parameter("replan_period", 1.0)
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("controller_id", "RPP")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")

        self.replan_period = self.get_parameter("replan_period").value
        self.planner_id = self.get_parameter("planner_id").value
        self.controller_id = self.get_parameter("controller_id").value
        self.goal_checker_id = self.get_parameter("goal_checker_id").value
        self.progress_checker_id = self.get_parameter("progress_checker_id").value

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Action clients ---
        self.compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")

        # --- Suscripción a las metas del 2D Goal Pose ---
        self.goal_subscription = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )

        self.get_logger().info("Orquestador iniciado. Esperando metas en /goal_pose...")

    # ─────────────────────────────────────────────────────────
    # Recepción de metas + arbitraje de meta nueva (Fase 06)
    # ─────────────────────────────────────────────────────────
    def goal_callback(self, message):
        x = message.pose.position.x
        y = message.pose.position.y
        z = message.pose.position.z
        qx = message.pose.orientation.x
        qy = message.pose.orientation.y
        qz = message.pose.orientation.z
        qw = message.pose.orientation.w

        self.get_logger().info(
            "\nMeta recibida:\n"
            f" \tframe:       {message.header.frame_id}\n"
            f" \tposición:    x={x:.2f}, y={y:.2f}, z={z:.2f}\n"
            f" \torientación: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}"
        )

        transform = self.get_robot_pose()
        if transform is None:
            self.get_logger().warning(
                "No se pudo consultar la posición actual del robot. ¿Está corriendo AMCL?"
            )
            return

        # Chequear la posición actual del robot
        # robot_x = transform.transform.translation.x
        # robot_y = transform.transform.translation.y
        # robot_z = transform.transform.translation.z
        # orientation = transform.transform.rotation
        # self.get_logger().info(
        #     "Posición actual del robot:\n"
        #     f"  x={robot_x:.2f}\n  y={robot_y:.2f}\n  z={robot_z:.2f}\n"
        #     f"  orientación quaternion: "
        #     f"x={orientation.x:.3f}, y={orientation.y:.3f}, "
        #     f"z={orientation.z:.3f}, w={orientation.w:.3f}"
        # )

        # ¿Estamos ocupados? (navegando, planificando o con un goal de control vivo)
        busy = (self.navigating or self._awaiting_plan
                or self.follow_goal_handle is not None or self._canceling)

        if busy:
            # Fase 06: guardamos la meta nueva y cancelamos la actual. Al confirmarse
            # la cancelación, se dispara la nueva desde after_cancel_callback.
            self.pending_goal = message
            if not self._canceling:
                self.get_logger().info("[PREEMPT] Meta nueva durante la navegación. Cancelando la actual...")
                self.cancel_current_navigation()
            else:
                self.get_logger().info("[PREEMPT] Otra meta más; se usará la última recibida.")
        else:
            self.start_navigation(message)

    def start_navigation(self, goal):
        self.session_id += 1          # nueva generación de meta
        self.current_goal = goal
        self.compute_path(goal)

    def get_robot_pose(self):
        try:
            return self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=1.0)
            )
        except Exception as error:
            self.get_logger().warning(f"No se pudo obtener map -> base_link: {error}")
            return None

    # ─────────────────────────────────────────────────────────
    # Fase 02 — ComputePathToPose (Planner)
    # ─────────────────────────────────────────────────────────
    def compute_path(self, goal_pose):
        session = self.session_id
        if not self.compute_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                "El planner (compute_path_to_pose) no está disponible. ¿Levantaste Nav2?"
            )
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = False

        self._awaiting_plan = True
        send_future = self.compute_path_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: self.compute_path_response_callback(f, session))

    def compute_path_response_callback(self, future, session):
        if session != self.session_id:       # esta meta ya fue reemplazada
            self._awaiting_plan = False
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._awaiting_plan = False
            self.report_failure("PLANNING", "el planner rechazó la meta")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.compute_path_result_callback(f, session))

    def compute_path_result_callback(self, future, session):
        self._awaiting_plan = False
        if session != self.session_id:       # meta reemplazada mientras planificaba
            return

        result = future.result().result

        if result.error_code != 0:
            if self.navigating:
                self.abort_navigation("PLANNING",
                                      f"la replanificación no encontró ruta (error_code={result.error_code})")
            else:
                self.report_failure("PLANNING",
                                    f"error_code={result.error_code} ({result.error_msg})")
                self.current_goal = None
            return

        path = result.path
        if len(path.poses) == 0:
            if self.navigating:
                self.abort_navigation("PLANNING", "la replanificación devolvió un path vacío")
            else:
                self.report_failure("PLANNING", "el planner devolvió un path vacío")
                self.current_goal = None
            return

        self.current_path = path
        tag = "REPLAN" if self.navigating else "PLANNING"
        self.get_logger().info(f"[{tag}] Path: {len(path.poses)} poses. Enviando al controller...")
        self.follow_path(path)

    # ─────────────────────────────────────────────────────────
    # Fase 03 — FollowPath (Controller)
    # ─────────────────────────────────────────────────────────
    def follow_path(self, path):
        if not self.follow_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("El controller (follow_path) no está disponible.")
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = self.controller_id
        goal_msg.goal_checker_id = self.goal_checker_id
        goal_msg.progress_checker_id = self.progress_checker_id

        self.follow_goal_id += 1          # invalida el goal de control anterior (replan/preempt)
        fid = self.follow_goal_id
        self._last_feedback_log = None

        send_future = self.follow_path_client.send_goal_async(
            goal_msg, feedback_callback=self.follow_path_feedback_callback
        )
        send_future.add_done_callback(lambda f: self.follow_path_response_callback(f, fid))

    def follow_path_feedback_callback(self, feedback_msg):
        now = self.get_clock().now()
        if (self._last_feedback_log is not None
                and (now - self._last_feedback_log) < Duration(seconds=1.0)):
            return
        self._last_feedback_log = now
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[CONTROL] Navegando... distancia a la meta: "
            f"{fb.distance_to_goal:.2f} m · velocidad: {fb.speed:.2f} m/s"
        )

    def follow_path_response_callback(self, future, fid):
        goal_handle = future.result()

        # Si este envío ya fue reemplazado, lo cancelamos para no dejarlo vivo.
        if fid != self.follow_goal_id:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if not goal_handle.accepted:
            if self.navigating:
                self.abort_navigation("CONTROL", "el controller rechazó el path")
            else:
                self.report_failure("CONTROL", "el controller rechazó el path")
            return

        self.follow_goal_handle = goal_handle

        if not self.navigating:
            self.navigating = True
            self.get_logger().info("[CONTROL] Path aceptado. Robot en movimiento.")
            self.start_replan_timer()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.follow_path_result_callback(f, fid))

    def follow_path_result_callback(self, future, fid):
        # Resultado de un goal ya reemplazado (replan) o cancelado (preempt/abort): ignorar.
        if fid != self.follow_goal_id:
            return

        status = future.result().status
        result = future.result().result

        # Fin real de la navegación.
        self.navigating = False
        self.follow_goal_handle = None
        self.current_goal = None
        self.stop_replan_timer()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.report_success()
        else:
            self.report_failure("CONTROL", f"status={status}, error_code={result.error_code}")

    # ─────────────────────────────────────────────────────────
    # Fase 04 — Replanificación periódica
    # ─────────────────────────────────────────────────────────
    def start_replan_timer(self):
        if self.replan_timer is None:
            self.replan_timer = self.create_timer(self.replan_period, self.replan_timer_callback)
            self.get_logger().info(f"[REPLAN] Replanificación activada (cada {self.replan_period:.1f}s).")

    def stop_replan_timer(self):
        if self.replan_timer is not None:
            self.replan_timer.cancel()
            self.destroy_timer(self.replan_timer)
            self.replan_timer = None
            self.get_logger().info("[REPLAN] Replanificación detenida.")

    def replan_timer_callback(self):
        if not self.navigating or self.current_goal is None:
            return
        if self._awaiting_plan:
            return
        self.compute_path(self.current_goal)

    # ─────────────────────────────────────────────────────────
    # Fase 06 — Cancelación limpia para arrancar meta nueva
    # ─────────────────────────────────────────────────────────
    def cancel_current_navigation(self):
        self._canceling = True
        self.session_id += 1        # invalida el plan en vuelo del goal viejo
        self.follow_goal_id += 1    # invalida el resultado del control viejo
        self._awaiting_plan = False
        self.navigating = False
        self.stop_replan_timer()

        handle = self.follow_goal_handle
        self.follow_goal_handle = None
        if handle is not None:
            cancel_future = handle.cancel_goal_async()   # frena el robot
            cancel_future.add_done_callback(self.after_cancel_callback)
        else:
            # No había control activo (p.ej. solo estaba planificando): arrancamos ya.
            self.start_pending()

    def after_cancel_callback(self, future):
        # El controller confirmó la cancelación (robot detenido): lanzamos la meta pendiente.
        self.get_logger().info("[PREEMPT] Navegación anterior cancelada.")
        self.start_pending()

    def start_pending(self):
        self._canceling = False
        if self.pending_goal is not None:
            goal = self.pending_goal
            self.pending_goal = None
            self.get_logger().info("[PREEMPT] Arrancando la nueva meta.")
            self.start_navigation(goal)

    # ─────────────────────────────────────────────────────────
    # Fase 05 — Reporte de finalización (etapa + resumen)
    # ─────────────────────────────────────────────────────────
    def report_success(self):
        self.get_logger().info("========== ✓ ÉXITO: meta alcanzada ==========")

    def report_failure(self, stage, detail):
        self.get_logger().error(f"========== ✗ FALLO en etapa {stage} — {detail} ==========")

    def abort_navigation(self, stage, detail):
        self.report_failure(stage, detail)
        self.session_id += 1
        self.follow_goal_id += 1
        handle = self.follow_goal_handle
        self.follow_goal_handle = None
        if handle is not None:
            handle.cancel_goal_async()   # frena el robot
        self.navigating = False
        self.current_goal = None
        self._awaiting_plan = False
        self.stop_replan_timer()


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()