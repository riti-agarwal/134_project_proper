# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Point, Pose, Quaternion
# from project_interfaces.msg import SplineSegment
# from std_msgs.msg import Float64
# import math

# from threedof.KinematicChain import KinematicChain  # For FK/Jacobian computation
# import math

# RATE = 100.0  
# DT   = 1.0 / RATE


# class BrainNode(Node):
#     def __init__(self, name):
#         super().__init__(name)

#         base_frame = "world"
#         tip_frame  = "tip"
#         expected_joint_names = ["base", "shoulder", "elbow"]
#         self.chain = KinematicChain(
#             node=self,
#             baseframe=base_frame,
#             tipframe=tip_frame,
#             expectedjointnames=expected_joint_names
#         )
#         self.pointsub = self.create_subscription(Point, '/balldetector/point', self.recv_point, 10)
#         self.posesub = self.create_subscription(Pose, '/balldetector/pose', self.recv_pose, 10)
#         self.spline_pub = self.create_publisher(SplineSegment, '/spline_segment', 10)

#         self.shutdown_after_publish = False
#         self.home_position = [np.pi/2, 0.0, np.pi/2]
#         self.position0 = self.grabfbk()
#         self.actpos = self.position0.copy()
#         self.lam = 20

#         self.get_logger().info("Brain Node Initialized and Listening to /balldetector/point and /balldetector/pose")


#     def grabfbk(self):
#         def cb(fbkmsg):
#             self.grabpos   = list(fbkmsg.position)
#             self.grabready = True

#         sub = self.create_subscription(JointState, '/joint_states', cb, 1)
#         self.grabready = False
#         while not self.grabready:
#             rclpy.spin_once(self)
#         self.destroy_subscription(sub)
#         return self.grabpos

#     def recvfbk(self, fbkmsg):
#         self.actpos = list(fbkmsg.position)
#         self.actvel = list(fbkmsg.velocity)
#         self.acteff = list(fbkmsg.effort)


#     def cartesian_to_joint_space_newton(self, pd, max_iters=100000, tol=1e-6):
#         """
#         Solves inverse kinematics using Newton-Raphson method.
        
#         Arguments:
#         - pd: Desired Cartesian position of end-effector.
#         - max_iters: Maximum number of iterations to refine the solution.
#         - tol: Convergence tolerance for position error.

#         Returns:
#         - Updated joint positions.
#         """
#         # Use an initial guess. You might choose the current configuration or a "home" position.
#         q = np.array(self.home_position)  
#         lam = 0.01
#         I = np.eye(len(q))
        
#         for i in range(max_iters):
#             ptip, Rtip, Jv, Jw = self.chain.fkin(q)
#             error = np.array(pd) - np.array(ptip)
            
#             if np.linalg.norm(error) < tol:
#                 print(f"Converged in {i} iterations with error norm {np.linalg.norm(error):.2e}.")
#                 break
            
#             # --- Damped Least Squares Step ---
#             # Instead of simply taking the pseudo-inverse of Jv,
#             # we form the damped pseudo-inverse:
#             #
#             #   delta_q = (Jv^T Jv + lam^2 I)^(-1) Jv^T * error
#             #
#             # This helps avoid numerical issues near singularities.
#             J_damped_inv = np.linalg.inv(Jv.T @ Jv + lam**2 * I) @ Jv.T
            
#             delta_q = J_damped_inv @ error
#             q = q + delta_q
            
#             # --- Optional: Enforce joint limits if they are defined ---
#             if hasattr(self, 'joint_lower_limits') and hasattr(self, 'joint_upper_limits'):
#                 q = np.clip(q, self.joint_lower_limits, self.joint_upper_limits)
                
#         else:
#             raise RuntimeError("Newton-Raphson IK did not converge within the maximum number of iterations.")
    
#         return list(q)
    
#     def cartesian_to_joint_space_ik(self, pf):
#         dt = DT  
#         q = np.array(self.actpos)  
#         ptip, Rtip, Jv, Jw = self.chain.fkin(q)
#         e = np.array(pf) - np.array(ptip)
#         Jpinv = np.linalg.pinv(Jv)
#         qdot = Jpinv @ (self.lam * e)
#         q_new = q + qdot * dt
#         return list(q_new)

#     def recv_point(self, msg):
#         """
#         Receives a target point in Cartesian space and converts it to joint space.
#         Sends the joint space trajectory via ROS2 message.
#         """
#         if self.shutdown_after_publish:
#             return 
        
#         home_msg = SplineSegment()
#         home_msg.qf = list(self.home_position)
#         home_msg.vf = [0.0, 0.0, 0.0]
#         home_msg.t_move = 10.0 
#         self.spline_pub.publish(home_msg)

#         pd = [msg.x, msg.y, msg.z]  
#         self.get_logger().info(f"Received Point: {pd}")

#         qf = self.cartesian_to_joint_space_newton(pd)

#         segment_msg = SplineSegment()
#         segment_msg.qf = qf
#         segment_msg.vf = [0.0, 0.0, 0.0] 
#         segment_msg.t_move = 10.0 
#         self.spline_pub.publish(segment_msg)

#         self.get_logger().info(f"Published SplineSegment for Point: {qf}")
#         self.shutdown_after_publish = True
#         self.get_logger().info("Published first SplineSegment. Shutting down.")
#         rclpy.shutdown()

#     def recv_pose(self, msg):
#         """
#         Receives a target pose in Cartesian space and calculates 3 waypoints.
#         Converts those waypoints to joint space using Newton-Raphson.
#         Publishes each waypoint as a separate SplineSegment.
#         """

#         if self.shutdown_after_publish:
#             return 
#         position = [msg.position.x, msg.position.y, msg.position.z]
#         orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
#         self.get_logger().info(f"Received Pose: Position={position}, Orientation={orientation}")

#         theta = 2 * np.arcsin(orientation[2]) 
#         dx = np.cos(theta)
#         dy = np.sin(theta)

#         point1 = np.array(position) + 0.05 * np.array([-dy, dx, 0])
#         point2 = np.array(position) + 0.05 * np.array([dy, -dx, 0])
#         midpoint = np.array(position) + np.array([0, 0, 0.05])

#         q1 = self.cartesian_to_joint_space_newton(point1)
#         q2 = self.cartesian_to_joint_space_newton(point2)
#         q_mid = self.cartesian_to_joint_space_newton(midpoint)

#         self.publish_spline_segment(q1, 10.0)
#         self.publish_spline_segment(q_mid, 5.0)
#         self.publish_spline_segment(q2, 10.0)

#         self.get_logger().info(f"Published 3 SplineSegments for Pose")
#         self.shutdown_after_publish = True
#         self.get_logger().info("Published first SplineSegment. Shutting down.")
#         rclpy.shutdown()

#     def publish_spline_segment(self, qf, t_move):
#         """
#         Publishes a SplineSegment message to the low-level node.
#         """
#         segment_msg = SplineSegment()
#         segment_msg.qf = qf
#         segment_msg.vf = [0.0, 0.0, 0.0]  
#         segment_msg.t_move = t_move  
#         self.spline_pub.publish(segment_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = BrainNode('brain')
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()


import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from project_interfaces.msg import SplineSegment, ObjectArray, Object, ControlCommand
from project_interfaces.msg import ExecutionStatus
from std_msgs.msg import Float64
import math

from threedof.KinematicChain import KinematicChain  

RATE = 100.0  
DT   = 1.0 / RATE

class BrainNode(Node):
    def __init__(self, name):
        super().__init__(name)

        # self.send_once = False

        self.ready = False

        # Home position (joint angles)
        self.home_position = [np.pi/2, 0.0, np.pi/2]

        # Store current joint position
        self.position0 = self.grabfbk()
        self.actpos = self.position0.copy()

        # Subscribe to detected objects
        self.obj_sub = self.create_subscription(ObjectArray, 'circledetector/detected_objects', self.recv_objects, 3)

        base_frame = "world"
        tip_frame  = "tip"
        expected_joint_names = ["base", "shoulder", "elbow"]
        self.chain = KinematicChain(
            node=self,
            baseframe=base_frame,
            tipframe=tip_frame,
            expectedjointnames=expected_joint_names
        )

        # Publisher for robot movement
        self.spline_pub = self.create_publisher(SplineSegment, '/spline_segment', 10)

        # Subscriber for low-level execution readiness
        self.status_sub = self.create_subscription(ExecutionStatus, '/execution_status', self.recv_status, 10)

        self.get_logger().info("Brain Node Initialized and Listening to detected_objects")

    def recv_status(self, msg):
        self.ready = msg.ready
        if self.ready:
            self.get_logger().info("Received 'ready' from low-level. Processing next detections.")

    def grabfbk(self):
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)
        return self.grabpos

    def recv_objects(self, msg):
        if not self.ready: 
            return

        self.ready = False
        
        if not msg.objects:
            self.get_logger().info("No objects detected.")
            return

        objects_sorted = sorted(msg.objects, key=lambda obj: obj.pose.position.x)
        self.go_to_home()
        
        for obj in objects_sorted:
            self.visit_object(obj)
            self.go_to_home() 
        
        self.send_once = True 

    def go_to_home(self):
        home_msg1 = SplineSegment()
        home_msg1.qf = [np.pi/2, 0.0, self.actpos[2]]
        home_msg1.vf = [0.0, 0.0, 0.0]
        home_msg1.t_move = 5.0  
        self.spline_pub.publish(home_msg1)
        self.get_logger().info(f"going to intermediate home: {home_msg1.qf}")

        home_msg2 = SplineSegment()
        home_msg2.qf = self.home_position
        home_msg2.vf = [0.0, 0.0, 0.0]
        home_msg2.t_move = 3.0  
        self.spline_pub.publish(home_msg2)
        self.get_logger().info(f"going to home: {home_msg2.qf}")

    def visit_object(self, obj):
        position = [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
        # self.get_logger().info(f"Visiting Object: Type={obj.type}, Position={position}")

        if obj.type == Object.DISK:
            qf = self.cartesian_to_joint_space_newton(position)
            self.publish_spline_segment(qf, 5.0)
            self.get_logger().info(f"going to disk: {qf}")

        elif obj.type == Object.STRIP:
            theta = 2 * np.arcsin(obj.pose.orientation.z)  
            dx = np.cos(theta)
            dy = np.sin(theta)

            point1 = np.array(position) + 0.05 * np.array([-dy, dx, 0])
            point2 = np.array(position) + 0.05 * np.array([dy, -dx, 0])
            midpoint = np.array(position) + np.array([0, 0, 0.05])

            q1 = self.cartesian_to_joint_space_newton(point1)
            q2 = self.cartesian_to_joint_space_newton(point2)
            q_mid = self.cartesian_to_joint_space_newton(midpoint)

            self.publish_spline_segment(q1, 5.0)
            self.get_logger().info(f"going to strip 1: {q1}")
            self.publish_spline_segment(q_mid, 3.0)
            self.get_logger().info(f"going to strip 2: {q_mid}")
            self.publish_spline_segment(q2, 5.0)
            self.get_logger().info(f"going to strip 3: {q2}")

        # self.get_logger().info(f"Completed visiting object at {position}")

    def cartesian_to_joint_space_newton(self, pd, max_iters=100, tol=1e-5):
        # self.get_logger().info(f"entering NR")
        q = np.array(self.home_position)  
        # q = np.array(self.actpos) 
        lam = 0.1
        I = np.eye(len(q))
        
        for i in range(max_iters):
            ptip, _, Jv, _ = self.chain.fkin(q)
            error = np.array(pd) - np.array(ptip)
            
            if np.linalg.norm(error) < tol:
                ptip, _, Jv, _ = self.chain.fkin(q)
                # self.get_logger().info(f"iteration broke at {i}, ptip {ptip}")
                break
            
            J_damped_inv = np.linalg.inv(Jv.T @ Jv + lam**2 * I) @ Jv.T
            delta_q = J_damped_inv @ error
            q = q + delta_q

        q = (q + np.pi) % (2 * np.pi) - np.pi

        return list(q)

    def publish_spline_segment(self, qf, t_move):
        segment_msg = SplineSegment()
        segment_msg.qf = qf
        segment_msg.vf = [0.0, 0.0, 0.0]  
        segment_msg.t_move = t_move  
        self.spline_pub.publish(segment_msg)
        # self.get_logger().info(f"Q!!! {qf}")

    def send_halt_command(self):
        msg = ControlCommand()
        msg.command = "halt"
        self.control_pub.publish(msg)
        self.get_logger().info("Sent halt command to low-level node.")

    def send_replace_command(self, new_segments):
        msg = ControlCommand()
        msg.command = "replace"
        msg.new_segments = new_segments 
        self.control_pub.publish(msg)
        self.get_logger().info("Sent replace command with new trajectory.")


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode('brain')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
