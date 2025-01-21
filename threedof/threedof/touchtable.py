# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Point
# from std_msgs.msg import Float64
# import math

# # KinematicChain import
# from threedof.KinematicChain import KinematicChain
# from threedof.TrajectoryUtils import *  # assumed your goto5 lives here

# RATE = 100.0  # Hertz
# DT   = 1.0 / RATE

# class DemoNode(Node):
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

#         self.lam = 20.0
#         self.e = np.zeros(3)  # IK error

#         self.A = -1.815
#         self.B = 0.0

#         self.subA = self.create_subscription(Float64, '/paramA', self.cb_A, 1)
#         self.subB = self.create_subscription(Float64, '/paramB', self.cb_B, 1)

#         self.test_gravity_mode = False

#         # For storing user-requested Cartesian points
#         self.point_queue = []
#         self.pointsub = self.create_subscription(Point, '/point', self.recvpoint, 10)
#         self.current_target = None

#         # Grab initial position from feedback
#         self.position0 = self.grabfbk()
#         self.get_logger().info("Initial positions: %r" % (self.position0,))

#         # Prepare a JointState message for publishing commands
#         self.cmdmsg = JointState()
#         self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

#         self.get_logger().info("Waiting for a /joint_commands subscriber...")
#         while not self.count_subscribers('/joint_commands'):
#             pass

#         # Continual feedback subscription
#         self.fbksub = self.create_subscription(JointState, '/joint_states', self.recvfbk, 10)

#         # Control-related variables
#         self.current_phase = "startup"
#         self.q_current     = self.position0
#         self.starttime     = self.get_clock().now()
#         self.moving_start_time = None

#         # Create a timer for the main control loop
#         self.timer = self.create_timer(DT, self.update)
#         self.get_logger().info("Sending commands at %f Hz" % RATE)

#         # For IK tracking
#         ptip, Rtip, Jv, Jw = self.chain.fkin(self.q_current)
#         self.x = np.array(ptip)  # current Cartesian tip position

#         # Variables used by various sub-phase/trajectories
#         self.cmd_positions  = [0.0, 0.0, 0.0]
#         self.cmd_velocities = [0.0, 0.0, 0.0]
#         self.cmd_effot = [0.0, 0.0, 0.0]

#         # Variables to manage a joint-spline (e.g. to waiting position)
#         self.joint_traj_active  = False
#         self.joint_traj_start   = None
#         self.joint_traj_end     = None
#         self.joint_traj_start_t = None
#         self.joint_traj_duration = 0.0

#         # Variables to manage a Cartesian-spline (IK)
#         self.cart_traj = []
#         self.cart_traj_start_t = None

#         # We might also store “intermediate” steps if doing a 2-step move to waiting.
#         # For simplicity, we’ll do a single-step direct approach in this example.

#     # ----------------------
#     # Subscription callbacks
#     # ----------------------
#     def recvpoint(self, pointmsg):
#         """Receive a Cartesian target point and enqueue it."""
#         x = pointmsg.x
#         y = pointmsg.y
#         z = pointmsg.z
#         point = (x, y, z)
#         self.point_queue.append(point)
#         self.get_logger().info(f"Received point: {point}")

#     def grabfbk(self):
#         """
#         Grab one set of feedback from the '/joint_states' topic (blocking).
#         Used at startup to get the initial position.
#         """
#         def cb(fbkmsg):
#             self.grabpos   = list(fbkmsg.position)
#             self.grabready = True

#         sub = self.create_subscription(JointState, '/joint_states', cb, 1)
#         self.grabready = False
#         while not self.grabready:
#             rclpy.spin_once(self)
#         self.destroy_subscription(sub)

#         return self.grabpos

#     def gravity(self, q):
#         shoulder = q[1]
#         tau_shoulder = self.A*math.sin(shoulder) + self.B*math.cos(shoulder)
#         return [0.0, tau_shoulder, 0.0]

#     def recvfbk(self, fbkmsg):
#         """Callback to continually receive feedback from '/joint_states'."""
#         self.actpos = list(fbkmsg.position)

#     def cb_A(self, msg):
#         self.A = msg.data
#         self.get_logger().info(f"Updated A to {self.A}")

#     def cb_B(self, msg):
#         self.B = msg.data
#         self.get_logger().info(f"Updated B to {self.B}")


#     def sendcmd(self, positions, velocities, effort=None):
#         """Sends joint commands to the hardware/robot simulator."""
#         self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
#         self.cmdmsg.name     = ['base', 'shoulder', 'elbow']
#         self.cmdmsg.position = positions
#         self.cmdmsg.velocity = velocities
#         if effort != None:
#             self.cmdmsg.effort = effort
#         self.cmdpub.publish(self.cmdmsg)

#     # ----------------------
#     # Helper functions
#     # ----------------------
#     def quintic_spline(self, q0, qT, T, t):
#         """
#         Compute array of positions and velocities for each DOF 
#         using a quintic polynomial between q0 and qT. 
#         q0, qT are arrays of the same length.
#         """
#         q0 = np.array(q0)
#         qT = np.array(qT)
#         positions  = []
#         velocities = []
#         for i in range(len(q0)):
#             a0 = q0[i]
#             a1 = 0.0
#             a2 = 0.0
#             a3 =  (10*(qT[i] - q0[i])) / (T**3)
#             a4 = (-15*(qT[i] - q0[i])) / (T**4)
#             a5 =   6*(qT[i] - q0[i])  / (T**5)

#             p = a0 + a1*t + a2*(t**2) + a3*(t**3) + a4*(t**4) + a5*(t**5)
#             v = a1 + 2*a2*t + 3*a3*(t**2) + 4*a4*(t**3) + 5*a5*(t**4)
#             positions.append(p)
#             velocities.append(v)

#         return positions, velocities

#     def cartesian_to_joint_space_ik(self, x, y, z, pd, vd, t):
#         """
#         Perform one incremental IK step at time t (0 ≤ t ≤ total_time).
#         - x, y, z is the final (static) desired point (just used for reference).
#         - pd, vd come from a 5th-order interpolation in Cartesian space.
#         """
#         dt = DT
#         q  = np.array(self.q_current)
#         ptip, Rtip, Jv, Jw = self.chain.fkin(q)
#         ptip = np.array(ptip)
#         J    = Jv
#         e    = pd - ptip   # Position error
#         Jpinv = np.linalg.pinv(J)
#         qdot  = Jpinv @ (vd + self.lam * e)

#         q_new = q + qdot * dt
#         # Update internal knowledge of current
#         self.q_current = q_new
#         self.x         = ptip
#         self.e         = e

#         # Return next position/velocity commands for the joints
#         positions = list(q_new)
#         velocities = list([0.0]*len(q_new))  # we can keep zero or use qdot
#         return positions, velocities

#     def build_trajectory(self, x0, xF, total_time=2.0, rate=100.0):
#         """
#         Precompute the entire spline in Cartesian space.
#         Returns a list of (pd, vd) from t=0 to t=2.0 in time increments of 1/rate.
#         """
#         dt = 1.0 / rate
#         steps = int(total_time * rate)

#         traj = []
#         for i in range(steps+1):
#             t = i * dt
#             if t > total_time:
#                 t = total_time
#             pd, vd = goto5(t, total_time, x0, xF)
#             traj.append( (pd, vd) )
        
#         return traj

#     # ---------------------------
#     # Non-blocking "initiate" methods
#     # ---------------------------
#     def initiate_move_to_waiting_position(self):
#         """
#         Example function to start a joint-space spline to a known waiting pose:
#           [pi/2, 0, pi/2]
#         We'll do a single move from the current position to that waiting pose.
#         """
#         self.joint_traj_active = True
#         self.joint_traj_start   = self.q_current[:]  # current
#         self.joint_traj_end     = [np.pi/2, 0.0, np.pi/2]
#         self.joint_traj_start_t = self.get_clock().now()
#         self.joint_traj_duration = 3.0  # seconds

#         # We’ll switch our phase so the update() method picks it up
#         self.current_phase = "move_to_waiting"

#     def initiate_move_joint_spline(self, q_start, q_goal, duration):
#         """
#         A general method to start a joint-space move from q_start to q_goal.
#         """
#         self.joint_traj_active = True
#         self.joint_traj_start   = q_start[:]
#         self.joint_traj_end     = q_goal[:]
#         self.joint_traj_start_t = self.get_clock().now()
#         self.joint_traj_duration = duration
#         # We can track a separate “phase” if needed, or reuse the same one.

#     def initiate_cartesian_move(self, target_point, duration=2.0):
#         """
#         Start a cartesian (IK-based) move from the current tip position 
#         to the 'target_point'.
#         """
#         ptip, Rtip, Jv, Jw = self.chain.fkin(self.q_current)
#         x0 = np.array(ptip)
#         xF = np.array(target_point)

#         self.cart_traj = self.build_trajectory(x0, xF, total_time=duration, rate=RATE)
#         self.cart_traj_start_t = self.get_clock().now()
#         self.current_phase = "moving"

#     # ---------------------------
#     # The main update loop
#     # ---------------------------
#     def update(self):
#         """
#         This method is called at RATE = 100 Hz. 
#         All command decisions are made here. At the end, we send out the next command.
#         """
#         if self.test_gravity_mode:
#             # Gravity only mode: publish torque commands that (hopefully) hold the arm
#             tau = self.gravity(self.actpos)
#             print(tau)
#             self.sendcmd([], [], tau)
#             return

#         # Default to holding current position
#         self.cmd_positions  = self.q_current if isinstance(self.q_current, list) else list(self.q_current)
#         self.cmd_velocities = [0.0]*len(self.cmd_positions)

#         # State machine approach
#         if self.current_phase == "startup":
#             # On first pass, we go to the waiting position
#             self.initiate_move_to_waiting_position()

#         elif self.current_phase == "move_to_waiting":
#             # We are moving along a joint-space spline to the waiting pose.
#             now     = self.get_clock().now()
#             elapsed = (now - self.joint_traj_start_t).nanoseconds * 1e-9
#             t       = min(elapsed, self.joint_traj_duration)

#             q0  = self.joint_traj_start
#             qT  = self.joint_traj_end
#             dur = self.joint_traj_duration

#             # Compute next step in joint space
#             positions, velocities = self.quintic_spline(q0, qT, dur, t)
#             self.cmd_positions  = positions
#             self.cmd_velocities = velocities

#             # Update internal q_current
#             self.q_current = positions

#             if t >= dur:
#                 # Done with move
#                 self.joint_traj_active = False
#                 self.current_phase = "waiting"

#         elif self.current_phase == "waiting":
#             # If we have a point, we begin a cartesian IK-based move
#             if self.point_queue:
#                 self.current_target = self.point_queue.pop(0)
#                 self.initiate_cartesian_move(self.current_target, duration=2.0)

#         elif self.current_phase == "moving":
#             # We are following a precomputed cartesian spline
#             now     = self.get_clock().now()
#             elapsed = (now - self.cart_traj_start_t).nanoseconds * 1e-9
#             idx = int(elapsed * RATE)

#             if idx < len(self.cart_traj):
#                 pd, vd = self.cart_traj[idx]
#                 # We use the incremental IK step
#                 xF = self.current_target  # final is just for reference
#                 t  = idx * (1.0 / RATE)
#                 positions, velocities = self.cartesian_to_joint_space_ik(xF[0], xF[1], xF[2], pd, vd, t)
#                 self.cmd_positions  = positions
#                 self.cmd_velocities = velocities
#             else:
#                 # Done with cartesian move, now we go back to waiting
#                 self.current_phase = "returning"

#         elif self.current_phase == "returning":
#             # Initiate a joint-space move back to waiting position
#             # then revert to "waiting" once completed
#             self.initiate_move_to_waiting_position()
#             # Switch the phase right away (the next iteration will actually do the spline)
#             # Because initiate_move_to_waiting_position() sets current_phase = "move_to_waiting"

#         # Finally, send out the command we decided on
#         self.effort = self.gravity(self.actpos)
#         self.sendcmd(self.cmd_positions, self.cmd_velocities, self.effort)

#         # self.sendcmd(self.cmd_positions, self.cmd_velocities)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DemoNode('demo')
#     try:
#         rclpy.spin(node)  # spin until shutdown
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
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import math

from threedof.KinematicChain import KinematicChain
from threedof.TrajectoryUtils import *

RATE = 100.0  
DT   = 1.0 / RATE

class DemoNode(Node):
    def __init__(self, name):
        super().__init__(name)

        base_frame = "world"
        tip_frame  = "tip"
        expected_joint_names = ["base", "shoulder", "elbow"]
        self.chain = KinematicChain(
            node=self,
            baseframe=base_frame,
            tipframe=tip_frame,
            expectedjointnames=expected_joint_names
        )

        self.lam = 20.0
        self.e = np.zeros(3)  

        self.lam = 20.0
        self.e = np.zeros(3)  # IK error

        self.A = -1.815
        self.B = 0.0

        self.subA = self.create_subscription(Float64, '/paramA', self.cb_A, 1)
        self.subB = self.create_subscription(Float64, '/paramB', self.cb_B, 1)

        self.test_gravity_mode = False

        self.point_queue = []
        self.pointsub = self.create_subscription(Point, '/point', self.recvpoint, 10)
        self.current_target = None

        # Grab initial position
        self.position0 = self.grabfbk()
        self.actpos = self.position0.copy()
        self.get_logger().info(f"Initial positions: {self.position0}")

        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while not self.count_subscribers('/joint_commands'):
            pass

        self.fbksub = self.create_subscription(JointState, '/joint_states', self.recvfbk, 10)

        self.current_phase = "startup"
        self.q_current = self.position0
        self.timer = self.create_timer(DT, self.update)
        self.get_logger().info(f"Sending commands at {RATE} Hz")

        ptip, Rtip, Jv, Jw = self.chain.fkin(self.q_current)
        self.x = np.array(ptip)

        self.cmd_positions  = list(self.q_current)
        self.cmd_velocities = [0.0]*len(self.cmd_positions)
        self.cmd_effort = [0.0]*len(self.cmd_positions)

        self.effort = self.gravity(self.actpos)

        # Joint-based spline variables
        self.joint_traj_active  = False
        self.joint_traj_start   = None
        self.joint_traj_end     = None
        self.joint_traj_start_t = None
        self.joint_traj_duration = 0.0
        self.waiting_sub_phase = None

        # Cartesian trajectory
        self.cart_traj = []
        self.cart_traj_start_t = None

        # --------------------
        # NEW: Torque-based contact detection only
        # --------------------
        self.contact_threshold_effort = 1.5 # e.g. 2 Nm
        self.contact_hold_start_time = None
        self.contact_hold_duration   = 1.0  # 1 second hold
        self.contact_hold_position   = None

    # ---------------------------------------------------------
    # Feedback & subscription callbacks
    # ---------------------------------------------------------
    def recvpoint(self, msg):
        self.point_queue.append((msg.x, msg.y, msg.z))
        self.get_logger().info(f"Received point: {(msg.x, msg.y, msg.z)}")

    def gravity(self, q):
        shoulder = q[1]
        tau_shoulder = self.A*math.sin(shoulder) + self.B*math.cos(shoulder)
        return [0.0, tau_shoulder, 0.0]

    def cb_A(self, msg):
        self.A = msg.data
        self.get_logger().info(f"Updated A to {self.A}")

    def cb_B(self, msg):
        self.B = msg.data
        self.get_logger().info(f"Updated B to {self.B}")

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

    def recvfbk(self, fbkmsg):
        self.actpos = list(fbkmsg.position)
        self.actvel = list(fbkmsg.velocity)
        self.acteff = list(fbkmsg.effort)

    # ---------------------------------------------------------
    # Publishing commands
    # ---------------------------------------------------------
    def sendcmd(self, positions, velocities, effort=None):
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name = ['base', 'shoulder', 'elbow']
        self.cmdmsg.position = positions
        self.cmdmsg.velocity = velocities
        if effort != None:
            self.cmdmsg.effort = effort
        self.cmdpub.publish(self.cmdmsg)

    # ---------------------------------------------------------
    # Helper motion functions (no direct sendcmd)
    # ---------------------------------------------------------
    def quintic_spline(self, q0, qT, T, t):
        q0 = np.array(q0)
        qT = np.array(qT)
        positions, velocities = [], []
        for i in range(len(q0)):
            a0 = q0[i]
            a1 = 0.0
            a2 = 0.0
            a3 =  (10*(qT[i] - q0[i]))/(T**3)
            a4 = (-15*(qT[i] - q0[i]))/(T**4)
            a5 =   6*(qT[i] - q0[i]) /(T**5)

            p = a0 + a1*t + a2*(t**2) + a3*(t**3) + a4*(t**4) + a5*(t**5)
            v = a1 + 2*a2*t + 3*a3*(t**2) + 4*a4*(t**3) + 5*a5*(t**4)
            positions.append(p)
            velocities.append(v)
        return positions, velocities

    def cartesian_to_joint_space_ik(self, x, y, z, pd, vd, t):
        dt = DT
        q  = np.array(self.q_current)
        ptip, Rtip, Jv, Jw = self.chain.fkin(q)
        e  = pd - np.array(ptip)
        Jpinv = np.linalg.pinv(Jv)
        qdot  = Jpinv @ (vd + self.lam * e)

        q_new = q + qdot * dt
        self.q_current = q_new
        self.x         = ptip
        self.e         = e

        positions  = list(q_new)
        velocities = [0.0]*len(q_new)
        return positions, velocities

    def build_trajectory(self, x0, xF, total_time=2.0, rate=100.0):
        dt = 1.0 / rate
        steps = int(total_time * rate)
        traj = []
        for i in range(steps+1):
            t = i*dt
            if t > total_time:
                t = total_time
            pd, vd = goto5(t, total_time, x0, xF)
            traj.append((pd, vd))
        return traj

    # ---------------------------------------------------------
    # Initiate moves
    # ---------------------------------------------------------
    def initiate_move_to_waiting_position_2step(self):
        self.current_phase = "move_to_waiting_2step"
        self.waiting_sub_phase = "step1"

        self.joint_traj_start = self.q_current[:]
        inter_shoulder = 0.0
        self.joint_traj_end = [
            self.q_current[0],
            inter_shoulder,
            self.q_current[2]
        ]
        self.joint_traj_duration = 2.0
        self.joint_traj_start_t  = self.get_clock().now()

    def initiate_cartesian_move(self, target_point, duration=2.0):
        ptip, Rtip, Jv, Jw = self.chain.fkin(self.q_current)
        x0 = np.array(ptip)
        xF = np.array(target_point)
        self.cart_traj = self.build_trajectory(x0, xF, total_time=duration, rate=RATE)
        self.cart_traj_start_t = self.get_clock().now()
        self.current_phase = "moving"

    # ---------------------------------------------------------
    # Torque-only contact detection
    # ---------------------------------------------------------
    def detect_contact_torque_only(self):
        """Return True if any actual torque exceeds the threshold."""
        # If we haven't received feedback yet, skip
        if not hasattr(self, 'acteff'):
            return False

        for i, eff in enumerate(self.effort):
            if abs(self.acteff[i] - eff) > self.contact_threshold_effort:
                self.get_logger().warn(
                f"[Contact Detect] Max torque {self.acteff[i]:.3f} exceeds {self.contact_threshold_effort:.3f}"
            )
                return True
        return False
                

    def handle_contact_event(self):
        """Immediately freeze the robot for 1s, then return to waiting."""
        self.get_logger().warn("[Contact] Blocking detected! Freezing now.")
        self.contact_hold_position = self.actpos[:]  # freeze where it is
        self.contact_hold_start_time = self.get_clock().now()
        self.current_phase = "contact_hold"

    # ---------------------------------------------------------
    # Main update loop
    # ---------------------------------------------------------
    def update(self):
        # Default = hold current
        if isinstance(self.q_current, list):
            self.cmd_positions = self.q_current[:]
        else:
            self.cmd_positions = list(self.q_current)
        self.cmd_velocities = [0.0]*len(self.cmd_positions)

        # If we're not already in a contact phase, check for contact
        if self.current_phase not in ("contact_hold", "contact_return"):
            if self.detect_contact_torque_only():
                self.handle_contact_event()

        # State machine
        if self.current_phase == "startup":
            self.initiate_move_to_waiting_position_2step()

        elif self.current_phase == "move_to_waiting_2step":
            now = self.get_clock().now()
            elapsed = (now - self.joint_traj_start_t).nanoseconds * 1e-9
            t = min(elapsed, self.joint_traj_duration)

            q0 = self.joint_traj_start
            qT = self.joint_traj_end
            dur = self.joint_traj_duration

            positions, velocities = self.quintic_spline(q0, qT, dur, t)
            self.cmd_positions  = positions
            self.cmd_velocities = velocities
            self.q_current      = positions

            if t >= dur:
                if self.waiting_sub_phase == "step1":
                    # Step 2
                    self.waiting_sub_phase = "step2"
                    self.joint_traj_start = self.q_current[:]
                    self.joint_traj_end   = [np.pi/2, 0.0, np.pi/2]
                    self.joint_traj_start_t = now
                    self.joint_traj_duration = 2.0
                elif self.waiting_sub_phase == "step2":
                    self.current_phase = "waiting"
                    self.waiting_sub_phase = None

        elif self.current_phase == "waiting":
            if self.point_queue:
                self.current_target = self.point_queue.pop(0)
                self.initiate_cartesian_move(self.current_target, duration=2.0)

        elif self.current_phase == "moving":
            now     = self.get_clock().now()
            elapsed = (now - self.cart_traj_start_t).nanoseconds * 1e-9
            idx = int(elapsed * RATE)

            if idx < len(self.cart_traj):
                pd, vd = self.cart_traj[idx]
                xF = self.current_target
                t = idx * (1.0 / RATE)
                positions, velocities = self.cartesian_to_joint_space_ik(xF[0], xF[1], xF[2], pd, vd, t)
                self.cmd_positions  = positions
                self.cmd_velocities = velocities
            else:
                self.current_phase = "returning"

        elif self.current_phase == "returning":
            self.initiate_move_to_waiting_position_2step()

        # ----------------
        # Contact phases
        # ----------------
        elif self.current_phase == "contact_hold":
            now     = self.get_clock().now()
            elapsed = (now - self.contact_hold_start_time).nanoseconds * 1e-9

            # Freeze in place
            self.cmd_positions  = self.contact_hold_position
            self.cmd_velocities = [0.0]*len(self.cmd_positions)
            self.q_current      = self.contact_hold_position

            # After 1s, go back
            if elapsed >= self.contact_hold_duration:
                self.current_phase = "contact_return"
                self.joint_traj_start   = self.q_current[:]
                self.joint_traj_end     = [np.pi/2, 0.0, np.pi/2]
                self.joint_traj_start_t = now
                self.joint_traj_duration = 2.0

        elif self.current_phase == "contact_return":
            now     = self.get_clock().now()
            elapsed = (now - self.joint_traj_start_t).nanoseconds * 1e-9
            t       = min(elapsed, self.joint_traj_duration)

            q0  = self.joint_traj_start
            qT  = self.joint_traj_end
            dur = self.joint_traj_duration

            positions, velocities = self.quintic_spline(q0, qT, dur, t)
            self.cmd_positions  = positions
            self.cmd_velocities = velocities
            self.q_current      = positions

            if t >= dur:
                self.current_phase = "waiting"

        # Finally, publish
        # self.sendcmd(self.cmd_positions, self.cmd_velocities)
        self.effort = self.gravity(self.actpos)
        self.sendcmd(self.cmd_positions, self.cmd_velocities, self.effort)

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode('demo')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()








