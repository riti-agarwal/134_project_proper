# #!/usr/bin/env python3
# #
# #   touchtable.py
# #
# #   Demonstration node to interact with the HEBIs.
# #
# import numpy as np
# import rclpy

# from rclpy.node         import Node
# from sensor_msgs.msg    import JointState

# from demo134.KinematicChain import KinematicChain

# from demo134.TrajectoryUtils import *


# #
# #   Definitions
# #
# RATE = 100.0            # Hertz


# #
# #   DEMO Node Class
# #
# class DemoNode(Node):
#     # Initialization.
#     def __init__(self, name):
#         # Initialize the node, naming it as specified
#         super().__init__(name)

#         # Create a temporary subscriber to grab the initial position.
#         self.q0 = self.grabfbk()
#         self.get_logger().info("Initial positions: %r" % self.q0)

#         # Create a message and publisher to send the joint commands.
#         self.cmdmsg = JointState()
#         self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

#         # Wait for a connection to happen.  This isn't necessary, but
#         # means we don't start until the rest of the system is ready.
#         self.get_logger().info("Waiting for a /joint_commands subscriber...")
#         while(not self.count_subscribers('/joint_commands')):
#             pass

#         # Create a subscriber to continually receive joint state messages.
#         self.actpos = self.q0.copy()
#         self.fbksub = self.create_subscription(
#             JointState, '/joint_states', self.recvfbk, 10)

#         # Create a timer to keep calculating/sending commands.
#         rate           = RATE
#         self.starttime = self.get_clock().now()
#         self.timer     = self.create_timer(1/rate, self.update)
#         self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
#                                (self.timer.timer_period_ns * 1e-9, rate))
        
#         self.current_phase = "startup"
#         self.e = np.zeros(3)
#         self.lam = 20.0
#         self.x = [0.0, 0.0, 0.0]
#         self.q = self.q0

#         self.homing_time = 2.0
#         self.dt = 0.01

#         # Variables to manage spline motion non-blockingly
#         self.motion_active = False
#         self.motion_start_time = None
#         self.motion_duration = None
#         self.motion_q0 = None
#         self.motion_qf = None
        
#         self.chain = KinematicChain(self, 'world', 'tip', ['base', 'shoulder', 'elbow'])

#         #TODO: change name, find coordinates of waiting position. 
#         self.waiting_pos, _, _, _ = self.chain.fkin(self.q)

    
#     def quintic_spline(self, q0, qT, T, t):
#         """
#         Compute the position, velocity, and acceleration using a quintic spline.
#         - param q0: Vector of initial positions
#         - param qT: Vector of final positions
#         - param T:  Total time for the motion
#         - param t:  Current time
#         - return: Vector: positions, Vector: velocities, Vector: accelerations 
#         """
#         positions, velocities, accelerations = [], [], []
    
#         for i in range(len(q0)):
#             a0 = q0[i]
#             a1 = 0.0
#             a2 = 0.0
#             a3 = (20 * (qT[i] - q0[i]) - (8 * 0.0 + 12 * 0.0) * T - (3 * 0.0 - 0.0) * T**2) / (2 * T**3)
#             a4 = (-30 * (qT[i] - q0[i]) + (14 * 0.0 + 16 * 0.0) * T + (3 * 0.0 - 2 * 0.0) * T**2) / (2 * T**4)
#             a5 = (12 * (qT[i] - q0[i]) - (6 * 0.0 + 6 * 0.0) * T - (0.0 - 0.0) * T**2) / (2 * T**5)
    
#             position = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
#             velocity = a1 + 2 * a2 * t + 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4
#             acceleration = 2 * a2 + 6 * a3 * t + 12 * a4 * t**2 + 20 * a5 * t**3
    
#             positions.append(position)
#             velocities.append(velocity)
#             accelerations.append(acceleration)
    
#         return positions, velocities, accelerations
    
#     def generate_cartesian_trajectory(self, start_pos, target_pos, duration, t):
#         """Generate a Cartesian trajectory using quintic splines."""
#         if t > duration:
#             t = duration  # Clamp time to duration

#         # Quintic spline coefficients for each Cartesian dimension
#         pd, vd, ad = [], [], []
#         for i in range(3):
#             p, v, a = self.quintic_spline(start_pos[i], target_pos[i], duration, t)
#             pd.append(p)
#             vd.append(v)
#             ad.append(a)

#         return pd, vd, ad


#     def move_with_spline(self, q_0, q_f, qdot_0, qdot_f, duration):
#         """Move between two joint positions using quintic spline."""
#         t_start = self.get_clock().now()
#         t_now = self.get_clock().now()
#         elapsed = (t_now - t_start).nanoseconds * 1e-9

#         while elapsed < duration:
#             t_now = self.get_clock().now()
#             elapsed = (t_now - t_start).nanoseconds * 1e-9
#             # TODO: Match input to spline5
#             pos, vel, _ = self.quintic_spline(q_0, q_f, duration, elapsed)
#             self.sendcmd(pos, vel)
#             rclpy.spin_once(self)
            
#         # Update last known positions - not sure whether to do here or in the update function
#         self.q = q_f


#     def move_to_waiting_state(self):
#         # TODO: find fixed waiting pos. 
#         """Move robot to the waiting state using quintic spline."""
#         current_pos = self.q  
#         # Step 1: Move upper arm vertical
#         intermediate_pos = [current_pos[0], 0.0, 0.0, 0.0, current_pos[2]]
#         self.move_with_spline(current_pos, intermediate_pos, 0.0, 0.0, 2.0)

#         # Step 2: Move pointer to default waiting position
#         waiting_pos = [np.pi/2, 0.0, np.pi / 2]  # We should change this once we know the exact position of our waiting state
#         self.move_with_spline(intermediate_pos, waiting_pos, 0.0, 0.0, 2.0)

#     def move_to_target(self, x, y, z):
#         """Move robot to a target point on the table directly using quintic spline."""
#         try:
#             now = self.get_clock().now()
#             t   = (now - self.starttime).nanoseconds * 1e-9
#             # TODO: quintic spline must go from inital position (which is postion in home state) to final target position
#             pd, vd, _ = self.generate_cartesian_trajectory(self.waiting_pos, [x, y, z], 2.0, t)
#             joint_angles = self.ikin(pd, vd)
#             # Move directly to the target - to move to target we do not need a 2 step process. 
#             self.move_with_spline(self.q, joint_angles, 0.0, 0.0, 2.0)
#         except ValueError as e:
#             self.get_logger().error(str(e))


#     # Grab a single feedback - DO NOT CALL THIS REPEATEDLY!
#     def grabfbk(self):
#         # Create a temporary handler to grab the position.
#         def cb(fbkmsg):
#             self.grabpos   = list(fbkmsg.position)
#             self.grabready = True

#         # Temporarily subscribe to get just one message.
#         sub = self.create_subscription(JointState, '/joint_states', cb, 1)
#         self.grabready = False
#         while not self.grabready:
#             rclpy.spin_once(self)
#         self.destroy_subscription(sub)

#         # Return the values.
#         return self.grabpos

#     # Send a command.
#     def sendcmd(self, pos, vel, eff = []):
#         # Build up the message and publish.
#         self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
#         self.cmdmsg.name         = ['base', 'shoulder', 'elbow']
#         self.cmdmsg.position     = pos
#         self.cmdmsg.velocity     = vel
#         self.cmdmsg.effort       = eff
#         self.cmdpub.publish(self.cmdmsg)


#     ######################################################################
#     # Handlers
#     # Receive feedback - called repeatedly by incoming messages.

#     def ikin(self, pd, vd):
#         # pd,vd come from spline:  is the path to the cartesian coordinates of where we want to go
#         dt = self.dt
#         ptip, rtip, Jv, Jw = self.chain.fkin(self.q)
#         Jpinv = np.linalg.pinv(Jv)
#         e = ptip - pd
#         qddot = Jpinv @ (vd + self.lam * e)
#         qd = self.q + qddot * dt

#         # update params
#         self.e = e
#         self.x = ptip
#         self.q = qd

#         return (qd, qddot)

#     def recvfbk(self, fbkmsg):
#         # Save the actual position.
#         self.actpos = fbkmsg.position

#     # # Timer (100Hz) update.
#     # def update(self):
#     #     # Grab the current time.
#     #     now = self.get_clock().now()
#     #     t   = (now - self.starttime).nanoseconds * 1e-9

#     #     # Compute the trajectory.
#     #     qd    = [0.0, 0.0, 0.0]
#     #     qddot = [0.0, 0.0, 0.0]
#     #     tau   = [0.0, 0.0, 0.0]

#     #     # To test the gravity compensation:
#     #     #qd    = []
#     #     #qddot = []
#     #     #tau   = [0.0, 0.0, 0.0]

#     #     # Send.
#     #     self.sendcmd(qd, qddot, tau)

#     def update(self):
#         """Main update loop."""
#         # TODO: set up a way to move between the states - decide on logic on how to move between the states.
#         now = self.get_clock().now()
#         t   = (now - self.starttime).nanoseconds * 1e-9


#         if self.current_phase == "startup":
#             self.move_to_waiting_state()
#             self.current_phase = "moving"

#         elif self.current_phase == "moving":
#             x, y, z = 0.3, 0.2, 0.0  # Target point hardcoded for now
#             self.move_to_target(x, y, z)
#             self.current_phase = "returning"

#         elif self.current_phase == "returning":
#             self.move_to_waiting_state()
#             self.current_phase = "waiting"


# #
# #   Main Code
# #
# def main(args=None):
#     # Initialize ROS.
#     rclpy.init(args=args)

#     # Instantiate the DEMO node.
#     node = DemoNode('demo')

#     # Spin the node until interrupted.
#     rclpy.spin(node)

#     # Shutdown the node and ROS.
#     node.shutdown()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3


# spasm code
# import numpy as np
# import rclpy

# from rclpy.node         import Node
# from sensor_msgs.msg    import JointState

# from demo134.KinematicChain import KinematicChain
# from demo134.TrajectoryUtils import *

# RATE = 100.0  # Hertz

# class DemoNode(Node):
#     def __init__(self, name):
#         super().__init__(name)

#         self.chain = KinematicChain(self, 'world', 'tip', ['base', 'shoulder', 'elbow'])
#         self.q0 = self.grabfbk()
#         self.get_logger().info("Initial positions: %r" % self.q0)

#         self.cmdmsg = JointState()
#         self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

#         self.get_logger().info("Waiting for a /joint_commands subscriber...")
#         while not self.count_subscribers('/joint_commands'):
#             pass

#         self.actpos = self.q0.copy()
#         self.fbksub = self.create_subscription(
#             JointState, '/joint_states', self.recvfbk, 10)

#         self.timer = self.create_timer(1/RATE, self.update)
#         self.get_logger().info(f"Sending commands with dt of {self.timer.timer_period_ns * 1e-9} seconds ({RATE}Hz)")

#         # Variables to manage spline motion non-blockingly
#         self.motion_active = False
#         self.motion_start_time = None
#         self.motion_duration = None
#         self.motion_q0 = None
#         self.motion_qf = None

#         self.current_phase = "startup"
#         self.e = np.zeros(3)
#         self.lam = 20.0
#         self.x = [0.0, 0.0, 0.0]
#         self.q = self.q0

#         self.homing_time = 2.0
#         self.dt = 1.0 / RATE
#         self.waiting_pos, _, _, _ = self.chain.fkin(self.q)

#     def quintic_spline(self, q0, qT, T, t):
#         """Compute position, velocity, acceleration for a scalar value using a quintic spline."""
#         a0 = q0
#         a1 = 0.0
#         a2 = 0.0
#         # Using boundary conditions of zero velocity and acceleration at start and end.
#         a3 = (10*(qT - q0))/(T**3)
#         a4 = (-15*(qT - q0))/(T**4)
#         a5 = (6*(qT - q0))/(T**5)

#         position = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
#         velocity = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
#         acceleration = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3
#         return position, velocity, acceleration

#     def move_with_spline_step(self, q_0, q_f, duration):
#         """Perform a single step of the spline motion non-blockingly."""
#         if not self.motion_active:
#             self.motion_active = True
#             self.motion_start_time = self.get_clock().now()
#             self.motion_duration = duration
#             self.motion_q0 = q_0
#             self.motion_qf = q_f

#         now = self.get_clock().now()
#         elapsed = (now - self.motion_start_time).nanoseconds * 1e-9

#         if elapsed > self.motion_duration:
#             # Motion complete
#             self.sendcmd(self.motion_qf, [0.0]*len(self.motion_qf))
#             self.q = self.motion_qf
#             self.motion_active = False
#             return True

#         # Compute spline for each joint incrementally
#         pos = []
#         vel = []
#         for i in range(len(q_0)):
#             p, v, _ = self.quintic_spline(q_0[i], q_f[i], duration, elapsed)
#             pos.append(p)
#             vel.append(v)

#         self.sendcmd(pos, vel)
#         return False

#     def move_to_waiting_state(self):
#         current_pos = self.q  
#         # Step 1: Move upper arm vertical
#         intermediate_pos = [current_pos[0], 0.0, 0.0]  # Adjust based on your robot's joint configuration
#         done = self.move_with_spline_step(current_pos, intermediate_pos, 2.0)
#         if done:
#             # Step 2: Move pointer to default waiting position
#             waiting_pos = [np.pi/2, 0.0, np.pi / 2]  # Adjust based on actual waiting state
#             self.move_with_spline_step(intermediate_pos, waiting_pos, 2.0)

#     def move_to_target(self, x, y, z):
#         try:
#             # Compute desired Cartesian trajectory step - this part would require non-blocking redesign
#             # For simplicity, we assume a direct inverse kinematics calculation for one step
#             pd = [x, y, z]
#             vd = [0.0, 0.0, 0.0]
#             qd, _ = self.ikin(pd, vd)
#             self.move_with_spline_step(self.q, qd, 2.0)
#         except ValueError as e:
#             self.get_logger().error(str(e))

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

#     def sendcmd(self, pos, vel, eff=[]):
#         self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
#         self.cmdmsg.name = ['base', 'shoulder', 'elbow']
#         self.cmdmsg.position = pos
#         self.cmdmsg.velocity = vel
#         self.cmdmsg.effort = eff
#         self.cmdpub.publish(self.cmdmsg)

#     def ikin(self, pd, vd):
#         dt = self.dt
#         ptip, rtip, Jv, Jw = self.chain.fkin(self.q)
#         Jpinv = np.linalg.pinv(Jv)
#         e = ptip - pd
#         qddot = Jpinv @ (vd + self.lam * e)
#         qd = self.q + qddot * dt
#         self.e = e
#         self.x = ptip
#         self.q = qd
#         return (qd, qddot)

#     def recvfbk(self, fbkmsg):
#         self.actpos = fbkmsg.position

#     # def update(self):
#         # """Main update loop using a simple state machine."""
#         # if self.current_phase == "startup":
#         #     self.move_to_waiting_state()
#         #     # Transition to next phase if waiting state reached (in practice, check if motion finished)
#         #     self.current_phase = "moving"

#         # elif self.current_phase == "moving":
#         #     # Hardcoded target; in practice, get this dynamically.
#         #     x, y, z = 0.3, 0.2, 0.0
#         #     self.move_to_target(x, y, z)
#         #     self.current_phase = "returning"

#         # elif self.current_phase == "returning":
#         #     self.move_to_waiting_state()
#         #     self.current_phase = "waiting"

#         # # For "waiting" phase, you might want to do nothing or wait for a new command.
#     def update(self):
#         if self.current_phase == "startup":
#             # Initialize waiting sequence variables once
#             if not hasattr(self, 'waiting_sequence_step'):
#                 self.waiting_sequence_step = 1
#                 self.sequence_start_pos = self.q.copy()
#                 self.sequence_intermediate = [self.q[0], 0.0, self.q[2]]  # adjust as needed
#                 self.sequence_waiting_pos = [np.pi/2, 0.0, np.pi/2]  # adjust as needed

#             # Step 1: Move upper arm vertical
#             if self.waiting_sequence_step == 1:
#                 done = self.move_with_spline_step(self.sequence_start_pos, 
#                                                 self.sequence_intermediate, 
#                                                 2.0)
#                 if done:
#                     # Proceed to next sub-phase
#                     self.waiting_sequence_step = 2
#                     self.motion_active = False  # reset for next motion
#                 return  # wait for next timer tick to continue step 1 or move to step 2

#             # Step 2: Move pointer parallel to table
#             if self.waiting_sequence_step == 2:
#                 done = self.move_with_spline_step(self.sequence_intermediate, 
#                                                 self.sequence_waiting_pos, 
#                                                 2.0)
#                 if done:
#                     # Reset sub-phase tracking for future use
#                     del self.waiting_sequence_step
#                     self.motion_active = False
#                     # Transition to next main phase
#                     self.current_phase = "moving"
#                 return

#         elif self.current_phase == "moving":
#             # Similar approach for moving phase
#             if not hasattr(self, 'target_set'):
#                 self.target_set = True
#                 self.target_x, self.target_y, self.target_z = 0.3, 0.2, 0.0
#                 self.moving_done = False

#             if not self.moving_done:
#                 self.move_to_target(self.target_x, self.target_y, self.target_z)
#                 # Check if motion completed by verifying that no motion is active
#                 if not self.motion_active:
#                     self.moving_done = True
#             else:
#                 self.current_phase = "returning"
#                 del self.target_set
#                 self.moving_done = False

#         elif self.current_phase == "returning":
#             # For returning, reuse startup logic or create a similar sequence
#             # For simplicity, transition to startup again
#             self.current_phase = "startup"


# def main(args=None):
#     rclpy.init(args=args)
#     node = DemoNode('demo')
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3


# """ Working code """

# import numpy as np
# import rclpy

# from rclpy.node import Node
# from sensor_msgs.msg import JointState

# from ikpy.chain import Chain
# import ikpy

# RATE = 100.0  # Hertz

# class DemoNode(Node):
#     def __init__(self, name):
#         # Initialize the node, naming it as specified
#         super().__init__(name)

#         # self.robot_chain = Chain.from_urdf_file("src/threedof/threedof/urdf/threedofexample.urdf", base_elements=["world"])

#         # Create a temporary subscriber to grab the initial position.
#         self.position0 = self.grabfbk()
#         self.get_logger().info("Initial positions: %r" % self.position0)

#         # Create a message and publisher to send the joint commands.
#         self.cmdmsg = JointState()
#         self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

#         # Wait for a connection to happen.  This isn't necessary, but
#         # means we don't start until the rest of the system is ready.
#         self.get_logger().info("Waiting for a /joint_commands subscriber...")
#         while(not self.count_subscribers('/joint_commands')):
#             pass

#         # Create a subscriber to continually receive joint state messages.
#         self.fbksub = self.create_subscription(
#             JointState, '/joint_states', self.recvfbk, 10)

#         # Create a timer to keep calculating/sending commands.
#         rate           = RATE
#         self.current_phase = "waiting"
#         self.last_joint_positions = self.position0
#         self.homing_time = 2.0
#         # self.homed = False
#         self.starttime = self.get_clock().now()
#         self.timer     = self.create_timer(1/rate, self.update)
#         self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
#                                (self.timer.timer_period_ns * 1e-9, rate))
#         self.q_current = self.position0

#     def quintic_spline(self, q0, qT, T, t):
#         """Compute position and velocity for a scalar value using a quintic spline."""
#         a0 = q0
#         a1 = 0.0
#         a2 = 0.0
#         a3 = (10*(qT - q0)) / (T**3)
#         a4 = (-15*(qT - q0)) / (T**4)
#         a5 = (6*(qT - q0)) / (T**5)

#         position = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
#         velocity = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
#         return position, velocity
    
#     def grabfbk(self):
#         # Create a temporary handler to grab the position.
#         def cb(fbkmsg):
#             self.grabpos   = list(fbkmsg.position)
#             self.grabready = True

#         # Temporarily subscribe to get just one message.
#         sub = self.create_subscription(JointState, '/joint_states', cb, 1)
#         self.grabready = False
#         while not self.grabready:
#             rclpy.spin_once(self)
#         self.destroy_subscription(sub)

#         # Return the values.
#         return self.grabpos
    
#     def recvfbk(self, fbkmsg):
#         # Save the actual position.
#         self.actpos = fbkmsg.position

#     def cartesian_to_joint_space(self, x, y, z):
#         # Link lengths (assumed from URDF for simplification)
#         L1 = 0.385  # shoulder to elbow length
#         L2 = 0.37   # elbow to tip length

#         # Compute the base rotation angle (q1) about the z-axis
#         q1 = np.arctan2(y, x)

#         # Project the target onto the plane defined by the shoulder and elbow after base rotation
#         r = np.sqrt(x**2 + y**2)  # horizontal distance from base axis to target

#         # Using planar 2R arm inverse kinematics in the plane defined by (r, z)
#         # Compute intermediate value for elbow angle calculation using the law of cosines
#         D = (r**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)

#         # Check if the target is reachable
#         if abs(D) > 1:
#             raise ValueError("Target is unreachable with the given arm configuration.")

#         # Elbow joint angle (q3) solution (elbow-up configuration assumed)
#         q3 = np.arccos(D)

#         # Compute the angle from the horizontal to the line connecting shoulder to target
#         phi = np.arctan2(z, r)

#         # Angle between link L1 and the line from shoulder to target
#         psi = np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3))

#         # Shoulder joint angle (q2)
#         q2 = phi - psi

#         return [q1, q2, q3]

#     def move_with_spline(self, q_start, q_goal, duration):
#         """Moves joints from q_start to q_goal over the given duration using a quintic spline."""
#         start_time = self.get_clock().now()
#         rate = self.create_rate(RATE)

#         while rclpy.ok():
#             # Calculate elapsed time
#             now = self.get_clock().now()
#             elapsed = (now - start_time).nanoseconds * 1e-9

#             # Clamp elapsed time to duration
#             t = min(elapsed, duration)

#             # Compute new positions and velocities for each joint
#             positions = []
#             velocities = []
#             for q0, qT in zip(q_start, q_goal):
#                 p, v = self.quintic_spline(q0, qT, duration, t)
#                 positions.append(p)
#                 velocities.append(v)

#             # Send command
#             self.sendcmd(positions, velocities)

#             # Break loop if motion complete
#             if t >= duration:
#                 self.sendcmd(positions, [0.0]*len(velocities))
#                 self.q_current = q_goal  # Update current joint state
#                 break

#     def sendcmd(self, positions, velocities):
#         """Sends joint commands."""
#         self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
#         self.cmdmsg.name = ['base', 'shoulder', 'elbow']  # Adjust joint names if necessary
#         self.cmdmsg.position = positions
#         self.cmdmsg.velocity = velocities
#         self.cmdpub.publish(self.cmdmsg)

#     def move_to_waiting_position(self):
#         """Directly move robot joints to the waiting position using a spline."""
#         duration = 2.0
#         intermediate_pos = [self.q_current[0], 0.0, self.q_current[2]]
#         self.move_with_spline(self.q_current, intermediate_pos, duration)
#         waiting_position = [np.pi/2, 0.0, np.pi/2]  
#         self.move_with_spline(self.q_current, waiting_position, duration)
#         self.get_logger().info("Reached waiting position.")

#     def update(self):
#         if self.current_phase == "waiting":
#             self.move_to_waiting_position()
#             self.current_phase = "moving"

#         elif self.current_phase == "moving":
#             x, y, z = 0.3, 0.2, 0.0  # Target point hardcoded for now
#             q_target = self.cartesian_to_joint_space(x, y, z)
#             # self.move_with_spline(self.q_current, [np.pi, np.pi/2, 0.0], 2.0)
#             self.move_with_spline(self.q_current, q_target, 2.0)
#             self.current_phase = "returning"

#         elif self.current_phase == "returning":
#             self.move_to_waiting_position()
#             self.current_phase = "waiting"

# def main(args=None):
#     rclpy.init(args=args)
#     node = DemoNode('demo')
#     try:
#         rclpy.spin(node)  # Keeps spinning until shutdown is requested
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


RATE = 100.0  # Hertz

class DemoNode(Node):
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # self.robot_chain = Chain.from_urdf_file("src/threedof/threedof/urdf/threedofexample.urdf", base_elements=["world"])
        self.point_queue = []

        self.pointsub = self.create_subscription(
            Point, '/point', self.recvpoint, 10)
        
        self.get_logger().info("Running %s" % name)
        self.current_target = None
        # Create a temporary subscriber to grab the initial position.
        self.position0 = self.grabfbk()
        self.get_logger().info("Initial positions: %r" % self.position0)

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)

        # Create a timer to keep calculating/sending commands.
        rate           = RATE
        self.current_phase = "startup"
        self.last_joint_positions = self.position0
        self.homing_time = 2.0
        # self.homed = False
        self.starttime = self.get_clock().now()
        self.timer     = self.create_timer(1/rate, self.update)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))
        self.q_current = self.position0

    def quintic_spline(self, q0, qT, T, t):
        """Compute position and velocity for a scalar value using a quintic spline."""
        a0 = q0
        a1 = 0.0
        a2 = 0.0
        a3 = (10*(qT - q0)) / (T**3)
        a4 = (-15*(qT - q0)) / (T**4)
        a5 = (6*(qT - q0)) / (T**5)

        position = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
        velocity = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
        return position, velocity
    
    def recvpoint(self, pointmsg):
        # Extract coordinates from message and enqueue them
        self.get_logger().info("Received a message on /point")
        x = pointmsg.x
        y = pointmsg.y
        z = pointmsg.z
        point = (x, y, z)
        self.point_queue.append(point)
        self.get_logger().info(f"Received point: {point}")

    
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return self.grabpos
    
    def recvfbk(self, fbkmsg):
        # Save the actual position.
        self.actpos = fbkmsg.position

    def cartesian_to_joint_space(self, x, y, z):
        # Link lengths (assumed from URDF for simplification)
        L1 = 0.385  # shoulder to elbow length
        L2 = 0.37   # elbow to tip length

        # Compute the base rotation angle (q1) about the z-axis
        q1 = np.arctan2(y, x)

        # Project the target onto the plane defined by the shoulder and elbow after base rotation
        r = np.sqrt(x**2 + y**2)  # horizontal distance from base axis to target

        # Using planar 2R arm inverse kinematics in the plane defined by (r, z)
        # Compute intermediate value for elbow angle calculation using the law of cosines
        D = (r**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)

        # Check if the target is reachable
        if abs(D) > 1:
            raise ValueError("Target is unreachable with the given arm configuration.")

        # Elbow joint angle (q3) solution (elbow-up configuration assumed)
        q3 = np.arccos(D)

        # Compute the angle from the horizontal to the line connecting shoulder to target
        phi = np.arctan2(z, r)

        # Angle between link L1 and the line from shoulder to target
        psi = np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3))

        # Shoulder joint angle (q2)
        q2 = phi - psi

        return [q1, q2, q3]

    def move_with_spline(self, q_start, q_goal, duration):
        """Moves joints from q_start to q_goal over the given duration using a quintic spline."""
        start_time = self.get_clock().now()
        rate = self.create_rate(RATE)

        while rclpy.ok():
            # Calculate elapsed time
            now = self.get_clock().now()
            elapsed = (now - start_time).nanoseconds * 1e-9

            # Clamp elapsed time to duration
            t = min(elapsed, duration)

            # Compute new positions and velocities for each joint
            positions = []
            velocities = []
            for q0, qT in zip(q_start, q_goal):
                p, v = self.quintic_spline(q0, qT, duration, t)
                positions.append(p)
                velocities.append(v)

            # Send command
            self.sendcmd(positions, velocities)

            # Break loop if motion complete
            if t >= duration:
                self.sendcmd(positions, [0.0]*len(velocities))
                self.q_current = q_goal  # Update current joint state
                break

    def sendcmd(self, positions, velocities):
        """Sends joint commands."""
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name = ['base', 'shoulder', 'elbow']  # Adjust joint names if necessary
        self.cmdmsg.position = positions
        self.cmdmsg.velocity = velocities
        self.cmdpub.publish(self.cmdmsg)

    def move_to_waiting_position(self):
        """Directly move robot joints to the waiting position using a spline."""
        duration = 2.0
        intermediate_pos = [self.q_current[0], 0.0, self.q_current[2]]
        self.move_with_spline(self.q_current, intermediate_pos, duration)
        waiting_position = [np.pi/2, 0.0, np.pi/2]  
        self.move_with_spline(self.q_current, waiting_position, duration)
        self.get_logger().info("Reached waiting position.")

    def update(self):
        if self.current_phase == "startup":
            self.move_to_waiting_position()
            self.current_phase = "waiting"
                    
        elif self.current_phase == "waiting":
            # Only start moving if there is a new point in the queue
            if self.point_queue:
                # Dequeue next point and transition to moving phase
                self.current_target = self.point_queue.pop(0)
                self.current_phase = "moving"
            # If no new points, remain in waiting phase without action

        elif self.current_phase == "moving":
            # x, y, z = 0.3, 0.2, 0.0  # Target point hardcoded for now
            # q_target = self.cartesian_to_joint_space(x, y, z)
            # # self.move_with_spline(self.q_current, [np.pi, np.pi/2, 0.0], 2.0)
            # self.move_with_spline(self.q_current, q_target, 2.0)
            # self.current_phase = "returning"
            x, y, z = self.current_target
            try:
                q_target = self.cartesian_to_joint_space(x, y, z)
            except ValueError:
                # Skip unreachable point and return to waiting phase
                self.get_logger().info("Target unreachable, skipping this point.")
                self.current_phase = "waiting"
                return

            self.move_with_spline(self.q_current, q_target, 2.0)
            self.current_phase = "returning"

        elif self.current_phase == "returning":
            self.move_to_waiting_position()
            self.current_phase = "waiting"

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode('demo')
    try:
        rclpy.spin(node)  # Keeps spinning until shutdown is requested
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


