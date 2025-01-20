import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

# KinematicChain import
from threedof.KinematicChain import KinematicChain
from threedof.TrajectoryUtils import *  # assumed your goto5 lives here

RATE = 100.0  # Hertz
DT   = 1.0/RATE

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

        self.point_queue = []
        self.pointsub = self.create_subscription(
            Point, '/point', self.recvpoint, 10)

        self.current_target = None

        self.position0 = self.grabfbk()
        self.get_logger().info("Initial positions: %r" % self.position0)

        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while not self.count_subscribers('/joint_commands'):
            pass

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)

        # Set up our timer for the main control loop
        self.current_phase = "startup"
        self.q_current     = self.position0
        self.starttime     = self.get_clock().now()

        self.moving_start_time = None

        self.timer = self.create_timer(DT, self.update)
        self.get_logger().info("Sending commands at %f Hz" % RATE)
        ptip, Rtip, Jv, Jw = self.chain.fkin(self.q_current)
        self.x = np.array(ptip)  

    def recvpoint(self, pointmsg):
        """Receive a Cartesian target point and enqueue it."""
        x = pointmsg.x
        y = pointmsg.y
        z = pointmsg.z
        point = (x, y, z)
        self.point_queue.append(point)
        self.get_logger().info(f"Received point: {point}")

    def grabfbk(self):
        """Grab one set of feedback from the '/joint_states' topic (blocking)."""
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
        """Callback to continually receive feedback from '/joint_states'."""
        self.actpos = list(fbkmsg.position)

    def sendcmd(self, positions, velocities):
        """Sends joint commands."""
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name     = ['base', 'shoulder', 'elbow']
        self.cmdmsg.position = positions
        self.cmdmsg.velocity = velocities
        self.cmdpub.publish(self.cmdmsg)

    def quintic_spline(self, q0, qT, T, t):
        """
        Compute array of positions and velocities for each DOF 
        using a quintic polynomial between q0 and qT. 
        q0, qT are arrays of the same length.
        """
        q0 = np.array(q0)
        qT = np.array(qT)
        positions  = []
        velocities = []
        for i in range(len(q0)):
            # For each DOF, compute the quintic
            a0 = q0[i]
            a1 = 0.0
            a2 = 0.0
            a3 =  (10*(qT[i] - q0[i])) / (T**3)
            a4 = (-15*(qT[i] - q0[i])) / (T**4)
            a5 =   6*(qT[i] - q0[i])  / (T**5)

            p = a0 + a1*t + a2*(t**2) + a3*(t**3) + a4*(t**4) + a5*(t**5)
            v = a1 + 2*a2*t + 3*a3*(t**2) + 4*a4*(t**3) + 5*a5*(t**4)

            positions.append(p)
            velocities.append(v)

        return positions, velocities

    def cartesian_to_joint_space_ik(self, x, y, z, pd, vd, t):
        """
        Perform one incremental IK step at time t (0 ≤ t ≤ 2.0).
        Called at 100Hz. We'll do a 5th-order interpolation from self.x0 to the final (x, y, z)
        and solve for a single small step in joint space.
        """
        dt = DT
        q = np.array(self.q_current)
        ptip, Rtip, Jv, Jw = self.chain.fkin(q)
        ptip = np.array(ptip)
        J    = Jv  
        p_f = np.array([x, y, z])
        # pd, vd = goto5(t, 2.0, self.p_waiting, p_f)  
        e = pd - ptip
        Jpinv = np.linalg.pinv(J)
        qdot = Jpinv @ (vd + self.lam * e)

        q_new = q + qdot * dt
        self.q_current = q_new
        self.x         = ptip  
        self.e         = e

        print(self.q_current)
        vel = [0.0]*len(self.q_current)
        self.q_current = list(self.q_current)
        vel = list(vel)

        self.sendcmd(self.q_current, vel)

    def build_trajectory(self, x0, xF, total_time=2.0, rate=100.0):
        """
        Precompute the entire spline in Cartesian space.
        Returns a list of (pd, vd) from t=0 to t=2.0 in time increments of 1/rate.
        """
        dt = 1.0 / rate
        steps = int(total_time * rate)

        traj = []
        for i in range(steps+1):
            t = i * dt
            if t > total_time:
                t = total_time
            pd, vd = goto5(t, total_time, x0, xF)
            traj.append( (pd, vd) )
        
        return traj


    def move_with_spline(self, q_start, q_goal, duration):
        """(Optional) The old function, left here if you still need it."""
        start_time = self.get_clock().now()
        while rclpy.ok():
            now     = self.get_clock().now()
            elapsed = (now - start_time).nanoseconds * 1e-9
            t = min(elapsed, duration)

            positions, velocities = self.quintic_spline(q_start, q_goal, duration, t)
            self.sendcmd(positions, velocities)

            if t >= duration:
                self.sendcmd(positions, [0.0]*len(velocities))
                self.q_current = q_goal
                break

    def move_to_waiting_position(self):
        """Example function that uses splines to move to a known waiting pose."""
        duration = 3.0
        # Move shoulder to 0 but keep base and elbow same
        intermediate_pos = [self.q_current[0], 0.0, self.q_current[2]]
        self.move_with_spline(self.q_current, intermediate_pos, duration)

        # Then move to [π/2, 0, π/2]
        waiting_position = [np.pi/2, 0.0, np.pi/2]
        self.move_with_spline(intermediate_pos, waiting_position, duration)
        self.get_logger().info("Reached waiting position.")


    def update(self):
        if self.current_phase == "startup":
            self.move_to_waiting_position()
            self.current_phase = "waiting"

        elif self.current_phase == "waiting":
            if self.point_queue:
                self.current_target = self.point_queue.pop(0)  
                
                ptip, Rtip, Jv, Jw = self.chain.fkin(self.q_current)
                x0 = np.array(ptip)
                
                xF = np.array(self.current_target)

                self.cart_traj = self.build_trajectory(x0, xF, total_time=2.0, rate=RATE)
                
                self.move_start_time = self.get_clock().now()
                self.traj_index      = 0

                self.current_phase = "moving"

        elif self.current_phase == "moving":
            now     = self.get_clock().now()
            elapsed = (now - self.move_start_time).nanoseconds * 1e-9
            idx = int(elapsed * RATE)

            if idx < len(self.cart_traj):
                pd, vd = self.cart_traj[idx]  
                xF = self.current_target  
                t  = idx * (1.0 / RATE)   
                self.cartesian_to_joint_space_ik(xF[0], xF[1], xF[2], pd, vd, t)
            else:
                self.current_phase = "returning"

        elif self.current_phase == "returning":
            self.move_to_waiting_position()
            self.current_phase = "waiting"



def main(args=None):
    rclpy.init(args=args)
    node = DemoNode('demo')
    try:
        rclpy.spin(node)  # Keeps spinning until shutdown
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



