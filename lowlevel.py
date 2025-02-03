# TODO: Replace and Halt
# TODO: if collision is detected - then move back to home position. 
# TODO: Send a message back to the brain once trajectory is completed.


import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from project_interfaces.msg import SplineSegment, ExecutionStatus, ControlCommand
from std_msgs.msg import Float64
import math

from threedof.KinematicChain import KinematicChain
from threedof.TrajectoryUtils import *

RATE = 100.0  
DT   = 1.0 / RATE


class Spline:
    def __init__(self, tcmd, pcmd, vcmd, segment):
        self.t0 = tcmd
        self.T = segment.Tmove
        
        p0 = np.array(pcmd)
        v0 = np.array(vcmd)
        pf = np.array(segment.qf)
        vf = np.array(segment.vf)
        T = self.T
        
        self.a = p0
        self.b = v0
        self.c = np.zeros_like(p0)
        self.d = +10 * (pf - p0) / T**3 - 6 * v0 / T**2 - 4 * vf / T**2
        self.e = -15 * (pf - p0) / T**4 + 8 * v0 / T**3 + 7 * vf / T**3
        self.f = +6 * (pf - p0) / T**5 - 3 * v0 / T**4 - 3 * vf / T**4
    
    def evaluate(self, t):
        t = t - self.t0
        p = self.a + self.b * t + self.c * t**2 + self.d * t**3 + self.e * t**4 + self.f * t**5
        v = self.b + 2 * self.c * t + 3 * self.d * t**2 + 4 * self.e * t**3 + 5 * self.f * t**4
        return (p.tolist(), v.tolist())
    
class Segment: 
    def __init__(self, qf, vf, Tmove):
        self.qf = qf
        self.vf = vf
        self.Tmove = Tmove


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
        self.A = -1.815
        self.B = 0.0

        # Grab initial position
        self.position0 = self.grabfbk()
        self.actpos = self.position0.copy()
        self.get_logger().info(f"Initial positions: {self.position0}")

        self.t_start = self.get_clock().now()

        # setup segment queue variables
        self.segments = []
        self.spline = None
        self.abort = False
        self.tcmd = self.now()
        self.pcmd = self.actpos.copy()
        self.vcmd = np.zeros_like(self.pcmd)

        self.contact_threshold_effort = 1.5
        self.effort = self.gravity(self.actpos)

        # subscriber to /spline_segment
        self.splinesub = self.create_subscription(SplineSegment, '/spline_segment', self.recvspline, 10)
        # self.get_logger().info("sub" + str(self.splinesub))

        # Publisher for sending "ready" status to the brain
        self.ready_pub = self.create_publisher(ExecutionStatus, '/execution_status', 10)

        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while not self.count_subscribers('/joint_commands'):
            pass

        self.fbksub = self.create_subscription(JointState, '/joint_states', self.recvfbk, 10)

        # Subscriber for control messages (halt/replace)
        self.control_sub = self.create_subscription(ControlCommand, '/control_command', self.recv_control, 10)

        self.timer = self.create_timer(DT, self.update)
        self.get_logger().info(f"Sending commands at {RATE} Hz")

        self.intermediate_home = [self.pcmd[0], 0.0, self.pcmd[2]]
        self.home = [np.pi/2, 0.0, np.pi/2]

        self.publish_status()


    def publish_status(self, collision=False):
        msg = ExecutionStatus()
        msg.ready = not collision  # Only "ready" if no collision
        msg.collision = collision
        self.ready_pub.publish(msg)
        if collision:
            self.get_logger().warn("Collision detected! Sending alert to brain.")
        else:
            self.get_logger().info("Low-level node is ready for new trajectory.")

    def gravity(self, q):
        shoulder = q[1]
        tau_shoulder = self.A*math.sin(shoulder) + self.B*math.cos(shoulder)
        return [0.0, tau_shoulder, 0.0]

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

    def now(self):
        time = self.get_clock().now() - self.t_start
        return time.nanoseconds * 1e-9

    def sendcmd(self, positions, velocities, effort=None):
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name = ['base', 'shoulder', 'elbow']
        self.cmdmsg.position = positions
        self.cmdmsg.velocity = velocities
        if effort != None:
            self.cmdmsg.effort = effort
        self.cmdpub.publish(self.cmdmsg)

    def recvspline(self, msg):
        self.get_logger().info("qf"+str(msg.qf))
        segment = self.create_test_segment(msg.qf, msg.vf, msg.t_move)
        self.enqueue_segment(segment)

    def recv_control(self, msg):
        if msg.command == "halt":
            self.get_logger().info("Halting motion!")
            self.segments.clear()
            self.abort = True  

        elif msg.command == "replace":
            self.get_logger().info("Replacing current trajectory!")
            self.segments = [Segment(s.qf, s.vf, s.t_move) for s in msg.new_segments]
            self.abort = True 

    def detect_contact_torque_only(self):
        if not hasattr(self, 'acteff'):
            return False
        for i, eff in enumerate(self.effort):
            if abs(self.acteff[i] - eff) > self.contact_threshold_effort:
                self.get_logger().warn(f"[Collision Detected] Torque {self.acteff[i]:.3f} exceeds threshold {self.contact_threshold_effort:.3f}")
                return True
        return False

    def create_test_segment(self, qf, vf, Tmove):
        return Segment(qf, vf, Tmove)

    def enqueue_segment(self, segment):
        self.segments.append(segment)
        # self.get_logger().info(f"Segment enqueued: {segment.qf}")


    # def update(self):
    #     t = self.now()
        
    #     if self.spline and (t - self.spline.t0 > self.spline.T) or self.abort:
    #         self.spline = None
    #         self.abort = False

    #         if not self.segments:
    #             self.publish_ready_status()
        
    #     if not self.spline and len(self.segments) > 0:
    #         self.spline = Spline(self.tcmd, self.pcmd, self.vcmd, self.segments.pop(0))
        
    #     if self.spline:
    #         (self.pcmd, self.vcmd) = self.spline.evaluate(t)
    #     else:
    #         self.pcmd = self.pcmd
    #         self.vcmd = [0.0 for v in self.vcmd]
        
    #     self.tcmd = t
    #     self.sendcmd(self.pcmd, self.vcmd, self.gravity(self.actpos))

    def update(self):
        t = self.now()

        if self.detect_contact_torque_only():
            self.publish_ready_status(collision=True) 
            self.segments.clear() 
            self.abort = True  
            return

        # If trajectory is complete OR an abort command is received, stop motion
        if (self.spline and (t - self.spline.t0 > self.spline.T)) or self.abort:
            self.spline = None
            self.abort = False

            # If no new segments exist, publish "ready" and stay still
            if not self.segments:
                self.publish_ready_status()  

        # If "abort" is set OR there are new segments to process, switch immediately
        if self.abort or (not self.spline and len(self.segments) > 0):
            self.spline = Spline(self.tcmd, self.pcmd, self.vcmd, self.segments.pop(0))
            self.abort = False  # Reset abort flag immediately

        # Continue executing the spline
        if self.spline:
            (self.pcmd, self.vcmd) = self.spline.evaluate(t)
        else:
            self.pcmd = self.pcmd  # Maintain last position
            self.vcmd = [0.0 for _ in self.vcmd]  # Ensure stopped velocity

        self.tcmd = t
        self.effort = self.gravity(self.actpos)
        self.sendcmd(self.pcmd, self.vcmd, self.effort)




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
