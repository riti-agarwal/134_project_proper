#!/usr/bin/env python3
#
#   balldetector.py
#
#   Detect the tennis balls with OpenCV.
#
#   Node:           /balldetector
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /balldetector/binary        Intermediate binary image
#                   /balldetector/image_raw     Debug (marked up) image
#
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image

# All the following todo's should be in the trajectory class
# TODO: figure out logic on recieving end, that will decide 
# if it saw a circle (just point) or a rectangle (point and pose)
# TODO: create a subscriber on the recieving end (trajectory file) for pose and point
# TODO: convert from quaternion to theta on recieving end
# TODO: figure out how to tap on both sides of the rectangle on the recieving end 
# (logic for doing this should be built into recieving end)
# TODO: build ability to tap on both sides of the strip: 
# If a Pose message is received, that is the detector found a strip, tap next to the long skinning sides. 
# I.e. tap on one side, then the other, lifting up only a few centimeters to move across the strip. 
# Do not tap on the strip directly
# TODO: once the arm is in motion and moving towards the target - 
# it should disregard all the other messages it recieves on the point and pose - 
# this cannot be implemented with a queue since we need the most recent point it recieves after completing the motion and queue will be FIFO
# We may just block/ignore incoming messages until we have come back to our waiting position, and once we are in wiaitng position we will take the most recent message. 

#
#  Detector Node Class
#
class DetectorNode(Node):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Thresholds in Hmin/max, Smin/max, Vmin/max
        # ball range
        # self.hsvlimits = np.array([[30, 42], [65, 110], [140, 240]])
        # orange objects ranje
        # self.hsvlimits = np.array([[13, 25], [65, 185], [135, 200]])
        # orange objects trying
        # self.hsvlimits = np.array([[10, 28], [60, 190], [130, 210]])
        # new hsv with shadows
        self.hsvlimits = np.array([[12, 25], [60, 240], [105, 185]])

        # Create a publisher for the processed (debugging) images.
        # Store up to three images, just in case.
        # TODO: create publisher for the point
        # TODO: create publisher for the pose (quaternion
        # TODO: import mapping in balldetector - to be able to convert coordinates to world coordinates 
        # to publish world coordinates to publisher
        self.pubrgb = self.create_publisher(Image, name+'/image_raw', 3)
        self.pubbin = self.create_publisher(Image, name+'/binary',    3)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)

        # Report.
        self.get_logger().info("Ball detector running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Process the image (detect the ball).
    def process(self, msg):

        # TODO: make logic to recognize whether it is a rectangle or a circle
        # TODO: if circle: 
            # find midpoint
            # convert midpoint to world coordinates
            # publish midpoint to point subscriber
        # TODO: if rectangle: 
            # find rectangle
            # find 4 corners of rectangle
            # use it to find the centre of the rectangle (x, y)
            # use 4 corners to find the midpoint of the top side of the rectangle (x2, y2)
            # (x_delta, y_delta) = x2 - x, y2 - y
            # theta = atan2 (y_delta, x_delta)
            # convert theta to quaternion: [0, 0, sin (theta/2), cos (theta/2)]
            # convert centre of rectangle to world coordinates
            # publish world coordinates and quaternion to pose publisher
        # TODO: build logic for what happens if both on circle and rectangle shown on display
        # # TODO: publish the point found to the publisher.
        # # TODO: publish pose found to publisher

        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Cheat: swap red/blue

        # Grab the image shape, determine the center pixel.
        (H, W, D) = frame.shape
        uc = W//2
        vc = H//2

        # Help to determine the HSV range...
        if True:
            # Draw the center lines.  Note the row is the first dimension.
            frame = cv2.line(frame, (uc,0), (uc,H-1), self.white, 1)
            frame = cv2.line(frame, (0,vc), (W-1,vc), self.white, 1)

            # Report the center HSV values.  Note the row comes first.
            self.get_logger().info(
                "HSV = (%3d, %3d, %3d)" % tuple(hsv[vc, uc]))

        
        # Threshold in Hmin/max, Smin/max, Vmin/max
        binary = cv2.inRange(hsv, self.hsvlimits[:,0], self.hsvlimits[:,1])

        # Erode and Dilate. Definitely adjust the iterations!
        # TODO: come back and fix if problem with detection
        iter_remove = 4
        iter_fillin = 10
        binary = cv2.erode( binary, None, iterations=iter_remove)
        binary = cv2.dilate(binary, None, iterations=iter_remove+iter_fillin)
        binary = cv2.erode( binary, None, iterations=iter_fillin)

        contours, hierarchy = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw all contours on the original image for debugging.
        cv2.drawContours(frame, contours, -1, self.blue, 2)

        detected_circles = []
        detected_rectangles = []

        # for contour in contours:
        #     # Approximate the contour to reduce the number of points
        #     peri = cv2.arcLength(contour, True)
        #     approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

        #     # Force fit a circle around the contour
        #     ((ur, vr), radius) = cv2.minEnclosingCircle(contour)
        #     ur = int(ur)
        #     vr = int(vr)
        #     radius = int(radius)

        #     # Calculate the area of the contour and the circle
        #     contour_area = cv2.contourArea(contour)
        #     circle_area = np.pi * (radius ** 2)

        #     # Calculate the overlap ratio
        #     overlap_ratio = contour_area / circle_area

        #     # Classify based on overlap ratio
        #     if overlap_ratio > 0.7:  # Circle classification threshold
        #         detected_circles.append((ur, vr, radius))

        #         # Draw the circle and its center
        #         cv2.circle(frame, (ur, vr), radius, self.yellow, 2)  # Enclosing circle
        #         cv2.circle(frame, (ur, vr), 5, self.red, -1)  # Circle center

        #         # Draw the force-fitted circle on the image
        #         cv2.circle(frame, (ur, vr), radius, self.yellow, 2)

        #         self.get_logger().info(
        #             f"Detected Circle: Center=({ur}, {vr}), Radius={radius}, Overlap={overlap_ratio:.2f}"
        #         )
        #     else:
        #         # Assume it's a rectangle
        #         rect = cv2.minAreaRect(contour)
        #         box = cv2.boxPoints(rect)
        #         box = np.int0(box)
        #         detected_rectangles.append(box)

        #         # Draw the bounding rectangle
        #         cv2.drawContours(frame, [box], 0, self.green, 2)

        #         # Calculate center of the rectangle
        #         M = cv2.moments(contour)
        #         if M["m00"] != 0:
        #             cx = int(M["m10"] / M["m00"])
        #             cy = int(M["m01"] / M["m00"])
        #         else:
        #             cx, cy = 0, 0

        #         # Draw the center point of the rectangle
        #         cv2.circle(frame, (cx, cy), 5, self.blue, -1)

        #         # Calculate midpoint of the top side
        #         sorted_box = sorted(box, key=lambda x: x[1])  # Sort by y
        #         top_side = sorted_box[:2]  # Top two points
        #         midpoint_x = int((top_side[0][0] + top_side[1][0]) / 2)
        #         midpoint_y = int((top_side[0][1] + top_side[1][1]) / 2)
        #         midpoint = (midpoint_x, midpoint_y)

        #         # Draw the midpoint of the top side
        #         cv2.circle(frame, midpoint, 5, self.blue, -1)

        #         # Calculate the orientation (theta)
        #         x_delta = midpoint_x - cx
        #         y_delta = midpoint_y - cy
        #         theta = np.arctan2(y_delta, x_delta)

        #         self.get_logger().info(
        #             f"Detected Rectangle: Center=({cx}, {cy}), Midpoint=({midpoint_x}, {midpoint_y}), Theta={np.degrees(theta):.2f}"
        #         )

        # # Handle case where both shapes are detected
        # if detected_circles and detected_rectangles:
        #     self.get_logger().info("Both circle and rectangle detected in the frame.")

        # # Convert the frame back into a ROS image and republish.
        # self.pubrgb.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

        # # Also publish the binary (black/white) image.
        # self.pubbin.publish(self.bridge.cv2_to_imgmsg(binary, "mono8"))
        for contour in contours:
            # Approximate the contour to reduce the number of points
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

            # Force fit a circle around the contour
            ((ur, vr), radius) = cv2.minEnclosingCircle(contour)
            ur = int(ur)
            vr = int(vr)
            radius = int(radius)

            # Calculate the area of the contour and the circle
            contour_area = cv2.contourArea(contour)
            circle_area = np.pi * (radius ** 2)

            # Calculate the overlap ratio
            overlap_ratio = contour_area / circle_area

            # Classify based on overlap ratio
            if overlap_ratio > 0.7:  # Circle classification threshold
                detected_circles.append((ur, vr, radius))

                # Draw the circle and its center
                cv2.circle(frame, (ur, vr), radius, self.yellow, 2)  # Enclosing circle
                cv2.circle(frame, (ur, vr), 5, self.red, -1)  # Circle center

                # Draw the force-fitted circle on the image
                cv2.circle(frame, (ur, vr), radius, self.yellow, 2)

                self.get_logger().info(
                    f"Detected Circle: Center=({ur}, {vr}), Radius={radius}, Overlap={overlap_ratio:.2f}"
                )
            else:
                # Assume it's a rectangle
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                detected_rectangles.append(box)

                # Draw the bounding rectangle
                cv2.drawContours(frame, [box], 0, self.green, 2)

                # Calculate center of the rectangle
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = 0, 0

                # Draw the center point of the rectangle
                cv2.circle(frame, (cx, cy), 5, self.blue, -1)

                # Find the shorter edge of the rectangle that is on top
                edges = [
                    (box[i], box[(i + 1) % 4]) for i in range(4)
                ]  # List of edges as pairs of points
                edges_with_lengths = [
                    (edge, np.linalg.norm(np.array(edge[0]) - np.array(edge[1])))
                    for edge in edges
                ]  # Compute lengths of edges

                # Find the two shorter edges
                edges_with_lengths.sort(key=lambda x: x[1])
                shorter_edges = edges_with_lengths[:2]

                # Determine which of the shorter edges is on top
                top_shorter_edge = min(
                    shorter_edges,
                    key=lambda edge: min(edge[0][0][1], edge[0][1][1]),  # Sort by min y-coord
                )[0]

                # Calculate midpoint of the top shorter edge
                midpoint_x = int((top_shorter_edge[0][0] + top_shorter_edge[1][0]) / 2)
                midpoint_y = int((top_shorter_edge[0][1] + top_shorter_edge[1][1]) / 2)
                midpoint = (midpoint_x, midpoint_y)

                # Draw the midpoint of the top shorter edge
                cv2.circle(frame, midpoint, 5, self.blue, -1)

                # Calculate the orientation (theta)
                x_delta = midpoint_x - cx
                y_delta = midpoint_y - cy
                theta = np.arctan2(y_delta, x_delta)

                self.get_logger().info(
                    f"Detected Rectangle: Center=({cx}, {cy}), Midpoint=({midpoint_x}, {midpoint_y}), Theta={np.degrees(theta):.2f}"
                )

        # Handle case where both shapes are detected
        if detected_circles and detected_rectangles:
            self.get_logger().info("Both circle and rectangle detected in the frame.")

        # Convert the frame back into a ROS image and republish.
        self.pubrgb.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

        # Also publish the binary (black/white) image.
        self.pubbin.publish(self.bridge.cv2_to_imgmsg(binary, "mono8"))




#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DetectorNode('balldetector')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
