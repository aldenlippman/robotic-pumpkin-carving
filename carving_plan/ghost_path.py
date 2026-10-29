import sys
import math

# ROS Client Library
import rclpy
from rclpy.node import Node

# YAML Export Utility
from carving_plan.export_utility import ExportUtility

# ROS Data Types
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker

# Project Specific Data Types
from pumpkin_msgs.srv import PlanMotion

PUSH_IN = 0.04


# Node
class GhostPumpkinPath(Node):

    def __init__(self):
        super().__init__('alden_pumpkin_planner')

        # Client
        self.cli = self.create_client(PlanMotion, 'generate_motion_plan')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('generate_motion_plan service not available, waiting again...')

        #Publisher
        self.pose_array_publisher_ = self.create_publisher(PoseArray, 'plot_carve', 10)
        self.line_publisher_ = self.create_publisher(Marker, 'plot_carve_lines', 10)
        self.pose_array_publisher_1_ = self.create_publisher(PoseArray, 'plot_carve_1', 10)
        self.line_publisher_1_ = self.create_publisher(Marker, 'plot_carve_lines_1', 10)
        self.pose_array_publisher_2_ = self.create_publisher(PoseArray, 'plot_carve_2', 10)
        self.line_publisher_2_ = self.create_publisher(Marker, 'plot_carve_lines_2', 10)
        self.pose_array_publisher_3_ = self.create_publisher(PoseArray, 'plot_carve_3', 10)
        self.line_publisher_3_ = self.create_publisher(Marker, 'plot_carve_lines_3', 10)
        self.pose_array_publisher_4_ = self.create_publisher(PoseArray, 'plot_carve_4', 10)
        self.line_publisher_4_ = self.create_publisher(Marker, 'plot_carve_lines_4', 10)
        self.pose_array_publisher_5_ = self.create_publisher(PoseArray, 'plot_carve_5', 10)
        self.line_publisher_5_ = self.create_publisher(Marker, 'plot_carve_lines_5', 10)
        self.pose_array_publisher_6_ = self.create_publisher(PoseArray, 'plot_carve_6', 10)
        self.line_publisher_6_ = self.create_publisher(Marker, 'plot_carve_lines_6', 10)
        self.pose_array_publisher_7_ = self.create_publisher(PoseArray, 'plot_carve_7', 10)
        self.line_publisher_7_ = self.create_publisher(Marker, 'plot_carve_lines_7', 10)
        self.pose_array_publisher_8_ = self.create_publisher(PoseArray, 'plot_carve_8', 10)
        self.line_publisher_8_ = self.create_publisher(Marker, 'plot_carve_lines_8', 10)

        # Initialize Request Structure
        self.req = PlanMotion.Request()


    def send_request(self):
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def generate_tool_path_bottom_left(self):
        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is bottom left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.0, 0.0))

        #Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.0, PUSH_IN))

        #Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.075, PUSH_IN))

        for x in range(0, 30, 5):
            ycoord = 16.67*((x*10**-3)-0.03)**2+0.06
            pumpkin_segment.poses.append(self.add_waypoint((x*10**-3),ycoord,PUSH_IN))

        pumpkin_segment.poses.append(self.add_waypoint(0.03, 0.06, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.03, 0.0, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.0, PUSH_IN))

        #Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        self.get_logger().info("Completed Tool Path Bottom Left")
        return pumpkin_segment
    
    def generate_tool_path_bottom_right(self):
        
        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is bottom left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.12, 0.0, 0.0))

        #Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.12, 0.0, PUSH_IN))

        #Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.12, 0.06, PUSH_IN))
        for x in range(120, 150, 5):
            ycoord = 16.67*((x*10**-3)-0.12)**2+0.06
            pumpkin_segment.poses.append(self.add_waypoint((x*10**-3),ycoord,PUSH_IN))
        
        pumpkin_segment.poses.append(self.add_waypoint(0.15, 0.075, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.15, 0.0, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.12, 0.0, PUSH_IN))

        #Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.12, 0.0, 0.0))

        self.get_logger().info("Completed Tool Path Bottom Right")
        return pumpkin_segment
    
    def generate_tool_path_left_eye(self):
        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is bottom left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.045, 0.1, 0.0))

        #Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.045,0.1,PUSH_IN))
        
        # Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.045,0.07,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.0675,0.07,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.0675,0.085,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.045,0.1,PUSH_IN))

        #Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.045, 0.1, 0.0))

        self.get_logger().info("Completed Tool Path Left Eye")

        return pumpkin_segment
    
    def generate_tool_path_right_eye(self):
        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is bottom left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.1, 0.085, 0.0))

        #Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.1, 0.085, PUSH_IN))

        # Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.1, 0.07, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.0825, 0.07, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.0825, 0.0775, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.1, 0.085, PUSH_IN))

        #Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.1, 0.085, 0.0))

        self.get_logger().info("Completed Tool Path Right Eye")

        return pumpkin_segment
    
    def generate_tool_path_top_left(self):
        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is bottom left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.085, 0.0))

        #Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.0,0.085,PUSH_IN))

        # Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.0,0.12,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.06,0.12,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.06,0.1174,PUSH_IN))

        for x in range(110, 180, 10):
            xcoord = 0.075 + 0.045*math.cos(x*(math.pi/180))
            ycoord = 0.075 + 0.045*math.sin(x*(math.pi/180))
            pumpkin_segment.poses.append(self.add_waypoint(xcoord,ycoord,PUSH_IN))

        pumpkin_segment.poses.append(self.add_waypoint(0.03,0.075,PUSH_IN))

        for x in range(30, 0, -5):
            ycoord = 11*((x*10**-3)-0.03)**2+0.075
            pumpkin_segment.poses.append(self.add_waypoint((x*10**-3),ycoord,PUSH_IN))

        pumpkin_segment.poses.append(self.add_waypoint(0.0,0.085,PUSH_IN))

        # Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.085, 0.0))

        self.get_logger().info("Completed Tool Path Top Left")
        return pumpkin_segment

    def generate_tool_path_top_right(self):
        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is bottom left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.15, 0.085, 0.0))

        #Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.15,0.085,PUSH_IN))

        # Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.15,0.12,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.09,0.12,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.09,0.1174,PUSH_IN))

        for x in range(70, 0, -10):
            xcoord = 0.075 + 0.045*math.cos(x*(math.pi/180))
            ycoord = 0.075 + 0.045*math.sin(x*(math.pi/180))
            pumpkin_segment.poses.append(self.add_waypoint(xcoord,ycoord,PUSH_IN))

        pumpkin_segment.poses.append(self.add_waypoint(0.12,0.075,PUSH_IN))

        for x in range(120, 150, 5):
            ycoord = 11*((x*10**-3)-0.12)**2+0.075
            pumpkin_segment.poses.append(self.add_waypoint((x*10**-3),ycoord,PUSH_IN))

        pumpkin_segment.poses.append(self.add_waypoint(0.15,0.085,PUSH_IN))

        # Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.15, 0.085, 0.0))

        self.get_logger().info("Completed Tool Path Top Right")
        return pumpkin_segment

    def generate_tool_path_triangle_1(self):
        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is bottom left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.035, 0.0, 0.0))

        # Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.035, 0.0, PUSH_IN))

        # Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.045, 0.015, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.055, 0.0, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.035, 0.0, PUSH_IN))

        # Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.035, 0.0, 0.0))

        self.get_logger().info("Completed Tool Path Triangle 1")
        return pumpkin_segment

    def generate_tool_path_triangle_2(self):
        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is bottom left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.065, 0.0, 0.0))

        # Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.065, 0.0, PUSH_IN))

        # Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.075, 0.015, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.085, 0.0, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.065, 0.0, PUSH_IN))

        # Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.065, 0.0, 0.0))

        self.get_logger().info("Completed Tool Path Triangle 2")
        return pumpkin_segment

    def generate_tool_path_triangle_3(self):
        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is bottom left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.095, 0.0, 0.0))

        # Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.095, 0.0, PUSH_IN))

        # Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.105, 0.015, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.115, 0.0, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.095, 0.0, PUSH_IN))

        # Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.095, 0.0, 0.0))

        self.get_logger().info("Completed Tool Path Triangle 3")
        return pumpkin_segment
    
    def plot_rviz(self, tool_path:PoseArray):

        # Generate lines
        tool_path_lines = Marker()
        tool_path_lines.header.frame_id = "pumpkin_face"
        tool_path_lines.type = Marker.LINE_STRIP
        tool_path_lines.action = Marker.ADD
        tool_path_lines.pose.orientation.w = 1.0
        tool_path_lines.scale.x = 0.002
        tool_path_lines.scale.y = 0.002
        tool_path_lines.scale.z = 0.002
        tool_path_lines.color.a = 0.8
        tool_path_lines.color.r = 1.0
        tool_path_lines.color.g = 1.0
        tool_path_lines.color.b = 1.0

        # Extract Points
        for pose in tool_path.poses:
            tool_path_lines.points.append(pose.position)

        return tool_path_lines

    def add_waypoint(self, px, py, pz, ox = 0.0, oy = 0.0, oz = 0.0):
        waypoint = Pose()
        waypoint.position.x = px
        waypoint.position.y = py
        waypoint.position.z = pz
        waypoint.orientation.x = ox
        waypoint.orientation.y = oy
        waypoint.orientation.z = oz
        waypoint.orientation.w = 1.0
        return waypoint

def main(args=None):
    rclpy.init(args=args)

    # Instantiate TPP Class
    minimal_client = GhostPumpkinPath()

    # Generate Pumpkin Toolpath
    tool_path_bottom_left = minimal_client.generate_tool_path_bottom_left()
    tool_path_bottom_right = minimal_client.generate_tool_path_bottom_right()
    tool_path_top_left = minimal_client.generate_tool_path_top_left()
    tool_path_top_right = minimal_client.generate_tool_path_top_right()
    tool_path_left_eye = minimal_client.generate_tool_path_left_eye()
    tool_path_right_eye = minimal_client.generate_tool_path_right_eye()
    tool_path_triangle_1 = minimal_client.generate_tool_path_triangle_1()
    tool_path_triangle_2 = minimal_client.generate_tool_path_triangle_2()
    tool_path_triangle_3 = minimal_client.generate_tool_path_triangle_3()
    minimal_client.req.path = [tool_path_left_eye, tool_path_right_eye, tool_path_triangle_1, tool_path_triangle_2, tool_path_triangle_3, tool_path_bottom_left, tool_path_bottom_right, tool_path_top_left, tool_path_top_right]

    # Visualize toolpath
    minimal_client.pose_array_publisher_.publish(tool_path_bottom_left)
    minimal_client.line_publisher_.publish(minimal_client.plot_rviz(tool_path_bottom_left))
    minimal_client.pose_array_publisher_1_.publish(tool_path_bottom_right)
    minimal_client.line_publisher_1_.publish(minimal_client.plot_rviz(tool_path_bottom_right))
    minimal_client.pose_array_publisher_.publish(tool_path_top_left)
    minimal_client.line_publisher_.publish(minimal_client.plot_rviz(tool_path_top_left))
    minimal_client.pose_array_publisher_1_.publish(tool_path_top_right)
    minimal_client.line_publisher_1_.publish(minimal_client.plot_rviz(tool_path_top_right))
    minimal_client.pose_array_publisher_2_.publish(tool_path_left_eye)
    minimal_client.line_publisher_2_.publish(minimal_client.plot_rviz(tool_path_left_eye))
    minimal_client.pose_array_publisher_3_.publish(tool_path_right_eye)
    minimal_client.line_publisher_3_.publish(minimal_client.plot_rviz(tool_path_right_eye))
    minimal_client.pose_array_publisher_4_.publish(tool_path_triangle_1)
    minimal_client.line_publisher_4_.publish(minimal_client.plot_rviz(tool_path_triangle_1))
    minimal_client.pose_array_publisher_4_.publish(tool_path_triangle_2)
    minimal_client.line_publisher_4_.publish(minimal_client.plot_rviz(tool_path_triangle_2))
    minimal_client.pose_array_publisher_4_.publish(tool_path_triangle_3)
    minimal_client.line_publisher_4_.publish(minimal_client.plot_rviz(tool_path_triangle_3))
    # minimal_client.get_logger().info("Attempted to publish")
    
    # Write out toolpath to YAML
    ExportUtility("log").write_pose_array_list_to_yaml(minimal_client.req.path)
    

    # Send Motion Plan Request
    minimal_client.get_logger().info("Attempting to call motion planner!")
    motion_plan_response = minimal_client.send_request()

    trajectory_msg = motion_plan_response.trajectory

    print(trajectory_msg)
    # minimal_client.get_logger().info(trajectory_msg)


    rclpy.spin_once(minimal_client)
    ExportUtility("log").write_joint_trajectory_to_yaml(trajectory_msg)


if __name__ == '__main__':
    main()