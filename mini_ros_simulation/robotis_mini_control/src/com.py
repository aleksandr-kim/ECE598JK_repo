#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs as tf_geo
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker


class MINI_CoM:

    def __init__(self):
        rospy.init_node('mini_com', anonymous=True)
        self.base_link_frame = rospy.get_param("~base_link_frame", "world")

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        self.init_link_inertial_info()
        self.init_marker()

    def init_link_inertial_info(self):
        robot = URDF.from_parameter_server()
        links = robot.link_map

        unnecessary_links = []
        for link in links:
            if links[link].inertial == None or links[link].inertial.origin == None:
                unnecessary_links.append(link)

        for link in unnecessary_links:
            del links[link]

        self.link_info = {}
        for link in links:
            self.link_info[link] = links[link].inertial.to_yaml()

    def init_marker(self):
        marker = Marker()
        marker.header.frame_id = self.base_link_frame
        marker.header.stamp = rospy.Time()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        self.marker = marker
        self.marker_pub = rospy.Publisher('com', Marker, queue_size=1)

    def transform_CoM(self):
        """
        TODO: transform all CoM positions from link frame to base frame

        Tips:
        - self.tfBuffer.lookup_transform(frame1, frame2) gets transform from two coord frames
        - Use tf_geo.do_transform_point(transform, point) to apply transform where point
            is ros_msg type geometry_msgs.msg.PointStamped()
        - To get position in link frame -> self.link_info[link]['origin']['xyz']
        - To get mass information -> self.link_info[link]['mass']
        """
        self.transformed_coms = []
        for link in self.link_info.keys():
            try:
                trans = self.tfBuffer.lookup_transform(
                    self.base_link_frame, link, rospy.Time())
                link_com = self.link_info[link]['origin']['xyz']
                mass = self.link_info[link]['mass']
                point_stamped = geometry_msgs.msg.PointStamped()
                point_stamped.header.frame_id = link
                point_stamped.point.x = link_com[0]
                point_stamped.point.y = link_com[1]
                point_stamped.point.z = link_com[2]
                transformed_point = tf_geo.do_transform_point(
                    point_stamped, trans)
                self.transformed_coms.append({
                    'link': link,
                    'transformed_com': transformed_point.point,
                    'mass': mass
                })

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Failed to transform CoM for link {link}: {e}")

    def calculate_CoM(self):
        """
        TODO: Calculate whole body CoM 
        """
        total_mass = 0
        weighted_sum_x = 0
        weighted_sum_y = 0
        weighted_sum_z = 0

        for com_data in self.transformed_coms:
            mass = com_data['mass']
            com = com_data['transformed_com']
            weighted_sum_x += mass * com.x
            weighted_sum_y += mass * com.y
            weighted_sum_z += mass * com.z
            total_mass += mass

        if total_mass > 0:
            self.com_x = weighted_sum_x / total_mass
            self.com_y = weighted_sum_y / total_mass
            self.com_z = weighted_sum_z / total_mass
        else:
            rospy.logwarn("Total mass is zero, cannot calculate CoM.")
            self.com_x = 0
            self.com_y = 0
            self.com_z = 0

        rospy.loginfo(
            f"Calculated CoM: ({self.com_x}, {self.com_y}, {self.com_z})")

    def visualize_CoM(self):
        self.marker.pose.position.x = self.com_x
        self.marker.pose.position.y = self.com_y
        self.marker.pose.position.z = self.com_z
        self.marker_pub.publish(self.marker)


if __name__ == '__main__':
    mini_com = MINI_CoM()
    rate = rospy.Rate(100)
    rospy.sleep(2.0)

    while not rospy.is_shutdown():
        mini_com.transform_CoM()
        mini_com.calculate_CoM()
        mini_com.visualize_CoM()

        rate.sleep()
