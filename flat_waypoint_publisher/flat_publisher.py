#! /usr/bin/env python

"""
A simple ROS way point publisher compatible with a modified PX4 Geometric Controller
The controller is based on this library: https://github.com/Jaeyoung-Lim/mavros_controllers/tree/master/geometric_controller
with a small modification to the controller and the flat_setpoint message to allow for yaw
commands to be simultaneously issued without needing a velocity command.
"""
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
# from std_msgs.msg import Float32
from mavros_msgs.msg import State
from controller_msgs.msg import FlatTarget


class Waypoint:
    def __init__(self):
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.accel = [0, 0, 0]
        self.jerk = [0, 0, 0]
        self.snap = [0, 0, 0]
        self.yaw = 0


class TakeoffListener:

    def __init__(self):
        self.wp_list = get_waypoint_list(case=1)
        self.n = len(self.wp_list)

        self.i = 0
        self.takeoff_detected = False
        rospy.init_node('waypoint_publisher', anonymous=True)
        self.rate = rospy.Rate(0.2)  # 0.2hz

        rospy.loginfo("Starting Listener!")
        rospy.Subscriber("/mavros/state", State, self.takeoff_callback)

        rospy.loginfo("Starting Publisher!")
        self.pub = rospy.Publisher('/reference/flatsetpoint', FlatTarget, queue_size=10)

        # rospy.loginfo("Waypoints: " + str(self.wp_list))

        rospy.spin()

    def takeoff_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard: " + str(data.armed))
        if data.armed:
            self.takeoff_detected = True
            self.waypoint_publisher()

    def waypoint_publisher(self):
        if not rospy.is_shutdown():
            self.rate.sleep()
            # rospy.loginfo("Takeoff Detected is... " + str(self.takeoff_detected))
            if self.takeoff_detected:

                wp = self.wp_list[self.i]
                vec3 = Vector3(x=wp.pos[0], y=wp.pos[1], z=wp.pos[2])
                ft = FlatTarget(yaw=wp.yaw, position=vec3)

                rospy.logdebug("Sending wp " + str(self.i) + "Position: " + str(wp.pos) + "Yaw: " + str(wp.yaw))

                self.pub.publish(ft)

                self.i = self.i + 1
                if self.i >= self.n:
                    self.i = 0


def get_waypoint_list(case=0):
    waypoint_list = []
    print("CASE: " + str(case))
    if case == 1:
        corners = [[0, 0, 3, 1.5], [-13, 0, 3, 1.5], [-13, 0, 3, 0], [-13, 18, 3, 0], [-13, 18, 3, -1.5],
                   [12, 18, 3, -1.5], [12, 18, 3, -3.1], [12, 0, 3, -3.1], [12, 0, 3, 1.5]]
        step_size = 2
        positions = generate_polygon_trajectory(corners, step_size)

    elif case == 2:
        corners = [[0, 0, 2, 1.5], [0, 1, 2, 1.5]]
        positions = generate_polygon_trajectory(corners)

    else:  # default case
        positions = [[0, 0, 2, 1.5], [0, 0, 7, 1.5], [5.4, 0, 7, 1.5], [5.4, 6, 7, 1.5], [5.5, 10, 7, 1.5],
                     [4.3, 10, 7, 1], [4.3, 10, 7, .5], [4.3, 10, 7, 0], [4.3, 10, 7, -.5],
                     [4.3, 10, 7, -1], [4.3, 10, 7, -1.5], [4.3, 10, 7, -2], [4.3, 10, 7, -2.5],
                     [4.3, 10, 7, 3], [4.3, 10, 7, 2.5], [4.3, 10, 7, 2], [4.3, 10, 7, 1.5]]

    for p in positions:
        wp = Waypoint()
        wp.pos = p[:3]
        wp.yaw = p[3]
        waypoint_list.append(wp)

    # add reverse steps to make a loop
    if case != 1 or case != 2:
        positions.reverse()
        for p in positions:
            wp = Waypoint()
            wp.pos = p[:3]
            wp.yaw = p[3]
            waypoint_list.append(wp)

    return waypoint_list


def generate_polygon_trajectory(corners, pos_step=2, yaw_step=45):
    n = len(corners)
    if corners[0] != corners[-1]:
        # close the loop
        corners.append(corners[0])
        n = n + 1

    waypoint_list = [corners[0]]

    for i in range(n-1):
        while waypoint_list[-1] != corners[i+1]:
            dp = dist(waypoint_list[-1][:3], corners[i+1][:3])
            dy = abs(waypoint_list[-1][3] - corners[i+1][3])

            next_pos = [0, 0, 0, 0]
            if dp > pos_step:
                # add increment in direction of next point
                inc = [0, 0, 0]
                for j in range(3):
                    inc[j] = (corners[i+1][j] - waypoint_list[-1][j])/dp*pos_step
                    next_pos[j] = waypoint_list[-1][j] + inc[j]

            else:
                # set equal to next point
                next_pos[:3] = corners[i+1][:3]

            if dy > yaw_step:
                next_pos[3] = waypoint_list[-1][3] + (corners[i+1][3] - waypoint_list[-1][3])/dy*yaw_step
            else:
                next_pos[3] = corners[i+1][3]

            waypoint_list.append(next_pos)

    return waypoint_list


def dist(p, q):
    sum_sqr = 0
    for i in range(len(p)):
        sum_sqr = sum_sqr + pow(p[i] - q[i], 2)
    return pow(sum_sqr, 0.5)


if __name__ == "__main__":
    rospy.loginfo("STARTING WAYPOINT PUBLISHER")
    try:
        tol = TakeoffListener()
        tol.waypoint_publisher()

    except rospy.ROSInterruptException:
        pass
