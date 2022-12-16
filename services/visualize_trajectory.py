import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from move_group_utils.kdl_kin import KdlKin
from move_group_utils.srv import VisualizeTrajectory


class TrajectoryVisualizer:

    def __init__(self, name: str):
        self.name = name
        self.kdl_kin = KdlKin()
        self.marker_pub = rospy.Publisher('trajectory_marker_array',
                                          MarkerArray, queue_size=1)
        self.traj_viz_srv = rospy.Service('/mgu/visualize_trajectory',
                                          VisualizeTrajectory, self._visualize_trajectory)
        self.rate = rospy.Rate(100)

    def run(self):
        rospy.loginfo(f'{self.name}: ready to visualize trajectories')
        while not rospy.is_shutdown():
            self.rate.sleep()

    def _visualize_trajectory(self, req):
        marker_array = MarkerArray()
        for i in range(len(req.joint_trajectory.points)):
            marker = Marker()
            marker.colors = [0.0, 0.0, 1.0, 1.0]
            marker.header = Header()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.scale = [0.01, 0.01, 0.01]
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = self.kdl_kin.forward_kinematics(
                req.joint_trajectory.points[i].positions)

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        return True


if __name__ == '__main__':
    rospy.init_node('trajectory_visualizer', anonymous=True)
    visualizer = TrajectoryVisualizer(rospy.get_name())
    visualizer.run()
