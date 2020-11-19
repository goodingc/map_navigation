#!/usr/bin/python
import sys

import actionlib
import roslaunch.parent
import roslaunch.rlutil
import rospy
import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Point, Quaternion, Pose, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from occupancy_grid import OccupancyGrid
from std_msgs.msg import Header
from std_srvs.srv import Empty
from typing import List, Optional

# Set to output of 'rospack find map_navigation'
PACKAGE_PATH = '/home/goodingc/ros/catkin_ws/src/map_navigation'


def launch_nav_stack():
    """
    @author Callum
    Launches fresh navigation stack
    """
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    parent = roslaunch.parent.ROSLaunchParent(uuid, [PACKAGE_PATH + '/launch/map_navigation.launch'])
    parent.start()


def get_header():
    """
    @return: Header where frame_id='map' and stamp correctly filled
    @rtype: Header
    """


class Position:
    x = 0
    y = 0
    theta = 0

    def __init__(self, x=0, y=0, theta=0):
        """
        @author Callum
        @type x: float
        @type y: float
        @param theta: euler angle
        @type theta: float
        """
        self.x = x
        self.y = y
        self.theta = theta

    def to_pose(self):
        """
        @return: pose message
        @rtype: Pose
        """

    def __str__(self):
        """
        @rtype: str
        """
        return "x: {}, y: {}, theta: {}".format(self.x, self.y, self.theta)


class MapNavigation:
    amcl_pos = None  # type: Optional[Position]
    marking = False
    rate_limiter = None  # type: rospy.Rate
    initialpose_pub = None  # type: rospy.Publisher
    action_client = None  # type: actionlib.SimpleActionClient
    occupancy_grid = None  # type: OccupancyGrid

    def __init__(self, occupancy_grid, synchronize=True):
        """
        Initializes:
            node,
            rate limiter as self.rate_limiter,
            amcl_pose topic subscriber with self.handle_amcl_pose as handler,
            action client with the move base action as self.action_client
            occupancy gris as self.occupancy_grid
        Additionally waits for simulation and action servers if synchronize is True
        @author Callum
        @type occupancy_grid: OccupancyGrid
        @param synchronize: True if the constructor should wait for the simulation and the action server to be ready
        @type synchronize: bool
        """
        rospy.init_node('map_navigation')
        self.rate_limiter = rospy.Rate(10)

        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.handle_amcl_pose)

        self.initialpose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

        self.action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.occupancy_grid = occupancy_grid

        if synchronize:
            self.synchronize()
            self.await_action_server()

    def synchronize(self):
        """
        Blocks until rospy.Time.now() returns a valid value
        """

    def await_action_server(self):
        """
        Blocks until action action server is ready
        """

    def set_initial_position(self, position):
        """
        Sends position to localize the nav stack and blocks until the amcl pose agrees
        @author Callum
        @param position: position for the nav stack to localize from
        @type position: Position
        """
        pose = PoseWithCovarianceStamped(
            header=get_header(),
            pose=PoseWithCovariance(
                pose=position.to_pose(),
                covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
            )
        )
        tries = 0
        while self.amcl_pos is None or \
                not (self.amcl_pos.x - 0.5 < position.x < self.amcl_pos.x + 0.5 and
                     self.amcl_pos.y - 0.5 < position.y < self.amcl_pos.y + 0.5 and
                     self.amcl_pos.theta - 0.5 < position.theta < self.amcl_pos.theta + 0.5):
            tries += 1
            self.initialpose_pub.publish(pose)
            self.rate_limiter.sleep()

        print 'Set pose to {} after {} tries'.format(position, tries)

    def send_goal(self, position, await=True, timeout=rospy.Duration(60)):
        """
        Sends goal to position stack and optionally awaits it's completion
        @param position: goal position
        @type position: Position
        @param await: True if the function should block until completion, False otherwise
        @type await: bool
        @param timeout: value of timeout passed when asking the action client to wait for a result
        @type timeout: rospy.Duration
        @return: True if goal is reached, False otherwise
        @rtype: bool
        """

    def move_along_path(self, checkpoints, marking=True):
        """
        Moves robot along checkpoints and optionally marks the occupancy grid. Ends early if a checkpoint fails
        @type checkpoints:  List[Position]
        @param marking: True if the occupancy grid should be marked, False otherwise
        @type marking: bool
        """

    def handle_amcl_pose(self, msg):
        """
        Parses mgs and sets self.amcl_pose with the current estimated pose.
        Optionally marks the occupancy grid if self.marking is True
        @type msg: PoseWithCovarianceStamped
        """


if __name__ == '__main__':
    # Reset simulation
    rospy.ServiceProxy('gazebo/reset_simulation', Empty)()
    launch_nav_stack()
    nav = MapNavigation(OccupancyGrid((10, 10), (20, 20), 0.05))
    nav.set_initial_position(Position(-3, 1))
    nav.move_along_path([
        Position(-1, 2),
        Position(5, 1),
        Position(6.5, -4.5),
        Position(1, 3),
        Position(-6.5, 2.5),
        Position(-6.5, -2)
    ])
