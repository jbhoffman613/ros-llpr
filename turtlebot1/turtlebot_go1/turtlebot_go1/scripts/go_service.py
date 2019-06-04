#!/usr/bin/env python
import rospy
import actionlib
import IPython

from turtlebot_go1.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class go_service(object):

    def __init__(self):
        # initialize the node
        rospy.init_node('go', anonymous=False)

        # set initial pose and publish to initialpose topic
        self.initPub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

        rospy.sleep(2)

        self.initial_pose = rospy.get_param("/turtlebot01/go/start")

        print(self.initial_pose)

        self.set_initial_pose()

        # create service
        self.s = rospy.Service('go_service', go_service_srv, self.handle_go_service)

        # load parameter value from launch file
        self.positions = []
        for i in range(5):
            self.positions.append(rospy.get_param("/turtlebot01/go/pos" + str(i)))


        # create move base action server
        self.goal_sent = False
        
        # What to do if shut down (e.g. Ctrl-C or failure)
        # rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("/turtlebot01/move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        
        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.spin()

    # to set initial position to /initialpose topic
    def set_initial_pose(self):
    	initPose_message = PoseWithCovarianceStamped()
        initPose_message.pose.pose.position.x = self.initial_pose[0]
        initPose_message.pose.pose.position.y = self.initial_pose[1]
        initPose_message.pose.pose.position.z = 0.0
        initPose_message.pose.pose.orientation.x = 0.0
        initPose_message.pose.pose.orientation.y = 0.0
        initPose_message.pose.pose.orientation.z = 0.0
        initPose_message.pose.pose.orientation.w = 1.0
        initPose_message.header.frame_id = "map"
        initPose_message.header.stamp = rospy.Time.now()

        self.initPub.publish(initPose_message)

    # return Pose message type
    def get_end_pose(self):
        listener = tf.TransformListener()
        trans = []
        quat = []
        rate = rospy.Rate(10.0)
        count = 0
        while count < 6:
        	try:
        		(trans,quat) = listener.lookupTransform('map', '/turtlebot01/base_link', rospy.Time(0))
        	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        		continue
        	count = count + 1
        	rate.sleep()
                
            
        end_pose = Pose()
        end_pose.position.x = trans[0]
        end_pose.position.y = trans[1]
        end_pose.position.z = trans[2]
        end_pose.orientation.x = quat[0]
        end_pose.orientation.y = quat[1]
        end_pose.orientation.z = quat[2]
        end_pose.orientation.w = quat[3]
        return end_pose

    # return ending image as Image message type
    def get_end_image(self):
        end_image = rospy.wait_for_message('/turtlebot01/camera/rgb/image_raw', Image)
        return end_image


    def handle_go_service(self, req):
        pos = self.positions[req.num.data]
        pos_x = pos[0]
        pos_y = pos[1]
        pos_dict = {'x': pos_x, 'y' : pos_y}

        re_message = self.go_to(pos_dict,
                {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000})

        p = self.get_end_pose()
        img = self.get_end_image()
        
        return go_service_srvResponse(p,img)



    def go_to(self, pos, quat): 
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result() 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")

        rospy.sleep(1)


if __name__ == '__main__':
    service = go_service()

