#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist
from turtlebot3_object_tracker.srv import GetError, GetErrorRequest, GetErrorResponse


class Controller:
    def __init__(self) -> None:
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()

        self.k_p_o = .005
        self.k_p_v = .005

        # The "p" parameter for your p-controller, TODO: you need to tune this
        self.angular_vel_coef = 1

        # TODO: Create a service proxy for your human detection service
        
        # TODO: Create a publisher for your robot "cmd_vel"
        self.cmd_publisher = rospy.Publisher('/follower/cmd_vel' , Twist , queue_size=10)
        self.rate = 1/.005
        self.r = rospy.Rate(self.rate)
    
    def get_error(self):

        rospy.wait_for_service('/mammad')

        try:

            GetNext = rospy.ServiceProxy('/mammad', GetError)

            next_dest : GetErrorResponse = GetNext(1.0)

            return next_dest
        
        except rospy.ServiceException as e:
            rospy.logerr("some fucking thig just happend")

    def control_effort(self, sub):

        err = 0

        if sub == 0 :
            err = self.get_error().error_x
            p = self.k_p_o*err
        if sub == 1 :
            err = self.get_error().error_y
            p = self.k_p_v*err

        return p 

    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                # TODO: Call your service, ride your robot

                self.move.angular.z = self.control_effort(0)

                self.move.linear.x = self.control_effort(1)

                self.cmd_publisher.publish(self.move)

                self.r.sleep()

                

        except rospy.exceptions.ROSInterruptException:
            pass
                

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
    

