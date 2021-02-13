
import rospy
from geometry_msgs.msg import Twist

class control_turtlebot():
    def __init__(self):
        rospy.init_node('Node_name', anonymous=True)
        
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel_object = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        vel_msg.linear.x = -0.2
        
        #vel_msg.linear.y = 0
        #vel_msg.linear.z = 0

        #vel_msg.angular.x = 0.2
        #vel_msg.angular.y = 0.2
        vel_msg.angular.z = 0.2
        #vel_msg.linear.y = 0

        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.cmd_vel_object.publish(vel_msg)
            r.sleep()
    
    def shutdown(self):
        rospy.sleep(1)
 
        
if __name__ == '__main__':
    try:    
        control_turtlebot()
    except rospy.ROSInterruptException:
        pass
    except:
        rospy.loginfo('Node Terminated')
