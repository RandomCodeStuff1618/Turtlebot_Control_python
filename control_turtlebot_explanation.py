import rospy
#rospy is a pure Python client library for ROS. 
# The rospy client API enables Python programmers to quickly interface with ROS Topics, Services, and Parameters. 
# The design of rospy favors implementation speed (i.e. developer time) over runtime performance so that algorithms 
# can be quickly prototyped and tested within ROS. It is also ideal for non-critical-path code, such as configuration 
# and initialization code. Many of the ROS tools are written in rospy to take advantage of the type introspection 
# capabilities. Many of the ROS tools, such as rostopic and rosservice, are built on top of rospy.

from geometry_msgs.msg import Twist
## This expresses velocity in free space broken into its linear and angular parts.
#Vector3  linear
#Vector3  angular

# I suggest you go through this, http://wiki.ros.org/rospy_tutorials
class control_turtlebot():
    def __init__(self):
        #some code  You can only have one node in a rospy process, so you can only call rospy.init_node() once. 
        #http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
        """
        The anonymous keyword argument is mainly used for nodes where you normally expect many of them to be 
        running and don't care about their names (e.g. tools, GUIs). It adds a random number to the end of your node's name, 
        to make it unique. Unique names are more important for nodes like drivers, where it is an error if more than one is running. 
        If two nodes with the same name are detected on a ROS graph, the older node is shutdown. 
        """
        #"Node" is the ROS term for an executable that is connected to the ROS network. 
        rospy.init_node('Node_name', anonymous=True)
        
        #Works like any print function 
        rospy.loginfo("To stop TurtleBot CTRL + C")
        # What to do you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        # You should also know something called , rospy.spin()
        # The spin() code simply sleeps until the is_shutdown() flag is True. 
        # It is mainly used to prevent your Python Main thread from exiting.
        #For manual shutdown
        #rospy.signal_shutdown(reason) where reason is a human readable string
        
        self.cmd_vel_object = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        """
        This section of code defines the talker's interface to the rest of ROS. 
        pub = rospy.Publisher("cmd_vel", String, queue_size=10) 
        declares that your node is publishing to the cmd_vel topic using the message type String.

        queue_size argument is New in ROS hydro and limits the amount of queued messages 
        if any subscriber is not receiving them fast enough. 
        In older ROS distributions just omit the argument. 
        """

        # Twist is a datatype for velocity
        vel_msg = Twist()
        vel_msg.linear.x = -0.2
        
        #vel_msg.linear.y = 0
        #vel_msg.linear.z = 0

        #vel_msg.angular.x = 0.2
        #vel_msg.angular.y = 0.2
        vel_msg.angular.z = 0.2
        #vel_msg.linear.y = 0

        r = rospy.Rate(30)
        #Offers a way to loop at desired rate, this'll make sure we loop 30 times per second
        #We need to take care that our processing time doesn't exceed this
        while not rospy.is_shutdown():
            #In this case, the "work" is a call to pub.publish(hello_str) that publishes a string to our cmd_vel topic.
            self.cmd_vel_object.publish(vel_msg)
            
            r.sleep()
            #(You may also run across rospy.sleep() which is similar to time.sleep() 
            # except that it works with simulated time as well (see Clock).) 
            # http://wiki.ros.org/Clock
    
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel_object.publish(Twist())
	    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
        
if __name__ == '__main__':
    try:    
        control_turtlebot()
    except rospy.ROSInterruptException:
        pass
        #In addition to the standard Python __main__ check, this catches a rospy.ROSInterruptException exception, 
        # which can be thrown by rospy.sleep() and rospy.Rate.sleep() methods when Ctrl-C is pressed or your Node 
        # is otherwise shutdown. 
        # The reason this exception is raised is so that you don't accidentally continue executing code after the sleep().
    except:
        rospy.loginfo('Node Terminated')

    """
    Well see some more examples and try stuff out by yourself, these are some ggood examples:
    https://www.programcreek.com/python/example/70251/geometry_msgs.msg.Twist
    """
