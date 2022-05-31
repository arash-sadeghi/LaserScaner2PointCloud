#!/usr/bin/python  
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np
class Converter:
    def __init__(self):
        rospy.init_node("laser2pc")
        # pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
        rospy.Subscriber("drone/laser", LaserScan, self.laser_callback,queue_size=1)
        rospy.Subscriber("drone/gt_pose", LaserScan, self.position_callback,queue_size=1)
         
        self.output_path='cut_out.txt'
        self.output_file=open(self.output_path,'w')

    def position_callback(self,data):
        orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.poses = np.array([data.position.x , data.position.y , data.position.z , yaw])

    def lasr_callback(self,msg):
        ranges=msg.ranges
        #! cheat
        ranges=
        self.output_file.write()
if __name__=='__main__':
    print("hello")
    coverter = Converter()
    rospy.spin()



