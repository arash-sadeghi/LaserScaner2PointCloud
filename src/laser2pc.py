#!/usr/bin/python  
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan

class Converter:
    def __init__(self):
        rospy.init_node("laser2pc")
        # pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
        rospy.Subscriber("drone/laser", LaserScan, self.scan_cb,queue_size=1)
        self.output_path='cut_out.txt'
        self.output_file=open(self.output_path,'w')

    def scan_cb(self,msg):
        self.output_file.write(str(msg.ranges))
        exit(0)
if __name__=='__main__':
    print("hello")
    coverter = Converter()
    rospy.spin()



