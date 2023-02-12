import rospy
from std_msgs.msg import Bool, String

def laserCallback(data):
    rospy.loginfo("Human Detection:{}".format(data))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/yolov5/detect_human", String, laserCallback)
    rospy.spin()

if __name__ == "__main__":
    listener()
