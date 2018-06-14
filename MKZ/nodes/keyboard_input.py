#!/usr/bin/env python

# roslib related
import rospy

# messages related
from std_msgs.msg import String


KEYBOARD_TOPIC = "mkz_key_command"

def pub_keys():
    pub = rospy.Publisher(KEYBOARD_TOPIC, String, queue_size=10)
    rospy.init_node('keyboard', anonymous=True)

    while not rospy.is_shutdown():
        while True:
            try:
                key = raw_input()
                break
            except (EOFError):
                pass
        pub.publish(key)

if __name__ == "__main__":
    pub_keys()
