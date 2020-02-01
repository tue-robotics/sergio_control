# ROS
import rospy
import std_msgs.msg


class EButton(object):
    """ Interfaces to the main ebutton topic.
    """
    def __init__(self, topic, ebutton_handler):
        """ Initialize

        :param topic: (str) main ebutton topic
        """
        self._sub = rospy.Subscriber(topic, std_msgs.msg.String, ebutton_handler, queue_size=1)
        self._handler = ebutton_handler
