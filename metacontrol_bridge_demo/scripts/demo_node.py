#!/usr/bin/env python
import rospy
from metacontrol_bridge.metacontrol_bridge import MetacontrolNode

class MyNode(MetacontrolNode):
    def __init__(self):
        MetacontrolNode.__init__(self)
    
    def on_configure(self, previous):
        print("CONFIGURING")

        param0_value = rospy.get_param('~param0', 1.0)
        param1_value = rospy.get_param('~param1', "hello")

        print("param0 value is [%lf]" % param0_value)
        print("param1 value is [%s]" % param1_value)

        return True

    def on_cleanup(self, previous):
        print("CLEANING UP")
        return True

    def on_activate(self, previous):
        print("ACTIVATING")
        return True

    def on_deactivate(self, previous):
        print("DEACTIVATING")
        return True

    def on_shutdown(self, previous):
        print("SHUTTING DOWN")
        return True

if __name__ == '__main__':
  rospy.init_node('listener')
  my_node = MyNode()

  rospy.spin()

