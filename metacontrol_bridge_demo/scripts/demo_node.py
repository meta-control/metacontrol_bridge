#!/usr/bin/env python
import rospy
from metacontrol_bridge.metacontrol_bridge import MetacontrolNode

class MyNode(MetacontrolNode):
    def __init__(self):
        MetacontrolNode.__init__(self)
    
    def on_configure(self):
        print("CONFIGURING")
        return True

    def on_configure(self):
        return True

    def on_cleanup(self):
       return True

    def on_activate(self):
       return True

    def on_deactivate(self):
       return True

    def on_shutdown(self):
       return True

if __name__ == '__main__':
  rospy.init_node('listener')
  my_node = MyNode()

  rospy.spin()

