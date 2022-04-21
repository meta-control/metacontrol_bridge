from sqlalchemy import true
import rospy

from lifecycle_msgs.srv import ChangeState, GetState, ChangeStateResponse, GetStateResponse
from lifecycle_msgs.msg import State, Transition

class MetacontrolNode:
    def __init__(self):
        print("Creating service")

        self.state = State()
        self.state.id = State.PRIMARY_STATE_UNCONFIGURED
        self.state.label = "unconfigured"

        self.set_state_srv = rospy.Service('~set_state', ChangeState, self.handle_change_state)
        self.get_state_srv = rospy.Service('~get_state', GetState, self.handle_get_state)

        print(self.state)
    
    def handle_change_state(self, req):
        if self.state.id == State.PRIMARY_STATE_UNCONFIGURED and req.transition.id == Transition.TRANSITION_CONFIGURE:
            ret_value = self.on_configure(self.state)
            if ret_value:
                self.state.id = State.PRIMARY_STATE_INACTIVE
                self.state.label = "inactive"
            return ChangeStateResponse(ret_value)
        elif self.state.id == State.PRIMARY_STATE_INACTIVE and req.transition.id == Transition.TRANSITION_ACTIVATE:
            ret_value = self.on_activate(self.state)
            if ret_value:
                self.state.id = State.PRIMARY_STATE_ACTIVE
                self.state.label = "active"
            return ChangeStateResponse(ret_value)
        elif self.state.id == State.PRIMARY_STATE_INACTIVE and req.transition.id == Transition.TRANSITION_CLEANUP:
            ret_value = self.on_cleanup(self.state)
            if ret_value:
                self.state.id = State.PRIMARY_STATE_UNCONFIGURED
                self.state.label = "unconfigured"
            return ChangeStateResponse(ret_value)
        elif self.state.id == State.PRIMARY_STATE_ACTIVE and req.transition.id == Transition.TRANSITION_DEACTIVATE:
            ret_value = self.on_deactivate(self.state)
            if ret_value:
                self.state.id = State.PRIMARY_STATE_INACTIVE
                self.state.label = "inactive"
            return ChangeStateResponse(ret_value)
        if self.state.id == State.PRIMARY_STATE_UNCONFIGURED and req.transition.id == Transition.TRANSITION_UNCONFIGURED_SHUTDOWN:
            ret_value = self.on_shutdown(self.state)
            if ret_value:
                self.state.id = State.PRIMARY_STATE_FINALIZED
                self.state.label = "finalized"
            return ChangeStateResponse(ret_value)
        elif self.state.id == State.PRIMARY_STATE_INACTIVE and req.transition.id == Transition.TRANSITION_INACTIVE_SHUTDOWN:
            ret_value = self.on_shutdown(self.state)
            if ret_value:
                self.state.id = State.PRIMARY_STATE_FINALIZED
                self.state.label = "finalized"
            return ChangeStateResponse(ret_value)
        elif self.state.id == State.PRIMARY_STATE_ACTIVE and req.transition.id == Transition.TRANSITION_ACTIVE_SHUTDOWN:
            ret_value = self.on_shutdown(self.state)
            if ret_value:
                self.state.id = State.PRIMARY_STATE_FINALIZED
                self.state.label = "finalized"
            return ChangeStateResponse(ret_value)
        return ChangeStateResponse(False)

    def handle_get_state(self, req):
        return GetStateResponse(self.state)

    def on_configure(self, previous):
        return True

    def on_cleanup(self, previous):
       return True

    def on_activate(self, previous):
       return True

    def on_deactivate(self, previous):
       return True

    def on_shutdown(self, previous):
       return True
