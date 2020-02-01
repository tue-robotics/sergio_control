# Hardware manager
from robot_part import RobotPart


class HardwareManager(object):
    """ Glues the dashboard, hardware status and ROS Control processes
    """
    def __init__(self):
        """ Constructor
        """
        self._parts = {}

    def _load_controller(self, name):
        self._load_srv.call(LoadControllerRequest(name=name))

    def _unload_controller(self, name):
        self._unload_srv.call(UnloadControllerRequest(name=name))

    def _start_controller(self, name):
        strict = SwitchControllerRequest.STRICT
        req = SwitchControllerRequest(start_controllers=[name],
                                      stop_controllers=[],
                                      strictness=strict)
        self._switch_srv.call(req)

    def _stop_controller(self, name):
        strict = SwitchControllerRequest.STRICT
        req = SwitchControllerRequest(start_controllers=[],
                                      stop_controllers=[name],
                                      strictness=strict)
        self._switch_srv.call(req)
