import numpy as np

import rclpy
from rclpy.node import Node

# from crazyflie_interfaces.msg import LogDataGeneric
# from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import Joy

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_aug')

        self.setParamsServiceTeleop = self.create_client(SetParameters, "/teleop/set_parameters")

        self.subscription1 = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        self.buttons_prev = None



    def joy_callback(self, msg: Joy):
        buttons = np.array(msg.buttons)
        
        if self.buttons_prev is None:
            self.buttons_prev = buttons
            return

        buttonsChange = buttons - self.buttons_prev
        self.buttons_prev = buttons

        # 0 - green button
        if buttonsChange[0] == 1:
            # switch to manual teleoperation
            self.setParamTeleopString("mode", "cmd_vel_world")

        # land button: switch back to regular lee controller!
        if buttonsChange[6] == 1:
            # make sure teleoperation is disabled
            self.setParamTeleopString("mode", "high_level")


    def setParamTeleopString(self, param_name, value):
        param_type = ParameterType.PARAMETER_STRING
        param_value = ParameterValue(type=param_type, string_value=str(value))
        req = SetParameters.Request()
        req.parameters = [Parameter(name=param_name, value=param_value)]
        self.setParamsServiceTeleop.call_async(req)

def main(args=None):
    rclpy.init(args=args)

    node = TeleopNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()