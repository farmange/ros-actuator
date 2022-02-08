
from threading import Event

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import GoalResponse
from rclpy.duration import Duration

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from actuator_msgs.action import ActuatorMove
from actuator_msgs.msg import ControlCommand
from actuator_msgs.msg import ControlMode
from actuator_msgs.msg import ActuatorState
from actuator_msgs.srv import SetControlMode

import time

class ActuatorActionServer(Node):

    def __init__(self):
        super().__init__('actuator_action_server')

        # self.action_done_event = Event()
        # self.callback_group = ReentrantCallbackGroup()
        self.callback_group = ReentrantCallbackGroup()
        self.service_done_event = Event()


        self.action_server_ = ActionServer(
            self,
            ActuatorMove,
            'actuator_move',
            self.execute_callback,
            goal_callback=self.goal_callback,
            callback_group=self.callback_group
            )

        self.actuator_state_sub_ = self.create_subscription(
            ActuatorState,
            'actuator_state',
            self._actuator_state_cb,
            1,
            callback_group=self.callback_group)
        self.actuator_state_sub_  # prevent unused variable warning
    
        self.control_command_pub_ = self.create_publisher(ControlCommand, 'control_command', 1)

        self.set_controle_mode_srvclient_ = self.create_client(SetControlMode, 'set_control_mode')


        self.actuator_state_ = None

        self.POSITION_THRESHOLD = 0.1
        self.TORQUE_LIMIT = 14.0
        self.TIMEOUT = 50.0

    def _actuator_state_cb(self, msg):
        self.actuator_state_ = msg

    def goal_callback(self, goal_request):
        self.get_logger().info('Checking new goal...')

        if(goal_request.mode.id == ControlMode.ID_MODE_TORQUE):
            if(abs(goal_request.cmd_value) > self.TORQUE_LIMIT):
                self.get_logger().warn('Torque command higher than max admissible torque (' + str(self.TORQUE_LIMIT) + ')')
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def service_end_callback(self, future):
        self.service_done_event.set()


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = ActuatorMove.Result()
        result.success = False

        if(self.actuator_state_ == None):
            self.get_logger().error('Action server not ready (no fresh actuator state received)')
            goal_handle.abort()
            return result

        command_msg = ControlCommand()
        command_msg.mode = goal_handle.request.mode

        command_msg.torque = 0.0
        command_msg.position = 0.0
        command_msg.velocity = 0.0

        if(goal_handle.request.mode.id == ControlMode.ID_MODE_POSITION):
            command_msg.position = goal_handle.request.cmd_value
        elif(goal_handle.request.mode.id == ControlMode.ID_MODE_TORQUE):
            command_msg.torque = goal_handle.request.cmd_value
        elif(goal_handle.request.mode.id == ControlMode.ID_MODE_SPEED):
            command_msg.velocity = goal_handle.request.cmd_value

        self.control_command_pub_.publish(command_msg)


        # if(self.actuator_state_ == None):
        #     if(self.actuator_state_.mode.id != goal_handle.request.mode.id):
        #         if(self.set_controle_mode_srvclient_.service_is_ready()):
        #             req = SetControlMode.Request()
        #             req.mode.id = goal_handle.request.mode.id
        #             self.service_done_event.clear()

        #             future = self.set_controle_mode_srvclient_.call_async(req)

        #             ### 1
        #             # while(not future.done()):
        #             #     self.get_logger().info('Wait service response...')
        #             #     time.sleep(0.1)

        #             ### 2 
        #             # self.get_logger().info('Wait service response...')
        #             # rclpy.spin_until_future_complete(self, future)

        #             ### 3
        #             future.add_done_callback(self.service_end_callback)
        #             self.get_logger().info('Waiting for goal execution...')
        #             self.service_done_event.wait()
        #         else:
        #             self.get_logger().error('Service "set_control_mode" unavailable')
        #             goal_handle.abort()
        #             return result
        # else:
        #     self.get_logger().error('Action server not ready (no fresh actuator state received)')
        #     goal_handle.abort()
        #     return result

        if(goal_handle.request.mode.id == ControlMode.ID_MODE_POSITION):
            start_time = self.get_clock().now()
            while(self.actuator_state_.position < (goal_handle.request.cmd_value - self.POSITION_THRESHOLD) 
            or self.actuator_state_.position > (goal_handle.request.cmd_value + self.POSITION_THRESHOLD)):
                self.get_logger().debug('current : ' + str(self.actuator_state_.position) + ' | desired : ' + str(goal_handle.request.cmd_value))
                if self.get_clock().now()-start_time > Duration(seconds=self.TIMEOUT):
                    # TODO improve timeout in action server
                    self.get_logger().warn('Timeout occured : hard limit of ' + str(self.TIMEOUT) + ' seconds should not be exceed in position control mode')
                    goal_handle.abort()
                    return result

        self.get_logger().info('Moving actuator done : goal succeed ! ')
        goal_handle.succeed()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)

    actuator_action_server = ActuatorActionServer()
    executor = MultiThreadedExecutor(2)

    rclpy.spin(actuator_action_server, executor)


if __name__ == '__main__':
    main()