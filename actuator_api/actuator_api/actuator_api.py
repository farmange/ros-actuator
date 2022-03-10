import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from actuator_msgs.action import ActuatorMove
from actuator_msgs.msg import ControlMode

class ActuatorApi:
  # Singleton
  instance = None

  def __init__(self):
    if not ActuatorApi.instance:
      ActuatorApi.instance = ActuatorApi.__ActuatorApi()

  def __getattr__(self, name):
    return getattr(self.instance, name)

  class __ActuatorApi(Node):
    def __init__(self):
      super().__init__('actuator_api')
      self._actuator_move_action_client = ActionClient(self, ActuatorMove, 'actuator_move')
      if(not self._actuator_move_action_client.wait_for_server(timeout_sec=1.0)):
        self.get_logger().error('Action server actuator_move not ready !')
        raise Exception("Action server actuator_move not ready !")
      pass

    def move_to(self, position):
        goal = ActuatorMove.Goal()
        goal.mode.id = ControlMode.ID_MODE_POSITION
        goal.cmd_value = position

        return self.__execute_action('actuator_move', ActuatorMove, goal)

    def apply_torque(self, torque):
        goal = ActuatorMove.Goal()
        goal.mode.id = ControlMode.ID_MODE_TORQUE
        goal.cmd_value = torque

        return self.__execute_action('actuator_move', ActuatorMove, goal)

    def stop(self):
        goal = ActuatorMove.Goal()
        goal.mode.id = ControlMode.ID_MODE_OFF
        goal.cmd_value = 0.0

        return self.__execute_action('actuator_move', ActuatorMove, goal)

    def __execute_action(self, action_name, action_msg_type, goal):
      future = self._actuator_move_action_client.send_goal_async(goal)
      rclpy.spin_until_future_complete(self, future)
        
      goal_handle = future.result()
      get_result_future = goal_handle.get_result_async()
      rclpy.spin_until_future_complete(self, get_result_future)

      get_action_future = get_result_future.result()
      return get_action_future.result.success


# This is an example code to test the ActuatorApi class, do not modify this code
def main(args=None):
    rclpy.init(args=args)
    print("=== Start test ===")

    try:
      actuator_api = ActuatorApi()

      for i in range(0,5):
        print("--- Iteration n°" + str(i) + " ---")

        print("Move to position 1")
        result = actuator_api.move_to(20.0)
        print("Result success: " + str(result))

        print("Sleep")
        time.sleep(1)

        print("Move to position 2")
        result = actuator_api.move_to(-20.0)
        print("Result success: " + str(result))

        torque = 3.0
        print("Apply torque " + str(torque) + "Nm")
        result = actuator_api.apply_torque(torque)
        print("Result success: " + str(result))

        print("Sleep")
        time.sleep(1)

        torque = -3.0
        print("Apply torque " + str(torque) + "Nm")
        result = actuator_api.apply_torque(torque)
        print("Result success: " + str(result))

        print("Sleep")
        time.sleep(1)

      print("Stop actuator ")
      result = actuator_api.stop()
      print("Result success: " + str(result))

    except Exception as e:
      print("Exception")
      print(e)
    print("=== Finish test ===")

if __name__ == '__main__':
    main()