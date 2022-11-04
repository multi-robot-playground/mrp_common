#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from mrp_common_msgs.msg import MonitoredNode
from mrp_common_msgs.srv import MonitoredNodeArray


class StateChangeRequestClient(Node):
    def __init__(self, robot_name='robot0'):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(MonitoredNodeArray, f'/{robot_name}/lifecycle_manager/change_monitored_nodes_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MonitoredNodeArray.Request()

    def send_request(self, monitored_nodes):
        self.req.nodes = monitored_nodes
        print("Headad")
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    monitored_node_list = []
    motion_planner_node = MonitoredNode()
    motion_planner_node.name = '/robot0/motion_planner_server'
    motion_planner_node.command = MonitoredNodeArray.Request.ACTIVATE
    monitored_node_list.append(motion_planner_node)

    change_state_cli = StateChangeRequestClient()
    res = change_state_cli.send_request(monitored_node_list)
    print(res)

    change_state_cli.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

