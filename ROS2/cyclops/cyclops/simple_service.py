import rclpy
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Trigger, 'service', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.success = True
        response.message = "this worked"
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
