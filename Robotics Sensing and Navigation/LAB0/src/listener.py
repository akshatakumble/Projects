import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.callback,
            10
        )

    def callback(self, msg):
        received_message = msg.data
        # Modify the string: swap the first two characters and append "!"
        if len(received_message) > 1:
            modified_message = received_message[1] + received_message[0] + received_message[2:] + "!"
        else:
            modified_message = received_message + "!"
        self.get_logger().info(f'I heard: "{modified_message}"')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

