#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from open_teleop_msgs.srv import ManageStream
import sys
import time

def main(args=None):
    rclpy.init(args=args)

    # Create a temporary node for the client
    node = rclpy.create_node('av_service_test_client')

    # Create the service client
    client = node.create_client(ManageStream, '/open_teleop_av_node/manage_stream')

    # Wait for the service to be available
    MAX_WAIT_SEC = 10.0
    wait_start_time = time.time()
    while not client.wait_for_service(timeout_sec=1.0):
        if time.time() - wait_start_time > MAX_WAIT_SEC:
            node.get_logger().error('Service /open_teleop_av_node/manage_stream not available after waiting.')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        node.get_logger().info('Service not available, waiting again...')

    # Create the request object
    request = ManageStream.Request()
    request.action = ManageStream.Request.ACTION_ADD # Use constant for clarity
    request.stream_id = "programmatic_test_stream" # Optional, can leave empty
    request.input_ros_topic = "/test/image_prog"
    request.output_ott_topic = "teleop.test.video_prog"
    request.encoding_format = "video/h264"
    request.encoder_params = "{ bitrate: 2000, quality: 25 }"

    node.get_logger().info(f'Sending request: {request}')

    # Call the service asynchronously
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        response = future.result()
        node.get_logger().info(
            f'Response received: success={response.success}, ' 
            f'message="{response.message}", ' 
            f'assigned_stream_id="{response.assigned_stream_id}"'
        )
    else:
        node.get_logger().error(f'Service call failed: {future.exception()}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 