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

    # --- ACTION_ADD Call --- 
    node.get_logger().info("--- Sending ACTION_ADD request ---")
    add_request = ManageStream.Request()
    add_request.action = ManageStream.Request.ACTION_ADD
    # add_request.stream_id = "test_stream_1" # Can specify or let node generate
    add_request.input_ros_topic = "/test/image_prog"
    add_request.output_ott_topic = "teleop.test.video_prog"
    add_request.encoding_format = "video/h264"
    add_request.encoder_params = "{ bitrate: 2000, quality: 25 }"

    node.get_logger().info(f'Sending ADD request: {add_request}')
    future = client.call_async(add_request)
    rclpy.spin_until_future_complete(node, future)

    assigned_id = None
    if future.result() is not None:
        add_response = future.result()
        node.get_logger().info(
            f'ADD Response received: success={add_response.success}, ' 
            f'message="{add_response.message}", ' 
            f'assigned_stream_id="{add_response.assigned_stream_id}"'
        )
        if add_response.success and add_response.assigned_stream_id:
            assigned_id = add_response.assigned_stream_id
        else:
            node.get_logger().error("ACTION_ADD failed or did not return a valid stream ID. Aborting.")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
    else:
        node.get_logger().error(f'ACTION_ADD service call failed: {future.exception()}')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Wait a moment before enabling (optional)
    time.sleep(1.0)

    # --- ACTION_ENABLE Call --- 
    if assigned_id:
        node.get_logger().info(f"--- Sending ACTION_ENABLE request for stream_id: {assigned_id} ---")
        enable_request = ManageStream.Request()
        enable_request.action = ManageStream.Request.ACTION_ENABLE
        enable_request.stream_id = assigned_id

        node.get_logger().info(f'Sending ENABLE request: {enable_request}')
        future = client.call_async(enable_request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            enable_response = future.result()
            node.get_logger().info(
                f'ENABLE Response received: success={enable_response.success}, ' 
                f'message="{enable_response.message}"'
                # assigned_stream_id is not relevant for enable response
            )
        else:
            node.get_logger().error(f'ACTION_ENABLE service call failed: {future.exception()}')
    else:
        node.get_logger().error("Cannot send ACTION_ENABLE, no valid stream ID received from ACTION_ADD.")


    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 