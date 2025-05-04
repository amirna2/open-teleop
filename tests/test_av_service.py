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

    # Wait a moment before enabling
    time.sleep(1.0)

    # --- FIRST ACTION_ENABLE Call --- 
    if assigned_id:
        node.get_logger().info(f"--- Sending first ACTION_ENABLE request for stream_id: {assigned_id} ---")
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
            )
        else:
            node.get_logger().error(f'ACTION_ENABLE service call failed: {future.exception()}')
    else:
        node.get_logger().error("Cannot send ACTION_ENABLE, no valid stream ID received from ACTION_ADD.")

    # Let the stream run for 30 seconds
    node.get_logger().info(f"--- Stream enabled and running, waiting for 30 seconds... ---")
    time.sleep(30.0)

    # --- ACTION_DISABLE Call --- 
    if assigned_id:
        node.get_logger().info(f"--- Sending ACTION_DISABLE request for stream_id: {assigned_id} ---")
        disable_request = ManageStream.Request()
        disable_request.action = ManageStream.Request.ACTION_DISABLE
        disable_request.stream_id = assigned_id

        node.get_logger().info(f'Sending DISABLE request: {disable_request}')
        future = client.call_async(disable_request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            disable_response = future.result()
            node.get_logger().info(
                f'DISABLE Response received: success={disable_response.success}, '
                f'message="{disable_response.message}"'
            )
        else:
            node.get_logger().error(f'ACTION_DISABLE service call failed: {future.exception()}')
    else:
        node.get_logger().error("Cannot send ACTION_DISABLE, no valid stream ID available.")

    # Wait a moment after disabling
    node.get_logger().info("--- Stream disabled, waiting for 5 seconds... ---")
    time.sleep(5.0)

    # --- SECOND ACTION_ENABLE Call --- 
    if assigned_id:
        node.get_logger().info(f"--- Sending second ACTION_ENABLE request for stream_id: {assigned_id} ---")
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
            )
        else:
            node.get_logger().error(f'ACTION_ENABLE service call failed: {future.exception()}')
    else:
        node.get_logger().error("Cannot send ACTION_ENABLE, no valid stream ID available.")

    # Let the stream run for 10 more seconds
    node.get_logger().info(f"--- Stream enabled again, waiting for 10 seconds... ---")
    time.sleep(10.0)

    # --- ACTION_REMOVE Call --- 
    if assigned_id:
        node.get_logger().info(f"--- Sending ACTION_REMOVE request for stream_id: {assigned_id} ---")
        remove_request = ManageStream.Request()
        remove_request.action = ManageStream.Request.ACTION_REMOVE
        remove_request.stream_id = assigned_id

        node.get_logger().info(f'Sending REMOVE request: {remove_request}')
        future = client.call_async(remove_request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            remove_response = future.result()
            node.get_logger().info(
                f'REMOVE Response received: success={remove_response.success}, '
                f'message="{remove_response.message}"'
            )
        else:
            node.get_logger().error(f'ACTION_REMOVE service call failed: {future.exception()}')
    else:
        node.get_logger().error("Cannot send ACTION_REMOVE, no valid stream ID available.")

    node.get_logger().info("--- Test script completed successfully ---")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 