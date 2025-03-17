#!/usr/bin/env python3
# zmq_client.py - Implementation for ZeroMQ client

import threading
import time
import json
import zmq
import logging

class ZmqClient:
    """
    Implementation of the ZeroMQ client for the ROS Gateway.
    Handles communication with the Open-Teleop Controller.
    """
    
    def __init__(self, controller_address, publish_address, buffer_size=1000, 
                 reconnect_interval_ms=1000, logger=None):
        """
        Initialize the ZeroMQ client.
        
        Args:
            controller_address: Address to connect to for sending messages to controller
            publish_address: Address to connect to for receiving messages from controller
            buffer_size: Size of the message buffer
            reconnect_interval_ms: Reconnection interval in milliseconds
            logger: Logger instance
        """
        self.controller_address = controller_address
        self.publish_address = publish_address
        self.buffer_size = buffer_size
        self.reconnect_interval_ms = reconnect_interval_ms
        self.logger = logger
        self.callbacks = {}
        self.running = False
        self.receive_thread = None
        
        # Set up ZMQ context
        self.context = zmq.Context()
        self.req_socket = None
        self.sub_socket = None
        
        # Initialize sockets
        self._initialize_sockets()
        
        if logger:
            logger.info(f"ZmqClient initialized: controller={controller_address}, pub={publish_address}")
    
    def _initialize_sockets(self):
        """Initialize ZeroMQ sockets"""
        # REQ socket for request-reply pattern
        self.req_socket = self.context.socket(zmq.REQ)
        self.req_socket.connect(self.controller_address)
        
        # SUB socket for subscription pattern
        self.sub_socket = self.context.socket(zmq.SUB)
        
        # Connect to the controller's publisher port
        self.sub_socket.connect(self.publish_address)
        if self.logger:
            self.logger.info(f"ZmqClient: Connected sub socket to {self.publish_address}")
        
        if self.logger:
            self.logger.info(f"ZmqClient: Sockets initialized and connected")
    
    def send_message(self, message):
        """
        Send a message to the controller.
        
        Args:
            message: The message to send (JSON string)
        
        Returns:
            Reply message from the controller
        """
        if not isinstance(message, str):
            message = json.dumps(message)
            
        if self.logger:
            self.logger.info(f"ZmqClient: Sending message: {message[:100]}...")
        
        try:
            self.req_socket.send_string(message)
            reply = self.req_socket.recv_string()
            
            if self.logger:
                self.logger.info(f"ZmqClient: Received reply: {reply[:100]}...")
                
            return reply
        except zmq.ZMQError as e:
            if self.logger:
                self.logger.error(f"ZmqClient: Error sending message: {e}")
            return None
    
    def request_config(self):
        """
        Request configuration from the controller.
        
        Returns:
            Configuration dictionary or None if request failed
        """
        config_request = {
            "type": "CONFIG_REQUEST",
            "timestamp": time.time()
        }
        
        if self.logger:
            self.logger.info("ZmqClient: Requesting configuration from controller")
        
        try:
            response_str = self.send_message(config_request)
            if response_str:
                response = json.loads(response_str)
                if response["type"] == "CONFIG_RESPONSE":
                    if self.logger:
                        self.logger.info("ZmqClient: Received configuration from controller")
                    return response["data"]
                else:
                    if self.logger:
                        self.logger.warning(f"ZmqClient: Unexpected response type: {response['type']}")
            return None
        except Exception as e:
            if self.logger:
                self.logger.error(f"ZmqClient: Error requesting configuration: {e}")
            return None
    
    def start_receiving(self, topic, callback):
        """
        Start receiving messages from the controller on a specific topic.
        
        Args:
            topic: Topic to subscribe to (e.g., "teleop.control.velocity")
            callback: Function to call when a message is received
        """
        if topic not in self.callbacks:
            self.callbacks[topic] = []
        
        self.callbacks[topic].append(callback)
        
        # Subscribe to the topic
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        
        if self.logger:
            self.logger.info(f"ZmqClient: Subscribed to topic: {topic}")
        
        if not self.running:
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
    
    def subscribe_to_config_updates(self, callback):
        """
        Subscribe to configuration updates from the controller.
        
        Args:
            callback: Function to call when a configuration update is received
        """
        self.start_receiving("configuration.", callback)
    
    def _receive_loop(self):
        """Main loop for receiving messages from the controller"""
        if self.logger:
            self.logger.info("ZmqClient: Starting receive loop")
        
        poller = zmq.Poller()
        poller.register(self.sub_socket, zmq.POLLIN)
        
        poll_count = 0
        last_log_time = time.time()
        
        while self.running:
            try:
                poll_count += 1
                current_time = time.time()
                
                # Log poll stats every 5 seconds
                if current_time - last_log_time > 5:
                    if self.logger:
                        self.logger.debug(f"ZmqClient: Polling socket (count: {poll_count} in last 5s)")
                    poll_count = 0
                    last_log_time = current_time
                
                socks = dict(poller.poll(timeout=100))
                
                if self.sub_socket in socks and socks[self.sub_socket] == zmq.POLLIN:
                    if self.logger:
                        self.logger.debug("ZmqClient: POLLIN event detected, receiving message...")
                    
                    # Receive the topic and message
                    topic = self.sub_socket.recv_string()
                    message = self.sub_socket.recv_string()
                    
                    if self.logger:
                        self.logger.info(f"ZmqClient: Received message on topic {topic}")
                        self.logger.info(f"ZmqClient: Message content: {message[:100]}...")
                    
                    # Find matching topic callbacks
                    processed = False
                    for registered_topic, callbacks in self.callbacks.items():
                        if topic.startswith(registered_topic):
                            processed = True
                            if self.logger:
                                self.logger.info(f"ZmqClient: Processing message for topic {topic} (matches {registered_topic})")
                            for callback in callbacks:
                                try:
                                    callback(message)
                                except Exception as e:
                                    if self.logger:
                                        self.logger.error(f"ZmqClient: Callback error for topic {topic}: {str(e)}")
                                        import traceback
                                        self.logger.error(traceback.format_exc())
                                    
                    if not processed and self.logger:
                        self.logger.warning(f"ZmqClient: No callback registered for topic {topic}")
                else:
                    # No messages available this poll cycle
                    pass
            except zmq.ZMQError as e:
                if self.logger:
                    self.logger.error(f"ZmqClient: Error in receive loop: {e}")
                time.sleep(0.1)  # Short sleep to avoid tight loop on error
    
    def shutdown(self):
        """Shutdown the client and clean up resources."""
        if self.logger:
            self.logger.info("ZmqClient: Shutting down")
        
        self.running = False
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        
        if self.req_socket:
            self.req_socket.close()
        
        if self.sub_socket:
            self.sub_socket.close()
        
        self.context.term() 