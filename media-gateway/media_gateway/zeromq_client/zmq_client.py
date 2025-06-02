#!/usr/bin/env python3
"""
ZeroMQ Client for Media Gateway.

This is EXACTLY the same implementation as ros-gateway's ZmqClient.
"""

import threading
import time
import json
import zmq
import logging


class ZmqClient:
    """
    Implementation of the ZeroMQ client for the Media Gateway.
    Handles communication with the Open-Teleop Controller.
    
    This is EXACTLY the same as ros-gateway's ZmqClient.
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
        self._init_lock = threading.Lock()
        self._initialized = False
        
        # Set up ZMQ context
        self.context = zmq.Context()
        self.req_socket = None
        self.sub_socket = None
        self.pub_socket = None  # Add publisher socket for sending discovery messages
        
        # Initialize sockets
        self._initialize_sockets()
        
        if logger:
            logger.info(f"ZmqClient initialized: controller={controller_address}, pub={publish_address}")
    
    def _initialize_sockets(self):
        """Initialize ZeroMQ sockets and connect to the controller."""
        with self._init_lock:
            if self._initialized:
                return
                
            try:
                # REQ socket for sending requests to controller
                self.req_socket = self.context.socket(zmq.REQ)
                self.req_socket.connect(self.controller_address)
                
                # SUB socket for receiving messages from controller
                self.sub_socket = self.context.socket(zmq.SUB)
                
                # Set timeout for receive operations
                self.sub_socket.setsockopt(zmq.RCVTIMEO, 1000)
                
                # Connect to the controller's publisher port
                self.sub_socket.connect(self.publish_address)
                
                # PUB socket for publishing discovery messages
                # Connect to controller's subscriber port (which is the same as publish_address)
                self.pub_socket = self.context.socket(zmq.PUB)
                self.pub_socket.connect(self.publish_address)
                
                # Set send timeout for publisher
                self.pub_socket.setsockopt(zmq.SNDTIMEO, 1000)
                
                self._initialized = True
                
                if self.logger:
                    self.logger.info(f"ZmqClient: Sockets initialized and connected")
            except Exception as e:
                if self.logger:
                    self.logger.error(f"ZmqClient: Failed to initialize sockets: {e}")
                raise
    
    def send_message(self, topic, message):
        """
        Send a message to the controller.
        
        Args:
            topic: The topic to send the message to
            message: The message to send (JSON string)
        
        Returns:
            Reply message from the controller
        """
        with self._init_lock:
            if not self._initialized:
                self._initialize_sockets()
                
        if not isinstance(message, str):
            message = json.dumps(message)
            
        if self.logger:
            self.logger.debug(f"ZmqClient: Sending message to {topic}: {message[:100]}...")
        
        try:
            self.req_socket.send_string(message)
            reply = self.req_socket.recv_string()
            
            if self.logger:
                self.logger.debug(f"ZmqClient: Received reply: {reply[:100]}...")
                
            return reply
        except zmq.ZMQError as e:
            if self.logger:
                self.logger.error(f"ZmqClient: Error sending message: {e}")
            return None
    
    def publish_message(self, topic, message_type, data):
        """
        Publish a message to a topic on the Open Teleop protocol.
        
        Args:
            topic: The topic to publish to (e.g., "teleop.media.available_sources")
            message_type: The message type (e.g., "DEVICE_DISCOVERY")
            data: The message data (dict)
        
        Returns:
            True if published successfully, False otherwise
        """
        with self._init_lock:
            if not self._initialized:
                self._initialize_sockets()
        
        # Create message in Open Teleop format
        message = {
            "type": message_type,
            "timestamp": time.time(),
            "data": data
        }
        
        message_json = json.dumps(message)
        
        if self.logger:
            self.logger.debug(f"ZmqClient: Publishing to topic '{topic}': {message_json[:100]}...")
        
        try:
            # Send topic first, then message (ZeroMQ PUB pattern)
            self.pub_socket.send_string(topic, zmq.SNDMORE)
            self.pub_socket.send_string(message_json)
            
            if self.logger:
                self.logger.debug(f"ZmqClient: Successfully published message to topic '{topic}'")
                
            return True
        except zmq.ZMQError as e:
            if self.logger:
                self.logger.error(f"ZmqClient: Error publishing message to topic '{topic}': {e}")
            return False
    
    def send_request_binary(self, binary_data):
        """
        Send raw binary data using REQ socket and wait for a string reply.

        Args:
            binary_data: The raw bytes to send.

        Returns:
            Reply string from the controller or None if failed.
        """
        if self.logger:
            self.logger.debug(f"ZmqClient: Sending binary request ({len(binary_data)} bytes)")
        
        try:
            # Send the raw binary data
            self.req_socket.send(binary_data)
            
            # Receive the string reply (expected to be JSON ACK)
            reply = self.req_socket.recv_string()
            
            if self.logger:
                self.logger.debug(f"ZmqClient: Received binary request reply: {reply[:50]}...")
                
            return reply
        except zmq.ZMQError as e:
            if self.logger:
                self.logger.error(f"ZmqClient: Error sending/receiving binary request: {e}")
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
            response_str = self.send_message("config", config_request)
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
    
    def start_receiving(self, teleop_topic, callback):
        """
        Start receiving messages from the controller on a specific topic.
        
        Args:
            topic: Topic to subscribe to (e.g., "teleop.control.velocity")
            callback: Function to call when a message is received
        """
        with self._init_lock:
            if not self._initialized:
                self._initialize_sockets()
                
            if teleop_topic not in self.callbacks:
                self.callbacks[teleop_topic] = []
            
            self.callbacks[teleop_topic].append(callback)
            
            # Subscribe to the topic
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, teleop_topic)
            
            if self.logger:
                self.logger.info(f"ZmqClient: Subscribed to message on teleop-topic: {teleop_topic}")
            
            if not self.running:
                self.running = True
                # Make the thread non-daemon to prevent abrupt termination
                self.receive_thread = threading.Thread(target=self._receive_loop)
                self.receive_thread.start()
    
    def subscribe_to_config_updates(self, callback):
        """
        Subscribe to configuration updates from the controller.
        
        Args:
            callback: Function to call when a config update is received
        """
        config_topic = "configuration.notification"
        self.start_receiving(config_topic, callback)
        
        if self.logger:
            self.logger.info("ZmqClient: Subscribed to configuration updates")
    
    def _receive_loop(self):
        """Main receive loop for the ZMQ SUB socket."""
        if self.logger:
            self.logger.info("ZmqClient: Starting receive loop")
        
        try:
            while self.running:
                try:
                    # Non-blocking receive with timeout
                    if self.sub_socket.poll(1000):  # 1 second timeout
                        topic = self.sub_socket.recv_string(zmq.NOBLOCK)
                        message = self.sub_socket.recv_string(zmq.NOBLOCK)
                        
                        if self.logger:
                            self.logger.debug(f"ZmqClient: Received message on topic '{topic}': {message[:100]}...")
                        
                        # Call appropriate callbacks
                        if topic in self.callbacks:
                            for callback in self.callbacks[topic]:
                                try:
                                    callback(message)
                                except Exception as e:
                                    if self.logger:
                                        self.logger.error(f"ZmqClient: Error in callback for topic '{topic}': {e}")
                        else:
                            if self.logger:
                                self.logger.debug(f"ZmqClient: No callbacks registered for topic '{topic}'")
                                
                except zmq.Again:
                    # Timeout, continue loop
                    continue
                except Exception as e:
                    if self.logger:
                        self.logger.error(f"ZmqClient: Error in receive loop: {e}")
                    time.sleep(1)  # Brief pause before continuing
                    
        except Exception as e:
            if self.logger:
                self.logger.error(f"ZmqClient: Fatal error in receive loop: {e}")
        finally:
            if self.logger:
                self.logger.info("ZmqClient: Receive loop stopped")
    
    def shutdown(self):
        """Shutdown the ZmqClient and close connections."""
        if self.logger:
            self.logger.info("ZmqClient: Shutting down...")
        
        # Stop the receive thread
        self.running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
        
        # Close sockets
        with self._init_lock:
            if self.req_socket:
                self.req_socket.close()
                self.req_socket = None
            
            if self.sub_socket:
                self.sub_socket.close()
                self.sub_socket = None
            
            if self.pub_socket:
                self.pub_socket.close()
                self.pub_socket = None
            
            if self.context:
                self.context.term()
                self.context = None
            
            self._initialized = False
        
        if self.logger:
            self.logger.info("ZmqClient: Shutdown complete") 