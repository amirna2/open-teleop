#!/usr/bin/env python3
# zmq_client.py - Enhanced stub implementation for ZeroMQ client

import threading
import time
import json
import random

class ZmqClient:
    """
    Enhanced stub implementation of the ZeroMQ client for the ROS Gateway.
    Includes simulation of inbound messages for testing.
    """
    
    def __init__(self, controller_address, publish_address, buffer_size=1000, 
                 reconnect_interval_ms=1000, logger=None):
        """
        Initialize the ZeroMQ client.
        
        Args:
            controller_address: Address to connect to for sending messages to controller
            publish_address: Address to bind to for receiving messages from controller
            buffer_size: Size of the message buffer
            reconnect_interval_ms: Reconnection interval in milliseconds
            logger: Logger instance
        """
        self.controller_address = controller_address
        self.publish_address = publish_address
        self.buffer_size = buffer_size
        self.reconnect_interval_ms = reconnect_interval_ms
        self.logger = logger
        self.callbacks = []
        self.running = False
        self.receive_thread = None
        
        if logger:
            logger.info(f"ZmqClient stub initialized with controller_address={controller_address}")
    
    def send_message(self, message):
        """
        Stub method for sending a message to the controller.
        
        Args:
            message: The message to send
        """
        if self.logger:
            self.logger.info(f"ZmqClient stub: send_message called with: {message[:100]}...")
        
    def start_receiving(self, callback):
        """
        Start simulating message reception from the controller.
        
        Args:
            callback: Function to call when a message is received
        """
        if self.logger:
            self.logger.info("ZmqClient stub: start_receiving called with simulation enabled")
        
        self.callbacks.append(callback)
        
        if not self.running:
            self.running = True
            self.receive_thread = threading.Thread(target=self._simulate_receive, daemon=True)
            self.receive_thread.start()
    
    def _simulate_receive(self):
        """Simulate receiving messages from the controller periodically"""
        if self.logger:
            self.logger.info("ZmqClient stub: Starting message simulation thread")
        
        while self.running:
            # Wait for a random interval between 1-5 seconds
            time.sleep(random.uniform(1.0, 5.0))
            
            # Create a simulated velocity command
            message = self._create_simulated_velocity_command()
            
            if self.logger:
                self.logger.info(f"ZmqClient stub: Simulating received message: {message}")
            
            # Call all registered callbacks with the simulated message
            for callback in self.callbacks:
                callback(message)
    
    def _create_simulated_velocity_command(self):
        """Create a simulated velocity command message"""
        # Simulate random motion commands
        linear_x = random.uniform(-0.5, 0.5)  # m/s
        angular_z = random.uniform(-0.3, 0.3)  # rad/s
        
        # Create a simple mock message format
        # In a real implementation, this would be properly formatted according to the protocol
        mock_message = {
            "topic": "teleop.control.velocity",
            "timestamp": time.time(),
            "data": {
                "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
            }
        }
        
        return json.dumps(mock_message)
    
    def shutdown(self):
        """Shutdown the client and stop simulation."""
        if self.logger:
            self.logger.info("ZmqClient stub: shutdown called")
        
        self.running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0) 