#!/usr/bin/env python3
# zmq_client.py - Stub implementation for ZeroMQ client

class ZmqClient:
    """
    Stub implementation of the ZeroMQ client for the ROS Gateway.
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
        
        if logger:
            logger.info(f"ZmqClient stub initialized with controller_address={controller_address}")
    
    def send_message(self, message):
        """
        Stub method for sending a message to the controller.
        
        Args:
            message: The message to send
        """
        if self.logger:
            self.logger.info("ZmqClient stub: send_message called (not implemented)")
        
    def start_receiving(self, callback):
        """
        Stub method for starting to receive messages from the controller.
        
        Args:
            callback: Function to call when a message is received
        """
        if self.logger:
            self.logger.info("ZmqClient stub: start_receiving called (not implemented)")
    
    def shutdown(self):
        """Stub method for shutting down the client."""
        if self.logger:
            self.logger.info("ZmqClient stub: shutdown called") 