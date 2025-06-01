"""
Connection Manager stub for Media Gateway.
"""

import structlog

class ConnectionManager:
    """Stub implementation of Connection Manager."""
    
    def __init__(self, endpoint):
        self.endpoint = endpoint
        self.logger = structlog.get_logger(__name__)
        
    async def connect(self):
        """Connect to Controller (stub)."""
        self.logger.info("Connecting to Controller (stub)", endpoint=self.endpoint)
        
    async def disconnect(self):
        """Disconnect from Controller (stub)."""
        self.logger.info("Disconnecting from Controller (stub)")
        
    def publish_frame(self, frame):
        """Publish frame to Controller (stub)."""
        pass 