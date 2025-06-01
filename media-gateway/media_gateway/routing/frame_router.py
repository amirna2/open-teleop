"""
Frame Router stub for Media Gateway.
"""

import structlog

class FrameRouter:
    """Stub implementation of Frame Router."""
    
    def __init__(self, connection_manager):
        self.connection_manager = connection_manager
        self.logger = structlog.get_logger(__name__)
        
    def on_frame_ready(self, frame_data, metadata):
        """Handle incoming frame (stub)."""
        self.logger.debug("Frame received (stub)", size=len(frame_data)) 