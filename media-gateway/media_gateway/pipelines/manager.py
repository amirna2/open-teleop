"""
Pipeline Manager stub for Media Gateway.
"""

import structlog

class PipelineManager:
    """Stub implementation of Pipeline Manager."""
    
    def __init__(self, frame_router):
        self.frame_router = frame_router
        self.logger = structlog.get_logger(__name__)
        
    async def create_stream(self, stream_config):
        """Create a new stream (stub)."""
        self.logger.info("Creating stream (stub)", device_id=stream_config.device_id)
        return True
        
    async def stop_all_streams(self):
        """Stop all streams (stub)."""
        self.logger.info("Stopping all streams (stub)")
        
    async def health_check(self):
        """Perform health check (stub)."""
        pass
        
    async def update_streams(self, new_config):
        """Update streams with new configuration (stub)."""
        self.logger.info("Updating streams (stub)") 