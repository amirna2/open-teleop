#!/usr/bin/env python3
# Copyright (c) 2023 Open-Teleop
# License: MIT

"""
System Diagnostics Bridge Node.

This node collects system metrics (CPU, memory, disk usage) and ROS2 node statuses,
and sends them to the Go Controller using FlatBuffers serialization and ZeroMQ.
"""

import time
import json
import psutil
import rclpy
import zmq
import flatbuffers
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import LoggingSeverity
import open_teleop_logger as log

# Import generated FlatBuffers code
from open_teleop.diagnostic import SystemMetrics, LoadAverage, Memory, Swap, NodeStatus


class SystemDiagnosticsNode(Node):
    """Collects system metrics and ROS node statuses and sends them to the controller."""

    def __init__(self):
        """Initialize the node."""
        super().__init__('system_diagnostics_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 'default_robot'),
                ('metrics_update_period', 1.0),
                ('nodes_update_period', 5.0),
                ('controller_address', 'localhost:8080'),
                ('zmq_port', 5555),
                ('console_log_level', 'INFO'),
            ]
        )
        
        self.robot_id = self.get_parameter('robot_id').value
        self.metrics_update_period = self.get_parameter('metrics_update_period').value
        self.nodes_update_period = self.get_parameter('nodes_update_period').value
        self.controller_address = self.get_parameter('controller_address').value
        self.zmq_port = self.get_parameter('zmq_port').value
        self.console_log_level = self.get_parameter('console_log_level').value
        
        log_level_map = {
            'DEBUG': log.DEBUG,
            'INFO': log.INFO,
            'WARNING': log.WARNING,
            'ERROR': log.ERROR,
            'CRITICAL': log.CRITICAL
        }
        console_level = log_level_map.get(self.console_log_level.upper(), log.INFO)
        
        self.logger = log.get_logger(
            name=f"{self.get_name()}",
            log_dir="/tmp/open_teleop_logs",
            console_level=console_level,
            file_level=log.DEBUG
        )
        
        self.validate_parameters()
        
        # Initialize ZeroMQ publisher
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        
        # Extract host from controller address (format: host:port)
        host = self.controller_address.split(':')[0]
        zmq_address = f"tcp://{host}:{self.zmq_port}"
        
        try:
            self.zmq_socket.connect(zmq_address)
            self.logger.info(f"Connected to ZeroMQ at {zmq_address}")
        except Exception as e:
            self.logger.error(f"Failed to connect to ZeroMQ at {zmq_address}: {e}")
        
        self.logger.info(f'Starting System Diagnostics Node for robot: {self.robot_id}')
        self.logger.info(f'Metrics update period: {self.metrics_update_period} seconds')
        self.logger.info(f'Nodes update period: {self.nodes_update_period} seconds')
        self.logger.info(f'Controller address: {self.controller_address}')
        
        self.metrics_timer = self.create_timer(
            self.metrics_update_period, 
            self.collect_and_send_system_metrics
        )
        
        self.nodes_timer = self.create_timer(
            self.nodes_update_period, 
            self.collect_and_send_node_statuses
        )

    def validate_parameters(self):
        """Validate and sanitize parameters to ensure they're within acceptable ranges."""
        if not self.robot_id or len(self.robot_id.strip()) == 0:
            self.logger.warning('Empty robot_id provided, using default_robot')
            self.robot_id = 'default_robot'
            
        if self.metrics_update_period <= 0:
            self.logger.warning(f'Invalid metrics_update_period: {self.metrics_update_period}, using default of 1.0')
            self.metrics_update_period = 1.0
            
        if self.nodes_update_period <= 0:
            self.logger.warning(f'Invalid nodes_update_period: {self.nodes_update_period}, using default of 5.0')
            self.nodes_update_period = 5.0
            
        if not self.controller_address or ':' not in self.controller_address:
            self.logger.warning(f'Invalid controller_address: {self.controller_address}, using localhost:8080')
            self.controller_address = 'localhost:8080'
            
        if self.zmq_port <= 0 or self.zmq_port > 65535:
            self.logger.warning(f'Invalid zmq_port: {self.zmq_port}, using default of 5555')
            self.zmq_port = 5555
            
        valid_log_levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
        if self.console_log_level.upper() not in valid_log_levels:
            self.logger.warning(f'Invalid console_log_level: {self.console_log_level}, using INFO')
            self.console_log_level = 'INFO'

    def collect_and_send_system_metrics(self):
        """Collect system metrics and send them to the controller using FlatBuffers."""
        try:
            cpu_percent = psutil.cpu_percent(interval=0.1)
            
            load_avg_1, load_avg_5, load_avg_15 = psutil.getloadavg()
            
            memory = psutil.virtual_memory()
            
            total_memory_mb = memory.total / (1024 * 1024)
            used_memory_mb = memory.used / (1024 * 1024) 
            available_memory_mb = memory.available / (1024 * 1024)
            
            memory_percent = memory.percent
            available_memory_percent = 100 - memory_percent
            
            swap = psutil.swap_memory()
            swap_total_mb = swap.total / (1024 * 1024)
            swap_used_mb = swap.used / (1024 * 1024)
            swap_percent = swap.percent
            
            disk = psutil.disk_usage('/')
            disk_percent = disk.percent
            
            # Build FlatBuffer
            builder = flatbuffers.Builder(1024)
            
            # Create string for robot_id
            robot_id_fb = builder.CreateString(self.robot_id)
            
            # Build load average object
            LoadAverage.Start(builder)
            LoadAverage.AddOneMin(builder, load_avg_1)
            LoadAverage.AddFiveMin(builder, load_avg_5)
            LoadAverage.AddFifteenMin(builder, load_avg_15)
            load_average = LoadAverage.End(builder)
            
            # Build memory object
            Memory.Start(builder)
            Memory.AddTotalMb(builder, float(round(total_memory_mb, 2)))
            Memory.AddUsedMb(builder, float(round(used_memory_mb, 2)))
            Memory.AddAvailableMb(builder, float(round(available_memory_mb, 2)))
            Memory.AddUsedPercent(builder, float(memory_percent))
            Memory.AddAvailablePercent(builder, float(available_memory_percent))
            memory_fb = Memory.End(builder)
            
            # Build swap object
            Swap.Start(builder)
            Swap.AddTotalMb(builder, float(round(swap_total_mb, 2)))
            Swap.AddUsedMb(builder, float(round(swap_used_mb, 2)))
            Swap.AddUsedPercent(builder, float(swap_percent))
            swap_fb = Swap.End(builder)
            
            # Start building system metrics
            SystemMetrics.Start(builder)
            SystemMetrics.AddSchemaVersion(builder, 1)  # Schema version
            SystemMetrics.AddTimestamp(builder, int(time.time() * 1000))  # Convert to milliseconds
            SystemMetrics.AddCpuUsage(builder, float(cpu_percent))
            SystemMetrics.AddLoadAverage(builder, load_average)
            SystemMetrics.AddMemory(builder, memory_fb)
            SystemMetrics.AddSwap(builder, swap_fb)
            SystemMetrics.AddDiskUsage(builder, float(disk_percent))
            SystemMetrics.AddRobotId(builder, robot_id_fb)
            
            # Add node statuses (empty list for now, populated in collect_and_send_node_statuses)
            SystemMetrics.AddNodeStatus(builder, 0)  # 0 means no vector
            
            # Finish the FlatBuffer
            metrics = SystemMetrics.End(builder)
            builder.Finish(metrics)
            
            # Get the serialized buffer
            buf = builder.Output()
            
            # Send the buffer via ZeroMQ
            try:
                self.zmq_socket.send(buf)
                self.logger.debug(f'Metrics sent to controller (size: {len(buf)} bytes)')
            except Exception as e:
                self.logger.error(f'Failed to send metrics: {e}')
            
            self.logger.info(f'System metrics: CPU={cpu_percent}%, Load={load_avg_1:.2f}, Memory={memory_percent}% ({round(used_memory_mb, 2)}MB/{round(total_memory_mb, 2)}MB), Disk={disk_percent}%, Swap={swap_percent}%')
            
        except Exception as e:
            self.logger.error(f'Error collecting system metrics: {e}')
            self.logger.trace(f'Stack trace for system metrics error')

    def collect_and_send_node_statuses(self):
        """Collect ROS node statuses and send them to the controller using FlatBuffers."""
        try:
            node_names = self.get_node_names()
            
            # Build FlatBuffer
            builder = flatbuffers.Builder(1024)
            
            # Create string for robot_id
            robot_id_fb = builder.CreateString(self.robot_id)
            
            # Create a vector of node statuses
            node_status_offsets = []
            for node_name in node_names:
                name_fb = builder.CreateString(node_name)
                status_fb = builder.CreateString("active")  # For now, just mark all as active
                
                NodeStatus.Start(builder)
                NodeStatus.AddName(builder, name_fb)
                NodeStatus.AddStatus(builder, status_fb)
                NodeStatus.AddPid(builder, 0)  # Would need more advanced methods to get PID
                node_status_offsets.append(NodeStatus.End(builder))
            
            # Create node status vector
            SystemMetrics.StartNodeStatusVector(builder, len(node_status_offsets))
            for offset in reversed(node_status_offsets):
                builder.PrependUOffsetTRelative(offset)
            node_statuses_vector = builder.EndVector()
            
            # Add placeholder objects for LoadAverage, Memory, and Swap
            LoadAverage.Start(builder)
            load_average = LoadAverage.End(builder)
            
            Memory.Start(builder)
            memory_fb = Memory.End(builder)
            
            Swap.Start(builder)
            swap_fb = Swap.End(builder)
            
            # Start building system metrics - this time focusing on node status
            SystemMetrics.Start(builder)
            SystemMetrics.AddSchemaVersion(builder, 1)  # Schema version
            SystemMetrics.AddTimestamp(builder, int(time.time() * 1000))  # Convert to milliseconds
            SystemMetrics.AddCpuUsage(builder, 0.0)  # Placeholder
            SystemMetrics.AddLoadAverage(builder, load_average)  # Placeholder
            SystemMetrics.AddMemory(builder, memory_fb)  # Placeholder
            SystemMetrics.AddSwap(builder, swap_fb)  # Placeholder
            SystemMetrics.AddDiskUsage(builder, 0.0)  # Placeholder
            SystemMetrics.AddRobotId(builder, robot_id_fb)
            SystemMetrics.AddNodeStatus(builder, node_statuses_vector)
            
            # Finish the FlatBuffer
            metrics = SystemMetrics.End(builder)
            builder.Finish(metrics)
            
            # Get the serialized buffer
            buf = builder.Output()
            
            # Send the buffer via ZeroMQ
            try:
                self.zmq_socket.send(buf)
                self.logger.debug(f'Node statuses sent to controller (size: {len(buf)} bytes)')
            except Exception as e:
                self.logger.error(f'Failed to send node statuses: {e}')
            
            self.logger.info(f'Found {len(node_names)} active ROS nodes')
            for node_name in node_names:
                self.logger.debug(f'  - Node: {node_name}')
            
        except Exception as e:
            self.logger.error(f'Error collecting node statuses: {e}')

    def __del__(self):
        """Clean up resources when the node is destroyed."""
        try:
            if hasattr(self, 'zmq_socket') and self.zmq_socket:
                self.zmq_socket.close()
            if hasattr(self, 'zmq_context') and self.zmq_context:
                self.zmq_context.term()
        except Exception as e:
            if hasattr(self, 'logger'):
                self.logger.error(f'Error cleaning up ZeroMQ resources: {e}')


def main(args=None):
    """Run the system diagnostics node."""
    rclpy.init(args=args)
    
    node = SystemDiagnosticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.logger.info('Shutting down System Diagnostics Node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
