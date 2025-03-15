#!/usr/bin/env python3
# Copyright (c) 2023 Open-Teleop
# License: MIT

"""
System Diagnostics Bridge Node.

This node collects system metrics (CPU, memory, disk usage) and ROS2 node statuses,
and sends them to the Go Controller. Initially uses a mock connection.
"""

import time
import json
import psutil
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import LoggingSeverity
import open_teleop_logger as log


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
                ('console_log_level', 'INFO'),
            ]
        )
        
        self.robot_id = self.get_parameter('robot_id').value
        self.metrics_update_period = self.get_parameter('metrics_update_period').value
        self.nodes_update_period = self.get_parameter('nodes_update_period').value
        self.controller_address = self.get_parameter('controller_address').value
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
        
        self.logger.info(f'Starting System Diagnostics Node for robot: {self.robot_id}')
        self.logger.info(f'Metrics update period: {self.metrics_update_period} seconds')
        self.logger.info(f'Nodes update period: {self.nodes_update_period} seconds')
        self.logger.info(f'Controller address: {self.controller_address} (not connected)')
        
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
            
        valid_log_levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
        if self.console_log_level.upper() not in valid_log_levels:
            self.logger.warning(f'Invalid console_log_level: {self.console_log_level}, using INFO')
            self.console_log_level = 'INFO'

    def collect_and_send_system_metrics(self):
        """Collect system metrics."""
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
            
            metrics = {
                "timestamp": time.time(),
                "cpu_usage": cpu_percent,
                "load_average": {
                    "1min": load_avg_1,
                    "5min": load_avg_5,
                    "15min": load_avg_15
                },
                "memory": {
                    "total_mb": round(total_memory_mb, 2),
                    "used_mb": round(used_memory_mb, 2),
                    "available_mb": round(available_memory_mb, 2),
                    "used_percent": memory_percent,
                    "available_percent": available_memory_percent
                },
                "swap": {
                    "total_mb": round(swap_total_mb, 2),
                    "used_mb": round(swap_used_mb, 2),
                    "used_percent": swap_percent
                },
                "disk_usage": disk_percent,
                "robot_id": self.robot_id
            }
            
            self.logger.info(f'System metrics: CPU={cpu_percent}%, Load={load_avg_1:.2f}, Memory={memory_percent}% ({round(used_memory_mb, 2)}MB/{round(total_memory_mb, 2)}MB), Disk={disk_percent}%, Swap={swap_percent}%')
            
            self.logger.info(f'Metrics payload: {json.dumps(metrics)}')
            
        except Exception as e:
            self.logger.error(f'Error collecting system metrics: {e}')
            self.logger.trace(f'Stack trace for system metrics error')

    def collect_and_send_node_statuses(self):
        """Collect ROS node statuses."""
        try:
            node_names = self.get_node_names()
            
            node_statuses = []
            for node_name in node_names:
                node_status = {
                    "name": node_name,
                    "status": "active",  # For now, just mark all as active
                    "pid": 0  # Would need more advanced methods to get PID
                }
                node_statuses.append(node_status)
            
            status_data = {
                "timestamp": time.time(),
                "robot_id": self.robot_id,
                "node_status": node_statuses
            }
            
            self.logger.info(f'Found {len(node_names)} active ROS nodes')
            for node_name in node_names:
                self.logger.debug(f'  - Node: {node_name}')
            
        except Exception as e:
            self.logger.error(f'Error collecting node statuses: {e}')


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
