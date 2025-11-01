#!/usr/bin/env python3
"""
MTRX3760 2025 Project 2: Warehouse Robot DevKit
File: robot_dashboard_bridge.py
Author(s): Dashboard Helper
Description: ROS2-to-Web Bridge for Robot Dashboard

This script bridges ROS2 topics to a web dashboard via HTTP server
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, BatteryState
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import threading
from datetime import datetime

class RobotDashboardBridge(Node):
    """Bridge between ROS2 topics and web dashboard"""
    
    def __init__(self):
        super().__init__('robot_dashboard_bridge')
        
        # State variables
        self.robot_mode = "IDLE"
        self.battery_level = 100.0
        self.battery_state = "normal"
        self.damage_count = 0
        self.delivery_count = 0
        self.delivery_pending = 0
        self.camera_image = None
        self.logs = []
        
        self.bridge = CvBridge()
        
        # Subscribers
        # Try to subscribe to ArUco debug image first, fallback to raw camera
        self.create_subscription(Image, '/aruco/debug_image', 
                                self.camera_callback, 10)
        # Also subscribe to raw camera as backup
        self.create_subscription(Image, '/camera/image_raw', 
                                self.camera_callback, 10)
        
        # Subscribe to battery topics - try turtlebot actual battery first
        self.create_subscription(BatteryState, '/battery_state',
                                self.turtlebot_battery_callback, 10)
        # Subscribe to virtual battery monitor (for simulation/testing)
        self.create_subscription(Float32, '/battery/level', 
                                self.battery_callback, 10)
        self.create_subscription(String, '/battery/state',
                                self.battery_state_callback, 10)
        
        # Publishers
        self.mode_pub = self.create_publisher(String, '/robot/mode', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Dashboard bridge initialized')
    
    def turtlebot_battery_callback(self, msg):
        """Update battery level from Turtlebot3's actual battery sensor"""
        # sensor_msgs/BatteryState has a percentage field (already 0-100 range)
        self.battery_level = msg.percentage
        
        # Determine state based on percentage (compare against 100 range)
        if msg.percentage <= 10.0:
            self.battery_state = "critical"
        elif msg.percentage <= 20.0:
            self.battery_state = "low"
        else:
            self.battery_state = "normal"
        
        # Log if state changes
        if self.battery_state in ['low', 'critical']:
            self.add_log(f'Battery {self.battery_state}: {self.battery_level:.1f}%')
    
    def battery_callback(self, msg):
        """Update battery level from virtual battery monitor"""
        self.battery_level = msg.data
    
    def battery_state_callback(self, msg):
        """Update battery state from virtual battery monitor"""
        self.battery_state = msg.data
        if msg.data in ['low', 'critical']:
            self.add_log(f'Battery {msg.data}: {self.battery_level:.1f}%')
    
    def camera_callback(self, msg):
        """Process camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_image)
            self.camera_image = buffer.tobytes()
        except Exception as e:
            self.get_logger().error(f'Camera processing failed: {e}')
    
    def get_state(self):
        """Get current robot state as JSON"""
        return {
            'mode': self.robot_mode,
            'battery': self.battery_level,
            'battery_state': self.battery_state,
            'damage_count': self.damage_count,
            'delivery_count': self.delivery_count,
            'delivery_pending': self.delivery_pending,
            'logs': self.logs[-50:]  # Last 50 logs
        }
    
    def set_mode(self, mode):
        """Set robot mode"""
        self.robot_mode = mode
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Mode set to: {mode}')
    
    def add_log(self, message):
        """Add log entry"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.logs.append(f'[{timestamp}] {message}')
        if len(self.logs) > 100:  # Keep last 100 logs
            self.logs.pop(0)


class DashboardHandler(BaseHTTPRequestHandler):
    """HTTP handler for dashboard requests"""
    
    bridge = None
    
    def do_GET(self):
        """Handle GET requests"""
        if self.path == '/':
            # Serve the HTML file
            with open('robot_dashboard.html', 'r') as f:
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                self.wfile.write(f.read().encode())
        
        elif self.path == '/api/state':
            # Return robot state as JSON
            state = DashboardHandler.bridge.get_state()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(state).encode())
        
        elif self.path == '/api/camera':
            # Return camera image
            if DashboardHandler.bridge.camera_image:
                self.send_response(200)
                self.send_header('Content-type', 'image/jpeg')
                self.end_headers()
                self.wfile.write(DashboardHandler.bridge.camera_image)
            else:
                self.send_response(204)  # No content
                self.end_headers()
        
        else:
            self.send_response(404)
            self.end_headers()
    
    def do_POST(self):
        """Handle POST requests"""
        if self.path == '/api/mode':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode())
            
            mode = data.get('mode', 'IDLE')
            DashboardHandler.bridge.set_mode(mode)
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({'status': 'ok'}).encode())
        
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        """Suppress log messages"""
        pass


def run_http_server(bridge, port=8080):
    """Run HTTP server in separate thread"""
    DashboardHandler.bridge = bridge
    server = HTTPServer(('', port), DashboardHandler)
    print(f'Dashboard server running on http://localhost:{port}')
    server.serve_forever()


def main():
    """Main function"""
    rclpy.init()
    bridge_node = RobotDashboardBridge()
    
    # Start HTTP server in background thread
    http_thread = threading.Thread(target=run_http_server, args=(bridge_node, 8080))
    http_thread.daemon = True
    http_thread.start()
    
    try:
        print('ROS2 Dashboard Bridge running. Open http://localhost:8080 in your browser')
        rclpy.spin(bridge_node)
    except (KeyboardInterrupt, SystemExit):
        print('Shutting down...')
    finally:
        try:
            bridge_node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

