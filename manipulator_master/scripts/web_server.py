#!/usr/bin/env python3

import os
import sys
import threading
import http.server
import socketserver
import urllib.parse
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

class ManipulatorWebHandler(http.server.SimpleHTTPRequestHandler):
    manipulator_web_dir = ""
    dist_dir = ""
    _pkg_cache = {}

    def translate_path(self, path):
        parsed = urllib.parse.urlparse(path)
        clean = urllib.parse.unquote(parsed.path)

        # Serve ROS package meshes / assets (e.g. /packages/manipulator_description/...)
        if clean.startswith('/packages/'):
            parts = clean.lstrip('/').split('/')
            if len(parts) >= 2:
                pkg_name = parts[1]
                allowed_pkgs = (
                    'manipulator_description',
                    'ugv_tracked_description',
                    'smart_master',
                    'smart_server',
                    'smart_sim2real',
                    'smart_moveit_config',
                )
                if pkg_name in allowed_pkgs:
                    try:
                        if pkg_name not in self._pkg_cache:
                            self._pkg_cache[pkg_name] = os.path.abspath(get_package_share_directory(pkg_name))
                        pkg_share = self._pkg_cache[pkg_name]
                        remainder = os.path.join(*parts[2:]) if len(parts) > 2 else ''
                        req_path = os.path.abspath(os.path.join(pkg_share, remainder))
                        if os.path.commonpath([pkg_share, req_path]) == pkg_share:
                            return req_path
                    except Exception:
                        pass
            return "/dev/null"

        # Serve full Vue Web UI 2.0 if requested under /app or /dist
        if clean.startswith('/app') or clean.startswith('/ui'):
            if self.dist_dir and os.path.exists(self.dist_dir):
                subpath = clean[4:].lstrip('/') if clean.startswith('/app') else clean[3:].lstrip('/')
                local_path = os.path.abspath(os.path.join(self.dist_dir, subpath))
                if not local_path.startswith(os.path.abspath(self.dist_dir)):
                    return "/dev/null"
                if not os.path.exists(local_path) and '.' not in os.path.basename(clean):
                    return os.path.join(self.dist_dir, 'index.html')
                return local_path

        # By default serve manipulator teleop web panel
        base_dir = self.manipulator_web_dir
        if not base_dir or not os.path.exists(base_dir):
            base_dir = self.dist_dir

        if not base_dir or not os.path.exists(base_dir):
            return "/dev/null"

        local_path = os.path.abspath(os.path.join(base_dir, clean.lstrip('/')))
        if not local_path.startswith(os.path.abspath(base_dir)):
            return "/dev/null"
        if not os.path.exists(local_path) and '.' not in os.path.basename(clean):
            return os.path.join(base_dir, 'index.html')
        return local_path

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server')
        self.declare_parameter('port', 8080)
        self.port = self.get_parameter('port').value

        try:
            ctrl_share = get_package_share_directory('manipulator_control')
            manipulator_web = os.path.join(ctrl_share, 'web')
        except Exception:
            manipulator_web = '/home/oleksandr/ros2_ws/src/manipulator_stack/manipulator_control/web'

        # Fallback to source directory if not yet in install
        if not os.path.exists(manipulator_web):
            manipulator_web = '/home/oleksandr/ros2_ws/src/manipulator_stack/manipulator_control/web'

        dist_path = '/home/oleksandr/ros2_ws/src/smart_robotics/Infrastructure_as_Code/web_ui_2.0/dist'

        ManipulatorWebHandler.manipulator_web_dir = manipulator_web
        ManipulatorWebHandler.dist_dir = dist_path

        self.get_logger().info(f"Serving Manipulator Web UI from: {manipulator_web}")
        if os.path.exists(dist_path):
            self.get_logger().info(f"Vue Web UI 2.0 available at: http://localhost:{self.port}/app")

        self.server_thread = threading.Thread(target=self.serve)
        self.server_thread.daemon = True
        self.server_thread.start()

    def serve(self):
        socketserver.TCPServer.allow_reuse_address = True
        try:
            with socketserver.TCPServer(("", self.port), ManipulatorWebHandler) as httpd:
                self.get_logger().info(f"🚀 Manipulator Web Server live at: http://localhost:{self.port}")
                httpd.serve_forever()
        except Exception as e:
            self.get_logger().error(f"Failed to start web server on port {self.port}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
