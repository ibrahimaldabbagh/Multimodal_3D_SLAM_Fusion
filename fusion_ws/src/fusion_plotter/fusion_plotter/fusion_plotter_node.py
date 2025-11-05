#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
import threading
import time
import math
import copy
from threading import Lock

class FusionPlotterNode(Node):
    def __init__(self):
        super().__init__('fusion_plotter_node')

        self.start_time = time.time()

        # A lock to protect concurrent access to self.data
        self.data_lock = Lock()

        # Data containers
        self.data = {
            "imu": {"t": [], "x": [], "y": [], "z": [], "yaw": []},
            "camera": {"t": [], "x": [], "y": [], "z": [], "yaw": []},
            "radar": {"t": [], "x": [], "y": [], "z": [], "yaw": []},
            "fused": {"t": [], "x": [], "y": [], "z": [], "yaw": []},
            "pressure": {"t": [], "z": []},
            "steps": {"t": [], "count": []}
        }

        # Subscribers
        # (IMU is PoseStamped, others are PoseWithCovarianceStamped or Int32)

        self.create_subscription(PoseWithCovarianceStamped, '/mvs/pose', self.camera_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose_estimation', self.radar_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/fused/pose', self.fused_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pressure/pose', self.pressure_callback, 10)
        self.create_subscription(Int32, '/step_count', self.step_callback, 10)

        # Start plot thread
        self.plot_thread = threading.Thread(target=self.plot_data)
        self.plot_thread.daemon = True
        self.plot_thread.start()

    def get_time(self):
        return time.time() - self.start_time

    def extract_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


    def camera_callback(self, msg):
        with self.data_lock:
            t = self.get_time()
            self.data["camera"]["t"].append(t)
            self.data["camera"]["x"].append(msg.pose.pose.position.x)
            self.data["camera"]["y"].append(msg.pose.pose.position.y)
            self.data["camera"]["z"].append(msg.pose.pose.position.z)
            self.data["camera"]["yaw"].append(self.extract_yaw(msg.pose.pose.orientation))

    def radar_callback(self, msg):
        with self.data_lock:
            t = self.get_time()
            self.data["radar"]["t"].append(t)
            self.data["radar"]["x"].append(msg.pose.pose.position.x)
            self.data["radar"]["y"].append(msg.pose.pose.position.y)
            self.data["radar"]["z"].append(msg.pose.pose.position.z)
            self.data["radar"]["yaw"].append(self.extract_yaw(msg.pose.pose.orientation))

    def fused_callback(self, msg):
        with self.data_lock:
            t = self.get_time()
            self.data["fused"]["t"].append(t)
            self.data["fused"]["x"].append(msg.pose.pose.position.x)
            self.data["fused"]["y"].append(msg.pose.pose.position.y)
            self.data["fused"]["z"].append(msg.pose.pose.position.z)
            self.data["fused"]["yaw"].append(self.extract_yaw(msg.pose.pose.orientation))

    def pressure_callback(self, msg):
        with self.data_lock:
            t = self.get_time()
            self.data["pressure"]["t"].append(t)
            self.data["pressure"]["z"].append(msg.pose.pose.position.z)

    def step_callback(self, msg):
        with self.data_lock:
            t = self.get_time()
            self.data["steps"]["t"].append(t)
            self.data["steps"]["count"].append(msg.data)

    def plot_data(self):
        plt.ion()
        fig_xyz, axs = plt.subplots(3, 1, figsize=(12, 10))
        fig_steps, ax_step = plt.subplots(1, 1, figsize=(10, 4))
        fig_xy, ax_xy = plt.subplots(1, 1, figsize=(8, 6))
        fig_yaw, ax_yaw = plt.subplots(1, 1, figsize=(10, 4))

        def safe_plot(ax, label, topic, axis, **kwargs):
            # Pull a local copy of time and data so they are consistent in length
            with self.data_lock:
                t_vals = self.data[topic]["t"][:]
                axis_vals = self.data[topic][axis][:]
            n = min(len(t_vals), len(axis_vals))
            if n > 0:
                ax.plot(t_vals[:n], axis_vals[:n], label=label, **kwargs)

        while rclpy.ok():
            # Clear axes
            axs[0].cla()
            axs[1].cla()
            axs[2].cla()
            ax_step.cla()
            ax_xy.cla()
            ax_yaw.cla()

            # Plot X
            #safe_plot(axs[0], "IMU X", "imu", "x")
            safe_plot(axs[0], "Camera X", "camera", "x")
            safe_plot(axs[0], "Radar X", "radar", "x")
            safe_plot(axs[0], "Fused X", "fused", "x")
            axs[0].set_ylabel("X")
            axs[0].legend()

            # Plot Y
            #safe_plot(axs[1], "IMU Y", "imu", "y")
            safe_plot(axs[1], "Camera Y", "camera", "y")
            safe_plot(axs[1], "Radar Y", "radar", "y")
            safe_plot(axs[1], "Fused Y", "fused", "y")
            axs[1].set_ylabel("Y")
            axs[1].legend()

            # Plot Z + Pressure
            #safe_plot(axs[2], "IMU Z", "imu", "z")
            safe_plot(axs[2], "Camera Z", "camera", "z")
            safe_plot(axs[2], "Fused Z", "fused", "z")
            # Pressure lines
            with self.data_lock:
                pt = self.data["pressure"]["t"][:]
                pz = self.data["pressure"]["z"][:]
            n_p = min(len(pt), len(pz))
            if n_p > 0:
                axs[2].plot(pt[:n_p], pz[:n_p], label="Pressure Z", linestyle='--')
            axs[2].set_ylabel("Z")
            axs[2].set_xlabel("Time (s)")
            axs[2].legend()

            # Step count
            with self.data_lock:
                st = self.data["steps"]["t"][:]
                sc = self.data["steps"]["count"][:]
            n_s = min(len(st), len(sc))
            if n_s > 0:
                ax_step.plot(st[:n_s], sc[:n_s], label="Step Count")
                ax_step.set_ylabel("Steps")
                ax_step.set_xlabel("Time (s)")
                ax_step.legend()

            # XY path
            def plot_xy(ax, label, topic, style='-', marker=''):
                with self.data_lock:
                    x_vals = self.data[topic]["x"][:]
                    y_vals = self.data[topic]["y"][:]
                n_xy = min(len(x_vals), len(y_vals))
                if n_xy > 0:
                    ax.plot(x_vals[:n_xy], y_vals[:n_xy],
                            label=label, linestyle=style, marker=marker)

            plot_xy(ax_xy, "Radar XY", "radar", style='-', marker='o')
            plot_xy(ax_xy, "Camera XY", "camera", style='--', marker='x')
            plot_xy(ax_xy, "Fused XY", "fused", style='--', marker='+')
            ax_xy.set_xlabel("X")
            ax_xy.set_ylabel("Y")
            ax_xy.set_title("XY Path (Radar vs Camera vs Fused)")
            ax_xy.legend()

            # Yaw
            safe_plot(ax_yaw, "IMU Yaw", "imu", "yaw")
            #safe_plot(ax_yaw, "Camera Yaw", "camera", "yaw")
            #safe_plot(ax_yaw, "Radar Yaw", "radar", "yaw")
            safe_plot(ax_yaw, "Fused Yaw", "fused", "yaw")
            ax_yaw.set_ylabel("Yaw (rad)")
            ax_yaw.set_xlabel("Time (s)")
            ax_yaw.set_title("Yaw from Camera, Radar, Fused")
            ax_yaw.legend()

            plt.pause(0.5)

        plt.ioff()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = FusionPlotterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

