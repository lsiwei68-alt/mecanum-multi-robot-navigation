"""Bounded, service-free Nav2 readiness checks for demo clients."""

import time

import rclpy


def wait_for_nav2(navigator, timeout=60.0):
    """Wait for localization and the namespaced Nav2 action interfaces.

    Lifecycle GetState and is_active service responses can occasionally be
    lost under a busy WSL DDS graph.  Topic delivery and action discovery do
    not have that single-response failure mode, so use AMCL's retained pose
    plus the planner, controller, and navigator action servers instead.
    """
    deadline = time.monotonic() + timeout
    last_status_log = 0.0
    stable_since = None

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)
        ready = {
            'localization': navigator.initial_pose_received,
            'planner': navigator.compute_path_to_pose_client.wait_for_server(
                timeout_sec=0.0),
            'controller': navigator.follow_path_client.wait_for_server(
                timeout_sec=0.0),
            'navigator': navigator.nav_to_pose_client.wait_for_server(
                timeout_sec=0.0),
        }
        now = time.monotonic()
        if all(ready.values()):
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= 1.0:
                navigator.get_logger().info('Nav2 is ready for use!')
                return
        else:
            stable_since = None

        if now - last_status_log >= 5.0:
            waiting = ', '.join(name for name, value in ready.items()
                                if not value)
            navigator.get_logger().info(
                f'Waiting for Nav2 interfaces: {waiting}')
            last_status_log = now

    raise TimeoutError(
        f'Nav2 did not become active within {timeout:.1f} seconds')


def start_navigation(navigator, pose, timeout=30.0):
    """Submit a navigation goal, retrying lifecycle-time rejections."""
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        pose.header.stamp = navigator.get_clock().now().to_msg()
        if navigator.goToPose(pose):
            return
        navigator.get_logger().warn(
            'Navigation action exists but is not accepting goals yet; '
            'retrying')
        time.sleep(0.5)

    raise TimeoutError(
        f'Navigation goal was not accepted within {timeout:.1f} seconds')
