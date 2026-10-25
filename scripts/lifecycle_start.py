#!/usr/bin/env python3

import argparse
import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class LifecycleStartup:
    def __init__(self, backend):
        self.backend = backend

    def start(self):
        states = self.backend.read_states()
        if states is None:
            return False
        active = {
            name: state == State.PRIMARY_STATE_ACTIVE
            for name, state in states.items()
        }
        if all(active.values()):
            self.backend.info('The lifecycle group is already active.')
            return True
        if any(active.values()):
            names = ', '.join(
                name for name, is_active in active.items() if is_active)
            self.backend.error(
                'The lifecycle group has mixed states. Active nodes: '
                f'{names}.')
            return False
        if not self.backend.request_start():
            return False
        return self.backend.wait_for_active()


class RosLifecycleBackend(Node):
    def __init__(self, manager, nodes, timeout):
        super().__init__('s5max_lifecycle_start')
        self.manager = manager.strip('/')
        self.nodes = [name.strip('/') for name in nodes]
        self.timeout = max(0.1, float(timeout))
        self.state_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in self.nodes
        }
        self.manager_client = self.create_client(
            ManageLifecycleNodes,
            f'/{self.manager}/manage_nodes',
        )

    def info(self, message):
        self.get_logger().info(message)

    def error(self, message):
        self.get_logger().error(message)

    def _wait_for_future(self, future, deadline):
        while (
                rclpy.ok()
                and not future.done()
                and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done() and future.result() is not None

    def _wait_for_service(self, client, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.1):
                return True
        return False

    def read_states(self):
        deadline = time.monotonic() + self.timeout
        states = {}
        for name, client in self.state_clients.items():
            if not self._wait_for_service(client, deadline):
                self.error(f'The lifecycle state service for {name} is unavailable.')
                return None
            future = client.call_async(GetState.Request())
            if not self._wait_for_future(future, deadline):
                self.error(f'The lifecycle state request for {name} timed out.')
                return None
            state = future.result().current_state
            states[name] = state.id
            self.info(f'{name} lifecycle state: {state.label}.')
        return states

    def request_start(self):
        deadline = time.monotonic() + self.timeout
        if not self._wait_for_service(self.manager_client, deadline):
            self.error(
                f'The lifecycle manager {self.manager} is unavailable.')
            return False

        states = self.read_states()
        if states is None:
            return False
        if all(
                state == State.PRIMARY_STATE_ACTIVE
                for state in states.values()):
            self.info('The lifecycle group became active before startup.')
            return True
        if any(
                state == State.PRIMARY_STATE_ACTIVE
                for state in states.values()):
            self.error('The lifecycle group became partially active before startup.')
            return False

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self.manager_client.call_async(request)
        if not self._wait_for_future(future, deadline):
            self.error('The lifecycle startup request timed out.')
            return False
        if not future.result().success:
            self.error('The lifecycle manager rejected startup.')
            return False
        return True

    def wait_for_active(self):
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            states = self.read_states()
            if states is None:
                return False
            if all(
                    state == State.PRIMARY_STATE_ACTIVE
                    for state in states.values()):
                self.info('The lifecycle group is active.')
                return True
            time.sleep(0.1)
        self.error('The lifecycle group did not become active.')
        return False


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--manager', required=True)
    parser.add_argument('--node', action='append', required=True)
    parser.add_argument('--timeout', type=float, default=30.0)
    options, _ = parser.parse_known_args(args)

    rclpy.init()
    backend = RosLifecycleBackend(
        options.manager,
        options.node,
        options.timeout,
    )
    success = False
    try:
        success = LifecycleStartup(backend).start()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        backend.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
