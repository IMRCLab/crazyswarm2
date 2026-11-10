#!/usr/bin/env python3

"""Run a minimal Crazyflie server with takeoff and land services."""

from functools import partial

from cflib.crazyflie.swarm import CachedCfFactory, Swarm
import cflib.crtp
from crazyflie_interfaces.srv import Land, Takeoff
import rclpy
from rclpy.node import Node


class CrazyflieServer(Node):
    """ROS 2 node that bridges Crazyflie firmware and ROS services."""

    def __init__(self):
        """Initialize the Crazyflie ROS 2 server node."""
        super().__init__(
            'crazyflie_server',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)

        self.uris = []
        # assign a all -> all mapping
        self.cf_dict = {'all': 'all'}
        self.uri_dict = {}
        self.type_dict = {}

        # Check if the Crazyflie library is initialized
        robot_data = self._ros_parameters['robots']

        # Create easy lookup tables for uri, name and types
        for crazyflie in robot_data:
            if robot_data[crazyflie]['enabled']:
                type_cf = robot_data[crazyflie]['type']
                # do not include virtual objects
                connection = self._ros_parameters['robot_types'][type_cf].get(
                    'connection', 'crazyflie'
                )
                if connection == 'crazyflie':
                    uri = robot_data[crazyflie]['uri']
                    self.uris.append(uri)
                    self.cf_dict[uri] = crazyflie
                    self.uri_dict[crazyflie] = uri
                    self.type_dict[uri] = type_cf

        # Setup Swarm class cflib with connection callbacks and open the links
        factory = CachedCfFactory(rw_cache='./cache')
        self.swarm = Swarm(self.uris, factory=factory)
        self.swarm.connected_crazyflie_cnt = 0

        # Initialize callbacks for each crazyflie
        for link_uri in self.uris:
            # Connect callbacks for different connection states of the crazyflie
            self.swarm._cfs[link_uri].cf.connected.add_callback(self._connected)
            self.swarm._cfs[link_uri].cf.disconnected.add_callback(self._disconnected)
            self.swarm._cfs[link_uri].cf.connection_failed.add_callback(
                self._connection_failed
            )

        # Now all crazyflies are initialized, open links!
        try:
            self.swarm.open_links()
        except Exception:
            # Close node if one of the Crazyflies can not be found
            self.get_logger().info('Error!: One or more Crazyflies can not be found. ')
            self.get_logger().info(
                'Check if you got the right URIs, if they are turned on'
                + ' or if your script have proper access to a Crazyradio PA'
            )
            exit()

    def _param_to_dict(self, param_ros):
        """Turn ROS 2 parameters from the node into a dict."""
        tree = {}
        for item in param_ros:
            t = tree
            for part in item.split('.'):
                if part == item.split('.')[-1]:
                    t = t.setdefault(part, param_ros[item].value)
                else:
                    t = t.setdefault(part, {})
        return tree

    def _connected(self, link_uri):
        """Handle the event when a Crazyflie reports connected."""
        self.get_logger().info(f'[{self.cf_dict[link_uri]}] is connected!')
        self.swarm.connected_crazyflie_cnt += 1

        if self.swarm.connected_crazyflie_cnt == len(self.cf_dict) - 1:
            self.get_logger().info('All Crazyflies are connected!')
            self._init_services()

        else:
            return

    def _disconnected(self, link_uri):
        self.get_logger().info(f'[{self.cf_dict[link_uri]}] is disconnected!')

    def _connection_failed(self, link_uri, msg):
        self.get_logger().info(f'[{self.cf_dict[link_uri]}] connection Failed')
        self.swarm.close_links()

    def _init_services(self):
        """Create takeoff and land services for each crazyflie and all."""
        for uri in self.cf_dict:
            if uri == 'all':
                continue

            name = self.cf_dict[uri]

            self.create_service(
                Takeoff, name + '/takeoff', partial(self._takeoff_callback, uri=uri)
            )
            self.create_service(Land, name + '/land', partial(self._land_callback, uri=uri))

        self.create_service(Takeoff, 'all/takeoff', self._takeoff_callback)
        self.create_service(Land, 'all/land', self._land_callback)

    def _takeoff_callback(self, request, response, uri='all'):
        """Handle a high-level takeoff request."""
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{self.cf_dict[uri]}] takeoff(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        if uri == 'all':
            for link_uri in self.uris:
                self.swarm._cfs[link_uri].cf.high_level_commander.takeoff(
                    request.height, duration
                )
        else:
            self.swarm._cfs[uri].cf.high_level_commander.takeoff(request.height, duration)

        return response

    def _land_callback(self, request, response, uri='all'):
        """Handle a high-level land request."""
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{self.cf_dict[uri]}] land(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        if uri == 'all':
            for link_uri in self.uris:
                self.swarm._cfs[link_uri].cf.high_level_commander.land(
                    request.height, duration, group_mask=request.group_mask
                )
        else:
            self.swarm._cfs[uri].cf.high_level_commander.land(
                request.height, duration, group_mask=request.group_mask
            )

        return response


def main(args=None):
    """Start the Crazyflie server node."""
    cflib.crtp.init_drivers()
    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()

    rclpy.spin(crazyflie_server)

    crazyflie_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
