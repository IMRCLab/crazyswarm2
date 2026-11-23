#!/usr/bin/env python3

"""Crazyflie server using cflib2."""

import asyncio
from functools import partial
import threading

from cflib2 import Crazyflie, LinkContext
from cflib2.toc_cache import FileTocCache
from crazyflie_interfaces.srv import Land, Takeoff
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
import rclpy
from rclpy.node import Node


def _is_float_type(type_cf_param):
    """
    Return True if a cflib2 parameter type is floating point.

    cflib2 reports types such as 'F16', 'F32', 'U8', or 'I16'. The case is
    ignored, since the cflib2 documentation uses lower-case names.
    """
    return type_cf_param.lower().startswith('f')


def _ros_param_type(type_cf_param):
    """Map a cflib2 parameter type to the corresponding ROS 2 parameter type."""
    if _is_float_type(type_cf_param):
        return ParameterType.PARAMETER_DOUBLE
    return ParameterType.PARAMETER_INTEGER


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
        self.cfs = {}

        # Get the robot data from the ROS parameters
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

        # cflib2 is async-first: run an asyncio loop in a background thread and
        # schedule all cflib2 calls onto it. The loop is a daemon thread, so it
        # is stopped when the node exits.
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._loop_thread.start()

        # Connect to all Crazyflies (blocks until all of them are connected)
        connect_future = asyncio.run_coroutine_threadsafe(self._connect(), self._loop)
        try:
            connect_future.result()
        except Exception as e:
            for line in str(e).splitlines():
                self.get_logger().error(line)
            exit()

        self._init_services()
        self._init_parameters()
        self.add_on_set_parameters_callback(self._parameters_callback)

        self.get_logger().info('All Crazyflies are connected!')

    async def _connect(self):
        """Connect to all Crazyflies through a shared link context."""
        self._link_context = LinkContext()
        toc_cache = FileTocCache('./cache')
        results = await asyncio.gather(
            *[
                Crazyflie.connect_from_uri(self._link_context, uri, toc_cache)
                for uri in self.uris
            ],
            return_exceptions=True,
        )
        self.cfs = {}
        errors = []
        for uri, result in zip(self.uris, results):
            if isinstance(result, BaseException):
                errors.append((uri, result))
            else:
                self.cfs[uri] = result
        if errors:
            # Disconnect the Crazyflies that did connect before failing
            await asyncio.gather(
                *[cf.disconnect() for cf in self.cfs.values()], return_exceptions=True
            )
            self.cfs = {}
            details = '\n'.join(
                f'[{self.cf_dict[uri]}] connecting to {uri} failed: {error}'
                for uri, error in errors
            )
            raise ConnectionError(
                'Error!: One or more Crazyflies can not be found:\n' + details
            )

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

    def _run(self, coroutine, timeout=None):
        """Run a cflib2 coroutine on the background asyncio loop and wait for its result."""
        return asyncio.run_coroutine_threadsafe(coroutine, self._loop).result(timeout)

    def _initial_param_value(self, uri, group, param):
        """
        Look up the value of a firmware parameter from crazyflies.yaml.

        Parameter sets for individual robots have priority, then robot types,
        then all (all robots). Returns None if the parameter is not specified.
        """
        sources = [
            ('all',),
            ('robot_types', self.type_dict[uri]),
            ('robots', self.cf_dict[uri]),
        ]
        param_value = None
        for source in sources:
            try:
                d = self._ros_parameters
                for key in source:
                    d = d[key]
                param_value = d['firmware_params'][group][param]
            except (KeyError, TypeError):
                pass
        return param_value

    async def _set_initial_params(self, uri):
        """Upload the firmware parameters from crazyflies.yaml to one Crazyflie."""
        param = self.cfs[uri].param()
        for name in sorted(param.names()):
            group, _, field = name.partition('.')
            value = self._initial_param_value(uri, group, field)
            if value is None:
                continue
            # YAML may yield ints for float params (e.g. 1 instead of 1.0)
            if _is_float_type(param.get_type(name)):
                value = float(value)
            try:
                await param.set(name, value)
                self.get_logger().info(f'[{self.cf_dict[uri]}] {name} is set to {value}')
            except Exception as e:
                self.get_logger().error(
                    f'[{self.cf_dict[uri]}] setting {name} to {value} failed: {e}'
                )

    async def _set_all_initial_params(self):
        """Upload the firmware parameters from crazyflies.yaml to all Crazyflies in parallel."""
        await asyncio.gather(*[self._set_initial_params(uri) for uri in self.cfs])

    async def _get_all_params(self, uri):
        """Return a dict name -> (type, value) of all parameters of one Crazyflie."""
        param = self.cfs[uri].param()
        result = {}
        for name in sorted(param.names()):
            result[name] = (param.get_type(name), await param.get(name))
        return result

    def _init_parameters(self):
        """Upload firmware parameters and mirror them into ROS 2 params."""
        self._run(self._set_all_initial_params())

        query_all = self._ros_parameters.get('firmware_params', {}).get(
            'query_all_values_on_connect', False
        )
        if query_all:
            # cflib2 downloads all parameter values on connect, so these are
            # read from the local cache (which includes the values set above).
            for uri in self.cfs:
                cf_name = self.cf_dict[uri]
                for name, (type_cf_param, value) in self._run(
                    self._get_all_params(uri)
                ).items():
                    self._declare_firmware_param(cf_name, name, type_cf_param, value)
                    # Based on the parameters from the first Crazyflie, set params for all
                    if not self.has_parameter('all.params.' + name):
                        self._declare_firmware_param('all', name, type_cf_param, value)

        self.get_logger().info('All Crazyflies parameters are initialized.')

    def _declare_firmware_param(self, prefix, name, type_cf_param, value):
        """Declare a firmware parameter as ROS 2 parameter; log and skip it on failure."""
        ros_name = prefix + '.params.' + name
        if _is_float_type(type_cf_param):
            value = float(value)
        else:
            value = int(value)
        try:
            self.declare_parameter(
                ros_name,
                value=value,
                descriptor=ParameterDescriptor(type=_ros_param_type(type_cf_param)),
            )
        except Exception as e:
            self.get_logger().error(
                f'Declaring ROS 2 parameter {ros_name} (type {type_cf_param}, '
                f'value {value}) failed: {e!r}'
            )

    async def _set_param(self, uris, name, value):
        """Set a firmware parameter on the given Crazyflies."""
        for uri in uris:
            param = self.cfs[uri].param()
            if _is_float_type(param.get_type(name)):
                value = float(value)
            await param.set(name, value)
            self.get_logger().info(f'[{self.cf_dict[uri]}] {name} is set to {value}')

    def _parameters_callback(self, params):
        """Apply ROS 2 parameter updates to Crazyflie firmware parameters."""
        for param in params:
            param_split = param.name.split('.')
            if len(param_split) < 2:
                return SetParametersResult(successful=False)

            if param_split[0] == 'all':
                uris = list(self.cfs.keys())
            elif param_split[0] in self.uri_dict:
                uris = [self.uri_dict[param_split[0]]]
            else:
                return SetParametersResult(successful=False)

            if param_split[1] == 'params' and len(param_split) == 4:
                name_param = param_split[2] + '.' + param_split[3]
                try:
                    self._run(self._set_param(uris, name_param, param.value), timeout=5.0)
                except Exception as e:
                    self.get_logger().error(f'Setting {name_param} failed: {e}')
                    return SetParametersResult(successful=False, reason=str(e))
            elif param_split[1] == 'logs':
                continue
            else:
                return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)

    def _takeoff_callback(self, request, response, uri='all'):
        """Handle a high-level takeoff request."""
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{self.cf_dict[uri]}] takeoff(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        self._schedule(self._takeoff(uri, request.height, duration, request.group_mask))

        return response

    def _land_callback(self, request, response, uri='all'):
        """Handle a high-level land request."""
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{self.cf_dict[uri]}] land(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        self._schedule(self._land(uri, request.height, duration, request.group_mask))

        return response

    def _schedule(self, coroutine):
        """Schedule a cflib2 coroutine on the background asyncio loop."""
        future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)
        future.add_done_callback(self._log_async_error)

    def _log_async_error(self, future):
        """Log an error if a scheduled cflib2 call failed."""
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f'cflib2 call failed: {e}')

    async def _takeoff(self, uri, height, duration, group_mask):
        """Send a high-level takeoff command to one or all Crazyflies."""
        if uri == 'all':
            await asyncio.gather(
                *[
                    cf.high_level_commander().take_off(height, None, duration, group_mask)
                    for cf in self.cfs.values()
                ]
            )
        else:
            await self.cfs[uri].high_level_commander().take_off(
                height, None, duration, group_mask
            )

    async def _land(self, uri, height, duration, group_mask):
        """Send a high-level land command to one or all Crazyflies."""
        if uri == 'all':
            await asyncio.gather(
                *[
                    cf.high_level_commander().land(height, None, duration, group_mask)
                    for cf in self.cfs.values()
                ]
            )
        else:
            await self.cfs[uri].high_level_commander().land(height, None, duration, group_mask)


def main(args=None):
    """Start the Crazyflie server node."""
    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()

    rclpy.spin(crazyflie_server)

    crazyflie_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
