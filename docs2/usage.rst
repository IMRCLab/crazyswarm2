.. _usage:

Usage
=====

.. warning::
    Do not forget to source your ROS 2 workspace in each terminal you would like to use it

    .. code-block:: bash

        . install/local_setup.bash


.. warning::
    If you work in a shared network (lab, classroom) or similar, you might want to avoid
    controlling other robots. This is in particular true for simulation. In this case,
    you can use

    .. code-block:: bash

        export ROS_LOCALHOST_ONLY=1
        export ROS_DOMAIN_ID=<unique number between 0 and 101>

    The first environment variable restricts ROS to your local machine. The second is useful
    if you have multiple user accounts on the same machine (e.g., a shared workstation computer).
    In such case, each user should use a unique `domain ID <https://docs.ros.org/en/iron/Concepts/Intermediate/About-Domain-ID.html>`_.


Configuration
-------------

All configuration files are in crazyflie/config.

* crazyflies.yaml : setting up everything related to the robots.
* server.yaml : setting up everything related to the server.
* motion_capture.yaml : configs for the motion capture package.
* teleop.yaml : configs for remote controls.

crazyflies.yaml
~~~~~~~~~~~~~~~

Each crazyflie should have an unique URI which can `be changed in Bitcraze's CFclient <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/#firmware-configuration/>`_.
They can also be enabled in case you don't want the server to connect with it.

.. code-block:: yaml

    robots:
        cf231:
            enabled: true
            uri: radio://0/80/2M/E7E7E7E7E7
            initial_position: [0, 0, 0]
            type: cf21  # see robot_types

        cf5:
            enabled: false
            uri: radio://0/80/2M/E7E7E7E705
            initial_position: [0, -0.5, 0]
            type: cf21  # see robot_types

The yaml file also contains different robot_types, to indicate differences between each platform:

.. code-block:: yaml

    robot_types:
        cf21:
            motion_capture:
                tracking: "librigidbodytracker"
                # the following settings are only relevant if "librigidbodytracker" is used; see motion_capture.yaml
                marker: default_single_marker
                dynamics: default
            big_quad: false
            battery:
                voltage_warning: 3.8  # V
                voltage_critical: 3.7 # V

        cf21_mocap_deck:
            motion_capture:
                tracking: "librigidbodytracker"
                # the following settings are only relevant if "librigidbodytracker" is used; see motion_capture.yaml
                marker: mocap_deck
                dynamics: default
            big_quad: false
            battery:
                voltage_warning: 3.8  # V
                voltage_critical: 3.7 # V

The yaml file also contain an 'all' field, in case you have parameters or logging that you want enabled for all the connected crazyflies.


.. code-block:: yaml

    all:
        firmware_logging:
            enabled: false
            default_topics:
                pose:
                frequency: 10 # Hz
            #custom_topics:
            #  topic_name1:
            #    frequency: 10 # Hz
            #    vars: ["stateEstimateZ.x", "stateEstimateZ.y", "stateEstimateZ.z", "pm.vbat"]
            #  topic_name2:
            #    frequency: 1 # Hz
            #    vars: ["stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw"]
        firmware_params:
            commander:
                enHighLevel: 1
            stabilizer:
                estimator: 2 # 1: complementary, 2: kalman
                controller: 2 # 1: PID, 2: mellinger

The above also contains an example of the firmware_logging field, where default topics can be enabled or custom topics based on the `existing log toc of the crazyflie <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs//>`_.
Default topics are ``pose``, ``scan``, ``odom``, ``status``, and ``imu``.
Moreover, it also contains the firmware_params field, where parameters can be set at startup.
Also see the `parameter list of the crazyflie <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/params//>`_ for that.


Mind that you can also place the firmware_params and firmware_logging fields per crazyflie in 'robots'  or the 'robot_types' field.
The server node will upon initialization, first look at the params/logs from the individual crazyflie's settings, then the robot_types, and then anything in 'all' which has lowest priority.

Positioning
-----------

The Crazyflie can be positioned in different ways, including motion capture and onboard positioning.
This blogpost provides a good overview of the different positioning systems if you are unsure about which one you are using: `Positioning System Overview <https://www.bitcraze.io/2021/05/positioning-system-overview/>`_.

Motion capture
~~~~~~~~~~~~~~

If you have a motion capture system, you can input the specifics in the motion_capture.yaml file.

.. code-block:: yaml

    /motion_capture_tracking:
        ros__parameters:
            type: "optitrack"
            hostname: "optitrackPC"

`type` can be replaced by "optitrack", "vicon", "qualisys" or any of the other supported motion capture systems of the `motion capture tracking package <https://github.com/IMRCLab/motion_capture_tracking/tree/ros2/>`_.
'hostname' is the hostname of the computer running the motion capture software which can either be the PC name or the IP.
Please check the documentation of the `motion capture tracking package <https://github.com/IMRCLab/motion_capture_tracking/tree/ros2/>`_ for any additional configuration that might be needed on the vendor software side.

There are two different modes for tracking rigid bodies:

1. Using the vendor-specific software (e.g., VICON Tracker or OptiTrack Motive), or
2. Using `librigidbodytracker`, which is a package built into the `motion capture tracking package <https://github.com/IMRCLab/motion_capture_tracking/tree/ros2/>`_ and allows to use identical marker configurations or a single marker per rigid body as long as the initial pose is known a-priori.

You can enable either mode in `crazyflies.yaml`:

.. code-block:: yaml

    robot_types:
    cf21:
        motion_capture:
            tracking: "librigidbodytracker" # use "vendor" for vendor-specific software or "librigidbodytracker" for custom frame-by-frame tracking using motion capture point clouds
            marker: default_single_marker
            dynamics: default
.. note::

    For most experiments, we recommend putting a single marker on top of the robots and use the "librigidbodytracker" with the "default_single_marker" configuration.
    In this mode, we use optimal task assignment at every frame, which makes this mode more robust to motion capture outliers compared to the duplicate marker arrangements.
    The main disadvantage is that the yaw angle cannot be observed without moving in the xy-plane.
    Nevertheless, it is possible to hover for 30 seconds in place for a Crazyflie 2.1, without causing flight instabilities.


For more in-depth information about the motion capture tracking package, see the `documentation <https://github.com/IMRCLab/motion_capture_tracking/tree/ros2/>`_.

Onboard positioning
~~~~~~~~~~~~~~~~~~~

The Crazyflie also supports several alternative positioning systems that provide direct onboard position or pose estimation.
In this case you do not need to receive positioning from an external system like with MoCap.

Instructions per positioning system:

* The `Loco positioning system <https://www.bitcraze.io/documentation/system/positioning/loco-positioning-system/>`_ - `Follow Bitcraze's tutorial on LPS here <https://www.bitcraze.io/documentation/tutorials/getting-started-with-loco-positioning-system/>`_.
* The `Lighthouse positioning system <https://www.bitcraze.io/documentation/system/positioning/lighthouse/>`_ - `Follow Bitcraze's tutorial on Lighthouse here <https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse-positioning-system/>`_. Make sure to review the system management for saving and loading a system config, such that you don't have to redo the basestation geometry estimation for each crazyflie.
* The `Flow deck <https://www.bitcraze.io/products/flow-deck-v2/>`_ -  `Follow Bitcraze's tutorial on the flowdeck here <https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/>`_. Note that the flow deck provides an relative positoing estimate and might conflict when you are flying with crazyswarm2 if you are flying with absolute coordinates instead of relative ones.

Also in this case, make sure that motion_capture is disabled in the crazyflies.yaml file:

.. code-block:: yaml

    robot_types:
        cf21:
            motion_capture:
                tracking: "vendor"

Also it is a good idea to turn on pose estimation logging such that you are able to see the poses and transforms of the Crazyflie updated in real life in rviz or the Swarm management gui.

.. code-block:: yaml

    firmware_logging:
        enabled: true
        default_topics:
            pose:
            frequency: 10 # Hz


Moreover, be aware that the motion capture node is enabled in the launch file by default, which can be turned off by adding 'mocap:=False' to the launch command.

Simulation
----------

Any usage of the ROS API, including high-level Python scripts, can be visualized before execution. The initial position and number of robots is taken from the crazyflies.yaml configuration file.
The simulation uses the firmware code as software-in-the-loop, and can optionally include the robot dynamics.
The configuration of the simulation (physics simulator, controller, etc.) can be changed in server.yaml.

Example:

.. code-block:: bash

    [terminal]$ ros2 launch crazyflie_examples launch.py script:=hello_world backend:=sim

which is a short-hand for the following two commands:

.. code-block:: bash

    [terminal1]$ ros2 launch crazyflie launch.py backend:=sim
    [terminal2]$ ros2 run crazyflie_examples hello_world --ros-args -p use_sim_time:=True

Physical Experiments
--------------------

.. _arming:

Arming
~~~~~~

Some platforms (Brushless, Bolt) need to be armed before they can fly.
There are several ways to arm:

* **Joystick**: If a joystick is connected (see `Teleoperation controller`_), press the arm button (yellow / Y button on an XBox controller by default, configurable as ``arm.button`` in teleop.yaml). Pressing the button again disarms the drone.
* **Python Script**:  See also the ``arming.py`` example script.
* **ROS Service**: ``/cfXXX/arm`` or ``/all/arm`` services.

Teleoperation controller
~~~~~~~~~~~~~~~~~~~~~~~~

We currently assume an XBox controller (the button mapping can be changed in teleop.yaml).
The teleoperation node and the joystick driver are started by default (disable with ``teleop:=False``):

.. code-block:: bash

    ros2 launch crazyflie launch.py

The ``mode`` in teleop.yaml selects how the joystick is used:

* ``high_level`` (default): The buttons arm/disarm, take off, land, and trigger an emergency stop for *all* Crazyflies (via the ``all/*`` services). The sticks are not used.
* ``cmd_vel_world``: Manual flight of a single Crazyflie by commanding velocities in the world frame. This requires a localization system (motion capture, Lighthouse, Flow deck, ...).
* ``cmd_rpy``: Manual flight of a single Crazyflie by commanding roll, pitch, yaw rate, and thrust directly (similar to manual flight in the cfclient).

For the manual flight modes (``cmd_vel_world`` and ``cmd_rpy``), you need to edit two files:

1. **teleop.yaml**: Set ``mode`` to ``cmd_vel_world`` or ``cmd_rpy`` and ``frequency`` to a positive value. If needed, adjust the axis mapping, the maximum values, and (for ``cmd_vel_world``) the workspace limits and the ``initial_position``.
2. **launch.py**: In the remappings of the teleop node, uncomment the lines for your mode and replace ``cf231`` with the name of your Crazyflie (as in crazyflies.yaml):

   * ``cmd_vel_world``: ``cmd_full_state`` and ``notify_setpoints_stop``
   * ``cmd_rpy``: ``cmd_vel_legacy`` and ``notify_setpoints_stop``

   Rebuild your workspace (``colcon build``) afterwards, unless you used ``--symlink-install``.

To fly, press the takeoff button. After the takeoff is completed, the sticks control the Crazyflie. Pressing the land button switches back to high-level mode and lands.
If the Crazyflie does not react to the sticks, check the output of the teleop node for warnings and verify that, e.g., ``ros2 topic echo /cf231/cmd_full_state`` shows messages after takeoff.

.. note::
    Teleoperation in ``cmd_vel_world`` mode tracks a setpoint that is integrated from the joystick input. With noisy onboard state estimates (e.g., Flow deck), the Crazyflie may oscillate; the PID controller is typically more forgiving than Mellinger or Lee in this case.

To fly two Crazyflies manually, each with its own joystick, use:

.. code-block:: bash

    ros2 launch crazyflie launch_teleop2.py cf_1:=cf231 cf_2:=cf5

This includes launch.py (all of its arguments, e.g., ``backend`` or ``mocap``, can be used) and starts one teleop and one joystick node per Crazyflie in the namespace of that Crazyflie.
Optional arguments are ``joy_device_id_1``/``joy_device_id_2`` (default 0 and 1) and ``teleop_yaml_file_1``/``teleop_yaml_file_2`` (default teleop.yaml).
No remappings need to be edited in this case, but the ``mode`` in teleop.yaml still needs to be set.


Python scripts
~~~~~~~~~~~~~~

In the first terminal run the server, in the second the desired script.
You may run the script multiple times or different scripts while leaving the server running.

.. code-block:: bash

    [terminal1]$ ros2 launch crazyflie launch.py
    [terminal2]$ ros2 run crazyflie_examples hello_world

If you only want to run a single script once, you can also use:

.. code-block:: bash

    [terminal]$ ros2 launch crazyflie_examples launch.py script:=hello_world

Swarm Management
----------------

The launch file will also start a swarm management tool that is a ROS node and web-based GUI.
In the upper pane is the location of the drone visualized in a 3D window, similar to rviz.
In the lower pane, the status as well as log messages are visible (tabbed per drone).
In the future, we are planning to add support for rebooting and other actions.
