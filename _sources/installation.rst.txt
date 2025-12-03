.. _installation:

Installation
============

Crazyswarm2 runs on **Ubuntu Linux** in one of the following configurations:

====== ======== ====== 
Ubuntu Python   ROS 2
------ -------- ------
22.04  3.10     Humble
------ -------- ------
24.04  3.12     Jazzy
====== ======== ======

.. warning::
   The `Windows Subsystem for Linux (WSL) <https://docs.microsoft.com/en-us/windows/wsl/about>`_ is experimentally supported but you'll have to use `usbipd-win <https://github.com/dorssel/usbipd-win/>`_.
   This program will link the crazyradio directly with WS, but beware of bugs. Check out their `WSL connect guide <https://github.com/dorssel/usbipd-win/wiki/WSL-support/>`_.

.. warning::
   Avoid using a virtual machine if possible: they add additional latency and might cause issues with the visualization tools.

First Installation
------------------

1. If needed, install ROS 2 using the instructions at https://docs.ros.org/en/jazzy/Installation.html.

2. Install dependencies

    .. tabs::

        .. group-tab:: Binary Installation

            .. code-block:: bash

                pip3 install rowan nicegui

        .. group-tab:: Source Installation

            .. code-block:: bash

                sudo apt install libboost-program-options-dev libusb-1.0-0-dev
                pip3 install rowan nicegui

   Then install the motion capture ROS 2 package (replace <DISTRO> with your version of ROS, namely humble or jazzy):

    .. code-block:: bash

        sudo apt-get install ros-<DISTRO>-motion-capture-tracking 

    If you are planning to use the CFlib backend, do:

    .. code-block:: bash
        
        pip3 install cflib transforms3d
        sudo apt-get install ros-<DISTRO>-tf-transformations

3. Set up your ROS 2 workspace

    .. tabs::

        .. group-tab:: Binary Installation

            Then install the crazyswarm2 stack (replace <DISTRO> with your version of ROS, namely humble or jazzy):

            .. code-block:: bash

                sudo apt-get install ros-<DISTRO>-crazyflie*

            To prepare your workspace, see "Custom ROS Package" section below.

        .. group-tab:: Source Installation
            Clone the Crazyswarm2 repository

            .. code-block:: bash

                mkdir -p ros2_ws/src
                cd ros2_ws/src
                git clone https://github.com/IMRCLab/crazyswarm2 --recursive

            Now build the ROS 2 workspace

            .. code-block:: bash

                cd ../
                source /opt/ros/DISTRO/setup.bash
                colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
         
            .. note::
                symlink-install allows you to edit Python and config files without running `colcon build` every time.
         
            .. note::
                If you install it for the first time, you will see a lot of warnings at first. 
                As long as the build of the package finish, you can ignore this for now. 
       

4. Set up Crazyradio

   For the Crazyradio, you need to setup usb rules in order to communicate with the Crazyflie. Find the instructions for that here `in Bitcraze's USB permission guide for Linux <https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/>`_.

   You will also need to update the Crazyradio firmware to the latest development branch to be able to use all features.
   
    - For Crazyradio PA (1), `follow these instructions <https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/building_flashing/>`_.
    - For Crazyradio 2, follow `these instuctions to flash <https://www.bitcraze.io/documentation/repository/crazyradio2-firmware/main/building-and-flashing/flash//>`_ the `latest release <https://github.com/bitcraze/crazyradio2-firmware/releases>`_ (we tested version 5.1).

5. Update the Crazyflies

   If this is the first time handling Crazyflies it is always good to start with `Bitcraze's getting started guide  <https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/>`_.

   You can update each Crazyflie firmware to the latest release via `these instructions of the Bitcraze Crazyflie client <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/#firmware-upgrade>`_ .

   While you are at it, make sure that each Crazyflie have an unique radio address which you can change in `the client via these instructions <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/#firmware-configuration>`_ .

6. Set up software-in-the-loop simulation (optional)

    This currently requires cloning the Crazyflie firmware of the latest tested release (2025.02) and building the Python bindings manually. In a separate folder (not part of your ROS 2 workspace!), 

    .. code-block:: bash

        git clone --branch 2025.02 --single-branch --recursive https://github.com/bitcraze/crazyflie-firmware.git

    First follow `the instructions to build the python bindings <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#build-python-bindings>`_ from the bitcraze website. Afterwards, make sure that the bindings can be found in the python path:

    .. code-block:: bash

        export PYTHONPATH=<replace-with-path-to>/crazyflie-firmware/build:$PYTHONPATH
        
    If you are working from an older version of the crazyflie-firmware (before tag 2023.02), then you will need to point to main folder of the repo by removing the '/build' part. 


Updating
--------

You can update your local copy using the following commands:

    .. tabs::

        .. group-tab:: Binary Installation

            .. code-block:: bash

                sudo apt update
                sudo apt upgrade

        .. group-tab:: Source Installation

            .. code-block:: bash

                cd ros2_ws/src/crazyswarm2
                git pull
                git submodule sync
                git submodule update --init --recursive
                cd ../../
                source /opt/ros/DISTRO/setup.bash
                colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


Custom ROS Package
------------------

In order to use Crazyswarm, it is best practice to use a custom ROS package that contains all necessary config files as well as your user scripts / ROS nodes.

1. Create a new package

    .. code-block:: bash

        mkdir -p ros2_ws/src
        cd ros2_ws/src
        ros2 pkg create --build-type ament_python --license MIT --node-name hello_world crazyflie_test

2. Replace hello_world.py with https://github.com/IMRCLab/crazyswarm2/blob/main/crazyflie_examples/crazyflie_examples/hello_world.py

3. Add `<depend>crazyflie_py</depend>` to package.xml

4. Copy config files `crazyflies.yaml` and `motion_capture.yaml` from https://github.com/IMRCLab/crazyswarm2/tree/main/crazyflie/config into the config folder

5. Add `launch/launch.py` with the following content

    .. code-block:: python

        import os

        from ament_index_python.packages import get_package_share_directory
        from launch import LaunchDescription
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource

        package_name = 'crazyflie_test'

        def generate_launch_description():

            crazyflies_yaml_path = os.path.join(
                get_package_share_directory(package_name),
                'config',
                'crazyflies.yaml')
            motion_capture_yaml_path = os.path.join(
                get_package_share_directory(package_name),
                'config',
                'motion_capture.yaml')

            return LaunchDescription(
                [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                os.path.join(
                                    get_package_share_directory('crazyflie'), 'launch'
                                ),
                                '/launch.py',
                            ]
                        ),
                        launch_arguments={
                            'crazyflies_yaml_file': crazyflies_yaml_path,
                            'motion_capture_yaml_file': motion_capture_yaml_path,
                        }.items(),
                    ),
                ]
            )

6. In `setup.py`, include the following lines as part of the `data_files` array:

    .. code-block:: python

        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))


