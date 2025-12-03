.. _introduction:

Crazyswarm2: A ROS 2 testbed for Aerial Robot Teams
===================================================

Crazyswarm2 is a ROS 2 stack to use aerial robots that use flight computers from Bitcraze AB, including the Crazyflie 2.1(+), Crazyflie 2.1 Brushless, Flapper Nimble+, and custom drones built using Crazyflie Bolt.

On the technical side Crazyswarm2 is a port of the original `Crazyswarm <https://crazyswarm.readthedocs.io>`_ to ROS 2.
It is fully open-source (MIT) and available on `github <https://github.com/IMRCLab/crazyswarm2>`_.

A good starting point is the `Aerial Swarm Tools and Applications Tutorial/Workshop <https://imrclab.github.io/workshop-aerial-swarms-rss2024/>`_. If you use our work in academic research, please cite the original paper (we are actively working on a dedicated publication for Crazyswarm2):

.. code-block:: none

    @inproceedings{crazyswarm,
      author    = {James A. Preiss* and
                   Wolfgang  H\"onig* and
                   Gaurav S. Sukhatme and
                   Nora Ayanian},
      title     = {Crazyswarm: {A} large nano-quadcopter swarm},
      booktitle = {{IEEE} International Conference on Robotics and Automation ({ICRA})},
      pages     = {3299--3304},
      publisher = {{IEEE}},
      year      = {2017},
      url       = {https://doi.org/10.1109/ICRA.2017.7989376},
      doi       = {10.1109/ICRA.2017.7989376},
      note      = {Software available at \url{https://github.com/USC-ACTLab/crazyswarm}},
    }

Related Packages:
  - `Aerostack2 <https://aerostack2.github.io/>`_
  - `CrazyChoir <https://github.com/OPT4SMART/crazychoir>`_
  - `Skybrush <https://skybrush.io/>`_
  - `Dynamic Swarms Crazyflies <https://dynamicswarms.github.io/ds-crazyflies/>`_

.. warning::
  Crazyswarm2 is being actively used and developed. There are currently the following major limitations:
  
  - Sometimes unicast packets get lost, which is a `known firmware bug <https://github.com/bitcraze/crazyflie-firmware/issues/703>`_.


Contents
--------

.. toctree::
   installation
   overview
   usage
   tutorials
   howto
   faq
   :maxdepth: 1


.. Indices and tables
.. ------------------

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`

