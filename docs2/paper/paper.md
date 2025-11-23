---
title: 'Crazyswarm2: A ROS 2-based Stack for Bitcraze Crazyflie Multirotor Robots'
tags:
  - ROS
  - robotics
  - multirotor
authors:
  - name: Wolfgang Hönig
    orcid: 0000-0002-0773-028X
    equal-contrib: true
    affiliation: "1, 2"
  - name: Kimberly N. McGuire 
    orcid: 0000-0002-5003-2049
    equal-contrib: true # (This is how you can denote equal contributions between multiple authors)
    affiliation: 3
affiliations:
 - name: TU Berlin, Germany
   index: 1
 - name: Robotics Institute Germany (RIG)
   index: 2
 - name: Independent Roboticist, Sweden
   index: 3
date: September 2025
bibliography: paper.bib
---

<!-- Compile using:

docker run --rm --volume $PWD:/data --user $(id -u):$(id -g) --env JOURNAL=joss openjournals/inara
-->

<!-- Relevant JOSS example papers:

- https://joss.theoj.org/papers/10.21105/joss.07481
- https://joss.theoj.org/papers/10.21105/joss.07473
- https://joss.theoj.org/papers/10.21105/joss.06771
- https://joss.theoj.org/papers/10.21105/joss.05647
-->

# Summary


Validation of multi-robot and robot swarm research in the physical world requires a *testbed*, i.e., easily accessible robots and a software stack that is well tested and simplifies the operation of common use cases.
We present Crazyswarm2, a software stack that uses the Robot Operating System 2 (ROS 2) [@ros2] at its core and enables simulation, visualization, and control of commercially off-the-shelf flying robots from Bitcraze AB.
These robots are popular amongst researchers because they are fully open (including schematics and low-level firmware), extendible using standardized connectors, and can be easily obtained world-wide.
Our software made significant changes to Crazyswarm [@crazyswarm], a popular ROS 1-based stack that has been widely used in the research community for planning, state estimation, controls, and even art.
While the high-level API is identical, we used the required breaking changes when moving to ROS 2 to re-visit some core design decisions and enable more sophisticated use-cases compared to the original Crazyswarm.

![Simple representation drawing of Crazyswarm2's functionality](frontimage.png){#frontimage width="100%"}

# Statement of Need

Testbeds are crucial for research in robotics as the simplify and accelerate data collection and validation experiments.
The de-facto standard for physical robots is the Robot Operating System 2 (ROS 2) [@ros2], because robot vendors typically provide drivers and examples using this middleware. 
Most research labs for flying robots either use large custom-built multirotors with a powerful companion computer (e.g., @mrs-uav-system) or the Crazyflie robots by Bitcraze AB.
However, the official vendor software does not have any ROS 2 support and does not scale well to control larger teams of robots.
There are parallel efforts to mitigate the first issue: CrazyChoir [@crazychoir] and AeroStack2 [@aerostack2].
Both rely on the official Python API, making it difficult to use larger teams, while our default backend is written in C++ and significantly faster.
The focus of CrazyChoir is on distributed optimization and for AeroStack2 on high-level missions.
In contrast, Crazyswarm2 provides just the essential tools for simulation, visualization and control to allow a wide variety of applications, including applications that require significant low-level firmware changes.
Differences between the systems and tutorials were provided at the "Aerial Swarm Tools and Applications" workshop at the Robotics Science and Systems conference [@aerialswarms-workshop].
A newer development is Dynamic Swarms Crazyflies [@ds-crazyflies], which has an interesting distributed architecture, where each Crazyflie is controlled by a single ROS 2 node.
Each node uses topics to communicate with a radio node that handles all the communication.
However, for this approach an adjusted version of the vendor-maintained software and firmware has to be installed and flashed, which might give challenges for long-term maintainability of the project


There are also some dedicated existing simulation tools for the Crazyflie robot, e.g., CrazySim [@crazysim], see our recent survey paper on simulation tools for a more detailed list [@aerial-sim-survey]. 
Most of the simulators are developed as a separate tool that have a different communication interface and API compared to those of the real robots.
In Crazyswarm2, the simulation is integrated as a backend, allowing to seamlessly test ROS 2 user-code simply by changing a launch file flag.


<!-- # Design and Implementation Choices -->
# Architecture

<!-- optional, but might be interesting and is something that the RobotDART paper has -->

![Architecture of Crazyswarm2](architecture.png){#architecture-diagram width="50%"}

The architecture of Crazyswarm2 can be found in \autoref{architecture-diagram}.
The Crazyflie server is the node that connects the Crazyflies to the ROS 2 framework.
Its main components contain the Crazyflie Server, the simulation framework, and a separate Python library that simplifies the command handling through ROS 2.

The Crazyflie server receives a list of Crazyflie URIs and ROS server parameters, and it will connect to multiple Crazyflies through multiple Crazyradios.
Moreover, this configuration YAML file will also contain all the logging and parameters that need to be initialized within the Crazyflie ecosystem.
It will then convert those specific logging and parameters to their ROS 2 equivalent and prepare them in proper topics and parameter types.

Moreover, the server also converts any control topics to their Crazyflie framework equivalent through the commander structure, which is used for both the pitch/roll/yaw and velocity/position commands for control in real time.
The [Crazyflie's high-level commander](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/commanders_setpoints/) framework is accessed through ROS 2 services, which only need to be called upon once and the Crazyflie will execute the command fully onboard. Moreover, services also exist to enable logging/parameters upon runtime, as well as an emergency service that will shut down the Crazyflie for safety.

The Crazyflie server includes three different backends: (1) the Crazyflie C++ library, (2) the Crazyflie Python library, and (3) the simulation backend.
A fourth backend that is written in Rust and uses the official Rust library by the vendor is currently work in progress.
(1) was a C++-based library that was developed in light of the Crazyswarm (1) project and reimplemented for Crazyswarm2.
In time, (2) the Crazyflie Python library was added and made almost feature complete with (1).
Cflib (Crazyflie Python Library) is the officially maintained communication library by Bitcraze AB, the developers of the Crazyflie.

The simulation backend acts as a gateway to the hybrid software-in-the-loop (SITL) simulation. The hybrid SITL consists of wrappers of the original Crazyflie's C-based firmware into Python functions, which can be called from the ROS 2 node.
The simulation backend also has various physics and visualization sub-backends to choose from.
The physics backends consist of various options, like a simple quadcopter dynamics based on the Python library NumPy and a dynamics library called [dynobench](https://github.com/quimortiz/dynobench).
The visualization backends consist of libraries like the ROS 2 native RViz2, or Blender for high-level rendering purposes for camera sensing.

Finally, the Crazyswarm2 architecture also consists of a separate ROS 2 package that is a Python library.
The main purpose of this library is to create a simpler interface for the users to control their Crazyflies as a layer above the full ROS 2 interface.
Instead of writing service calls and topic publishers, they can call simple functions per Crazyflie entity in this library, which handles the ROS 2 calling on the backend.

Additionally, the Crazyswarm2 architecture supports integration with motion capture systems which feed positioning data into the Crazyflie server.
To help users get started with the framework, Crazyswarm2 also includes a collection of example scripts that demonstrate common use cases, ranging from simple "Hello World" demonstrations to more complex multi-trajectory coordination scenarios.

Compared to Crazyswarm (1), there are two key differences: 1) the motion capture support is not tightly integrated and instead properly separated; 2) the simulation supports the full ROS interface and physics (including inter-robot interaction forces).

![Communication Latency of Crazyswarm2.](latency.png){#latency-figure width="100%"}

Crazyswarm2 continuously measures the latency of the radio communication by sending the current timestamp to an "echo"-service and recording the timestamp once the packet is returned. The latency is the time difference between those two timestamp.
The latencies for the two backends are shown in \autoref{latency-figure}.
Here, we show the distribution of all latencies, i.e. stacked over all robots for a case with motion capture information being transmitted at 100 Hz.
We consider two cases: 1) external localization, e.g., using a motion capture system. In that case, the position/pose information of all robots is computed centrally and sent to the robots. The cpp-backend uses broadcast messages, while the cflib uses unicast messages, explaining the big difference especially for larger team sizes. 2) self-localization, e.g., by using the LightHouse localization system or on-board sensors. Here, both backends only rely on unicast messages and the difference between the two backends is less pronounced.


# Scientific Impact

<!-- optional, perhaps better suited to integrate into the statement of need? Essentially, we want to already list "users" here -->

Crazyswarm2 has already been used by several researchers, mostly to validate novel algorithms on physical hardware. The following is a non-exhaustive list, demonstrating the usefulness of the package in different areas of robotics.

- Exploration / Active Sensing [@2025-dong-TimeoptimalErgodicSearch;@liu2025probabilistic;@2025-pagano-DistributedMultiRobotActiveSensing;@2025-liu-MultiRobotErgodicTrajectory]
- Novel hardware design / SW frameworks[@2025-boegeat-InnovativeFrugalDesign;@2024-chiun-STARSwarmTechnologya]
- Motion Planning
[@2024-moldagalieva-DbCBSDiscontinuityBoundedConflictBased;@2024-wahba-EfficientOptimizationBasedCable;@2025-wahba-PcdbCBSKinodynamicMotion;@2024-wahba-KinodynamicMotionPlanning;@2024-toumieh-HighSpeedMotionPlanning;@2025-li-TRUSTPlannerTopologyguidedRobust;@2024-khan-SwarmPathDroneSwarm]
- Online learning [@2025-tseng-HybridGradientBasedPolicy;@2025-cobo-briesewitz-NeuralAugmentedIncrementalNonlinear;@2025-lorentz-CrazyMARLDecentralizedDirect]
- Controls [@2024-engl-CoordinatedControlGround;@2025-karasahin-TrajectoryTrackingZeroShot;@2024-yan-CollisionFreeFormationControl;@2023-aram-LeaderFollowerBasedFormation]
- Perception [@2023-moldagalieva-VirtualOmnidirectionalPerception]
- Intergration witih ROS2 [@crazysim]

# Conclusion and Future Work

Crazyswarm2 is a ROS 2 software stack that allows researchers to simulate and physically validate (multi-)robot algorithms.
Compared to other ROS 2 solutions, it supports more (low-level) features, similar to the vendor's official software stack.
Compared to the official software, we add ROS 2 integration, we improve the scalability and usability for larger teams by using broadcast communication, and we include an easy way to simulate robots with the same (ROS 2) API.

In the future, we are planning to add and default to a Rust backend, which will allow us to reuse more of the official vendor's software stack without sacrificing performance for larger teams. We are also planning to improve the swarm management tools, so that the use case of distributed operation is better supported.

# Conflict of Interest

Kimberly N. McGuire started her work on Crazyswarm2 while being employed at Bitcraze AB, Sweden, the vendor of the robots supported in this stack. She is currently an independent roboticist with no financial relationship to Bitcraze AB.

# Acknowledgements

The work was supported by Deutsche Forschungsgemeinschaft (DFG, German Research Foundation) under Grant 448549715 and the Federal Ministry for Research, Technology and Aeronautics Germany (BMFTR) under Grant 16ME1000.

We would also like to thank one of the original author of the predecessor project, Crazyswarm, namely James A. Preiss as well as the contributors of Crazyswarm2.

<!-- # TODO

- add a cool figure
- repo clean-up (docs folder; ros_ws folder) -->

# References