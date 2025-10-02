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

# Statement of Need

Testbeds are crucial for research in robotics as the simplify and accelerate data collection and validation experiments.
The de-facto standard for physical robots is the Robot Operating System 2 (ROS 2) [@ros2], because robot vendors typically provide drivers and examples using this middleware. 
Most research labs for flying robots either use large custom-built multirotors with a powerful companion computer (e.g., @mrs-uav-system) or the Crazyflie robots by Bitcraze AB.
However, the official vendor software does not have any ROS 2 support and does not scale well to control larger teams of robots.
There are two parallel efforts to mitigate the first issue: CrazyChoir [@crazychoir] and AeroStack2 [@aerostack2]. Both rely on the official Python API, making it difficult to use larger teams, while our default backend is written in C++ and significantly faster. The focus of CrazyChoir is on distributed optimization and for AeroStack2 on high-level missions. In contrast, Crazyswarm2 provides just the essential tools for simulation, visualization and control to allow a wide variety of applications, including applications that require significant low-level firmware changes. Differences between the systems and tutorials were provided at the "Aerial Swarm Tools and Applications" workshop at the Robotics Science and Systems conference [@aerialswarms-workshop].

AI W: add statement about dynamicsSwarm

<!-- # Design and Implementation Choices -->
# Architecture

<!-- optional, but might be interesting and is something that the RobotDART paper has -->

AI: K
  - architecture diagram + text

AI: W
  - changes compared to CS1, broadly speaking explain reasoning
  - latency measurement cpp/cflib backend over number of CFs (see example from kimberly)

# Scientific Impact

<!-- optional, perhaps better suited to integrate into the statement of need? Essentially, we want to already list "users" here -->

WH

# Conclusion and Future Work

# Conflict of Interest

Kimberly N. McGuire started her work on Crazyswarm2 while being employed at Bitcraze AB, Sweden, the vendor of the robots supported in this stack. She is currently an independent roboticist with no financial relationship to Bitcraze AB.

# Acknowledgements

The work was supported by Deutsche Forschungsgemeinschaft (DFG, German Research Foundation) under Grant 448549715 and the Federal Ministry for Research, Technology and Aeronautics Germany (BMFTR) under Grant 16ME1000.

We would also like to thank one of the original author of the predecessor project, Crazyswarm, namely James A. Preiss as well as the contributors of Crazyswarm2.

# TODO

- mention crazysim (?) [@crazysim]
- mention our prior survey paper on simulation tools [@aerial-sim-survey]
- create a curated list of users / relevant papers that use Crazyswarm2
- latency measurements or other interesting results for the Design / Implementation Choices?
- add a cool figure
- repo clean-up (docs folder; ros_ws folder)
- add dynamicSwarms

# References