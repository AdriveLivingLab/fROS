<h1 align="center" style="display: block; font-size: 2.5em; font-weight: bold; margin-block-start: 1em; margin-block-end: 1em;">
<a name="logo" href="https://www.hs-kempten.de"><img align="center" src="images/logo.png" alt="FRos Home" style="width:80%;height:80%"/></a>
  <br /><br /><strong>FRos</strong>
</h1>

---

## Introduction[:pushpin:](#introduction)

**FRos** is a node framework for the Robot Operating System (ROS1 noetic) to integrate an Ixxat FRC-EP170 gateway device, which is to our knowledge one of the very few gateway devices on the market that can be easily integrated into an ROS environment.
This node handles all the functionalities for interacting with the device and publishes the received messages as topics in ROS. Only required, of course, is the FlexRay network's description in a .xml or .dbc format.

---

## Table of contents[:pushpin:](#table-of-contents)
- [Introduction:pushpin:](#introductionpushpin)
- [Table of contents:pushpin:](#table-of-contentspushpin)
- [Motivation:pushpin:](#motivationpushpin)
- [Code generation over handwork:pushpin:](#code-generation-over-handworkpushpin)
- [Composition:pushpin:](#compositionpushpin)
- [Software build:pushpin:](#software-buildpushpin)
- [Gateway Setup:pushpin:](#gateway-setuppushpin)
    - [ACT Project setup](#act-project-setup)

---

## Motivation[:pushpin:](#motivation)
The robot operating system (ROS) is widely used for research and in commercial products in the area of robotics. It allows for an easy integration of different sensor types into a common environment and due to its open source nature, many different software packages and drivers are already openly available.
It makes sense to also expand the ecosystem beyond the commonly used CAN and Ethernet standard to also include FlexRay to be able to listen to a modern car's internal network. 

This project aims to extend the usage of ROS for FlexRay networks by providing a framework for the currently available Ixxat FRC-EP170 gateway.


[ [↑ to top ↑](#table-of-contents) ]

---

## Code generation over handwork[:pushpin:](#more-than-embedded)

In the area of research, oftentimes the necessary tools need to be developed and be available fast and the role of tools is overshadowed by potential results.
The requirements for a research project may change and the adaption and expansion of the previous research topic needs to happen fast.
Thus, the here presented framework is built with auto code generation in mind. A provided .dbc network description is used to automatically generate all the relevant code snippets and files.

[ [↑ to top ↑](#table-of-contents) ]

---

## Composition[:pushpin:](#composition)

Currently FRos consists of:
1. [The framework itself](./framework/main/) for ROS that handle the connection to the device and the ROS publishers.
2. [ROSCoderdbc](./framework/roscoderdbc/) is a fork of the c-coderdbc by github.com user [astand](https://github.com/astand). This program was created to output C source code from a given .dbc file. It has been modified for this use case to output a more elegant C code based on C structs and also outputs copy-paste ready code snippets to handle the initialization of ROS publishers, to call the appropriate functions based on the message ID and also to generate the necessary .msg files for ROS.

The FRos framework is setup as such, that the relevant code snippets can be easily integrated.

[ [↑ to top ↑](#table-of-contents) ]

---

## Software build[:pushpin:](#software-build)

The framework is in itself complete after you have gone through the code generation with the c-coderdbc program. The package can then be build with ```catkin_make```, just like any other ROS package you know. [catkin_make tutorial](http://wiki.ros.org/catkin/commands/catkin_make)

After compilation, 
```source devel/setup.bash```
and start the node with 
```roslaunch ixxat_gw ixxat_gw.launch```

[ [↑ to top ↑](#table-of-contents) ]

---

## Gateway Setup[:pushpin:](#software-integration)

Generally, the customer support and engineers at HMS will be able to assist you with your project plans. In the following paragraph, we can provide you with hints how to setup your environment so the outcome fits best for you.

#### ACT Project setup

We adapted this project to work two specific possible configurations. One configuration maps a selection of whole frames to GenEthernet, the other maps a selection of whole PDUs to GenEthernet. A third possible configuration, the mapping of all frames to GenEthernet, is not included in this release. This would require a different processing of the message ID.

:writing_hand: Note: FlexRay allows for PDUs, and as such also its signals, to appear in more than one frame. This way, a specific PDU might be updated more frequently by, e.g. publishing it in two separate frames. Thus, with the analysis of only one of the two frames, there is data lost and the observed updaterate of the signals is halved. We can come up with two possible ways to work around that in case this becomes an issue:
- Observe all frames that include this PDU, and with that the signal, and fuse them together in post-processing.
- Change the configuration to map PDUs to GenEthernet, as such the signals will be updated by all the frames the PDU happens to be included.
