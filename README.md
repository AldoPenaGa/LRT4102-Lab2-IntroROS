<p align="center">
  <h2 align="center">Intro to ROS</h2>

  <p align="justify">
  This is the second laboratory report for the course titled Robotic Systems Design (LRT4102). This report will serve as an introduction to the Robot Operating System (ROS) environment.
	  
  <br>Universidad de las Américas Puebla (UDLAP) - Proffessor Dr. César Martínez Torres. "https://www.linkedin.com/in/c%C3%A9sar-martinez-torres-617b5347/?originalSubdomain=mx>" 
  </p>
</p>
<be>

## Table of contents
- [Introduction](#introduction)
- [Problems](#problems)
- [Codes](#codes)
- [Conclusion](#conclusion)
- [Contributors](#codes)

<div align= "justify">

### Introduction

**ROS**

**Control of Systems**

### Problems
The problems were divided into three sections: basic, medium and advanced.

**Basic**
- Create a ROS package named `practicas_lab` with the following dependencies: `rospy`, `roscpp` and `std_msgs`.
- Copy inside the `listener.py` and `talker.py`.
- Compile the package.
- Execute the talker and the listener.
- Conclude about their interaction.

**Medium**
- Create a keyboard control for the turtlesim node.
- Draw a square and an equilateral triangle using turtlesim without the control just made but with functions instead.

**Advanced**
- Create a position P controller for the turtlesim.
- Create a position PI controller for the turtlesim.
- Create a position PID controller for the turtlesim.

### Codes

All the codes come in their original `ROS package`, so they can be easily added to any `catkin_ws`. 

**Basic**

The basic codes can be found in a ROS package called `practicas_lab` inside the folder named `Basic`. This package contains the `listener.py` and the `talker.py`. To understand the functioning of these elements we recommend looking at: https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29 . Nevertheless, a brief explanation is also provided: In ROS, the main communication structure is publishers and subscribers, in which the publishers send data with an specific structure called `messages` through `topics` (for further information type rosmsg and rostopic in a ROS installed environment), these two elements work within subscribers. Lets say a publisher (which can be any node that will be sending information at certain conditions) called Alice (A) wants to send its information (messages) to Bob (B), which is subscribed to the information of Alice because he needs to perform any specific task whenever Alice's information shows up. Bob can be subscribed to any other publisher because he can perform other tasks depending on the messages he's getting. Anyway, when Alice sends messages to Bob, Bob will listen and perform a pre-defined instrucion depending on Alice's message. That is how publishers and subscribers work in ROS. A publisher sends a message through a topic and a subscriber listens and performs. 

In this very specific scenario, our publisher node called `chatter` will print a `string` message that says 'hello world' and the number of the salutation. In the mean time, our subscriber, called `listener` will print 'I heard' + the message recieved, so we know that it was subscribed and listening. This are the basics at ROS communication.

**Medium**

The medium codes can be found in two different ROS packages called `square_turtlesim` and `teleopTurtle` inside the folder named `Medium`. In which both have launch files, so they can be initiated with `roslaunch` command. The `square_turtlesim` package will draw a square and an equilateral triangle too and the `teleopTurtle` will let the user control the turtlesim_node with the keyboard, nevertheless, it is important to mention that this last one does not follow the common conventions for naming a package and can be conflictive so, it is recommended to proceed with caution.

**Advanced**

Finally, the advanced codes can be found in a ROS package called `pid_control_turtlesim` inside the folder named `Advanced`, here a P, PD and PID controllers for the turtlesim node can be found. These codes were a little bit difficult to develop, even though they do the labor intended, they may not reach high expectatives. Although, there will be future updates to the codes so they have a better performance. 

### Conclusion

### Contributors

| Name                          | Github                               |
|-------------------------------|--------------------------------------|
| Aldo Oziel Peña Gamboa        | https://github.com/AldoPenaGa        |
| Joan Carlos Monfil Huitle     | https://github.com/JoanCarlosMonfilHuitle  |
