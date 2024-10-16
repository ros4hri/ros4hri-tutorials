# From Zero to an Interactive Social Robot using ROS4HRI and LLMs

Welcome!

This tutorial will guide you through the installation and use of the ROS4HRI
framework, a set of ROS nodes and tools to build interactive social robots.

We will use a set of pre-configured Docker containers to simplify the setup
process.

We will also explore how a simply yet complete social robot architecture can be
assembled using ROS 2, PAL Robotics' toolset to quickly generate robot
application templates, and a LLM backend.

![Social interaction simulator](../images/social-interaction-simulator.jpg)


**Note: the content on this page is not final, and will be updated before the
tutorial day.**

## Pre-requisites

To follow 'hands-on' the tutorial, you will need to be able to run a Docker
container on your machine, with access to a X server (to display graphical
applications like `rviz` and `rqt`). We will also use the webcam of your
computer.

Any recent Linux distribution should work, as well as MacOS (with XQuartz
installed).


## Preparing your environment

Fetch the `PAL tutorials` public Docker image:

```
docker pull palrobotics/public-tutorials-alum-devel
```

Then, run the container, with access to your webcam and your X server.

```
docker run -it --rm --name ros4hri --device /dev/video0:/dev/video0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix palrobotics/public-tutorials-alum-devel bash
```

> **Note**: The `--device` option is used to pass the webcam to the container, and the
> `-e: DISPLAY` and `-v /tmp/.X11-unix:/tmp/.X11-unix` options are used to display
> graphical applications on your screen.

For convenience, you can also mount a folder from your host machine to the
container, to share files between the two environments:

```
docker run -it --rm --name ros4hri --device /dev/video0:/dev/video0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /path/to/your/folder:/exchange palrobotics/public-tutorials-alum-devel bash
```

## Face detection

### Install hri_face_detect

We first want to detect faces in our test bag file.

We will use a ROS4HRI-compatible node for that purpose: [`hri_face_detect`](https://github.com/ros4hri/hri_face_detect/)

To install it:

First, let's get the code:

```
cd ws/src
git clone https://github.com/ros4hri/hri_face_detect.git
cd ..
```

Then, build it:

```
source /opt/ros/noetic/setup.bash
catkin build hri_face_detect
```

> 💡 all the dependencies are already included in your container, so the
> build step should work straight away. Otherwise, you would have had to install
> manually `mediapipe` (`pip3 install mediapipe`) and all the other ROS
> dependencies (`rosdep install -r -y --from-paths src`).

### Start the face detection node

The `hri_face_detect` node has been installed in the `install/` subfolder. We
need to tell ROS to look into that folder when starting a node. Type:

```
source ./install/setup.bash
```

Then, you can start the face detection node, remapping the default image topic
to the one actually published by the camera:

```
roslaunch hri_face_detect detect.launch rgb_camera:=usb_cam filtering_frame:=head_camera
```

You should immediately see on the console that some faces are indeed detected.
Let's visualise them.


#### Visualise the result

Open another terminal, and source ROS.

We can check that the faces are detected and published at ROS message by simply typing:

```
rostopic echo /humans/faces/tracked
```

We can also use `rviz` to display the faces with the facial landmarks.
Then, in `rviz`, set the fixed frame to `head_camera`, and enable the `Humans` and TF plugins:

![rviz human plugin](../images/rviz-humans-plugin.png)

Configure the `Humans` plugin to use the `/usb_cam/image_raw` topic. You should see the
face being displayed, as well as its estimated 6D position:

![rviz displaying faces](../images/rviz-faces.png)

We are effectively running the face detector in a Docker container, running in a virtual machine somewhere in a Github datacentre!

### Install hri_fullbody

Next, let's detect 3D skeletons in the image.

We will use the ROS4HRI-compatible [`hri_fullbody`](https://github.com/ros4hri/hri_fullbody/) node.

To install it:

First, let's get the code:

```
cd ws/src
git clone https://github.com/ros4hri/hri_fullbody.git
cd ..
```


Then, build it:

```
catkin build hri_fullbody
```

> 💡 again, all the dependencies are already installed. To do it manually: `pip3
> install mediapipe ikpy` followed by `rosdep install -r -y --from-paths src`.

### Start the body detection

First, go back to the terminal playing the bag file. Stop it (Ctrl+C), and start
the second bag file:

```
rosbag play --loop --clock severin-sitting-table.bag
```

Now, open a new terminal, and source `install/setup.bash` (this will also
automatically source `/opt/ros/noetic.setup.bash`):

```
cd ws
source install/setup.bash
```

Start the body detector:

```
roslaunch hri_fullbody hri_fullbody.launch rgb_camera:=usb_cam
```

Re-open the browser tab with `rviz`: you should now see the skeleton being
detected, in addition to the face:

![Body and face, visualised in rviz](../images/body-face.png)

## 'Assembling' full persons

Now that we have a face and a body, we can build a 'full' person.

![ROS4HRI IDs](../images/ros4hri-ids.png)

Until now, we were running two ROS4HRI perception module: `hri_face_detect` and
`hri_fullbody`.

The face detector is assigning a unique identifier to each face that it
detects (and since it only *detects* faces, but does not *recognise* them, a
new identifier might get assigned to the same actual face if it disappears and
reappears later); the body detector is doing the same thing for bodies.

Next, we are going to run a node dedicated to managing full *persons*. Persons
are also assigned an identifier, but the person identifier is meant to be permanent.

First, to avoid generating too many new people, we are going to only publish the
few same frames from the video. Switch back to your `rosbag` terminal. Stop the
current bag (Ctrl+C), and run:

```
rosbag play --loop --clock -s 3 -u 1 severin-sitting-table.bag
```

Then, open a new terminal and install `hri_person_manager`:

```
cd ws/src
git clone https://github.com/ros4hri/hri_person_manager.git
cd ..
catkin build hri_person_manager
```

Source again `install/setup.bash`, configure some general parameters (needed
because we are using a webcam, not an actual robot, [check the doc](https://github.com/ros4hri/hri_person_manager?tab=readme-ov-file#hri_person_manager) to know more), and start
`hri_person_manager`:

```
source install/setup.bash
rosparam set /humans/reference_frame head_camera
rosparam set /humans/robot_reference_frame head_camera
rosrun hri_person_manager hri_person_manager
```

If the face and body detector are still running, you might see that
`hri_person_manager` is already creating some *anonymous* persons: the node
knows that some persons must exist (since faces and bodies are detected), but it
does not know *who* these persons are (you can ignore the warning regarding TF
frames: they come from the use of bag files instead of real 'live' data).

To get 'real' people, we need a node able to match for instance a *face* to a unique and
stable *person*: a face identification node.

### Display the person feature graph

We can use a small utility tool to display what the person manager understand of
the current situation.

Open a new terminal and run:

```
source /opt/ros/noetic/setup.bash
cd ws/src/hri_person_manager/scripts/
./show_humans_graph.py
```

In a different terminal, run:

```
 evince /tmp/graph.pdf
 ```

You should see a graph similar to:

![ROS4HRI graph](../images/ros4hri-graph.png)

### Connecting the person feature graph

First, let's manually tell `hri_person_manager` that the face and body are
indeed parts of the same person. TO do so, we need to publish a *match* between
the two ids (in this example, `rlkas` (the face) and `mnavu` (the body), but
your IDs might be different, as they are randomly chosen)

In a new terminal (with ROS sourced):

```
rostopic pub /humans/candidate_matches hri_msgs/IdsMatch "{id1: 'rlkas', id1_type: 2, id2: 'mnavu', id2_type: 3, confidence: 0.9}"
```

The graph updates to:

![ROS4HRI graph](../images/ros4hri-graph-2.png)

> ⚠️  do not forget to change the face and body IDs to match the ones in your system!

> 💡 the values `2` and `3` correspond respectively to a face and a body. See
> [hri_msgs/IdsMatch](https://github.com/ros4hri/hri_msgs/blob/master/msg/IdsMatch.msg)
> for the list of constants.

### Manually identifying the person

To turn our *anonymous* person into a known person, we need to match the face ID
(or the body ID) to a person ID:

For instance:

```
rostopic pub /humans/candidate_matches hri_msgs/IdsMatch "{id1: 'rlkas', id1_type: 2, id2: 'severin', id2_type: 0, confidence: 0.9}"
```

The graph updates to:

![ROS4HRI graph](../images/ros4hri-graph-3.png)

Know that the person is 'known' (ie, at least one person 'part' is associated to
a person ID)m the automatically-generated 'anonymous' person is replaced by the
actual person.

We are doing it manually here, but in practice, we want to do it automatically.

### Installing and running automatic face identification

Let's install the `hri_face_identification` node:

```
cd ws/src
git clone https://github.com/ros4hri/hri_face_identification.git
cd ..
```


Then, build it:

```
catkin build hri_face_identification
```

> 💡 again, all the dependencies are already installed in the container. To do
> it manually, run `rosdep install -r -y --from-paths src`.


Start the node:

```
source install/setup.bash
roslaunch hri_face_identification face_identification.launch
```

You can now check in the graph (or directly on the `/humans/candidate_matches`
topic): the face should now be automatically associated to a person.

### Probabilistic feature matching

The algorithm used by `hri_person_manager` exploits the probabilities of *match*
between each and all personal features perceived by the robot to find the most
likely set of *partitions* of features into persons.

For instance, from the following graph, try to guess which are the most likely
'person' associations:

![complex ROS4HRI graph](../images/ex1.png)

Response in the paper (along with the exact algorithm!): [the 'Mr Potato' paper](https://academia.skadge.org/publis/lemaignan2024probabilistic.pdf).

## If you want more...!

Here a few additional tasks you might want to try, to further explore ROS4HRI:

- Write a small Python script that list on the console the people around the
  robot ([hint!](https://www.phind.com/search?cache=rhu3n4zmjwshfp0h3vp29b9w)).

- write a node (C++ or Python) to automatically match faces and bodies. One
  approach consists in computing the overlap of the regions of interest of pairs
  of (face, body), and compute a likelihood based on that.

  Check the [`pyhri` API documentation](https://pyhri.readthedocs.io/en/latest/)
  here, and the [C++ `libhri` API
  documentation](http://docs.ros.org/en/noetic/api/hri/html/c++/) here.