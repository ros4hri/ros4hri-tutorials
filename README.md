# ros4hri-tutorials

## Start the GitHub Codespaces

Start the Github Codespaces by clicking on `Code`, then `Codespaces`.

### Connect to the remote desktop

![Port forwarding](images/vscode-ports.png)

In the Codespaces' VSCode interface, click on the `PORTS` tab (next to
`TERMINAL`), and click on the *Forwarded Address* URL next to the `6080` port forward. Click on the
small globe icon to open a VNC connection to the remote desktop. The password is
`vscode`.

This desktop environment is connected to the Codespaces: any GUI application
that you start from the Codespaces' terminal (like `rviz`) will open in the remote desktop.


## Prepare the environment

### Initial environment preparation

Your environment (called a *devcontainer*) is based on [ROS
noetic](http://wiki.ros.org/noetic). It already contains almost all the standard
ROS tools that we need. Let's add a couple of additional packages required for
this tutorial:

```
sudo apt update
sudo apt install evince python3-graphviz wget python3-pip python3-catkin-tools
```

Next, we create a basic ROS *workspace*, so that we can compile ROS nodes:

```
mkdir -p ws/src
cd ws
catkin init
catkin config --install
cd ..
rosdep update
```

## Playing ROS 'bags'

ROS bags are files containing ROS data (ROS *messages*) that have been
previously recorded. We can *play* ROS bag to re-create the datastream as they
were when they were recorded.

### Start ROS

In the same terminal, source your ROS environment:

```
source /opt/ros/noetic
```

**Note: each time you open a new terminal, you first need to source your ROS
environment with this command.**

Then, start `roscore`

```
roscore
```

### Download the bag files

I have prepared some bags for today's tutorial. Download them with `wget`:

```
cd bags
wget https://skadge.org/data/severin-head.bag
wget https://skadge.org/data/severin-sitting-table.bag
cd ..
```

### Display the content of the bag file

Open a new terminal by clicking on the `+` at the right of the termnial panel.

Source the ROS environment, and play the pre-recorded bag file:

```
source /opt/ros/noetic/setup.bash
rosbag play --loop bags/severin-head.bag
```

Type `rostopic list` to list the available ROS topic (ie, the ROS data
channels). You should see at least `/usb_cam/image_raw` that contains the raw
image pixels data.

Open yet another terminal, source ROS, and open `rqt_image_view`:

```
source /opt/ros/noetic/setup.bash
rqt_image_view
```

Switch to the remote desktop tab. You should see the RQT `image_view`
interface. Select the `/usb_cam/image_raw` topic in the drop-down list. It
should display the video stream.

![rqt_image_view](images/rqt_image_view.png)

We can alos visualize the bag file in `rviz`, the main ROS tool for data
visualization. Stop `rqt_image_view` (either close the window, or press Ctrl+C
in the terminal), and start `rviz` instead:

```
rviz
```

*Add* an `Image` plugin, and select the `/usb_cam/image_raw` topic like on the
screenshot below:

![rviz](images/rviz.png)


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

Then, let's install the dependencies:

```
pip3 install mediapipe
rosdep install -r -y --from-paths src
```

Finally, build it:

```
catkin build hri_face_detect
```

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

We can also use `rviz` to display the faces with the facial landmarks. First,
install the `rviz` ROS4HRI plugin:

```
sudo apt install ros-noetic-hri-rviz
```

Then, start `rviz`, set the fixed frame to `head_camera`, and enable the `Humans` and TF plugins:

![rviz human plugin](images/rviz-humans-plugin.png)

Configure the `Humans` plugin to use the `/usb_cam/image_raw` topic. You should see the
face being displayed, as well as its estimated 6D position:

![rviz displaying faces](images/rviz-faces.png)

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

Then, let's install the dependencies:

```
pip3 install ikpy
rosdep install -r -y --from-paths src
```

Finally, build it:

```
catkin build hri_fullbody
```

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
roslaunch hri_fullbody detect.launch rgb_camera:=usb_cam
```

Re-open the browser tab with `rviz`: you should now see the skeleton being
detected, in addition to the face:

![Body and face, visualised in rviz](images/body-face.png)

## 'Assembling' full persons

Now that we have a face and a body, we can build a 'full' person.

![ROS4HRI IDs](images/ros4hri-ids.png)

Until now, we were running two ROS4HRI perception module: `hri_face_detect` and
`hri_fullbody`.

The face detector is assigning a unique identifier to each face that it
detects (and since it only *detects* faces, but does not *recognise* them, a
new identifier might get assigned to the same actual face if it disappears and
reappears later); the body detector is doing the same thing for bodies.

Next, we are going to run a node dedicated to managing full *persons*. Persons
are also assigned an identifier, but the person identifier is meant to be permanent.

Let install `hri_person_manager`:

```
cd ws/src
git clone https://github.com/ros4hri/hri_person_manager.git
cd ..
rosdep install -r -y --from-paths src
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
does not know *who* these persons are.

For that, we need a node able to match for instance a *face* to a unique and
stable *person*: a face identification node.

### 'Manual' face identification

Before doing it automatically with a dedicated node, let's do it manually, to
understand how this work.

### Installing and running automatic face identification

### Probabilistic feature matching

The algorithm used by `hri_person_manager` exploits the probabilities of *match*
between each and all personal features perceived by the robot to find the most
likely set of *partitions* of features into persons.

If you want to know more about the exact algorithm, [check the 'Mr Potato' paper!](https://academia.skadge.org/publis/lemaignan2024probabilistic.pdf).
