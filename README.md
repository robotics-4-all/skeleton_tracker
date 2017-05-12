openni2_tracker
===============

`openni2_tracker` is a ROS Wrapper for the OpenNI2 and NiTE2 Skeleton Tracker.

### Installation

#### Install OpenNI2

Installation instructions: https://github.com/occipital/openni2

1. Install dependencies:

```bash
$ sudo apt-get install libusb-1.0-0-dev libudev-dev python g++ freeglut3-dev openjdk-6-jdk
```

2. Clone and build the OpenNI2 repository from Github:

    ```bash
    git clone git@github.com:occipital/OpenNI2.git
    ```

    Then, switch to the OpenNI2 directory and build:

    ```bash
    cd OpenNI2
    make
    ```

3. Package the build. Navigate to the `Packaging/` directory and execute:

```bash
$ ReleaseVersion.py [x86|x64|arm|android]
```

depending on the machine architecture.
The correcsponding `OpenNI-Linux-{arch}-2.2` will be generated, where \{arch\} is the machine architecture specified above.

3. Run the `install.sh` script located at: `OpenNI-Linux-{arch}-2.2`. This will generate a file that exports ENV variables for Include and Library Paths.

```bash
$ ./install.sh
```

The generated OpenNIDevEnvironment file contains the following (Example for x64 architecture):

```bash
export OPENNI2_INCLUDE=/home/klpanagi/Desktop/OpenNI2/Packaging/OpenNI-Linux-x64-2.2/Include
export OPENNI2_REDIST=/home/klpanagi/Desktop/OpenNI2/Packaging/OpenNI-Linux-x64-2.2/Redist
```

These can be used to include OpenNI2 in your own C++ project

4. For simplicity, if you are going to builde a project that depends on OpenNI2, copy those commands in your shell rc file. For example, if your default shell is `bash`:

```bash
$ echo "export OPENNI2_INCLUDE=/home/klpanagi/Desktop/OpenNI2/Packaging/OpenNI-Linux-x64-2.2/Include" >> ~/.bashrc
$ echo "export OPENNI2_REDIST=/home/klpanagi/Desktop/OpenNI2/Packaging/OpenNI-Linux-x64-2.2/Redist" >> ~/.bashrc
```

*or*

```bash
$ cat OpenNIDevEnvironment >> ~/.bashrc
```

#### Download and Install NiTE2

1. Download [NiTE2-Linux-x64-2.2](http://ilab.usc.edu/packages/forall/current/NiTE-Linux-x64-2.2.tar.bz2)

2. Change directories to the NiTE location and install with

```bash
$ cd NiTE-Linux-x64-2.2
$ ./install.sh
```

This will generate a file that exports ENV variables for Include and Library Paths.

The generated NiTEDevEnvironment file contains the following (Example for x64 architecture):

```bash
export NITE2_INCLUDE=/home/klpanagi/Desktop/NiTE-Linux-x64-2.2/Include
export NITE2_REDIST64=/home/klpanagi/Desktop/NiTE-Linux-x64-2.2/Redist
```

These can be used to include NiTE2 in your own C++ project

3. For simplicity, if you are going to build a project that depends on NiTE, copy those commands in your shell rc file. For example, if your default shell is `bash`:

```bash
$ cat NiTEDevEnvironment >> ~/.bashrc
```
####  Build/Comnpile skeleton_tracker

Build the `skeleton_tracker` package as a regular ros package (in a catkin workspace):

```bash
$ cd {catkin_ws_dir} && catkin_make --pkg skeleton_tracker
```

####  Run skeleton_tracker

```bash
roslaunch skeleton_tracker tracker.launch
```

In the lauch file, you can rename both the tracker name and the tracker's relative frame.  I have included a static publisher that aligns the tracker frame to the world frame, approximately 1.25m off the floor.

```xml
<!-- openni2_tracker Launch File -->
<launch>

<arg name="tracker_name" default="tracker" />

<node name="tracker" output="screen" pkg="skeleton_tracker" type="tracker" >
<param name="tf_prefix" value="$(arg tracker_name)" />
<param name="relative_frame" value="/$(arg tracker_name)_depth_frame" />
</node>

<!-- TF Static Transforms to World -->
<node pkg="tf" type="static_transform_publisher" name="world_to_tracker" args=" 0 0 1.25 1.5707 0 1.7707  /world /$(arg tracker_name)_depth_frame 100"/>

</launch>
```

Currently, this node will broadcast TF frames of the joints of any user being tracked by the tracker.  The frame names are based on the tracker name, currently `/tracker/user_x/joint_name`. The node will also publish the Point Cloud and the Video stream.
