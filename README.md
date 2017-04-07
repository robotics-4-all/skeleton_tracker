openni2_tracker
===============

`openni2_tracker` is a ROS Wrapper for the OpenNI2 and NiTE2 Skeleton Tracker.

### Installation

## Install OpenNI2

Installation instructions: https://github.com/occipital/openni2

1. Install dependencies:

```bash
$ sudo apt-get install libusb-1.0-0-dev libudev-dev python g++ freeglut3-dev openjdk-6-jdk
```

2. Clone and build the OpenNI2 repository from Github:

    ```bash
    git clone git@github.com:robotics-4-all/OpenNI2.git
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

For simplicity, if you are going to builde a project that depends on OpenNI2, copy those commands in your shell rc file. For example, if your default shell is `bash`:

```bash
$ echo "export OPENNI2_INCLUDE=/home/klpanagi/Desktop/OpenNI2/Packaging/OpenNI-Linux-x64-2.2/Include" >> ~/.bashrc
$ echo "export OPENNI2_REDIST=/home/klpanagi/Desktop/OpenNI2/Packaging/OpenNI-Linux-x64-2.2/Redist" >> ~/.bashrc
```

*or*

```bash
$ cat OpenNIDevEnvironment >> ~/.bashrc
```


2. Install Nite2 from the OpenNI Website [here](http://www.openni.org/files/nite/?count=1&download=http://www.openni.org/wp-content/uploads/2013/10/NiTE-Linux-x64-2.2.tar1.zip).  Be sure to match the version (x86 or x64) with the version of OpenNI2 you installed above.
You will probably need to create a free account.

    Change directories to the NiTE location and install with

    ```bash
    cd NiTE-Linux-x64-2.2
    sudo ./install.sh
    ```
    Try and run one of the examples in `.../NiTE-Linux-x64-2.2/Samples/Bin/NiTE2`.  If these don't work, then something went wrong with your installation.

3. Clone `openni2_tracker` to your ROS workspace.

    ```bash
    git clone git@github.com:futureneer/openni2-tracker.git
    ```

4. Configure CMake
    In the CMakeLists.txt file inside the `openni2_tracker` package, you will need to change the path where CMake will look for OpenNI2 and NiTE2.  These two lines:

    ```makefile
    set(OPENNI2_DIR ~/OpenNI2)
		set(OPENNI2_WRAPPER /opt/ros/indigo/include/openni2_camera/)
		set(NITE2_DIR ~/Openni2/NiTE-2.0.0/)
		set(NITE2_LIB ~/Openni2/NiTE-2.0.0/Redist/libNiTE2.so)

		link_directories(${OPENNI2_DIR}/Bin/x64-Release)

		include_directories(${OPENNI2_DIR}/Bin/x64-Release)
		include_directories(/usr/include/openni2/)
		include_directories(${OPENNI2_DIR}/Include)
		include_directories(${OPENNI2_WRAPPER})
		include_directories(${NITE2_DIR}/Include)
		include_directories(${OpenCV_INCLUDE_DIRS}/include)
    ```
    need to point to the root directories of where you extracted or cloned OpenNI2 and NiTE2.


5. Make openni2_tracker

    ```bash
    cd your_catkin_workspace/
    catkin_make
    ```

6. Set up NiTE2: Right now, NiTE requires that any executables point to a training sample directory at `.../NiTE-Linux-x64-2.2/Samples/Bin/NiTE2`.  If you run the NiTE sample code, this works fine because those examples are in that same directory.  However, to be able to roslaunch or rosrun openni2_tracker from any current directory, I have created a workaround script `setup_nite.bash`.  This script creates a symbolic link of the NiTE2 directory in your .ros directory (the default working directory for roslaunch / rosrun).  You will need to modify this file so that it points to YOUR NiTE2 and .ros locations.  I would be pleased if anyone has a better solution to this.
7. Run openni2_tracker

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

### THANKS!
Please let me know if something doesnt work, or if you have suggestions (or feel free to add stuff and send a pull request).
