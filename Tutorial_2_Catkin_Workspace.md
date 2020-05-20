# Catkin Workspace

### Catkin packages
ROS software is organized and distributed into packages, which are directories that might contain source code for ROS nodes, libraries, datasets, and more. Each package also contains a file with build instructions - the `CMakeLists.txt` file - and a `package.xml` file with information about the package. Packages enable ROS users to organize useful functionality in a convenient and reusable format.

### Catkin workspaces
A catkin workspace is a top-level directory where you build, install, and modify catkin packages. The workspace contains all of the packages for your project, along with several other directories for the catkin system to use when building executables and other targets from your source code.

# Chapter 1: Create a Catkin Workspace

### Step 1
All of the ROS related code you develop will reside in your catkin workspace. You only need to create and initialize the workspace once.

First, create the top level catkin workspace directory and a sub-directory named `src` (pronounced source). The top level directory’s name is arbitrary, but is often called `catkin_ws` (an abbreviation of catkin_workspace), so we will follow this convention. You can create these two directories with a single command:

`$ mkdir -p ~/catkin_ws/src`

### Step 2

Next, navigate to the src directory with the `cd` command:

`$ cd ~/catkin_ws/src`

### Step 3

Now you can initialize the catkin workspace:

`$ catkin_init_workspace`

Let’s list the contents of the current directory to see what changed.

`$ ls -l`
Notice that a symbolic link (`CMakeLists.txt`) has been created to `/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake`

Step 4: cd ~/catkin_ws
Return to the top level directory,

$ cd ~/catkin_ws
Step 5: catkin_make
and build the workspace.

Note: you must issue this command from within the top level directory (i.e., within catkin_ws NOT catkin_ws/src)

$ catkin_make
While it is not essential that you have a deep understanding of what the catkin build system is, particularly if you are doing most of your development work in Python, it is helpful to learn about it. The curious reader is encouraged to read the ROS wiki.

After the command is executed you will notice the output of the build processes being echoed to your display. When it has finished you should see the following lines at the end of the output:

-- BUILD_SHARED_LIBS is on
-- Configuring done
-- Generating done
-- Build files have been written to: /home/robo/catkin_ws/build
####
#### Running command: "make -j2 -l2" in "/home/robo/catkin_ws/build"
####
robo@robo-virtual-machine:~/catkin_ws$
But what else has changed? Use the ls command again to see what is new.

$ ls

You now have two new directories: build and devel. The aptly named build directory is the build space for C++ packages and, for the most part, you will not interact with it. The devel directory does contain something of interest, a file named setup.bash. This setup.bash script must be sourced before using the catkin workspace.

Step 6: Commentary
Congratulations! You just created your first catkin workspace.

Before you begin to work with and develop your own ROS package, you should take a moment to get acquainted with catkin workspace conventional directory structure as described in the ROS Enhancement Proposal (REP) 128: http://www.ros.org/reps/rep-0128.html

