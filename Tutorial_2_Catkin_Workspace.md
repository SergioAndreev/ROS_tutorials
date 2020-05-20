# Catkin Workspace

### Catkin packages
ROS software is organized and distributed into packages, which are directories that might contain source code for ROS nodes, libraries, datasets, and more. Each package also contains a file with build instructions - the `CMakeLists.txt` file - and a `package.xml` file with information about the package. Packages enable ROS users to organize useful functionality in a convenient and reusable format.

### Catkin workspaces
A catkin workspace is a top-level directory where you build, install, and modify catkin packages. The workspace contains all of the packages for your project, along with several other directories for the catkin system to use when building executables and other targets from your source code.

# Chapter 1: Create a Catkin Workspace

### Step 1
All of the ROS related code you develop throughout this course will reside in your catkin workspace. You only need to create and initialize the workspace once.

First, create the top level catkin workspace directory and a sub-directory named `src` (pronounced source). The top level directory’s name is arbitrary, but is often called `catkin_ws` (an abbreviation of catkin_workspace), so we will follow this convention. You can create these two directories with a single command:

`$ mkdir -p ~/catkin_ws/src`

### Step 2

Next, navigate to the src directory with the `cd` command:

`$ cd ~/catkin_ws/src`

### Step 3

Now you can initialize the catkin workspace:

`$ catkin_init_workspace`

The expected output:

```bash
Creating symlink "/home/alexander/catkin_ws/src/CMakeLists.txt" pointing to "/opt/ros/melodic/share/catkin/cmake/toplevel.cmake"

```

Let’s list the contents of the current directory to see what changed.

`$ ls -l`
Notice that a symbolic link (`CMakeLists.txt`) has been created to `/opt/ros/kinetic/share/melodic/cmake/toplevel.cmake`
