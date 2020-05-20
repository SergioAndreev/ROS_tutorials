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

The expected output:

```bash
Creating symlink "/home/alexander/catkin_ws/src/CMakeLists.txt" pointing to "/opt/ros/melodic/share/catkin/cmake/toplevel.cmake"
```

Let’s list the contents of the current directory to see what changed.

`$ ls -l`
Notice that a symbolic link (`CMakeLists.txt`) has been created to `/opt/ros/melodic/share/catkin/cmake/toplevel.cmake`

### Step 4
Return to the top level directory,

`$ cd ~/catkin_ws`

and build the workspace.

**Note**: you must issue this command from within the top level directory (i.e., within `catkin_ws` NOT `catkin_ws/src`)

`$ catkin_make`

While it is not essential that you have a deep understanding of what the catkin build system is, particularly if you are doing most of your development work in Python, it is helpful to [learn about it](http://wiki.ros.org/catkin/conceptual_overview).

After the command is executed you will notice the output of the build processes being echoed to your display. When it has finished you should see the following lines at the end of the output:

```bash
-- BUILD_SHARED_LIBS is on
-- Configuring done
-- Generating done
-- Build files have been written to: `/home/robo/catkin_ws/build`
####
#### Running command: "make -j2 -l2" in "/home/robo/catkin_ws/build"
####
robo@robo-virtual-machine:~/catkin_ws$
```

But what else has changed? Use the `ls` command again to see what is new.
`$ ls`

You now have two new directories: `build` and `devel`. The aptly named `build` directory is the build space for C++ packages and, for the most part, you will not interact with it. The `devel` directory does contain something of interest, a file named `setup.bash`. This `setup.bash` script must be sourced before using the catkin workspace.

### Step 5
You just created your first catkin workspace.

Before you begin to work with and develop your own ROS package, you should take a moment to get acquainted with catkin workspace conventional directory structure as described in the ROS Enhancement Proposal (REP) 128:[here](http://www.ros.org/reps/rep-0128.html)

<details><summary>Click to see</summary>
<p>
  
```bash
workspace_folder/        --WORKSPACE
  src/                   --SOURCE SPACE
    CMakeLists.txt       --This is symlinked to catkin/cmake/toplevel.cmake
    package_1/
      CMakeLists.txt
      package.xml
    ...
    package_n/
      CATKIN_IGNORE      --Optionally place this marker file to exclude package_n from being processed. Its file type (e.g. regular file, directory) and contents don't matter. It may even be a dangling symlink.
      CMakeLists.txt
      package.xml
  build/                 --BUILD SPACE(this is where build system is invoked, not necessarily within workspace)
    CATKIN_IGNORE        --Marking the folder to be ignored when crawling for packages (necessary when source space is in the root of the workspace, the file is emtpy)
  devel/                 --DEVEL SPACE (targets go here, parameterizable, but defaults to peer of Build Space)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin              --Marking the folder as a development space (the file contains a semicolon separated list of Source space paths)
    env.bash
    setup.bash
    setup.sh
    ...
  install/               --INSTALL SPACE (this is where installed targets for test installations go, not necessarily within workspace)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin              --Marking the folder as an install space (the file is empty)
    env.bash
    setup.bash
    setup.sh
    ...
```

</p>
</details>

# Chapter 2: Add a package

### Step 1: Cloning the simple_arm Package
One of the biggest benefits of using ROS is that it has a really large community of users and developers, so there is a lot of code that you can use.

Let’s clone an existing package and add it to our newly created workspace.

You will start by navigating to the src directory and cloning the simple_arm package for this lesson from its github repo.

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/simple_arm_01.git simple_arm
```

### Step 2: Building the simple_arm package
After the repo has finished cloning, you can change directory to the top-level of the ros workspace and build the new package.

```bash
$ cd ~/catkin_ws
$ catkin_make
```

I see a CMake Error. "Could not find a package configuration file provided by controller_manager"

### Step 3: Installing Missing Packages Using apt-get
I happen to know that controller_manager refers to a ROS package from ROS Control. We can fix this by installing the associated Debian package. If I didn't already know this, I would probably have to rely on a Google search to figure out the exact name of the package required.

```$ sudo apt-get install ros-kinetic-controller-manager```

Some students have had success using the following commands to install missing packages:

```bash
$ source devel/setup.bash 
$ rosdep install simple_arm
```

OK, now that we have the controller-manager package let’s try building again. I'm still in the top level directory, so I can just type `catkin_make` and hit enter.

```$ catkin_make```

Looks like the build worked. Great, that wasn't so bad. Let’s run some of this code that we just cloned!
