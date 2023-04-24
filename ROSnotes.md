Notes:
- "sudo" means "super user do" and executes command with admin priviledges
- pip virtual environments can be created using the venv command
- ~~I'm running Ubuntu 22.04 Jammy with Python 3.10~~
- I've downgraded to Ubuntu 20.04 and python 3.8.10; turns out doing a dual boot between 22.04 and 20.04 is relatively easy


Python setup steps taken:
- `sudo apt update` to update latest packages and updates for Ubuntu
- `sudo apt install python3-pip` to get pip packages to install
- `sudo apt install python3.8-venv` to enable the creation of virtual environments
- ~~`sudo apt install python3.10-venv` to get virtual envirionments for Ubuntu python 3.10~~ use this for python 3.10
- `python3 -m venv ROS544proj` in a new folder (`cd ~/pyenvs`) to create virtual environment named "ROS544proj"
- `source ~/pyenvs/ROS544proj/bin/activate` to activate/open up a specific environment
- `pip install numpy` to install numpy - repeat for desired packages
- `deactivate` to exit virtual environment

ROS setup steps:
- go to <wiki.ros.org/noetic/installation/Ubuntu> and follow the steps outlined
- add `source /opt/ros/noetic/setup.bash` to your ~/.bashrc file
    - you can do this using : `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`

To create a ROS package:
- Create a directory for the workspace, and give it a subfolder "src" `mkdir -p ~/rosPkgEx_ws/src` the '-p' tells the shell to create any parent directories as needed
- navigate inside the directory `cd ~/rosPkgEx_ws`
- execute `catkin_make`  - this will build, amongst other details, a "build" and "devel" folder in your directory in parallel to the "src" file you've already created
- navigate inside the "src" folder (`cd /src`) and run `catkin_create_pkg package_name rospy type_of_ROS_pkg`
    - this will create a folder for your package, /rosPkgEx_ws/src/package_name and populate a package.xml and CMakeLists.txt
    - most times,  you will not need to specify the type of package
    - Python code, e.g., node.py can be created within the package_name directory
    - be sure to add any python files to CMakeLists.txt:
        ```bash
        ## Declare a python executable
        add_executable(node src/node.py)
        ## add dependencies to the executable
        add_dependencies(node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
        ## Specify libraries to link a library or executable target against
        target_link_libraries(node ${catkin_LIBRARIES})
        ```
- navigate back up to your workspace parent directory `cd ..` and run `catkin_make` again to compile changes and create the package dependencies- this will populate your build folder with the package files
- can add `source ~/rosPkgEx/devel/setup.bash` to the .bashrc so that the system can find your package when running

Connecting nodes/ Network stuff:
- connect to orangepi over ssh using `ssh orangepi@192.168.1.29`
- connect to stanley-linux using `ssh sage@192.168.1.33`

Running ROS:
- The following steps have been aliased and invoked in my .bashrc to streamline ROS operations:
    - run  `source ~/Documents/ROS_helloworld/exp2_ws/devel/setup.bash` to get all the ROS packages available from your workspace
    - run `source ~/pyenvs/ROS544proj/bin/activate` to activate the python environment required
- in a new terminal tab, run `roscore`
- in a new terminal tab, run `roslaunch <package_name> launch_file.launch`
    - This will invoke the xml-formatted .launch command that can initiate nodes, set up graphs, etc.

- To activate nodes manually:
    - run `rosrun waypoint_follower robotNode.py` (rosrun package_name node_name)
- To publish individual messages:
    - run `rostopic pub /pose geometry_msgs/Pose2D "x: 0.0 y: 0.0 theta: 0.0"` (rostopic pub /ros_topic message_type message)
- run `rqt_graph` to visualize nodes
- run `rviz` to visualize robot position, path, data, etc. Note that an rviz session can be saved as a config.rviz file and placed into a launch file for easy re-running
- run `rqt_plot` to graph topic data
- run `rostopic list` or `rosnode list` to visualize topics and nodes, respectively

Running OpenCV:
- refer to [OpenCV-Python Tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- used [OpenCV-Basics Tutorial](https://github.com/nicknochnack/OpenCV-Basics) to help set up my image capture using openCV packages