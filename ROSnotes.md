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
- `python -m venv ROS544proj` to create virtual environment named "ROS544proj"
- `source ROS544proj/bin/activate` to activate/open up a specific environment
- `pip install numpy` to install numpy - repeat for desired packages
- `deactivate` to exit virtual environment
ROS setup steps:
- go to <wiki.ros.org/noetic/installation/Ubuntu> and follow the steps outlined
- add `source /opt/ros/noetic/setup.bash` to your ~/.bashrc file
    - you can do this using : `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`
To create a ROS package:
- Create a directory for the package, and give it a subfolder "src" `mkdir -p ~/rosPkgEx/src` the '-p' tells the shell to create any parent directories as needed
- navigate inside the directory `cd ~/rosPkgEx`
- execute `catkin_make`  - this will build, amongst other details, a "build" and "devel" folder in your directory in parallel to the "src" file you've already created
- navigate inside the "src" folder `cd /src` and run `catkin_create_pkg package_name rospy type_of_ROS_pkg`
    - this will create a folder for your package, /rosPkgEx/src/package_name and populate a package.xml and CMakeLists.txt
- navigate back up to your workspace parent directory `cd ..` and run `catkin_make` to create the package - this will populate your build folder with the package files
- can add `source ~/rosPkgEx/devel/setup.bash` to the .bashrc so that the system can find your package when running
Connecting nodes/ Network stuff:
- connect to orangepi over ssh using `ssh orangepi@192.168.1.29`
- connect to stanley-linux using `ssh sage@192.168.1.33`
