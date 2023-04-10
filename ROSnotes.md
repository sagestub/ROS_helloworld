Notes:
- "sudo" means "super user do" and executes command with admin priviledges
- pip virtual environments can be created using the venv command
- ~~I'm running Ubuntu 22.04 Jammy with Python 3.10~~
- I've downgraded to Ubuntu 20.04

Python setup steps taken:
- `sudo apt update` to update latest packages and updates for Ubuntu
- `sudo apt install python3-pip` to get pip packages to install
- ~~`pip install virtualenv` to enable the creation of virtual environments~~ did not work- see next line
- `sudo apt install python3.10-venv` to get virtual envirionments for Ubuntu python 3.10
- `python -m venv ROS544proj` to create virtual environment named "ROS544proj"
- `source ROS544proj/bin/activate` to activate/open up a specific environment
- `pip install numpy` to install numpy - repeat for desired packages
- `deactivate` to exit virtual environment
ROS setup steps:
- go to <wiki.ros.org/noetic/installation/Ubuntu> and follow the steps outlined

