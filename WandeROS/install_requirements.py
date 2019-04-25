import os
import sys
import fileinput


def replace_line(file_, pattern, newline):
    for line in fileinput.input(file_, inplace=True):
        if pattern in line:
            line =  str(newline) + "\n"
        sys.stdout.write(line)


def add_ros_repo():
    os.system("sudo sh -c "+
              "'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" "
              +"> /etc/apt/sources.list.d/ros-latest.list'")
    os.system("wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - "
              +"| sudo apt-key add - ")
    os.system("sudo apt update")



def install_ros():
    os.system("sudo apt install ros-melodic-desktop-full")
    os.system("sudo rosdep init & " 
              + "rosdep update ")


def source_ros():
    os.system("echo \"source /opt/ros/melodic/setup.bash\" >> ~/.bashrc & "
              +". ~/.bashrc ")

def install_ros_packages():
    os.system("sudo apt install ros-melodic-roslaunch "
              +"ros-melodic-turtlebot3 ros-melodic-turtlebot3-gazebo " 
              +"ros-melodic-catkin ros-melodic-catkin-virtualenv "
              +"ros-melodic-kobuki-core ros-melodic-kobuki-msgs ")


def fix_ros_ignition():
    os.system("echo '---\\nservers:\\n  -\\n    name: osrf\\n    " 
              +"url: https://api.ignitionrobotics.org' " 
              +"> ~/.ignition/fuel/config.yaml ")
         


def source_turtlebot():
    os.system("echo \"export TURTLEBOT3_MODEL=burger\" >> ~/.bashrc & " 
              +". ~/.bashrc ")
    
    
if __name__ == '__main__':
    add_ros_repo()
    install_ros()
    source_ros()
    install_ros_packages()
    fix_ros_ignition()
    source_turtlebot()
