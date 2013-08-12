Questions/Support: brad@udel.edu

Thanks to GT for allowing me to model this panel
from the 'hubo init" program.

RViz panel plugin for measurement related tasks
such as fitting a plane, measuring the length of 
a line, etc.


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
_______________________________  IF  YOU  ARE  NEW  TO  ROS  _________________________________
I strongly recommend looking over the tutorials for ROS: http://www.ros.org/wiki/ROS/Tutorials
You will not technically need to understand ROS or how it works to use ud_measurement_panel, but
it is not difficult to learn and will certainly be worthwhile down the road. Regardless,
below I am providing instructions to install and run ud_measurement_panel which should be sufficient
whether or not you are familiar with ROS.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
___________________________ PREREQUISITES ______________________________

This package depends on ROS.

To obtain ROS, follow the instructions here (BE SURE TO INSTALL GROOVY):
http://www.ros.org/wiki/groovy/Installation/Ubuntu

It is also assumed that you have the ROS UD_imarker package installed
as this package is designed to interface with it as the backend. You
can get it here: https://github.com/qiaosongwang/ud_imarker.git

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


~~~~~~~~~~~~~~~~~~~~~~~~~~
_________________Installation (after satisfying the Prerequisites above):____________________

If you do not have a catkin workspace (or you do not know what a catkin workspace is), copy/paste
the following block into a terminal:

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

From now on, the directory ~/catkin_ws/src will be where you keep your ROS package code.


To install ud_measurement_panel, copy/paste the following block into a terminal:

cd ~/catkin_ws/src
git clone https://github.com/qiaosongwang/ud_measurement_panel.git
cd ~/catkin_ws
catkin_make
~~~~~~~~~~~~~~~~~~~~~~~~~~~


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
____________________________ USAGE ___________________________________
To use ud_measurement_panel, do the following:

1) Open a terminal and type in:
    $ roscore
   This will eat up the terminal. Minimize it or move it out of the way.

2) From your workstation (whatever desktop or laptop you use), run
    $ rosrun rviz rviz
    
3) Run UD_iMarker

4) If this is your first time using ud_measurement_panel, do the following:
    a) In the menu bar at the very top, click on 'Panels'
    b) Click on 'Add new panel'
    c) Select ud_measurement_panel
    d) A panel should appear in rviz with measurement related options

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

