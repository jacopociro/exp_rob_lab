# Experimental Robotics Laboratory - Assignment 1

## Jacopo Ciro Soncini 5050695

### Preliminary actions

The package is containing the onthology used by the SherlockBot, but it will need to be paired with the Armor package Professor Luca Buoncompagni and Alessio Capitanelli developed,
downloadable at https://github.com/EmaroLab/armor. 

### Content description

* Cluedo onthology: onthology given in the assignment and used for reasoning
* CMakeLists.txt: the cmake file of the package
* package.xml: the XML file describing the package requisites
* Doxyfile: file used to generate the documentation
* Docs: folder containing the documentation
* launch: folder containing the launch file (launch.launch) to run every node needed by the robot (not working for now)
* msg: folder containing the custom message (Hint.msg) used by the "hint_publisher" node.
* scripts: folder contaning the four main scripts (hint_publisher.py, hypothesis_maker.py, oracle_service.py and state_machine.py).
* srv: folder containing the custom services (Hypothesis.srv and oracle.srv) used by the "hypothesis_maker2" node and the "oracle" node.

### RQT graph

### State machine graph

### Compiling and running

To compile the package you need to clone it in your ros workspace and then run the command 

> catkin_make

Then you need to run the command 
> roslaunch exp_rob_lab launch.launch
and this should launch all the nodes that are needed, including the armor service.

In case this doesn't work, or if you want to check how the robot works with more feedback, you may need to launch every single node individually by running
> roscore 
>> add '&' to run it in background
> rosrun armor execute it.emarolab.armor.ARMORMainService 
>> **be careful! You must have downloaded the armor package linked in the Preliminary actions and the package must be called armor**

> rosrun exp_rob_lab hint_publisher.py 
> rosrun exp_rob_lab oracle_service.py
> rosrun exp_rob_lab hypothesis_maker.py
> rosrun exp_rob_lab state_machine.py


### Robot behaviour 

### Architectural choices made

### Limitations and possible imporvements


