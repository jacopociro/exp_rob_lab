# Experimental Robotics Laboratory - Assignment 1

# Jacopo Ciro Soncini 5050695
E-mail: jacopo.soncini@gmail.com

# Brief introduction
The project implements a game of cluedo. This is a first simulation where all of the action are simulated but the updating of the onthology. The robot collects hints and sends them to the armor service that returns any complete hypothesis. Then the robot return home and checks if the hypothesis is correct. In this implemenation hint are all correct and building an incosistent hypothesis should be impossible.

# Preliminary actions

The package is containing the onthology used by the SherlockBot, but it will need to be paired with the Armor package Professor Luca Buoncompagni and Alessio Capitanelli developed, downloadable at https://github.com/EmaroLab/armor. 

# Software Architecture
SherlockBot is handled with a finite state machine and 3 nodes that handle hints, hypothesis and the oracle. The state machine sets the behaviour of the robot and communicates with different services or subscribers. More on the system architecture to be explained with the graphs, first I will explain the contents of the package.
The package is composed of:
- One launch file:
    - launch.launch
- One custom message file:
    - Hint.msg
- Two custom service files:
    - Hypothesis.srv
    - oracle.srv
- Four nodes:
    - hint_publisher.py
    - hypothesis_maker.py
    - oracle_service.py
    - state_machine.py

## Custom messages and services
### Hint message
The message is composed of three string: id, name and class. These are needed to be readable from armor service and upload them correctly on the onthology.
### Hypothesis service
The request has the same format as the Hint message. The response is composed of 3 strings (id, who, what, where) and 1 boolean value (consistent). This service is called every time i upload one hint and return any complete hint, with a boolean value that checks if it is consistent.
### oracle service 
The request is a string with the id of the hypotesis. The response is a boolean value that checks if the hypothesis is correct.

## Nodes
### hint_publisher.py
This node generates random hint from an array of hints and, using a ROS publisher, sends them to whoever subscribes. There are 18 different hint divided in 6 identifiers, each one is consistent.
### hypothesis_maker.py
This node is the one tasked with communicating with the armor service and updating the onthology. It receives the hints and adds them to the onthology, then it checks if there is a complete hypothesis. In case there is it returns the complete hypothesis, otherwise is returns an empty message.
### oracle_service.py
This node is tasked with checking if the complete hypothesis is the correct one. It does so by simply checking if the hypotesis' id is "HP3" or whatever the user sets.

## Architecture diagram
![System Architecture](images/exp_rob1.jpg)

![Rqt Graph](images/rosgraph.png)

## State Machine
![State Machine](images/sm_sys.GIF)
## Temporal sequence diagram


# Installation and Running Procedure

# Running code

# Working Hypothesis and Environment

## System's Features

## System's Limitations

## Possible Techinical Imporvements


