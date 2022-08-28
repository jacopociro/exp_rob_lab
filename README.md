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

>string id
>string name
>string class_id

### Hypothesis service
The request has the same format as the Hint message. The response is composed of 3 strings (id, who, what, where) and 1 boolean value (consistent). This service is called every time i upload one hint and return any complete hint, with a boolean value that checks if it is consistent.
**Request**
>string id
>string name
>string class_id
**Response**
>string id
>string who
>string where
>string what
>bool consistent

### oracle service 
The request is a string with the id of the hypotesis. The response is an int32 value that checks if the hypothesis is correct.
**Request**
>string id

**Response**
>int32 right

## Nodes
### hint_publisher.py
This node generates random hint from an array of hints and, using a ROS publisher, sends them to whoever subscribes. There are 18 different hint divided in 6 identifiers, each one is consistent.
### hypothesis_maker.py
This node is the one tasked with communicating with the armor service and updating the onthology. It receives the hints and adds them to the onthology, then it checks if there is a complete hypothesis. In case there is it returns the complete hypothesis, otherwise is returns an empty message.
### oracle_service.py
This node is tasked with checking if the complete hypothesis is the correct one. It does so by simply checking if the hypotesis' id is "HP3" or whatever the user sets.
### state_machine.py
This node is the central node and handles the behaviour of the robot, basing it off of states. There are 3 states: move, clues and hyp. The first one handles the movement of the robot to a random room, the second one collects hints and the last one checks if the hypothesis formulated is correct.
## Architecture diagram
![System Architecture](images/exp_rob1.jpg)

![Rqt Graph](images/rosgraph.png)

As we can see the architecture is composed of 5 nodes. The central one is the state machine, which subscribes to the hint publisher and call the services in hypothesis_maker and oracle. 
The hypothesis maker node also call the service in armor, in order to update the cluedo onthology.
## State Machine
![State Machine](images/sm_sys.GIF)

The state machine has 3 states: move, clues and hyp. The machine starts in the move state, from which will move to the clues state.
The clues state has two possible outputs, based on if the received hypothesis is complete or not. If the hypothesis is complete then it will go to the hyp state, to check if it is correct. Otherwise the machine will return to the move state to collect more hints.
The hyp state has two outcomes, if the hypothesis is correct then the game ends, otherwise the machine returns to the move state, to collect more hints.
## Temporal sequence diagram
![System Architecture](images/exp_rob1_seq.jpg)
This image shows a temporal sequence diagram for the simulation, in case every output is ok at the first run. Probably the state machine will be called again and again but the number of times is not fixed, so it is not represented.

# Installation and Running Procedure

# Running code

# Working Hypothesis and Environment

## System's Features

## System's Limitations

## Possible Techinical Imporvements


