# Experimental Robotics Laboratory - Assignment 1

# Jacopo Ciro Soncini 5050695
E-mail: jacopo.soncini@gmail.com

# Brief introduction
The project implements a game of cluedo. This is a first simulation where all of the action are simulated but the updating of the onthology. The robot collects hints and sends them to the armor service that returns any complete hypothesis. Then the robot return home and checks if the hypothesis is correct. In this implemenation hint are all correct and building an incosistent hypothesis should be impossible.

# Preliminary actions

The package is containing the onthology used by the SherlockBot, but it will need to be paired with the Armor package Professor Luca Buoncompagni and Alessio Capitanelli developed,
downloadable at https://github.com/EmaroLab/armor. 

# Software Architecture
The package is composoed of:
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

# Installation and Running Procedure

# Running code

# Working Hypothesis and Environment

## System's Features

## System's Limitations

## Possible Techinical Imporvements


