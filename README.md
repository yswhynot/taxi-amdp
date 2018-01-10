# Abstract MDP for Taxi Problem
Implemented the paper from ICAPS 2017: [Planning with Abstract Markov Decision Processes](https://pdfs.semanticscholar.org/2fa6/b3f543d9b934069fd2f083535638bbc2486d.pdf)  

The original single taxi problem is with the *master* branch. To build the project, do
```
catkin b -vi
```
in your catkin workspace.

The project includes three nodes: **canvas**, **taxi**, **passenger**. Don't forget to run `roscore` before you run any node.
## Canvas
To display the map in ros:
```
rosrun taxi_amdp canvas.py
```
## Passenger
To add a passenger to the map:
```
rosrun taxi_amdp passenger.py
```
Note that passenger node listens to the ros service `/pas_serv`. You need to call the service when you need to change the passenger's state, e.g. get on the taxi.
## Taxi
To run the taxi and the core planning algorithm:
```
rosrun taxi_amdp taxi.py
```  


The multiple taxi extension of the original algorithm could be find in the *multi_pas* branch. 
