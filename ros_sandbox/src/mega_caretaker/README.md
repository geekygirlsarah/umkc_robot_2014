###Commands


To start up both the mega_caretaker node and the rosserial node
```
roslaunch mega_caretaker mega_caretaker.launch
```


If it complains about not finding the package, execute this at the top of your catkin workspace.
```
source devel/setup.bash
```


### Communicating with the rest of board

####Rostopics
* /mega/command 
* /mega/response


On the topic /mega/command, publish the following to tell it to get tools.
```
rostopic pub -1 /mega/command mega_caretaker/MasterPacket '{msgType: 0, payload: 0}'
```

Once it is done, the node will publish a command to the /mega/response topic with msgType:2, payload:0.



On the topic /mega/command, publish the following to tell it to start crossing waves.
```
rostopic pub -1 /mega/command mega_caretaker/MasterPacket '{msgType: 0, payload: 1}'
```

Once it is done, the node will publish a command to the /mega/response topic with msgType:2, payload:1.



