# SIRA - Socially Intelligent Robot Assistant
--------------------------------
Our lab is motivated to increase functional independence in those with high level spinal cord injuries.  We believe autonomous robots, like Hello Robot’s Stretch model, could be a key part of regaining functional independence and increasing quality of life.  For example, someone who is paralyzed may have trouble picking things up from the ground, or they may rely on somebody else to accomplish this task for them.  However, SIRA is able to pick an object up from the ground and hand it over to the person, eliminating the need for a caregiver and giving them a sense of independence they had previously lost.

# Background Nodes
--------------------------------
## ArUco Marker Detection
The ArUco Marker detection node is utilized for this demo.  An ArUco marker is a black and white square image that can be used to identify things with computer vision (similar to a QR code).  An image of one is shown below: 

![image](https://user-images.githubusercontent.com/87331189/234057458-bbacafe4-9a56-442e-9ff3-80e2037acd54.png)

Stretch has the ability to search for and detect these markers.  You can place them on objects or in specific areas so that the robot has an easier time identifying things.  More information on Stretch's ArUco Marker detection can be found here: [ArUco Marker Detection](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/aruco_marker_detection/)

# USE 
--------------------------------
## Pickup Hat Demo

First, SIRA must be operated untethered.  See the untethered operation guide if you need help, it can be found here: [Untethered Operation Guide](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/untethered_operation/#ros-remote-master).

Then, you must map the robot to its environment, if you have not already done so.  In order to do this, follow the FUNMAP mapping instructions: [Stretch FUNMAP Guide](https://github.com/hello-robot/stretch_ros/tree/master/stretch_funmap).  If you have already done this, make sure you have the name of the map ready.  It should look something like merged_map_20230405112159.  

You are ready to start the demo, run the following command in a terminal: 
```
roslaunch sira hat_pickup.launch map_yaml:=/home/hello-robot/stretch_user/debug/merged_maps/merged_map_name rviz:=false 
```
Where merged_map_name is the correct map for the environment from the last step. 

Then, open RVIZ using the following command 
```
rviz -d $(rospack find sira)/rviz/hat_pickup.rviz 
```
This will open RVIZ with the correct vizualization of the environment.

![Screenshot 2023-04-24 124421](https://user-images.githubusercontent.com/87331189/234063502-dedeb7b7-2d21-462a-b7da-a01c4db7dfa4.png)

You can always manually keyboard teleop Stretch around (e.g. during the mapping step or during the hat pickup demo if FUNMAP fails to find a valid base plan). A menu for keyboard teleop prints in the terminal if you press a key like '4'. 

You must then localize SIRA using the following command in another terminal: 
```
rosservice call /funmap/trigger_global_localization "{}" 
```
This should complete before anything else (it takes a few minutes).  The mapping of the environment in RVIZ should now reflect the actual environment.

![Screenshot 2023-04-24 124442](https://user-images.githubusercontent.com/87331189/234063830-ba259a14-fe5a-4b66-bc49-6ed5584cca14.png)

Now, the demo is ready to initiate.  First the hat needs to be placed and located located.  Use the commands: 
```
rosservice call /hat_locator/clear_saved_locations 
``` 
in order to clear locations (in case it was seen before placement), then initiate a head scan that looks for the hat Aruco marker.   
```
rosservice call /hat_locator/marker_scan
```

Next, we want to make sure that the robot saw the hat.  Use the command: 
```
rosservice call /hat_locator/get_marker_location 
```
in order to see if we successfully located the hat.  The success message should say True. The hat location should be shown in RVIZ as well.

![Screenshot 2023-04-24 124529](https://user-images.githubusercontent.com/87331189/234064283-86e85463-ddd8-4707-8584-195317e7352e.png)

Now, we want to try and pick up the hat.  Use the command: 
```
rosservice call /grasp_off_floor/trigger_grasp_marker "{}" 
```
This should have the robot plan a path to the hat, then execute and grab the hat.  If the plan fails, it will reposition base and try again twice before giving up.  Note: sometimes it does give up even after three tries, we have found usually if you manually teleoperate the base a few inches forward or back then try again, it will have an easier time the next try.  If the plan is successful, you will see the path displayed in RVIZ.

![Screenshot 2023-04-24 124623](https://user-images.githubusercontent.com/87331189/234064412-be6fc235-5b91-4577-badb-2bdf59b3080b.png)

Next, we need to locate the person to hand the hat over to, run the following commands: 
```
rosservice call /julia_locator/clear_saved_locations 
```
First, we clear the saved locations, in case the robot already saw earlier, then we do a marker scan to try and find Julia. 
```
rosservice call /julia_locator/marker_scan 
```

Next, we want to make sure that the robot saw Julia.  Use the command
```
rosservice call /julia_locator/get_marker_location 
```
This should return a success of True, so we know SIRA knows where Julia is. 

Lastly, we want to hand the hat over to Julia.  Run the following command
```
rosservice call /handover_to_julia/trigger_handover "{}" 
```
This should have the robot plan a path to the person, then execute and handover the hat to right in front of their Aruco marker. 

IMPORTANT: This Demo was done with Julia, but can be done with anyone wearing the Aruco marker tag, none of the commands change. 

IMPORTANT: At any time, you can use the command 
```
rosservice list 
```
in order to see what services are currently available to be called. Or the command 
```
rosservice list | grep KEYWORD 
```
where the keyword is in the service you are looking for, for example 
```
rosservice list | grep grasp 
```
will show all the services you can currently call that have to do with grasp (have grasp in the name) 

