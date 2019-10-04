# mkr_tm_ws
*NOT IN PROGRESS*  
<br/>
<b>To-do list and status:  </b>
<br/>
<i>Part I</i>  
Base robot class with properties - TU !Please check scripts folder
<br/>
Input to create robots  - TU
<br/>
Spawn robots in world  
<br/>   
<i>Part II</i>   
A* function  
Planning for each robot  
Checking of colisions/deadlock  
Path modification and veryfication  
<br/>  
<i>Part III</i>  
Generating of waypoints  
Translating of waypoints to robots  
Position update for each time base  
<br/>
<br/>
<b>Initialize robots:  </b>
<br/>
1) Set starting positions of the robot in "robot_start_positions.csv" file, which can be found in "data" folder.
<br/>
The configuration is normal: x,y,theta
<br/>
Each row is one robot
<br/>
2) Consider x and y of goal position
<br/>
3) Run command "python robot_input.py x y", where x and y are the considered coordinates
<br/>
4) Check the output of terminal, it should state info about robots.
<br/>
5) The info is stored in "robot_data.csv" file in "data" folder
<br/>
<br/>
<br/>
<b>Suggestions for finishing the first part:</b>
<br/>
Have a look at robots.launch in simulator_e130 folder.
<br/>
You will probably need to write a script that changes its configurations so that the robots are inserted there.
<br/>
Have fun! :)
<br/>
<b>Guide:  </b>
<br/>
If you are working under any function/task, change top line to *IN PROGRESS*, commit and push it to repo with message like "MI - in progress" - this will let us know if work in progress and let us to avoid any merge conflict (git commit -m "MI - in progress").  
<br/>
!!! Don't forget to pull and fetch actual repo before you start working on (see command below) !!!  
!!! Don't forget to push after work is done and update this readme !!!  
<br/>
It would be also nice to indicate which function you are working on, for example:  
A* planning - *MI*  
<br/>
Finished work to be indicated as:  
A* planning - MI  
<br/>
Feel free to split any task on any number of sub-tasks, it's easier to do few small tasks rather than one big.  
<br />
Install stdr_simulator and turtlebot_simulator packages yourself, because it depends on the version of ROS that you have  
<br/>
<b>Useful Git commands:  </b>
<br/>
To clone repo  
git clone https://github.com/Misha91/mkr_tm_ws.git  
<br/>
Check current status (files modified)  
git status  
<br/>
Stage files to be commited:  
git add --all  
<br/>
Commit:  
git commit -m "Message"  
<br/>
Push:  
git push  
<br/>
Pull and fetch:  
git pull  
git fetch  
<br/>




