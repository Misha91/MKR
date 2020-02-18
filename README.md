# mkr_tm_ws
<br/>
Part I - Multi robot planning 

[video](<https://youtu.be/jGYrxGyDIUE>)

<br/>
Part II - Kalman filter

[video](<https://youtu.be/V-C31kcMgH8>)

<br/>
Part III - Particle filter

[video](<https://youtu.be/sZPrtwbSUmE>)

<br/>

<br/>
<br/>
Part I
<br/>

Launch command
<b>roslaunch motion start.launch</b>

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
<br/>
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
<br/>
<br/>
Ivanov & Uzakov



