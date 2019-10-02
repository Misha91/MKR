# mkr_tm_ws
*NOT IN PROGRESS*  

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
<b>To-do list and status:  </b>
<br/>
Part I  
Base robot class with properties  
Input to create robots  
Spawn robots in world  
<br/>   
Part II  
A* function  
Planning for each robot  
Checking of colisions/deadlock  
Path modification and veryfication  
<br/>  
Part III   
Generating of waypoints  
Translating of waypoints to robots  
Position update for each time base  


