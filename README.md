# mkr_tm_ws
*NOT IN PROGRESS*  

Guide:  
If you are working under any function/task, change top line to *IN PROGRESS*, commit and push it to repo with message like "MI - in progress" - this will let us know if work in progress and let us to avoid any merge conflict (git commit -m "MI - in progress").  

!!! Don't forget to pull and fetch actual repo before you start working on (see command below) !!!  
!!! Don't forget to push after work is done and update this readme !!!  

It would be also nice to indicate which function you are working on, for example:  

A* planning - *MI*  


Finished work to be indicated as:  

A* planning - MI  


Feel free to split any task on any number of sub-tasks, it's easier to do few small tasks rather than one big.  


Useful Git commands:  
To clone repo  
git clone https://github.com/Misha91/mkr_tm_ws.git  

Check current status (files modified)  
git status  

Stage files to be commited:  
git add --all  

Commit:  
git commit -m "Message"  

Push:  
git push  

Pull and fetch:  
git pull  
git fetch  
  

To-do list and status:  

Part I  

Base robot class with properties  
Input to create robots  
Spawn robots in world  
  
  
Part II  

A* function  

Planning for each robot  
Checking of colisions/deadlock  
Path modification and veryfication  
   
  
Part III   

Generating of waypoints  
Translating of waypoints to robots  
Position update for each time base  


