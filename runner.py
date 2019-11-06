import os
import subprocess
#roslaunch simulator_e130 simulator.launch
#;roslaunch simulator_e130 0.launch
#source devel/setup.bash;
import time
subprocess.call(["pwd", "source devel/setup.bash", "roslaunch simulator_e130 simulator.launch"])

"""
subprocess.call(["pwd;", \
"pwd;source devel/setup.bash"])
time.sleep(1)
subprocess.call("source devel/setup.bash;roslaunch simulator_e130 1.launch")
time.sleep(1)
subprocess.call("source devel/setup.bash;roslaunch motion start.launch")
"""
