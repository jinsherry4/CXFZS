#!/usr/bin/env python3
import subprocess, yaml
out = subprocess.run(
    'source /opt/ros/humble/setup.bash && source ~/dev_ws/install/setup.bash'
    ' && timeout 10 ros2 topic echo /model_states --once --yaml',
    shell=True, executable='/bin/bash', capture_output=True, text=True).stdout
d = yaml.safe_load(out)
i = d['name'].index('obstacle_1')
p = d['pose'][i]['position']
print('obstacle_1:', round(p['x'], 2), round(p['y'], 2), round(p['z'], 2))
j = d['name'].index('six_arm')
q = d['pose'][j]['position']
print('six_arm  :', round(q['x'], 2), round(q['y'], 2), round(q['z'], 2))
