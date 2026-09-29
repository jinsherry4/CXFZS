#!/usr/bin/env python3
# 封门：obstacle_1 传送至漏斗口，保持原 z
import rclpy, time, math
from rclpy.node import Node
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState

X, Y = -0.9, -7.0
if len(__import__('sys').argv) > 2:
    X, Y = float(__import__('sys').argv[1]), float(__import__('sys').argv[2])

rclpy.init()
n = Node('seal')
st = {'z': 0.05, 'pos': None}
def cb(m):
    for i, nm in enumerate(m.name):
        if nm == 'obstacle_1':
            st['z'] = m.pose[i].position.z
            st['pos'] = (m.pose[i].position.x, m.pose[i].position.y)
n.create_subscription(ModelStates, '/model_states', cb, 10)
for _ in range(30):
    rclpy.spin_once(n, timeout_sec=0.1)
    if st['pos']:
        break
c = n.create_client(SetEntityState, '/set_entity_state')
c.wait_for_service(timeout_sec=3)
r = SetEntityState.Request()
r.state = EntityState()
r.state.name = 'obstacle_1'
r.state.reference_frame = 'world'
r.state.pose.position.x = float(X)
r.state.pose.position.y = float(Y)
r.state.pose.position.z = float(st['z'])
r.state.pose.orientation.w = 1.0
f = c.call_async(r)
t0 = time.monotonic()
while not f.done() and time.monotonic() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
ok = bool(f.result() and f.result().success)
landed = False
for _ in range(40):
    rclpy.spin_once(n, timeout_sec=0.1)
    if st['pos'] and math.hypot(st['pos'][0]-X, st['pos'][1]-Y) < 0.2:
        landed = True
        break
print('seal rpc-ok:', ok, 'landed:', landed, 'z:', round(st['z'], 3))
