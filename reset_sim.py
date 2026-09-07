"""
Script: reset_sim.py
Author: Arda Gencer
Resets the Gazebo simulation back to its initial state.
"""
import gz.transport13 as transport
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
import time

node = transport.Node()  # transport node for pub/sub

req = WorldControl()  # world control request message
req.reset.all = True  # flag to reset the whole world

result, response = node.request("/world/empty/control", req, WorldControl, Boolean, 3000)  # send request, get result and reply
if result:  # check whether the reset request actually succeeded
    print(f"reset sent, ack: {response.data}")
else:
    print("reset request failed/timed out - is gz sim actually running?")

print("waiting for the drop to finish before handing off...")
time.sleep(6.0)  # wait for the drop to settle
print("reset complete")
