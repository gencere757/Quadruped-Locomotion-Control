"""
Script: test2.py
Author: Arda Gencer
Publishes commands to all 8 leg joints to check wiring.
"""
import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
import time

node = transport.Node()  # transport node for pub/sub
legs = ["FL", "FR", "BL", "BR"]  # leg name labels
pubs = {}  # dict of publisher handles

for leg in legs:  # loop over each leg
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)  # hip publisher for this leg
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)  # knee publisher for this leg

time.sleep(1.5)  # give it a sec to find all 8 subscribers

hip_target = 0.0  # target hip angle
knee_target = -0.6  # target knee angle

for _ in range(20):  # repeat this many times
    for leg in legs:  # loop over each leg
        h, k = Double(), Double()  # new hip and knee messages
        h.data = hip_target  # set hip message value
        k.data = knee_target  # set knee message value
        pubs[f"{leg}_HIP"].publish(h)
        pubs[f"{leg}_KNEE"].publish(k)
    time.sleep(0.1)  # pause between publish cycles

print("done — robot should be holding a crouched stance")
