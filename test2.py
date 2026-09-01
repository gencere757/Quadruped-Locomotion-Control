import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
import time

node = transport.Node()
legs = ["FL", "FR", "BL", "BR"]
pubs = {}

for leg in legs:
    pubs[f"{leg}_HIP"] = node.advertise(f"/model/my_quadruped/joint/{leg}_HIP/cmd_pos", Double)
    pubs[f"{leg}_KNEE"] = node.advertise(f"/model/my_quadruped/joint/{leg}_KNEE/cmd_pos", Double)

time.sleep(1.5)  # let gz-transport finish discovering all 8 subscribers

hip_target = 0.0
knee_target = -0.6

for _ in range(20):
    for leg in legs:
        h, k = Double(), Double()
        h.data = hip_target
        k.data = knee_target
        pubs[f"{leg}_HIP"].publish(h)
        pubs[f"{leg}_KNEE"].publish(k)
    time.sleep(0.1)

print("done — robot should be holding a crouched stance")