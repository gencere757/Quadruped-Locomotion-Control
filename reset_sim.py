import gz.transport13 as transport
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
import time

node = transport.Node()

req = WorldControl()
req.reset.all = True

result, response = node.request("/world/empty/control", req, WorldControl, Boolean, 3000)
if result:
    print(f"reset sent, ack: {response.data}")
else:
    print("reset request failed/timed out - is gz sim actually running?")

print("waiting for the drop to finish before handing off...")
time.sleep(6.0)
print("reset complete")
