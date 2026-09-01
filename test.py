import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
import time

node = transport.Node()
topic = "/model/my_quadruped/joint/FL_KNEE/cmd_pos"
pub = node.advertise(topic, Double)

time.sleep(1.5)  # let gz-transport finish discovering the subscriber before we publish

msg = Double()
msg.data = -0.4

for i in range(20):
    pub.publish(msg)
    time.sleep(0.1)

print("done publishing")