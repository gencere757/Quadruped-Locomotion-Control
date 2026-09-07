"""
Script: test.py
Author: Arda Gencer
Publishes a single joint command to check pub/sub wiring.
"""
import gz.transport13 as transport
from gz.msgs10.double_pb2 import Double
import time

node = transport.Node()  # transport node for pub/sub
topic = "/model/my_quadruped/joint/FL_KNEE/cmd_pos"  # topic name for the knee joint
pub = node.advertise(topic, Double)  # publisher handle for the topic

time.sleep(1.5)  # give it a sec to find the subscriber before publishing

msg = Double()  # message object to send
msg.data = -0.4  # target position value

for i in range(20):  # loop counter, value unused
    pub.publish(msg)
    time.sleep(0.1)  # short pause between publishes

print("done publishing")
