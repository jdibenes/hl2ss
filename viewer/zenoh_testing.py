import time
import sys

import zenoh
from zenoh import config, QueryTarget
from zenoh import Reliability

import hl2ss_schema


zenoh.init_logger()
conf = zenoh.Config()
session = zenoh.open(conf)

replies = session.get("hl2/cfg/vid/pv/dev00", zenoh.Queue(), target=QueryTarget.BEST_MATCHING())
desc_reply = next(replies).ok
desc = hl2ss_schema.Hololens2StreamDescriptor.deserialize(desc_reply.payload, has_header=False)

sample_store = {}


def listen(s):
    sample_store[s.key_expr] = s.payload
    print("got", s.key_expr)


sub = session.declare_pull_subscriber(desc.stream_topic, listen, reliability=Reliability.RELIABLE())

while True:
    sub.pull()

sub.undeclare()
session.close()
