import time
import sys
import zenoh
import hl2ss_schema
import logging
import argparse
import yaml
import dataclasses
import json


class EnhancedJSONEncoder(json.JSONEncoder):
    def default(self, o):
        if dataclasses.is_dataclass(o):
            return dataclasses.asdict(o)
        return super().default(o)


# Create a logger object.
logger = logging.getLogger(__name__)

topic_handler_map = {
    cls.__idl_typename__: cls for _, cls in hl2ss_schema.__dict__.items() if
    isinstance(cls, type) and issubclass(cls, hl2ss_schema.IdlStruct)
}
print(topic_handler_map.keys())


class TopicHandler:
    def __init__(self, session, query, root_type):
        self.session = session
        self.query = query
        if root_type not in topic_handler_map:
            raise ValueError(f"Invalid root_type: {root_type} for query: {query}")
        self.root_type = topic_handler_map[root_type]
        print(f"subscribe: {query} with root_type: {root_type}")

    def handler(self, sample):
        try:
            message = self.root_type.deserialize(sample.payload)
            new_topic = f"{sample.key_expr}_json"
            self.session.put(new_topic,
                             json.dumps(message, cls=EnhancedJSONEncoder).encode('utf-8'),
                             encoding=zenoh.Encoding.TEXT_JSON())
        except Exception as e:
            logger.error("Error while handling message")
            logger.exception(e)


def main():
    parser = argparse.ArgumentParser(description='Replicate CDR Messages as json.')
    parser.add_argument('--config', metavar='config', type=str, default='json_replicator.yml',
                        help='config file for json_replicator')

    args = parser.parse_args()

    config = None
    with open(args.config, "r") as f:
        config = yaml.load(f, yaml.CLoader)

    logging.basicConfig(level=logging.DEBUG)

    z = zenoh.open(zenoh.Config())

    subscribers = []
    handlers = []
    for item in config.get("topics", []):
        query = item["query"]
        root_type = item["root_type"]
        handlers.append(TopicHandler(z, query, root_type))

    for idx, item in enumerate(handlers):
        subscriber = z.declare_subscriber(item.query, item.handler)
        subscribers.append(subscriber)

    print("Enter 'q' to quit...")
    c = '\0'
    while c != 'q':
        c = sys.stdin.read(1)
        if c == '':
            time.sleep(1)

    subscriber.undeclare()


if __name__ == "__main__":
    main()
