import time
import sys
import zenoh
import hl2ss_schema
import coloredlogs
import logging
import argparse

# Create a logger object.
logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', fmt='%(message)s')

LVL_MAP = {
    hl2ss_schema.Hololens2LogLevel.HL2_LOG_ERROR: logger.error,
    hl2ss_schema.Hololens2LogLevel.HL2_LOG_WARNING: logger.warning,
    hl2ss_schema.Hololens2LogLevel.HL2_LOG_INFO: logger.info,
    hl2ss_schema.Hololens2LogLevel.HL2_LOG_DEBUG: logger.debug,
    hl2ss_schema.Hololens2LogLevel.HL2_LOG_TRACE: logger.debug,
}


def log_handler(sample):
    try:
        message = hl2ss_schema.Hololens2LogMessage.deserialize(sample.payload)
        sender = message.header.frame_id
        for item in message.items:
            LVL_MAP[item.severity]("{} >> {}".format(sender, item.message[:-1]))
    except Exception as e:
        logger.error("Error while decoding the Hololens2LogMessage.")


def main():
    parser = argparse.ArgumentParser(description='Display Distributed Logs.')
    parser.add_argument('--query', metavar='query', type=str, default='tcn/loc/*/*/str/logs',
                        help='query for retrieving the logs')

    args = parser.parse_args()

    log_query = args.query

    logging.basicConfig(level=logging.DEBUG)

    z = zenoh.open(zenoh.Config())
    print(f"Listening for logs on '{log_query}'")
    subscriber = z.declare_subscriber(log_query, log_handler)

    print("Enter 'q' to quit...")
    c = '\0'
    while c != 'q':
        c = sys.stdin.read(1)
        if c == '':
            time.sleep(1)

    subscriber.undeclare()


if __name__ == "__main__":
    main()
