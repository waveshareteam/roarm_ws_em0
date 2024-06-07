import argparse

from ros2web.verb.server import ServerVerb

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    cli = ServerVerb()
    cli.add_arguments(parser, None)
    args = parser.parse_args()
    cli.main(args=args)
