from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class WebCommand(CommandExtension):
    def __init__(self):
        super().__init__()
        self._subparser = None

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        add_subparsers_on_demand(parser, cli_name, '_verb', 'ros2web.verb', required=False)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            self._subparser.print_help()
            return 0
        extension = getattr(args, '_verb')
        return extension.main(args=args)
    