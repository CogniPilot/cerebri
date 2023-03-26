# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

'''format_command.py

Formats the code using clang-format
'''

import subprocess
import os
import re

from west.commands import WestCommand  # your extension must subclass this
from west import log                   # use this for user output

class FormatCommand(WestCommand):

    def __init__(self):
        super().__init__(
            'format',               # gets stored as self.name
            'runs formatting',  # self.help
            # self.description:
            '''Runs clang-format''')

    def do_add_parser(self, parser_adder):
        # This is a bit of boilerplate, which allows you full control over the
        # type of argparse handling you want. The "parser_adder" argument is
        # the return value of an argparse.ArgumentParser.add_subparsers() call.
        parser = parser_adder.add_parser(self.name,
                                         help=self.help,
                                         description=self.description)

        # Add some example options using the standard argparse module API.
        #parser.add_argument('-o', '--optional', help='an optional argument')
        #parser.add_argument('required', help='a required argument')

        return parser           # gets stored as self.parser

    def do_run(self, args, unknown_args):
        # This gets called when the user runs the command, e.g.:
        #
        #   $ west my-command-name -o FOO BAR
        #   --optional is FOO
        #   required is BAR
        #log.inf('--optional is', args.optional)
        #log.inf('required is', args.required)
        log.inf('running clang-format')

        regex = re.compile('(.*\.c$)|(.*\.cpp$)|(.*\.h$)|(.*\.hpp$)')

        def format_dir_files(path):
            for root, dirs, files in os.walk(path):
                for file in files:
                    if regex.match(file):
                        file_path = os.path.join(root, file)
                        cmd_str = f'\tclang-format -i -style WebKit {file_path}'
                        print(cmd_str)
                        subprocess.run(cmd_str, shell=True)

        paths = [
         './app',
         './boards',
         './drivers',
         './dts',
         './include',
         './lib',
         './scripts',
         './tests',
         './zephyr',
        ]

        for path in paths:
            print(f'formatting {path}')
            format_dir_files(path)

