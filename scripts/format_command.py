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

import cerebri_common

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
        parser.add_argument('-c', '--check', help='run formatting in check mode', action='store_true')
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

        # check version
        regex = re.compile('(.*\.c$)|(.*\.cpp$)|(.*\.h$)|(.*\.hpp$)')
        format_ok = True

        for path in cerebri_common.source_paths:
            for root, dirs, files in os.walk(path):
                for file in files:
                    if regex.match(file):
                        file_path = os.path.join(root, file)
                        if args.check:
                            cmd_str = ['clang-format-14', '--dry-run', '--Werror', '-style', 'WebKit', f'{file_path}']
                        else:
                            cmd_str = ['clang-format-14', '-i', '-style', 'WebKit', f'{file_path}']
                        try:
                            res = subprocess.run(cmd_str, capture_output=False, check=True)
                        except Exception as e:
                            print(e)
                            format_ok = False
        if not format_ok:
            exit(1)
