# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

'''tidy_command.py

Runs clang-tidy
'''

import subprocess
import os
import re

import cerebri_common

from west.commands import WestCommand  # your extension must subclass this
from west import log                   # use this for user output

class TidyCommand(WestCommand):

    def __init__(self):
        super().__init__(
            'tidy',               # gets stored as self.name
            'runs tidy',  # self.help
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
        log.inf('running clang-tidy')

        regex = re.compile('(.*\.c$)|(.*\.cpp$)|(.*\.h$)|(.*\.hpp$)')

        def format_dir_files(path):
            for root, dirs, files in os.walk(path):
                for file in files:
                    if regex.match(file):
                        file_path = os.path.join(root, file)
                        cmd_str = ['clang-tidy', '-p', 'build/', f'{file_path}']
                        print(cmd_str)
                        subprocess.run(cmd_str, shell=True)

        for path in cerebri_common.source_paths:
            print(f'calling tidy on {path}')
            format_dir_files(path)

