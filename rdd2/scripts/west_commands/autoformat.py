# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

from west.commands import WestCommand

CLANG_FORMAT_VERSION = "18.1.8"
CLANG_FORMAT_PACKAGE = f"clang-format=={CLANG_FORMAT_VERSION}"
SUPPORTED_SUFFIXES = {
    ".c",
    ".cc",
    ".cpp",
    ".cxx",
    ".h",
    ".hh",
    ".hpp",
    ".hxx",
}
FORMAT_BATCH_SIZE = 100

AUTOFORMAT_DESCRIPTION = f"""\
Format tracked C/C++ files with a pinned clang-format toolchain.

This command bootstraps a workspace-local virtualenv in `.west/tools/autoformat`
and installs `{CLANG_FORMAT_PACKAGE}` there. By default it formats tracked
source files under the current app directory (`rdd2/`). Pass files or
directories to target other tracked paths in the workspace.
"""


class Autoformat(WestCommand):
    def __init__(self):
        super().__init__(
            "autoformat",
            "format tracked C/C++ files with pinned clang-format",
            AUTOFORMAT_DESCRIPTION,
            accepts_unknown_args=False,
        )

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            description=self.description,
            formatter_class=argparse.RawDescriptionHelpFormatter,
        )
        parser.add_argument(
            "--check",
            action="store_true",
            help="report formatting drift without modifying files",
        )
        parser.add_argument(
            "paths",
            nargs="*",
            help="tracked files or directories to format; defaults to the current app directory",
        )
        return parser

    def do_run(self, args, unknown_args):
        if unknown_args:
            self.die(f"unexpected arguments: {' '.join(unknown_args)}")

        topdir = Path(self.topdir).resolve()
        app_dir = Path(self.manifest.abspath).resolve().parent
        style_file = topdir / "zephyr" / ".clang-format"

        if not style_file.is_file():
            self.die(f"missing style file: {style_file}")

        files = self._collect_files(app_dir, args.paths)
        if not files:
            self.die("no tracked C/C++ files matched")

        clang_format = self._ensure_clang_format(topdir)
        mode = "checking" if args.check else "formatting"

        self.inf(
            f"{mode} {len(files)} file(s) with {clang_format} "
            f"({CLANG_FORMAT_PACKAGE})"
        )

        self._run_clang_format(clang_format, style_file, files, args.check)

    def _collect_files(self, app_dir, raw_paths):
        groups = defaultdict(list)
        targets = list(raw_paths) if raw_paths else [str(app_dir)]

        for raw_path in targets:
            candidate = Path(raw_path)
            if not candidate.is_absolute():
                candidate = (Path.cwd() / candidate).resolve()
            else:
                candidate = candidate.resolve()

            if not candidate.exists():
                self.die(f"path does not exist: {candidate}")

            repo_root = self._git_toplevel(candidate)
            try:
                relative = candidate.relative_to(repo_root)
            except ValueError as err:
                self.die(f"cannot resolve {candidate} inside git repo {repo_root}: {err}")
            groups[repo_root].append(relative)

        files = set()
        for repo_root, pathspecs in groups.items():
            for relpath in self._git_ls_files(repo_root, pathspecs):
                abspath = (repo_root / relpath).resolve()
                if abspath.suffix.lower() in SUPPORTED_SUFFIXES and abspath.is_file():
                    files.add(abspath)

        return sorted(files)

    def _ensure_clang_format(self, topdir):
        tools_dir = topdir / ".west" / "tools" / "autoformat"
        venv_dir = tools_dir / "venv"
        stamp = tools_dir / "clang-format.version"
        python = self._venv_executable(venv_dir, "python")
        clang_format = self._venv_executable(venv_dir, "clang-format")
        expected = CLANG_FORMAT_PACKAGE + "\n"

        tools_dir.mkdir(parents=True, exist_ok=True)

        needs_install = (
            not python.is_file()
            or not clang_format.is_file()
            or not stamp.is_file()
            or stamp.read_text(encoding="utf-8") != expected
        )

        if needs_install:
            if not python.is_file():
                self.inf(f"creating autoformat virtualenv at {venv_dir}")
                self._run([sys.executable, "-m", "venv", str(venv_dir)])

            self.inf(f"installing {CLANG_FORMAT_PACKAGE}")
            self._run(
                [
                    str(python),
                    "-m",
                    "pip",
                    "install",
                    "--disable-pip-version-check",
                    "--upgrade",
                    "pip",
                ]
            )
            self._run(
                [
                    str(python),
                    "-m",
                    "pip",
                    "install",
                    "--disable-pip-version-check",
                    CLANG_FORMAT_PACKAGE,
                ]
            )
            stamp.write_text(expected, encoding="utf-8")

        if not clang_format.is_file():
            self.die(f"missing clang-format executable after install: {clang_format}")

        return clang_format

    def _run_clang_format(self, clang_format, style_file, files, check):
        base_cmd = [
            str(clang_format),
            f"-style=file:{style_file}",
        ]

        if check:
            base_cmd.extend(["--dry-run", "--Werror"])
        else:
            base_cmd.append("-i")

        for index in range(0, len(files), FORMAT_BATCH_SIZE):
            batch = files[index:index + FORMAT_BATCH_SIZE]
            self._run(base_cmd + [str(path) for path in batch])

    def _git_toplevel(self, path):
        cwd = path if path.is_dir() else path.parent
        output = self._capture(
            ["git", "-C", str(cwd), "rev-parse", "--show-toplevel"]
        )
        return Path(output.strip()).resolve()

    def _git_ls_files(self, repo_root, pathspecs):
        cmd = ["git", "-C", str(repo_root), "ls-files", "-z", "--"]
        cmd.extend(self._pathspec(path) for path in pathspecs)
        output = self._capture(cmd, text=False)
        return [
            Path(entry.decode("utf-8"))
            for entry in output.split(b"\0")
            if entry
        ]

    @staticmethod
    def _pathspec(path):
        if str(path) == ".":
            return "."
        return path.as_posix()

    @staticmethod
    def _venv_executable(venv_dir, name):
        scripts_dir = "Scripts" if os.name == "nt" else "bin"
        executable = name + (".exe" if os.name == "nt" else "")
        return venv_dir / scripts_dir / executable

    def _capture(self, cmd, text=True):
        try:
            result = subprocess.run(
                cmd,
                check=True,
                capture_output=True,
                text=text,
            )
        except subprocess.CalledProcessError as err:
            stderr = err.stderr.decode("utf-8", errors="replace") if not text else err.stderr
            stdout = err.stdout.decode("utf-8", errors="replace") if not text else err.stdout
            message = stderr or stdout or str(err)
            self.die(message.strip())

        return result.stdout

    def _run(self, cmd):
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as err:
            self.die(str(err))
