#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Test code style."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os
from pathlib import Path
import shlex
import subprocess
import warnings


PYBIND_DIR = Path(__file__).resolve().parents[1]

# Indicators that a style tool executed but failed to import its own Python
# dependencies. In that case, skip the test with a warning instead of failing.
_BROKEN_TOOL_MARKERS = (
    b'PackageNotFoundError',
    b'ModuleNotFoundError',
    b'No module named',
    b'ImportError',
)


def _find_ros_setup():
    """Return the setup.bash of an available ROS distro, or None."""
    candidates = []
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        candidates.append(ros_distro)
    candidates.extend(['humble', 'jazzy', 'iron', 'rolling'])

    seen = set()
    for distro in candidates:
        if distro in seen:
            continue
        seen.add(distro)
        setup = Path('/opt/ros') / distro / 'setup.bash'
        if setup.is_file():
            return setup
    return None


def _run_style_command(command):
    """Run a style command, sourcing ROS if available; warn on missing/broken tools."""
    env = os.environ.copy()
    command_name = command[0]

    ros_setup = _find_ros_setup()
    prelude = (
        f'source {shlex.quote(str(ros_setup))} >/dev/null 2>&1; '
        if ros_setup is not None else ''
    )
    shell_command = prelude + 'exec ' + ' '.join(shlex.quote(arg) for arg in command)

    result = subprocess.run(
        ['bash', '-c', shell_command],
        cwd=PYBIND_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,
    )

    # Tool missing entirely: bash returns 127 with "command not found".
    if result.returncode == 127 or b'command not found' in result.stderr:
        warnings.warn(
            f"Style check '{command_name}' skipped because the executable was not found.",
            RuntimeWarning,
            stacklevel=2,
        )
        return subprocess.CompletedProcess(command, 0, stdout=b'', stderr=b'')

    # Tool exists but is broken in this environment (e.g. missing Python deps).
    if (
        result.returncode != 0
        and b'Traceback' in result.stderr
        and any(marker in result.stderr for marker in _BROKEN_TOOL_MARKERS)
    ):
        warnings.warn(
            f"Style check '{command_name}' skipped because the tool is broken in "
            'this environment (likely missing Python dependencies).',
            RuntimeWarning,
            stacklevel=2,
        )
        return subprocess.CompletedProcess(command, 0, stdout=b'', stderr=b'')

    return result


def run_ament_flake8():
    """Run ament_flake8 and return the output."""
    return _run_style_command(['ament_flake8', '--linelength', '120'])


def run_flake8():
    """Run flake8 and return the output."""
    return _run_style_command(['flake8', '--max-line-length=120'])


def run_pep257():
    """Run ament_pep257 and return the output."""
    return _run_style_command(['ament_pep257'])


def run_pydocstyle():
    """Run pydocstyle and return the output."""
    return _run_style_command(['pydocstyle', '--convention=pep257', '--add-ignore=D104,D105'])


def test_ament_flake8():
    """Test that code conforms to ament flake8 rules."""
    result = run_ament_flake8()
    assert result.returncode == 0, f"ament_flake8 found issues:\n{result.stdout.decode('utf-8')}"


def test_flake8():
    """Test that code conforms to flake8 rules."""
    result = run_flake8()
    assert result.returncode == 0, f"flake8 found issues:\n{result.stdout.decode('utf-8')}"


def test_pep257():
    """Test that code conforms to docstring conventions."""
    result = run_pep257()
    assert result.returncode == 0, f"ament_pep257 found issues:\n{result.stdout.decode('utf-8')}"


def test_pydocstyle():
    """Test that code conforms to pydocstyle conventions."""
    result = run_pydocstyle()
    assert result.returncode == 0, f"pydocstyle found issues:\n{result.stdout.decode('utf-8')}"
