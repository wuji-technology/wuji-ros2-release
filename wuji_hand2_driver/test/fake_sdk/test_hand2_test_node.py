# Copyright 2026 Wuji Technology
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os
from pathlib import Path
import subprocess
import time


def test_terminal_mode_exits_after_grace_window_and_reports_cleanup_order():
    executable = Path(os.environ['WUJI_HAND2_TEST_NODE'])
    assert executable.is_file()
    environment = os.environ.copy()
    environment['WUJI_HAND2_TEST_MODE'] = 'terminal'
    started_at = time.monotonic()

    completed = subprocess.run(
        [str(executable)],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
        timeout=5.0,
    )

    output = completed.stdout + completed.stderr
    elapsed = time.monotonic() - started_at
    assert completed.returncode != 0
    assert elapsed < 3.0, (completed.returncode, output)
    marker = 'WUJI_HAND2_TEST_TERMINAL_CALLS='
    assert marker in output
    calls = output.split(marker, maxsplit=1)[1].splitlines()[0].split(',')
    subscription_close = calls.index('sub_close')
    publisher_close = calls.index('close_joint_command_publisher')
    disable = calls.index('disable')
    disconnect = calls.index('disconnect')
    release = calls.index('release')
    shutdown = calls.index('shutdown')
    assert subscription_close < publisher_close < disable < disconnect < release < shutdown


def test_integer_for_double_parameter_exits_with_readable_error():
    executable = Path(os.environ['WUJI_HAND2_NODE'])
    assert executable.is_file()

    completed = subprocess.run(
        [str(executable), '--ros-args', '-p', 'connect_timeout:=5'],
        capture_output=True,
        check=False,
        text=True,
        timeout=5.0,
    )

    output = completed.stdout + completed.stderr
    assert completed.returncode == 1, output
    assert 'invalid parameter' in output
    assert 'connect_timeout' in output
