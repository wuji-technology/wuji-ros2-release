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
import signal
import subprocess
import threading
import time

import pytest


CASES = [
    ('required_end', 1),
    ('required_error', 1),
    ('optional_end', 0),
    ('optional_error', 0),
    ('normal_shutdown', 0),
]

EVENT_PREFIX = 'WUJI_GLOVE_TEST_EVENT '
SUBSCRIPTIONS = {
    'subscribe_joint_angles': 'joint_angles',
    'subscribe_emf_poses': 'emf_poses',
    'subscribe_tip_poses': 'tip_poses',
    'subscribe_hand_skeleton': 'hand_skeleton',
    'subscribe_imu_palm': 'imu_palm',
    'subscribe_tactile': 'tactile',
    'subscribe_tactile_residual': 'tactile_residual',
    'subscribe_tactile_binary': 'tactile_binary',
    'subscribe_tactile_zones': 'tactile_zones',
    'subscribe_tactile_point_cloud': 'tactile_point_cloud',
}


class ProcessOutput:

    def __init__(self, process):
        self._process = process
        self._lines = []
        self._condition = threading.Condition()
        self._eof = False
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for raw_line in self._process.stdout:
            with self._condition:
                self._lines.append(raw_line.rstrip('\n'))
                self._condition.notify_all()
        with self._condition:
            self._eof = True
            self._condition.notify_all()

    def wait_for_event(self, event, deadline):
        exact_line = EVENT_PREFIX + event
        with self._condition:
            while exact_line not in self._lines:
                if self._eof:
                    raise AssertionError(self.dump())
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AssertionError(self.dump())
                self._condition.wait(timeout=remaining)

    def finish(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise subprocess.TimeoutExpired(self._process.args, 5.0)
        return_code = self._process.wait(timeout=remaining)
        self._reader.join(timeout=max(0.0, deadline - time.monotonic()))
        return return_code

    def events_snapshot(self):
        with self._condition:
            return [
                line.removeprefix(EVENT_PREFIX)
                for line in self._lines
                if line.startswith(EVENT_PREFIX)
            ]

    def dump(self):
        with self._condition:
            return '\n'.join(self._lines)


class _DelayedStdout:

    def __init__(self, line, release):
        self._line = line
        self._release = release
        self._returned = False
        self.reader_started = threading.Event()

    def __iter__(self):
        return self

    def __next__(self):
        if self._returned:
            raise StopIteration
        self.reader_started.set()
        assert self._release.wait(timeout=1.0)
        self._returned = True
        return self._line


class _ExitedProcess:

    def __init__(self, stdout):
        self.stdout = stdout

    def poll(self):
        return 1


def test_process_output_drains_event_after_process_exit():
    release = threading.Event()
    stdout = _DelayedStdout(EVENT_PREFIX + 'late\n', release)
    output = ProcessOutput(_ExitedProcess(stdout))
    assert stdout.reader_started.wait(timeout=1.0)
    timer = threading.Timer(0.05, release.set)
    timer.start()
    try:
        output.wait_for_event('late', time.monotonic() + 1.0)
    finally:
        release.set()
        timer.join(timeout=1.0)


def test_glove_test_node_rejects_explicit_empty_selector_value():
    executable = Path(os.environ['WUJI_GLOVE_TEST_NODE'])
    environment = os.environ.copy()
    environment['WUJI_GLOVE_FAKE_SCENARIO'] = 'normal_shutdown'
    environment['WUJI_GLOVE_EXPECTED_SELECTOR_KIND'] = 'unique_discovery'
    environment['WUJI_GLOVE_EXPECTED_SELECTOR_VALUE'] = ''
    result = subprocess.run(
        [str(executable)],
        env=environment,
        capture_output=True,
        text=True,
        timeout=5.0,
        check=False,
    )
    assert result.returncode == 1
    assert 'WUJI_GLOVE_EXPECTED_SELECTOR_VALUE is invalid' in result.stderr
    assert EVENT_PREFIX not in result.stdout


def assert_cleanup_order(events):
    opened = [event for event in events if event in SUBSCRIPTIONS]
    assert opened == list(SUBSCRIPTIONS), events

    disconnect_index = events.index('disconnect')
    closed = [event for event in events[:disconnect_index] if event.startswith('close:')]
    assert len(closed) == len(SUBSCRIPTIONS), events
    assert set(closed) == {f'close:{stream}' for stream in SUBSCRIPTIONS.values()}, events
    assert 'close:' not in '\n'.join(events[disconnect_index + 1:]), events


def assert_optional_fault_isolated(events):
    opened = [event for event in events if event in SUBSCRIPTIONS]
    assert opened == list(SUBSCRIPTIONS), events
    closed = [event for event in events if event.startswith('close:')]
    assert closed == ['close:emf_poses'], events
    assert 'disconnect' not in events, events


@pytest.mark.parametrize(('scenario', 'expected_exit_code'), CASES)
def test_glove_test_node_subprocess(scenario, expected_exit_code):
    executable = Path(os.environ['WUJI_GLOVE_TEST_NODE'])
    assert executable.is_file()
    environment = os.environ.copy()
    environment['WUJI_GLOVE_FAKE_SCENARIO'] = scenario
    deadline = time.monotonic() + 5.0
    process = subprocess.Popen(
        [str(executable)],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    output = ProcessOutput(process)

    try:
        output.wait_for_event('ready', deadline)
        if scenario.startswith('optional_'):
            output.wait_for_event('close:emf_poses', deadline)
            output.wait_for_event('optional_fault_processed', deadline)
            assert process.poll() is None, output.dump()
            assert_optional_fault_isolated(output.events_snapshot())
            process.send_signal(signal.SIGINT)
        elif scenario == 'normal_shutdown':
            assert process.poll() is None, output.dump()
            process.send_signal(signal.SIGINT)

        return_code = output.finish(deadline)
        assert return_code == expected_exit_code, output.dump()
        assert_cleanup_order(output.events_snapshot())
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=0.5)


def test_integer_for_double_parameter_exits_with_readable_error():
    executable = Path(os.environ['WUJI_GLOVE_NODE'])
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
