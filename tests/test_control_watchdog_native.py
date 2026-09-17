import shutil
import subprocess
import tempfile
from pathlib import Path

import unittest


class ControlWatchdogTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'host g++ required; run under WSL/Linux')
    def test_deadline_latch_and_clock_wrap(self):
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / 'watchdog-test'
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Werror',
                            str(Path(__file__).with_name('test_control_watchdog.cpp')), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
