"""Real logging-mp startup ordering in a fresh interpreter; no robot imports."""
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BrowserStartup(unittest.TestCase):
    def check_startup(self, follow_head, fail=False):
        program = '''
from pathlib import Path
import tempfile
import logging_mp
from head_camera import browser_teleop as browser
events=[]
original=logging_mp.basicConfig
def install():
    logging_mp.getLogger('hook.arm')
    events.append('installed')
    return lambda:events.append('restored')
browser.install_browser_hooks=install
with tempfile.TemporaryDirectory() as directory:
    entry=Path(directory)/'official.py'
    entry.write_text("import logging_mp\\nlogging_mp.basicConfig(level=logging_mp.INFO)\\nlogging_mp.getLogger('official')\\n" + ("raise ValueError('entry failure')\\n" if FAIL else ""))
    try:
        browser.run_official(entry,FOLLOW)
    except ValueError:
        assert FAIL
    else:
        assert not FAIL
assert logging_mp.basicConfig==original
assert events==(['installed','restored'] if FOLLOW else []),events
'''
        program = 'FOLLOW='+repr(follow_head)+'\nFAIL='+repr(fail)+'\n'+program
        result = subprocess.run([sys.executable, '-c', program], cwd=ROOT,
                                capture_output=True, text=True, timeout=12)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_head_hooks_import_loggers_after_official_logging_configuration(self):
        self.check_startup(True)

    def test_plain_browser_keeps_official_logging_startup(self):
        self.check_startup(False)

    def test_entry_failure_restores_logging_function_and_head_hooks(self):
        self.check_startup(True, fail=True)
