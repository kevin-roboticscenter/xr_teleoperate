"""Dashboard reconnects without a live master and isolates input TCP streams."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from wuji_glove.dashboard_gateway import ssh_command


class InputSsh(unittest.TestCase):
    def test_installed_key_does_not_require_or_reuse_the_temporary_login(self):
        with tempfile.TemporaryDirectory() as directory:
            key=Path(directory)/'operator-key'
            key.touch()
            with patch.dict(os.environ, {'H2_SSH_KEY':str(key)}):
                command=ssh_command('192.0.2.1','/missing/master')
            self.assertEqual(command[command.index('-S')+1],'none')
            self.assertEqual(command[command.index('-i')+1],str(key))
            self.assertIn('ControlMaster=no',command)
            self.assertIn('IdentitiesOnly=yes',command)
            self.assertIn('BatchMode=yes',command)
            self.assertIn('-C',command)
            self.assertNotIn('/missing/master',command)
            self.assertEqual(command[-1],'unitree@192.0.2.1')

    def test_existing_master_remains_available_for_setups_without_a_key(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(os.environ, {'H2_SSH_KEY':str(Path(directory)/'missing-key')}):
                command=ssh_command('192.0.2.1','/existing/master')
            self.assertEqual(command[command.index('-S')+1],'/existing/master')
            self.assertNotIn('-i',command)
            self.assertIn('BatchMode=yes',command)
