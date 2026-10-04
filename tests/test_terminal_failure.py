"""Real PTY input and discarded inference responses on manual failure."""
from concurrent.futures import Future
import os
import termios
import unittest

from simulation.terminal_failure import TerminalFailureKey, wait_prediction


class TerminalFailureTests(unittest.TestCase):
    def setUp(self):
        self.master,self.slave=os.openpty()
        self.stream=os.fdopen(self.slave,'r')
        self.original=termios.tcgetattr(self.stream.fileno())
        self.keys=TerminalFailureKey(self.stream)

    def tearDown(self):
        self.keys.close()
        self.stream.close()
        os.close(self.master)

    def test_f_without_enter_and_reset_keys_discarded(self):
        os.write(self.master,b'f')
        self.assertTrue(self.keys.poll(active=True))
        self.assertFalse(self.keys.poll(active=True))
        os.write(self.master,b'f')
        self.assertFalse(self.keys.poll(active=False))
        self.assertFalse(self.keys.poll(active=True))

    def test_terminal_settings_restored_and_ctrl_c_preserved(self):
        current=termios.tcgetattr(self.stream.fileno())
        self.assertTrue(current[3] & termios.ISIG)
        self.assertFalse(current[3] & termios.ICANON)
        self.keys.close()
        self.assertEqual(termios.tcgetattr(self.stream.fileno()),self.original)

    def test_failure_during_inference_does_not_use_old_response(self):
        old=Future()
        old.set_running_or_notify_cancel()
        failed=[]
        os.write(self.master,b'f')
        self.assertIsNone(wait_prediction(old,self.keys,lambda:False,lambda:failed.append('manual_failure')))
        self.assertEqual(failed,['manual_failure'])
        self.assertFalse(old.done())  # No wait for the model to finish.
        old.set_result({'actions':'old trial'})
        new=Future()
        new.set_result({'actions':'new trial'})
        self.assertEqual(wait_prediction(new,self.keys,lambda:False,lambda:None),{'actions':'new trial'})


if __name__=='__main__':
    unittest.main()
