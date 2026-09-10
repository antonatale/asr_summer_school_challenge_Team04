"""Exercise the actual mission transitions with bounded fake action responses."""
import unittest
from unittest.mock import Mock
from types import SimpleNamespace as NS
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
from asr_summer_school.mission_orchestrator import Mission


def mission():
    n=object.__new__(Mission)
    n.future=None;n.goal=None;n.running=True;n.returning=False;n.return_attempts=0
    n.target={'x':1.,'y':0.,'yaw':0.};n.blacklist=[];n.pending_kind=None
    n.event=Mock();n.halt=Mock();n.nav=Mock();n.planner=Mock()
    return n


class ActionTests(unittest.TestCase):
    def test_start_requires_safety_gate(self):
        n=mission();n.running=False;n.started=None;n.ready=lambda:True;n.enabled=False
        result=n.start(None,Trigger.Response());self.assertFalse(result.success)

    def test_planner_rejection_does_not_navigate(self):
        n=mission();n.pending_kind='plan_accept';n.pending_at=0
        n.future=Mock();n.future.done.return_value=True;n.future.result.return_value=NS(accepted=False)
        n.process_future(1);n.nav.send_goal_async.assert_not_called();self.assertIsNone(n.future)

    def test_empty_path_does_not_navigate(self):
        n=mission();n.pending_kind='plan_result';n.pending_at=0
        n.future=Mock();n.future.done.return_value=True
        n.future.result.return_value=NS(status=GoalStatus.STATUS_SUCCEEDED,result=NS(path=NS(poses=[])))
        n.process_future(1);n.nav.send_goal_async.assert_not_called()

    def test_late_exploration_accept_cancels_on_return(self):
        n=mission();n.returning=True;n.pending_kind='nav_accept';n.pending_at=0
        handle=Mock();handle.accepted=True
        n.future=Mock();n.future.done.return_value=True;n.future.result.return_value=handle
        n.process_future(1);handle.cancel_goal_async.assert_called_once();self.assertEqual(n.cancel_at,1)

    def test_timeout_halts(self):
        n=mission();n.pending_kind='nav_accept';n.pending_at=0;n.future=Mock()
        n.future.done.return_value=False;n.process_future(9);n.halt.assert_called_once()

    def test_return_failure_is_bounded(self):
        n=mission();n.returning=True
        for _ in range(3):n.reject('no_path')
        n.halt.assert_called_once_with('return_path_failed')

    def test_return_discards_exploration_plan(self):
        n=mission();n.returning=True;n.pending_kind='plan_result';n.pending_at=0
        n.future=Mock();n.future.done.return_value=True
        n.future.result.return_value=NS(status=GoalStatus.STATUS_SUCCEEDED,result=NS(path=NS(poses=[1])))
        n.process_future(1);n.nav.send_goal_async.assert_not_called();self.assertIsNone(n.future)

if __name__=='__main__':unittest.main()
