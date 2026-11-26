"""Process evidence must exercise repeated signals and preserve timeout facts."""
import copy
import unittest

from tests.ros_common.shutdown_process import validate_result


def result(timeout=False):
    stopped = dict(status=6, bytes=0, width=0)
    return dict(returncode=1 if timeout else 0, stopped=[stopped], samples=[stopped],
                repeated=True, late_signal_sent=True, peer_events=['hello', 'stop'],
                peer_error=None, elapsed_seconds=.5,
                finalization=dict(status='failed' if timeout else 'succeeded',
                                  reason='incomplete_shutdown' if timeout else '',
                                  worker_stopped=0 if timeout else 1))


class ShutdownResultTest(unittest.TestCase):
    def test_valid_normal_repeated_and_timeout_outcomes(self):
        validate_result(result(), repeat_signal=True)
        validate_result(result(True), expect_timeout=True, shutdown_hook=True)

    def test_requested_repeat_cannot_be_silently_omitted(self):
        report = result()
        report['repeated'] = False
        with self.assertRaises(AssertionError):
            validate_result(report, repeat_signal=True)

    def test_timeout_requires_the_reason_and_an_unstopped_worker(self):
        for field, value in [('reason', 'storage_error'), ('worker_stopped', 1)]:
            for missing in (False, True):
                report = result(True)
                if missing:
                    report['finalization'].pop(field)
                else:
                    report['finalization'][field] = value
                with self.subTest(field=field, missing=missing), self.assertRaises(AssertionError):
                    validate_result(report, expect_timeout=True)

    def test_existing_terminal_exit_and_deadline_checks_remain_strict(self):
        original = result(True)
        for field, value in [('returncode', 0), ('late_signal_sent', False),
                             ('elapsed_seconds', 8), ('stopped', [])]:
            report = copy.deepcopy(original)
            report[field] = value
            with self.subTest(field=field), self.assertRaises(AssertionError):
                validate_result(report, expect_timeout=True, shutdown_hook=True)


if __name__ == '__main__':
    unittest.main()
