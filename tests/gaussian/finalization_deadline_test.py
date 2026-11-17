#!/usr/bin/env python3
"""Pause the real owner across its repair deadline; requires a local GDB.

The binary's ordinary suite skips this externally controlled case. No production
clock or scheduler hooks are used. Run this separately from GPU/bag validation.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    args = parser.parse_args()
    cases = [
        ('CompletionCheckAfterDeadlineCannotSucceed', [
            'break livo_atlas::OnlineGaussianMapper::FinalizationRepairsComplete', 'run']),
        ('RebuildDispatchAfterDeadlineIsSkipped', [
            'break livo_atlas::OnlineGaussianMapper::FinalizationRepairsComplete', 'run',
            'delete 1', 'break livo_atlas::GaussianMapIndex::MakeTarget(unsigned long, unsigned long) const', 'ignore 2 1', 'continue']),
        ('CandidateValidationAfterDeadlineIsRejected', [
            'break livo_atlas::OnlineGaussianMapper::Finalize', 'run', 'delete 1',
            'break livo_atlas::OnlineGaussianMapper::CanAcceptSubmapCandidate', 'continue', 'delete 2',
            'break livo_atlas::OnlineGaussianMapper::CurrentTarget', 'continue'])]
    environment = dict(os.environ, LIVO_TEST_FINALIZATION_DEADLINE='1')
    failed = False
    for name, preparation in cases:
        command = ['gdb', '-q', '-batch', '--return-child-result']
        for action in ['set pagination off', 'set print thread-events off',
                       'set breakpoint pending on', *preparation,
                       'python import time; time.sleep(0.3)', 'delete breakpoints', 'continue']:
            command += ['-ex', action]
        command += ['--args', str(args.binary.resolve()),
                    '--gtest_filter=GaussianFinalizationTest.' + name]
        result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=30)
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        failed |= (result.returncode != 0 or '[  PASSED  ] 1 test.' not in result.stdout or
                   '[  SKIPPED ]' in result.stdout)
    return 1 if failed else 0



if __name__ == '__main__':
    sys.exit(main())
