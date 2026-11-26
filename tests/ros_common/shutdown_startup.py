"""An unreachable ROS1 master must not prevent startup cancellation."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    args = parser.parse_args()
    output = Path(tempfile.mkdtemp(prefix='livo-startup-stop-'))
    with socket.socket() as unused_port, (output / 'node.log').open('w') as log:
        # Reserve an endpoint without listening so no other master can occupy it.
        unused_port.bind(('127.0.0.1', 0))
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:' +
                   str(unused_port.getsockname()[1]), ROS_IP='127.0.0.1')
        process = subprocess.Popen([str(args.executable)], env=env, stdout=log,
                                   stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 8
            while 'Failed to contact master' not in (output / 'node.log').read_text():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Did not reach blocked master registration: ' + str(output))
                time.sleep(.01)
            began = time.monotonic()
            process.send_signal(signal.SIGINT)
            try:
                code = process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                code = None
            report = dict(directory=str(output), returncode=code,
                          elapsed_seconds=time.monotonic() - began)
            (output / 'result.json').write_text(json.dumps(report) + '\n')
            assert code in (0, 1), 'Startup ignored SIGINT: ' + str(report)
            print(json.dumps(report))
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=3)


if __name__ == '__main__':
    main()
