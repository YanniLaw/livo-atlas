import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest


class SetupDocumentationTest(unittest.TestCase):
    def test_source_install_ignores_existing_pip_config(self):
        repo = Path(__file__).resolve().parents[2]
        text = (repo / "docs/p6-03b-gpu-probe.md").read_text(encoding="utf-8")
        blocks = re.findall(r"```bash\n(.*?)```", text, flags=re.DOTALL)
        commands = []
        for block in blocks:
            lines = iter(block.splitlines())
            for line in lines:
                if not line.startswith("env "):
                    continue
                command = line
                while line.endswith("\\"):
                    line = next(lines)
                    command += "\n" + line
                if "-m pip install --no-build-isolation" in command:
                    commands.append(command)
        self.assertEqual(len(commands), 1)
        with tempfile.TemporaryDirectory(prefix="livo-pip-doc-") as directory:
            root = Path(directory)
            fake_python = root / "python"
            fake_python.write_text(
                '#!/bin/sh\nprintf "%s\\n" "$PIP_CONFIG_FILE" "$@" > "$TRACE"\n'
            )
            fake_python.chmod(0o755)
            environment = dict(os.environ, LIVO_ENV=str(root), LIVO_PHASE=str(root),
                               LIVO_PYTHON=str(fake_python), TRACE=str(root / "trace"),
                               PIP_CONFIG_FILE=str(root / "existing-pip.conf"))
            subprocess.run(["bash", "-c", commands[0]], env=environment, check=True)
            trace = (root / "trace").read_text().splitlines()
            self.assertEqual(trace[0], "/dev/null")
            self.assertIn("--no-cache-dir", trace)


if __name__ == "__main__":
    unittest.main()
