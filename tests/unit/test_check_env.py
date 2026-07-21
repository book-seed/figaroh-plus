"""Tests for the environment verification script (scripts/check_casadi_env.py)."""

import subprocess
import sys
from pathlib import Path

import pytest

SCRIPT_PATH = Path(__file__).resolve().parent.parent.parent / "scripts" / "check_casadi_env.py"


class TestCheckEnvScript:
    """Test that check_casadi_env.py exists, is executable, and produces valid output."""

    def test_script_exists(self):
        """The check_casadi_env.py script must exist."""
        assert SCRIPT_PATH.is_file(), f"Script not found at {SCRIPT_PATH}"

    def test_script_executable(self):
        """The check_casadi_env.py script must be executable."""
        assert SCRIPT_PATH.stat().st_mode & 0o111, (
            f"Script {SCRIPT_PATH} is not executable "
            f"(chmod +x {SCRIPT_PATH})"
        )

    def test_script_runs_without_error(self):
        """Running the script must exit with code 0 (all good) or 1 (some missing)."""
        result = subprocess.run(
            [sys.executable, str(SCRIPT_PATH)],
            capture_output=True,
            text=True,
            timeout=30,
        )
        # The script should exit with 0 or 1 (never crash with a traceback)
        assert result.returncode in (0, 1), (
            f"Script crashed with return code {result.returncode}.\n"
            f"stderr:\n{result.stderr}"
        )

    def test_script_output_format(self):
        """The script output must contain expected section headers."""
        result = subprocess.run(
            [sys.executable, str(SCRIPT_PATH)],
            capture_output=True,
            text=True,
            timeout=30,
        )
        output = result.stdout
        stderr = result.stderr

        # Filter library warnings from stderr (pinocchio, etc.)
        stderr_filtered = "\n".join(
            line for line in stderr.strip().split("\n")
            if "RuntimeWarning" not in line and "DeprecationWarning" not in line
        )
        assert not stderr_filtered.strip(), f"Script produced unexpected stderr:\n{stderr}"

        # Must contain title
        assert "FIGAROH 环境验证脚本" in output, (
            "Missing title 'FIGAROH 环境验证脚本'"
        )

        # Must contain section headers
        assert "[1] 基础依赖" in output, "Missing section '[1] 基础依赖'"
        assert "[2] IPOPT 线性求解器" in output, (
            "Missing section '[2] IPOPT 线性求解器'"
        )

        # Must contain PASS/FAIL markers for each check
        assert "[PASS]" in output or "[FAIL]" in output, (
            "No check results ([PASS]/[FAIL]) found in output"
        )

        # Must contain summary line
        assert "结果:" in output and "通过" in output, (
            "Missing summary line '结果: X/Y 通过'"
        )

    def test_script_invoked_directly(self):
        """The script should be runnable via `python scripts/check_casadi_env.py`."""
        cwd = SCRIPT_PATH.parent.parent
        result = subprocess.run(
            [sys.executable, "scripts/check_casadi_env.py"],
            capture_output=True,
            text=True,
            timeout=30,
            cwd=cwd,
        )
        assert result.returncode in (0, 1)

    def test_check_casadi_section_has_pass_fail(self):
        """CasADi checks must show PASS or FAIL status."""
        result = subprocess.run(
            [sys.executable, str(SCRIPT_PATH)],
            capture_output=True,
            text=True,
            timeout=30,
        )
        output = result.stdout

        # Should have at least one check result line mentioning "casadi"
        # with [PASS] or [FAIL] (hint lines also contain "casadi" but lack the tag)
        casadi_check_lines = [
            line for line in output.splitlines()
            if "casadi" in line.lower() and ("[PASS]" in line or "[FAIL]" in line)
        ]
        assert casadi_check_lines, (
            "No casadi check result ([PASS]/[FAIL]) found in output.\n"
            f"Full output:\n{output}"
        )


if __name__ == "__main__":
    pytest.main([__file__])
