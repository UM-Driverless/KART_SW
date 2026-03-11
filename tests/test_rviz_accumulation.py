"""Diagnostic test: rviz / GUI process accumulation prevention.

This script statically analyzes all launch files and shell scripts in kart_brain
to ensure that any file spawning GUI processes (rviz2, rqt_image_view, hud_viewer)
also includes cleanup logic to kill stale instances before spawning new ones.

The goal is to prevent the Orin freeze bug caused by accumulated rviz2 / GUI
processes across relaunches.

Run:
    python3 tests/test_rviz_accumulation.py
"""

import ast
import os
import re
import sys
import unittest

# -- Configuration --

KART_BRAIN_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_DIRS = [
    os.path.join(KART_BRAIN_ROOT, "src"),
]
SHELL_SCRIPTS = [
    os.path.join(KART_BRAIN_ROOT, "run_live.sh"),
    os.path.join(KART_BRAIN_ROOT, "run_live_3d.sh"),
]

# GUI executables that consume GPU/display resources and must not accumulate
GUI_EXECUTABLES = {"rviz2", "rqt_image_view"}

# Patterns that indicate cleanup logic is present
CLEANUP_PATTERNS_SHELL = [
    re.compile(r"killall.*(?:rviz2|rqt_image_view)", re.IGNORECASE),
    re.compile(r"pkill.*(?:rviz2|rqt_image_view)", re.IGNORECASE),
]

# In launch files, we look for ExecuteProcess actions that run killall/pkill
CLEANUP_PATTERNS_LAUNCH = [
    re.compile(r"killall.*(?:rviz2|rqt_image_view)", re.IGNORECASE),
    re.compile(r"pkill.*(?:rviz2|rqt_image_view)", re.IGNORECASE),
]


def find_launch_files(search_dirs):
    """Find all .launch.py files under the given directories."""
    results = []
    for d in search_dirs:
        for root, _dirs, files in os.walk(d):
            for f in files:
                if f.endswith(".launch.py"):
                    results.append(os.path.join(root, f))
    return results


def launch_file_spawns_gui(filepath):
    """Check if a launch file spawns any GUI executable (rviz2, rqt_image_view).

    Returns a list of GUI executable names found.
    """
    found = []
    try:
        with open(filepath, "r") as fh:
            content = fh.read()
    except OSError:
        return found

    # Simple regex: look for executable='rviz2' or executable="rviz2" etc.
    for exe in GUI_EXECUTABLES:
        # Match both string literal assignments and standalone references
        pattern = re.compile(
            r"""(?:executable\s*[=:]\s*['"]""" + re.escape(exe) + r"""['"])"""
            r"""|(?:ros2\s+run\s+\S+\s+""" + re.escape(exe) + r""")""",
            re.MULTILINE,
        )
        if pattern.search(content):
            found.append(exe)

    return found


def launch_file_has_cleanup(filepath):
    """Check if a launch file contains cleanup logic for GUI processes.

    We look for ExecuteProcess or similar actions that run killall/pkill on
    GUI executables.
    """
    try:
        with open(filepath, "r") as fh:
            content = fh.read()
    except OSError:
        return False

    for pattern in CLEANUP_PATTERNS_LAUNCH:
        if pattern.search(content):
            return True
    return False


def shell_script_spawns_gui(filepath):
    """Check if a shell script launches GUI processes."""
    found = []
    try:
        with open(filepath, "r") as fh:
            content = fh.read()
    except OSError:
        return found

    for exe in GUI_EXECUTABLES:
        # Match: ros2 run <pkg> <exe> or direct invocation
        pattern = re.compile(
            r"(?:ros2\s+run\s+\S+\s+" + re.escape(exe) + r")"
            r"|(?:^" + re.escape(exe) + r"\b)",
            re.MULTILINE,
        )
        if pattern.search(content):
            found.append(exe)

    return found


def shell_script_has_cleanup(filepath):
    """Check if a shell script has cleanup logic for GUI processes."""
    try:
        with open(filepath, "r") as fh:
            content = fh.read()
    except OSError:
        return False

    for pattern in CLEANUP_PATTERNS_SHELL:
        if pattern.search(content):
            return True
    return False


class TestRvizAccumulation(unittest.TestCase):
    """Ensure all files that spawn GUI processes also include cleanup."""

    def test_launch_files_have_cleanup(self):
        """Every .launch.py that spawns rviz2 or rqt_image_view must include
        a cleanup action (killall/pkill) to prevent process accumulation."""
        launch_files = find_launch_files(LAUNCH_DIRS)
        self.assertTrue(
            len(launch_files) > 0, "No launch files found -- check LAUNCH_DIRS"
        )

        violations = []
        for lf in launch_files:
            gui_procs = launch_file_spawns_gui(lf)
            if gui_procs and not launch_file_has_cleanup(lf):
                rel = os.path.relpath(lf, KART_BRAIN_ROOT)
                violations.append(
                    f"  {rel} spawns {gui_procs} but has no cleanup action"
                )

        self.assertEqual(
            len(violations),
            0,
            "Launch files spawn GUI processes without cleanup:\n"
            + "\n".join(violations),
        )

    def test_shell_scripts_have_cleanup(self):
        """Every shell script that backgrounds GUI processes must kill stale
        instances at startup."""
        violations = []
        for script in SHELL_SCRIPTS:
            if not os.path.exists(script):
                continue
            gui_procs = shell_script_spawns_gui(script)
            if gui_procs and not shell_script_has_cleanup(script):
                rel = os.path.relpath(script, KART_BRAIN_ROOT)
                violations.append(
                    f"  {rel} spawns {gui_procs} but has no cleanup logic"
                )

        self.assertEqual(
            len(violations),
            0,
            "Shell scripts spawn GUI processes without cleanup:\n"
            + "\n".join(violations),
        )

    def test_display_zed_cam_has_singleton_guard(self):
        """display_zed_cam.launch.py must check for an existing rviz2 process
        before spawning a new one (singleton guard)."""
        target = os.path.join(
            KART_BRAIN_ROOT,
            "src",
            "ThirdParty",
            "zed_display_rviz2",
            "launch",
            "display_zed_cam.launch.py",
        )
        if not os.path.exists(target):
            self.skipTest("display_zed_cam.launch.py not found")

        with open(target, "r") as fh:
            content = fh.read()

        # Must contain either a pgrep guard or cleanup + kill before spawning
        has_pgrep = bool(re.search(r"pgrep.*rviz2", content))
        has_cleanup = bool(re.search(r"killall.*rviz2|pkill.*rviz2", content))

        self.assertTrue(
            has_pgrep or has_cleanup,
            "display_zed_cam.launch.py spawns rviz2 without a singleton guard "
            "or cleanup action. Add pgrep check or killall before Node spawn.",
        )

    def test_no_backgrounded_gui_without_trap(self):
        """Shell scripts that background GUI processes should have a trap for
        cleanup on exit."""
        for script in SHELL_SCRIPTS:
            if not os.path.exists(script):
                continue
            gui_procs = shell_script_spawns_gui(script)
            if not gui_procs:
                continue

            with open(script, "r") as fh:
                content = fh.read()

            has_trap = bool(re.search(r"trap\s+", content))
            rel = os.path.relpath(script, KART_BRAIN_ROOT)
            self.assertTrue(
                has_trap,
                f"{rel} backgrounds GUI processes but has no trap for cleanup on exit.",
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
