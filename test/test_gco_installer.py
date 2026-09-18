"""Offline installer tests: real local Git repos, temporary files, fake services."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "install_helpers/gco-routines.sh"
URL = "https://github.com/OpenAMSOrg/gco-routines.git"

# Models the dependency installer's interface. Its own implementation/tests live
# in gco-routines; this repository tests orchestration and ownership safeguards.
INSTALLER = '''import argparse
from pathlib import Path
p = argparse.ArgumentParser()
p.add_argument('--klipper', required=True)
a = p.parse_args()
source = Path(__file__).resolve().parents[1] / 'klippy_extra/gco_routines'
dest = Path(a.klipper) / 'klippy/extras/gco_routines'
if not dest.is_symlink():
    dest.symlink_to(source, target_is_directory=True)
assert dest.resolve() == source
'''


class GcoInstallerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="openams-gco-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.remote = self.root / "remote"
        self.checkout = self.root / "checkout with spaces"
        self.klipper = self.root / "klipper"
        self.config = self.root / "config"
        (self.klipper / "klippy/extras").mkdir(parents=True)
        (self.klipper / "scripts").mkdir()
        self.config.mkdir()
        (self.config / "moonraker.conf").write_text("[server]\n")
        (self.config / "printer.cfg").write_text("[printer]\nkinematics: none\n")
        self.original_macros = "# Existing printer tuning must survive\n[gcode_macro USER]\ngcode:\n    G4 P0\n"
        (self.config / "oams_macros.cfg").write_text(self.original_macros)
        (self.config / "oams.cfg").write_text("# existing OAMS config\n")
        git_config = self.root / "gitconfig"
        git_config.write_text('[user]\nname = Installer Test\nemail = installer@example.invalid\n'
                              '[url "file://%s"]\ninsteadOf = %s\n' % (self.remote, URL))
        self.env = dict(os.environ, GIT_CONFIG_GLOBAL=str(git_config), GIT_CONFIG_NOSYSTEM="1",
                        GIT_TERMINAL_PROMPT="0", GIT_ALLOW_PROTOCOL="file",
                        GCO_ROUTINES_PATH=str(self.checkout), KLIPPER_PATH=str(self.klipper))
        self.run_cmd("git", "init", "-b", "main", str(self.remote))
        self.write_remote("tools/install_gco_routines.py", INSTALLER)
        self.write_remote("klippy_extra/gco_routines/__init__.py", "# fixture\n")
        self.commit_remote("Initial fixture")

    def run_cmd(self, *args, check=True, **kwargs):
        return subprocess.run(args, env=self.env, text=True, capture_output=True,
                              check=check, timeout=20, **kwargs)

    def write_remote(self, name, text):
        path = self.remote / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)

    def commit_remote(self, message):
        self.run_cmd("git", "-C", str(self.remote), "add", ".")
        self.run_cmd("git", "-C", str(self.remote), "commit", "-m", message)

    def helper(self, body="prepare_gco_routines; install_gco_routines", check=True):
        return self.run_cmd("bash", "-c", 'set -e; source "$1"; ' + body, "test", str(HELPER), check=check)

    def clone(self):
        self.run_cmd("git", "clone", "--branch", "main", URL, str(self.checkout))

    def test_first_install_and_rerun_are_idempotent(self):
        self.helper()
        target = self.klipper / "klippy/extras/gco_routines"
        self.assertTrue(target.is_symlink())
        self.assertEqual(target.resolve(), self.checkout / "klippy_extra/gco_routines")
        self.helper()
        self.assertEqual((self.config / "oams_macros.cfg").read_text(), self.original_macros)

    def test_prepare_fetches_but_does_not_advance_installed_source(self):
        self.helper()
        self.write_remote("klippy_extra/gco_routines/new.py", "# new module\n")
        self.commit_remote("New version")
        self.helper("prepare_gco_routines")
        self.assertFalse((self.checkout / "klippy_extra/gco_routines/new.py").exists())
        self.helper()
        self.assertTrue((self.checkout / "klippy_extra/gco_routines/new.py").exists())

    def test_refuses_non_git_directory_without_modifying_it(self):
        self.checkout.mkdir()
        sentinel = self.checkout / "preserve"
        sentinel.write_text("keep")
        result = self.helper(check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("not a Git checkout", result.stderr)
        self.assertEqual(sentinel.read_text(), "keep")

    def test_refuses_foreign_extra_symlink_even_when_dangling(self):
        target = self.klipper / "klippy/extras/gco_routines"
        target.symlink_to(self.root / "old-manual-install")
        result = self.helper(check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("not this checkout", result.stderr)
        self.assertEqual(target.readlink(), self.root / "old-manual-install")
        self.assertFalse(self.checkout.exists())

    def test_refuses_real_extra_directory(self):
        target = self.klipper / "klippy/extras/gco_routines"
        target.mkdir()
        self.assertNotEqual(self.helper(check=False).returncode, 0)
        self.assertTrue(target.is_dir())
        self.assertFalse(target.is_symlink())

    def test_accepts_its_own_dangling_symlink(self):
        target = self.klipper / "klippy/extras/gco_routines"
        target.symlink_to(self.checkout / "klippy_extra/gco_routines")
        self.helper()
        self.assertTrue(target.is_dir())

    def test_dirty_branch_and_origin_checks_preserve_sources(self):
        for problem in ("dirty", "branch", "origin", "detached"):
            with self.subTest(problem=problem):
                checkout = self.root / problem
                self.env["GCO_ROUTINES_PATH"] = str(checkout)
                self.run_cmd("git", "clone", "--branch", "main", URL, str(checkout))
                if problem == "dirty":
                    (checkout / "untracked.txt").write_text("keep")
                elif problem == "branch":
                    self.run_cmd("git", "-C", str(checkout), "switch", "-c", "work")
                elif problem == "origin":
                    self.run_cmd("git", "-C", str(checkout), "remote", "set-url", "origin", "https://example.invalid/other.git")
                else:
                    self.run_cmd("git", "-C", str(checkout), "checkout", "--detach")
                before = self.run_cmd("git", "-C", str(checkout), "rev-parse", "HEAD").stdout
                self.assertNotEqual(self.helper(check=False).returncode, 0)
                self.assertEqual(before, self.run_cmd("git", "-C", str(checkout), "rev-parse", "HEAD").stdout)

    def test_local_unpublished_commits_are_not_silently_installed(self):
        self.clone()
        (self.checkout / "local.txt").write_text("keep")
        self.run_cmd("git", "-C", str(self.checkout), "add", "local.txt")
        self.run_cmd("git", "-C", str(self.checkout), "commit", "-m", "Local work")
        result = self.helper(check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("cannot fast-forward", result.stderr)
        self.assertEqual((self.checkout / "local.txt").read_text(), "keep")

    def fake_services(self):
        binary = self.root / "bin"
        binary.mkdir()
        sudo = binary / "sudo"
        sudo.write_text('''#!/bin/bash
printf '%s\\n' "$*" >> "$OPENAMS_TEST_SERVICE_LOG"
if [[ "$*" == *list-units* ]]; then
    echo "klipper-test.service loaded active running"
fi
''')
        sudo.chmod(0o755)
        self.service_log = self.root / "services.log"
        self.env.update(PATH=str(binary) + os.pathsep + self.env["PATH"],
                        OPENAMS_TEST_SERVICE_LOG=str(self.service_log))

    def install(self, *extra):
        if os.geteuid() == 0:
            self.skipTest("The actual OpenAMS installer intentionally rejects root")
        return self.run_cmd("bash", str(ROOT / "install-openams.sh"),
                            "-k", str(self.klipper), "-c", str(self.config),
                            "-s", "klipper-test", "-r", str(self.checkout), *extra,
                            check=False, input="n\n")

    def test_full_install_preserves_macros_and_does_not_enable_extension(self):
        self.fake_services()
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((self.klipper / "klippy/extras/gco_routines").is_symlink())
        self.assertNotIn("gco_routines", (self.config / "printer.cfg").read_text())
        self.assertEqual((self.config / "oams_macros.cfg").read_text(), self.original_macros)
        self.assertEqual(self.service_log.read_text().count("systemctl stop klipper-test"), 1)
        self.assertEqual(self.service_log.read_text().count("systemctl start klipper-test"), 1)

    def test_prepare_failure_does_not_stop_service(self):
        self.fake_services()
        self.checkout.mkdir()
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("systemctl stop", self.service_log.read_text())

    def test_network_failure_does_not_stop_service(self):
        self.fake_services()
        self.remote.rename(self.root / "offline")
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("systemctl stop", self.service_log.read_text())

    def test_incomplete_dependency_is_rejected_before_stopping_service(self):
        self.fake_services()
        (self.remote / "tools/install_gco_routines.py").unlink()
        self.commit_remote("Simulated incomplete release")
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing tools/install_gco_routines.py", result.stderr)
        self.assertNotIn("systemctl stop", self.service_log.read_text())

    def test_dependency_install_failure_restarts_service_and_propagates_error(self):
        self.fake_services()
        self.write_remote("tools/install_gco_routines.py", "raise SystemExit(17)\n")
        self.commit_remote("Simulated install failure")
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("systemctl start klipper-test", self.service_log.read_text())
        self.assertEqual((self.config / "oams_macros.cfg").read_text(), self.original_macros)

    def test_openams_uninstall_retains_independent_dependency(self):
        self.fake_services()
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        result = self.install("-u")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((self.klipper / "klippy/extras/gco_routines").is_symlink())
        self.assertTrue(self.checkout.exists())
        self.assertEqual((self.config / "oams_macros.cfg").read_text(), self.original_macros)


if __name__ == "__main__":
    unittest.main()
