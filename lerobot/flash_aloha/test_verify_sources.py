import hashlib
from pathlib import Path
import tempfile
import unittest

from verify_sources import verify_sources


class SourceVerificationTests(unittest.TestCase):
    def test_changed_and_missing_reference_files_fail_verification(self):
        content = b"pinned reference\n"
        manifest = {"repositories": {"reference": {"files": [
            {"path": "reference.py", "sha256": hashlib.sha256(content).hexdigest()}
        ]}}}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            file = root / "reference.py"
            self.assertEqual(verify_sources(manifest, {"reference": root})[0]["status"], "missing")
            file.write_bytes(content)
            self.assertEqual(verify_sources(manifest, {"reference": root})[0]["status"], "matched")
            file.write_bytes(content + b"modified\n")
            self.assertEqual(verify_sources(manifest, {"reference": root})[0]["status"], "mismatch")

    def test_unsupplied_checkouts_are_not_claimed_as_verified(self):
        manifest = {"repositories": {"not_present": {"files": []}}}
        self.assertEqual(verify_sources(manifest, {}), [])


if __name__ == "__main__":
    unittest.main()
