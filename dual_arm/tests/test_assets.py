"""Offline regression tests: no downloads or robot transports."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('assets', Path(__file__).parents[1] / 'tools/assets.py')
assets = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assets)


class AssetTests(unittest.TestCase):
    def entry(self):
        return {'path': 'sample', 'bytes': 4, 'sha256': assets.digest(b'good'),
                'url': 'https://files.waveshare.com/wiki/RoArm-M3/sample'}

    def test_corrupt_existing_file_is_preserved_and_not_refetched(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'sample').write_bytes(b'bad!')
            with patch.object(assets.urllib.request, 'urlopen') as request:
                with self.assertRaisesRegex(ValueError, 'mismatch'):
                    assets.fetch({'assets': [self.entry()]}, root)
                request.assert_not_called()
            self.assertEqual((root / 'sample').read_bytes(), b'bad!')

    def test_bad_download_is_not_written(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(assets.urllib.request, 'urlopen') as request:
                request.return_value.__enter__.return_value.read.return_value = b'bad!'
                with self.assertRaisesRegex(ValueError, 'mismatch'):
                    assets.fetch({'assets': [self.entry()]}, root)
            self.assertFalse((root / 'sample').exists())

    def test_cache_path_cannot_escape(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'escapes'):
                assets.checked_path(Path(directory), '../elsewhere')

    def test_verified_existing_file_does_not_need_network(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'sample').write_bytes(b'good')
            with patch.object(assets.urllib.request, 'urlopen') as request:
                assets.fetch({'assets': [self.entry()]}, root)
                request.assert_not_called()


if __name__ == '__main__':
    unittest.main()
