"""A failed checkpoint transfer must not be published as a usable cache."""
import base64
import hashlib
import google_crc32c
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dataset.openpi_download import public_checkpoint


def metadata(name, content, *, composite=False):
    result = {'name': 'checkpoints/test/params/' + name, 'size': str(len(content)), 'generation': '1'}
    if composite:
        result['crc32c'] = base64.b64encode(google_crc32c.Checksum(content).digest()).decode()
    else:
        result['md5Hash'] = base64.b64encode(hashlib.md5(content).digest()).decode()
    return result


class Response:
    def __init__(self, *, listing=None, content=b''):
        self.listing = listing
        self.content = content

    def raise_for_status(self):
        pass

    def json(self):
        return self.listing

    def iter_content(self, _):
        yield self.content

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass


class CheckpointDownloadTests(unittest.TestCase):
    def run_download(self, content, directory):
        objects = [metadata('_METADATA', b'meta'), metadata('d/weights', b'correct', composite=True)]

        def get(url, **kwargs):
            if 'generation' not in kwargs['params']:
                return Response(listing={'items': objects})
            return Response(content=b'meta' if '_METADATA' in url else content)

        with patch('dataset.openpi_download.requests.get', side_effect=get), \
             patch('dataset.openpi_download.time.sleep'):
            return public_checkpoint('gs://openpi-assets/checkpoints/test/params', directory)

    def test_invalid_transfer_preserves_existing_file_and_raises(self):
        with tempfile.TemporaryDirectory() as temporary:
            weights = Path(temporary) / 'openpi-assets/checkpoints/test/params/d/weights'
            weights.parent.mkdir(parents=True)
            weights.write_bytes(b'existing')
            with self.assertRaisesRegex(ValueError, 'size/checksum mismatch'):
                self.run_download(b'corrupt', temporary)  # Same size; checksum must catch it.
            self.assertEqual(weights.read_bytes(), b'existing')
            self.assertFalse(weights.with_name('weights.download').exists())
            self.assertFalse(weights.parents[1].with_suffix('.verified.json').exists())

    def test_valid_transfer_replaces_corrupt_cache(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = self.run_download(b'correct', temporary)
            self.assertEqual((directory / 'd/weights').read_bytes(), b'correct')
            self.assertTrue(directory.with_suffix('.verified.json').is_file())


if __name__ == '__main__':
    unittest.main()
