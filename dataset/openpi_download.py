"""Verified anonymous download of public OpenPI checkpoint objects."""
import base64
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import logging
from pathlib import Path
import time
from urllib.parse import quote, urlparse

import filelock
import google_crc32c
import requests


def _valid(path, metadata):
    if not path.is_file() or path.stat().st_size != int(metadata['size']):
        return False
    # Composite GCS objects (the large weight shards) have CRC32C, no MD5.
    digest = google_crc32c.Checksum() if 'crc32c' in metadata else hashlib.md5()
    with path.open('rb') as stream:
        while chunk := stream.read(8 * 1024 * 1024):
            digest.update(chunk)
    expected = metadata['crc32c'] if 'crc32c' in metadata else metadata['md5Hash']
    return base64.b64encode(digest.digest()).decode() == expected


def public_checkpoint(url, cache_root):
    """Publish each file atomically only after size and GCS checksum verification.

    The pinned upstream fallback can publish an incomplete download without
    surfacing its worker's exception. Never rely on directory existence alone.
    All requests are anonymous, and existing valid files are reused.
    """
    parsed = urlparse(url)
    if parsed.scheme != 'gs' or parsed.netloc != 'openpi-assets':
        raise ValueError('Only public gs://openpi-assets checkpoints are supported.')
    prefix = parsed.path.strip('/') + '/'
    directory = Path(cache_root).resolve() / parsed.netloc / parsed.path.strip('/')
    directory.parent.mkdir(parents=True, exist_ok=True)
    with filelock.FileLock(str(directory.with_suffix('.lock'))):
        objects = []
        params = {'prefix': prefix, 'maxResults': 1000}
        while True:
            response = requests.get(
                f'https://storage.googleapis.com/storage/v1/b/{parsed.netloc}/o',
                params=params, timeout=(10, 60))
            response.raise_for_status()
            listing = response.json()
            objects.extend(listing.get('items', []))
            if 'nextPageToken' not in listing:
                break
            params['pageToken'] = listing['nextPageToken']
        if not objects or not any(x['name'] == prefix + '_METADATA' for x in objects):
            raise ValueError(f'No checkpoint metadata at {url}')

        def fetch(metadata):
            relative = metadata['name'][len(prefix):]
            path = directory / relative
            if '..' in Path(relative).parts or not metadata['name'].startswith(prefix):
                raise ValueError('Invalid checkpoint object path')
            if _valid(path, metadata):
                return
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_name(path.name + '.download')
            if _valid(temporary, metadata):
                temporary.replace(path)
                logging.info('Recovered verified checkpoint file %s', relative)
                return
            media_url = (f'https://storage.googleapis.com/download/storage/v1/b/{parsed.netloc}/o/'
                         f'{quote(metadata["name"], safe="")}')
            for attempt in range(3):
                try:
                    logging.info('Downloading checkpoint file %s (%s bytes)', relative, metadata['size'])
                    with requests.get(media_url, params={'alt': 'media', 'generation': metadata['generation']},
                                      stream=True, timeout=(10, 120)) as response:
                        response.raise_for_status()
                        with temporary.open('wb') as stream:
                            for chunk in response.iter_content(8 * 1024 * 1024):
                                stream.write(chunk)
                    if not _valid(temporary, metadata):
                        raise ValueError(f'Checkpoint size/checksum mismatch: {relative}')
                    temporary.replace(path)
                    logging.info('Verified checkpoint file %s', relative)
                    return
                except (requests.RequestException, ValueError):
                    temporary.unlink(missing_ok=True)
                    if attempt == 2:
                        raise
                    time.sleep(1)

        with ThreadPoolExecutor(max_workers=2) as pool:
            # Consume the iterator so every worker error reaches the caller.
            list(pool.map(fetch, objects))
        summary = {'url': url, 'files': len(objects), 'bytes': sum(int(x['size']) for x in objects),
                   'verification': 'GCS object size and CRC32C (MD5 fallback)', 'passed': True}
        directory.with_suffix('.verified.json').write_text(json.dumps(summary, indent=2) + '\n')
        logging.info('Public checkpoint verified: %s', directory)
    return directory
