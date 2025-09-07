import hashlib
import zlib

def get_batch_checksum(batch):
    batch_str = ''.join(f"{wert}:{zeit}" for wert, zeit in batch)
    return hashlib.sha256(batch_str.encode('utf-8')).hexdigest()

def get_crc(wert, zeit):
    data = f"{wert:.1f}:{zeit}".encode('utf-8')
    return zlib.crc32(data) & 0xffffffff
