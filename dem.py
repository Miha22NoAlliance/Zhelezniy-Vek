from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path


TYPE_SIZES = {
    1: 1,   # BYTE
    2: 1,   # ASCII
    3: 2,   # SHORT
    4: 4,   # LONG
    5: 8,   # RATIONAL
    6: 1,   # SBYTE
    7: 1,   # UNDEFINED
    8: 2,   # SSHORT
    9: 4,   # SLONG
    10: 8,  # SRATIONAL
    11: 4,  # FLOAT
    12: 8,  # DOUBLE
}


def _unpack_values(data: bytes, offset: int, typ: int, count: int, endian: str):
    size = TYPE_SIZES.get(typ)
    if size is None:
        raise ValueError(f"Неподдерживаемый TIFF type {typ}")

    total = size * count
    fmt_map = {
        1: f"{endian}{count}B",
        2: f"{endian}{count}s",
        3: f"{endian}{count}H",
        4: f"{endian}{count}I",
        6: f"{endian}{count}b",
        8: f"{endian}{count}h",
        9: f"{endian}{count}i",
        11: f"{endian}{count}f",
        12: f"{endian}{count}d",
    }
    fmt = fmt_map.get(typ)
    if fmt is None:
        raise ValueError(f"Неподдерживаемый TIFF type {typ}")

    raw = data[offset:offset + total]
    if len(raw) != total:
        raise ValueError("Оборванное TIFF-поле")
    return list(struct.unpack(fmt, raw))


def _ascii_value(data: bytes, offset: int, count: int):
    raw = data[offset:offset + count]
    if len(raw) != count:
        return ""
    return raw.rstrip(b"\\x00").decode("ascii", errors="replace").strip()


class GeoTiffDEM:
    """Минимальный офлайн-чтение GeoTIFF DEM/COG без GDAL/Pillow.

    Поддерживает:
    - Classic TIFF little/big endian
    - tiled и stripped TIFF
    - DEFLATE (zlib) и uncompressed blocks
    - Predictor 1 и floating-point Predictor 3
    - FLOAT32 / signed or unsigned integer samples
    - ModelPixelScale/ModelTiepoint и ModelTransformation
    """

    def __init__(self, path: Path):
        self.path = Path(path)
        self._fp = self.path.open("rb")
        self.endian = "<"

        self.width = 0
        self.height = 0
        self.bits_per_sample = 0
        self.sample_format = 1
        self.samples_per_pixel = 1
        self.compression = 1
        self.predictor = 1

        self.tiled = False
        self.tile_width = 0
        self.tile_height = 0
        self.rows_per_strip = 0
        self.block_offsets = []
        self.block_sizes = []

        self.scale = None
        self.tiepoint = None
        self.transform = None
        self.nodata = None

        self._tile_cache = {}
        self._parse()

    def close(self):
        try:
            self._fp.close()
        except Exception:
            pass

    def __del__(self):
        self.close()

    def _read_at(self, offset: int, size: int) -> bytes:
        self._fp.seek(offset)
        data = self._fp.read(size)
        if len(data) != size:
            raise ValueError("Оборванный GeoTIFF")
        return data

    def _u16(self, offset: int) -> int:
        return struct.unpack(self.endian + "H", self._read_at(offset, 2))[0]

    def _u32(self, offset: int) -> int:
        return struct.unpack(self.endian + "I", self._read_at(offset, 4))[0]

    def _parse(self):
        head = self._read_at(0, 8)
        if head[:2] == b"II":
            self.endian = "<"
        elif head[:2] == b"MM":
            self.endian = ">"
        else:
            raise ValueError("Это не TIFF: неверный byte order")

        if struct.unpack(self.endian + "H", head[2:4])[0] != 42:
            raise ValueError("Поддерживается только Classic TIFF (magic 42)")

        ifd_offset = struct.unpack(self.endian + "I", head[4:8])[0]
        count = self._u16(ifd_offset)
        tags = {}

        pos = ifd_offset + 2
        for _ in range(count):
            entry = self._read_at(pos, 12)
            tag, typ, n = struct.unpack(self.endian + "HHI", entry[:8])
            raw_value = entry[8:12]
            size = TYPE_SIZES.get(typ)
            if size is None:
                raise ValueError(f"Неподдерживаемый TIFF type {typ}")
            total = size * n
            if total <= 4:
                value_offset = pos + 8
            else:
                value_offset = struct.unpack(self.endian + "I", raw_value)[0]
            tags[tag] = (typ, n, value_offset)
            pos += 12

        def vals(tag, default=None):
            item = tags.get(tag)
            if item is None:
                return default
            typ, n, off = item
            return _unpack_values(self._read_at(off, TYPE_SIZES[typ] * n), 0, typ, n, self.endian)

        def ascii_val(tag, default=None):
            item = tags.get(tag)
            if item is None:
                return default
            typ, n, off = item
            if typ not in (2, 7, 1):
                return default
            return _ascii_value(self._read_at(off, n), 0, n)

        self.width = int(vals(256, [0])[0])
        self.height = int(vals(257, [0])[0])
        self.bits_per_sample = int(vals(258, [32])[0])
        self.compression = int(vals(259, [1])[0])
        self.samples_per_pixel = int(vals(277, [1])[0])
        self.sample_format = int(vals(339, [1])[0])
        self.predictor = int(vals(317, [1])[0])
        self.nodata = None

        nodata_text = ascii_val(42113)
        if nodata_text:
            try:
                self.nodata = float(nodata_text.replace(",", "."))
            except ValueError:
                pass

        scale = vals(33550)
        tiepoint = vals(33922)
        transform = vals(34264)

        if transform and len(transform) >= 16:
            self.transform = tuple(float(x) for x in transform[:16])
        elif scale and tiepoint and len(scale) >= 2 and len(tiepoint) >= 6:
            self.scale = (float(scale[0]), float(scale[1]))
            self.tiepoint = tuple(float(x) for x in tiepoint[:6])
        else:
            raise ValueError("GeoTIFF не содержит геопривязку ModelTransform или Scale/Tiepoint")

        tile_offsets = vals(324)
        tile_sizes = vals(325)
        if tile_offsets and tile_sizes:
            self.tiled = True
            self.tile_width = int(vals(322, [0])[0])
            self.tile_height = int(vals(323, [0])[0])
            self.block_offsets = [int(x) for x in tile_offsets]
            self.block_sizes = [int(x) for x in tile_sizes]
        else:
            strip_offsets = vals(273)
            strip_sizes = vals(279)
            self.rows_per_strip = int(vals(278, [self.height])[0])
            if not strip_offsets or not strip_sizes:
                raise ValueError("GeoTIFF не содержит StripOffsets/StripByteCounts")
            self.block_offsets = [int(x) for x in strip_offsets]
            self.block_sizes = [int(x) for x in strip_sizes]

        if self.width <= 0 or self.height <= 0:
            raise ValueError("Некорректный размер DEM")
        if self.samples_per_pixel != 1:
            raise ValueError("DEM должен содержать одну полосу (SamplesPerPixel=1)")
        if self.bits_per_sample not in (16, 32):
            raise ValueError(f"Неподдерживаемая глубина DEM: {self.bits_per_sample} bit")
        if self.compression not in (1, 8, 32946):
            raise ValueError(
                f"Неподдерживаемое сжатие GeoTIFF: {self.compression}. "
                "Для Copernicus COG ожидается DEFLATE."
            )
        if self.predictor not in (1, 3):
            raise ValueError(f"Неподдерживаемый TIFF Predictor: {self.predictor}")
        if self.predictor == 3 and self.sample_format != 3:
            raise ValueError("Predictor=3 требует IEEE floating-point DEM")
        if self.predictor == 3 and self.bits_per_sample not in (16, 32, 64):
            raise ValueError("Predictor=3 поддержан только для 16/32/64-bit float")

    @property
    def bounds(self):
        corners = [
            self.pixel_to_geo(0, 0),
            self.pixel_to_geo(self.width - 1, 0),
            self.pixel_to_geo(0, self.height - 1),
            self.pixel_to_geo(self.width - 1, self.height - 1),
        ]
        lons = [x for x, _ in corners]
        lats = [y for _, y in corners]
        return min(lats), min(lons), max(lats), max(lons)

    def pixel_to_geo(self, col: float, row: float):
        if self.transform is not None:
            m = self.transform
            x = m[0] * col + m[1] * row + m[2] * 0 + m[3]
            y = m[4] * col + m[5] * row + m[6] * 0 + m[7]
            return y, x

        sx, sy = self.scale
        t = self.tiepoint
        # ModelTiepoint for north-up DEM: raster row increases southward.
        x = t[3] + (col - t[0]) * sx
        y = t[4] - (row - t[1]) * sy
        return y, x

    def geo_to_pixel(self, lat: float, lon: float):
        if self.transform is not None:
            m = self.transform
            a, b, d, e = m[0], m[1], m[4], m[5]
            tx = lon - m[3]
            ty = lat - m[7]
            det = a * e - b * d
            if abs(det) < 1e-15:
                raise ValueError("В GeoTIFF не удалось обратить affine transform")
            col = (tx * e - b * ty) / det
            row = (a * ty - tx * d) / det
            return col, row

        sx, sy = self.scale
        t = self.tiepoint
        col = (lon - t[3]) / sx + t[0]
        row = (t[4] - lat) / sy + t[1]
        return col, row

    def _decode_float_predictor(self, raw: bytes, width: int, rows: int):
        bps = self.bits_per_sample // 8
        row_bytes = width * bps
        out = bytearray(raw)

        expected = row_bytes * rows
        if len(out) < expected:
            raise ValueError("Размер GeoTIFF tile/strip меньше ожидаемого")
        if len(out) > expected:
            out = out[:expected]

        for r in range(rows):
            start = r * row_bytes
            end = start + row_bytes
            row = bytearray(out[start:end])
            n = width

            # Predictor 3 first stores separate byte planes, each horizontally
            # differenced. Undo the cumulative sum in each plane.
            for plane in range(bps):
                acc = 0
                for i in range(plane, n * bps, bps):
                    acc = (acc + row[i]) & 0xFF
                    row[i] = acc

            # Re-interleave into normal sample byte order.
            rebuilt = bytearray(row_bytes)
            for i in range(n):
                for byte in range(bps):
                    src_plane = (bps - 1 - byte) if self.endian == "<" else byte
                    rebuilt[i * bps + byte] = row[src_plane * n + i]
            out[start:end] = rebuilt

        return bytes(out)

    def _decode_block(self, index: int, block_width: int, block_height: int):
        key = (index, block_width, block_height)
        cached = self._tile_cache.get(key)
        if cached is not None:
            return cached

        offset = self.block_offsets[index]
        size = self.block_sizes[index]
        blob = self._read_at(offset, size)

        if self.compression in (8, 32946):
            raw = zlib.decompress(blob)
        else:
            raw = blob

        expected = block_width * block_height * (self.bits_per_sample // 8)
        if self.predictor == 3:
            raw = self._decode_float_predictor(raw, block_width, block_height)
        elif len(raw) < expected:
            raise ValueError("Повреждённый или неполный GeoTIFF block")

        if len(raw) < expected:
            raise ValueError("Недостаточно данных GeoTIFF block")

        if self.bits_per_sample == 32 and self.sample_format == 3:
            fmt = self.endian + "f"
        elif self.bits_per_sample == 16 and self.sample_format == 2:
            fmt = self.endian + "h"
        elif self.bits_per_sample == 16 and self.sample_format == 1:
            fmt = self.endian + "H"
        elif self.bits_per_sample == 32 and self.sample_format == 1:
            fmt = self.endian + "I"
        elif self.bits_per_sample == 32 and self.sample_format == 2:
            fmt = self.endian + "i"
        else:
            raise ValueError(
                f"Неподдерживаемый DEM: {self.bits_per_sample} bit, sample format {self.sample_format}"
            )

        count = block_width * block_height
        values = struct.unpack(self.endian + fmt[-1] * count, raw[:expected])
        self._tile_cache[key] = values
        if len(self._tile_cache) > 24:
            self._tile_cache.pop(next(iter(self._tile_cache)))
        return values

    def _sample_pixel(self, col: int, row: int):
        if col < 0 or row < 0 or col >= self.width or row >= self.height:
            return None

        if self.tiled:
            tx = col // self.tile_width
            ty = row // self.tile_height
            tiles_x = (self.width + self.tile_width - 1) // self.tile_width
            index = ty * tiles_x + tx
            if index >= len(self.block_offsets):
                return None
            bw = self.tile_width
            bh = self.tile_height
            values = self._decode_block(index, bw, bh)
            x = col - tx * self.tile_width
            y = row - ty * self.tile_height
            value = values[y * bw + x]
        else:
            strip = row // self.rows_per_strip
            if strip >= len(self.block_offsets):
                return None
            block_rows = min(self.rows_per_strip, self.height - strip * self.rows_per_strip)
            values = self._decode_block(strip, self.width, block_rows)
            x = col
            y = row - strip * self.rows_per_strip
            if y >= block_rows:
                return None
            value = values[y * self.width + x]

        if not math.isfinite(value):
            return None
        if self.nodata is not None and abs(value - self.nodata) < 1e-6:
            return None
        return float(value)

    def sample(self, lat: float, lon: float):
        col_f, row_f = self.geo_to_pixel(lat, lon)
        col = int(round(col_f))
        row = int(round(row_f))
        return self._sample_pixel(col, row)


def find_dem(data_dir: Path):
    preferred = data_dir / "Copernicus_DSM_COG_10_N52_00_E039_00_DEM.tif"
    if preferred.exists():
        return preferred

    candidates = sorted(
        p for p in data_dir.iterdir()
        if p.is_file() and p.suffix.lower() in {".tif", ".tiff"}
    )
    return candidates[0] if candidates else None


def open_dem(data_dir: Path):
    path = find_dem(data_dir)
    if path is None:
        return None, "DEM не найден в data/ (нужен .tif/.tiff)"
    dem = GeoTiffDEM(path)
    return dem, path.name
