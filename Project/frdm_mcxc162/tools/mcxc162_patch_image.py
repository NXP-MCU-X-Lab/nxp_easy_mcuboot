#!/usr/bin/env python3
"""Patch an MCXC162 A-slot Intel HEX image with a backup-boot header."""

from __future__ import annotations

import argparse
import re
import struct
import sys
import tempfile
from pathlib import Path


MAGIC = 0x4D435831
HEADER_VERSION = 1
HEADER_FORMAT = "<IHHIIII10I"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
CRC_FIELD_OFFSET = 16
PHRASE_SIZE = 16


class HexError(ValueError):
    pass


def parse_int(value: str) -> int:
    return int(value, 0)


def parse_c_int_literal(value: str) -> int:
    text = value.strip()
    while text.startswith("(") and text.endswith(")"):
        text = text[1:-1].strip()
    text = re.sub(r"[uUlL]+$", "", text)
    return int(text, 0)


def load_version_from_header(path: Path, macro: str) -> int:
    pattern = re.compile(r"^\s*#\s*define\s+" + re.escape(macro) + r"\s+(.+?)\s*$")

    for line_no, line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = line.split("//", 1)[0].split("/*", 1)[0].strip()
        match = pattern.match(line)
        if match:
            try:
                return parse_c_int_literal(match.group(1))
            except ValueError as exc:
                raise HexError(f"{path}:{line_no}: invalid {macro} value") from exc

    raise HexError(f"{path}: missing macro {macro}")


def resolve_version(args: argparse.Namespace) -> int:
    version_arg = getattr(args, "version", None)
    version_header = getattr(args, "version_header", None)
    version_macro = getattr(args, "version_macro", "APP_IMAGE_VERSION")

    if version_arg is not None and version_header:
        raise HexError("--version and --version-header are mutually exclusive")

    if version_header:
        version = load_version_from_header(Path(version_header), version_macro)
    else:
        version = 0 if version_arg is None else version_arg

    if version < 0 or version > 0xFFFFFFFF:
        raise HexError("version must fit in uint32")
    return version


def expand_output_path(output: str, version: int) -> Path:
    return Path(output.replace("{version}", f"{version:08X}"))


def crc32_ieee(data: bytes, crc: int = 0xFFFFFFFF) -> int:
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
            crc &= 0xFFFFFFFF
    return crc


def crc32_finish(crc: int) -> int:
    return crc ^ 0xFFFFFFFF


def parse_hex_record(line: str, line_no: int) -> tuple[int, int, bytes]:
    text = line.strip()
    if not text:
        raise HexError(f"line {line_no}: empty line")
    if not text.startswith(":"):
        raise HexError(f"line {line_no}: missing ':'")
    body = text[1:]
    if len(body) < 10 or (len(body) % 2) != 0:
        raise HexError(f"line {line_no}: invalid record length")
    try:
        raw = bytes.fromhex(body)
    except ValueError as exc:
        raise HexError(f"line {line_no}: invalid hex digit") from exc

    byte_count = raw[0]
    expected_len = byte_count + 5
    if len(raw) != expected_len:
        raise HexError(f"line {line_no}: byte count mismatch")
    if (sum(raw) & 0xFF) != 0:
        raise HexError(f"line {line_no}: checksum mismatch")

    address = (raw[1] << 8) | raw[2]
    record_type = raw[3]
    data = raw[4 : 4 + byte_count]
    return record_type, address, data


def load_hex(path: Path) -> dict[int, int]:
    data: dict[int, int] = {}
    upper = 0
    saw_eof = False

    with path.open("r", encoding="ascii") as file:
        for line_no, line in enumerate(file, 1):
            record_type, address, payload = parse_hex_record(line, line_no)

            if saw_eof:
                raise HexError(f"line {line_no}: data after EOF")

            if record_type == 0x00:
                base = upper + address
                for offset, byte in enumerate(payload):
                    absolute = base + offset
                    previous = data.get(absolute)
                    if previous is not None and previous != byte:
                        raise HexError(f"line {line_no}: conflicting data at 0x{absolute:08X}")
                    data[absolute] = byte
            elif record_type == 0x01:
                if payload:
                    raise HexError(f"line {line_no}: EOF record has payload")
                saw_eof = True
            elif record_type == 0x02:
                if len(payload) != 2:
                    raise HexError(f"line {line_no}: invalid extended segment address")
                upper = (((payload[0] << 8) | payload[1]) << 4)
            elif record_type == 0x04:
                if len(payload) != 2:
                    raise HexError(f"line {line_no}: invalid extended linear address")
                upper = (((payload[0] << 8) | payload[1]) << 16)
            elif record_type in (0x03, 0x05):
                pass
            else:
                raise HexError(f"line {line_no}: unsupported record type 0x{record_type:02X}")

    if not saw_eof:
        raise HexError("missing EOF record")
    return data


def make_hex_record(address: int, record_type: int, data: bytes) -> str:
    if len(data) > 0xFF:
        raise ValueError("record data too long")
    fields = bytes(
        [len(data), (address >> 8) & 0xFF, address & 0xFF, record_type]
    ) + data
    checksum = (-sum(fields)) & 0xFF
    return ":" + fields.hex().upper() + f"{checksum:02X}"


def check_range(data: dict[int, int], slot_base: int, slot_size: int) -> None:
    slot_end = slot_base + slot_size
    for address in data:
        if address < slot_base or address >= slot_end:
            raise HexError(
                f"address 0x{address:08X} is outside slot "
                f"0x{slot_base:08X}-0x{slot_end - 1:08X}"
            )


def align_up(value: int, alignment: int) -> int:
    return (value + alignment - 1) & ~(alignment - 1)


def header_is_patchable(image: bytearray, header_offset: int) -> bool:
    header = bytes(image[header_offset : header_offset + HEADER_SIZE])
    if all(byte in (0x00, 0xFF) for byte in header):
        return True
    current_magic = struct.unpack_from("<I", header, 0)[0]
    return current_magic == MAGIC


def build_image(
    data: dict[int, int],
    slot_base: int,
    slot_size: int,
    header_offset: int,
    version: int,
) -> tuple[bytearray, int]:
    if not data:
        raise HexError("input HEX has no data records")
    if header_offset + HEADER_SIZE > slot_size:
        raise HexError("header does not fit in slot")
    if version < 0 or version > 0xFFFFFFFF:
        raise HexError("version must fit in uint32")

    check_range(data, slot_base, slot_size)
    highest = max(data)
    image_size = align_up(
        max((highest - slot_base) + 1, header_offset + HEADER_SIZE),
        PHRASE_SIZE,
    )
    if image_size > slot_size:
        raise HexError(
            f"image size 0x{image_size:X} exceeds slot size 0x{slot_size:X}"
        )

    image = bytearray([0xFF] * image_size)
    for address, byte in data.items():
        index = address - slot_base
        if index < image_size:
            image[index] = byte

    if not header_is_patchable(image, header_offset):
        raise HexError(
            f"header region 0x{slot_base + header_offset:08X}-"
            f"0x{slot_base + header_offset + HEADER_SIZE - 1:08X} is not blank"
        )

    zero_crc_header = struct.pack(
        HEADER_FORMAT,
        MAGIC,
        HEADER_VERSION,
        HEADER_SIZE,
        version,
        image_size,
        0,
        0,
        *([0] * 10),
    )
    image[header_offset : header_offset + HEADER_SIZE] = zero_crc_header
    crc = crc32_finish(crc32_ieee(bytes(image)))

    final_header = struct.pack(
        HEADER_FORMAT,
        MAGIC,
        HEADER_VERSION,
        HEADER_SIZE,
        version,
        image_size,
        crc,
        0,
        *([0] * 10),
    )
    image[header_offset : header_offset + HEADER_SIZE] = final_header
    return image, crc


def write_hex(path: Path, slot_base: int, image: bytes) -> None:
    lines: list[str] = []
    current_upper: int | None = None

    for offset in range(0, len(image), 16):
        address = slot_base + offset
        upper = address >> 16
        if upper != current_upper:
            lines.append(make_hex_record(0, 0x04, upper.to_bytes(2, "big")))
            current_upper = upper
        lines.append(make_hex_record(address & 0xFFFF, 0x00, image[offset : offset + 16]))

    lines.append(make_hex_record(0, 0x01, b""))
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def patch_file(args: argparse.Namespace) -> tuple[int, int]:
    input_path = Path(args.input)
    args.version = resolve_version(args)
    output_path = expand_output_path(args.output, args.version)
    data = load_hex(input_path)
    image, crc = build_image(
        data,
        args.slot_base,
        args.slot_size,
        args.header_offset,
        args.version,
    )
    write_hex(output_path, args.slot_base, image)
    return len(image), crc


def run_self_test() -> None:
    crc = crc32_finish(crc32_ieee(b"123456789"))
    if crc != 0xCBF43926:
        raise AssertionError(f"CRC32 self-test failed: 0x{crc:08X}")

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        raw_path = tmp_path / "app.raw.hex"
        out_path = tmp_path / "app_v{version}_upgrade.hex"
        expected_out_path = tmp_path / "app_v00010000_upgrade.hex"
        payload = struct.pack("<II", 0x20001000, 0x00004101)
        raw_path.write_text(
            "\n".join(
                [
                    make_hex_record(0, 0x04, (0).to_bytes(2, "big")),
                    make_hex_record(0x4000, 0x00, payload),
                    make_hex_record(0, 0x01, b""),
                ]
            )
            + "\n",
            encoding="ascii",
        )
        args = argparse.Namespace(
            input=str(raw_path),
            output=str(out_path),
            slot_base=0x4000,
            slot_size=0x6000,
            header_offset=0x200,
            version=0x00010000,
        )
        image_size, _ = patch_file(args)
        if out_path.exists():
            raise AssertionError("version template was not expanded in output path")
        output_data = load_hex(expected_out_path)
        header_base = args.slot_base + args.header_offset
        header = bytes(output_data[header_base + i] for i in range(HEADER_SIZE))
        magic, header_ver, header_size, version, size, image_crc, flags = struct.unpack_from(
            "<IHHIIII", header, 0
        )
        if (
            magic != MAGIC
            or header_ver != HEADER_VERSION
            or header_size != HEADER_SIZE
            or version != args.version
            or size != image_size
            or flags != 0
        ):
            raise AssertionError("patched header fields are invalid")

        image = bytearray(output_data[args.slot_base + i] for i in range(image_size))
        struct.pack_into("<I", image, args.header_offset + CRC_FIELD_OFFSET, 0)
        if crc32_finish(crc32_ieee(bytes(image))) != image_crc:
            raise AssertionError("patched image CRC is invalid")

        header_path = tmp_path / "app_version.h"
        header_out_template = tmp_path / "app_v{version}_from_header.hex"
        expected_header_out = tmp_path / "app_v00010002_from_header.hex"
        header_path.write_text(
            "#ifndef APP_VERSION_H_\n"
            "#define APP_VERSION_H_\n"
            "#define APP_IMAGE_VERSION (0x00010002u)\n"
            "#endif\n",
            encoding="ascii",
        )
        args = argparse.Namespace(
            input=str(raw_path),
            output=str(header_out_template),
            slot_base=0x4000,
            slot_size=0x6000,
            header_offset=0x200,
            version=None,
            version_header=str(header_path),
            version_macro="APP_IMAGE_VERSION",
        )
        image_size, _ = patch_file(args)
        if header_out_template.exists():
            raise AssertionError("version header output template was not expanded")
        output_data = load_hex(expected_header_out)
        header = bytes(output_data[header_base + i] for i in range(HEADER_SIZE))
        _, _, _, version, size, _, _ = struct.unpack_from("<IHHIIII", header, 0)
        if version != 0x00010002 or size != image_size:
            raise AssertionError("version header fields are invalid")

        try:
            resolve_version(
                argparse.Namespace(
                    version=0x00010000,
                    version_header=str(header_path),
                    version_macro="APP_IMAGE_VERSION",
                )
            )
            raise AssertionError("conflicting version sources were accepted")
        except HexError:
            pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", help="raw Intel HEX generated by objcopy")
    parser.add_argument("--output", help="patched upgrade Intel HEX")
    parser.add_argument("--slot-base", type=parse_int, default=0x4000)
    parser.add_argument("--slot-size", type=parse_int, default=0x6000)
    parser.add_argument("--header-offset", type=parse_int, default=0x200)
    parser.add_argument("--version", type=parse_int)
    parser.add_argument("--version-header", help="C header containing the image version macro")
    parser.add_argument("--version-macro", default="APP_IMAGE_VERSION")
    parser.add_argument("--self-test", action="store_true")
    return parser


def main(argv: list[str]) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        if args.self_test:
            run_self_test()
            print("self-test passed")
            return 0

        if not args.input or not args.output:
            parser.error("--input and --output are required unless --self-test is used")
        image_size, crc = patch_file(args)
        output_path = expand_output_path(args.output, args.version)
        print(
            f"patched {output_path}: size=0x{image_size:X}, "
            f"version=0x{args.version:08X}, crc32=0x{crc:08X}"
        )
        return 0
    except (HexError, OSError, AssertionError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
