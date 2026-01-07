#!/usr/bin/env python3
import argparse
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class UartFormat:
    data_bits: int
    parity: str  # N/E/O
    stop_bits: int


def parse_uart_format(fmt: str) -> UartFormat:
    fmt = fmt.strip().upper()
    if len(fmt) != 3:
        raise ValueError("format must look like 8N1, 7E1, 8O2, etc.")
    data_bits = int(fmt[0])
    parity = fmt[1]
    stop_bits = int(fmt[2])

    if data_bits not in (5, 6, 7, 8, 9):
        raise ValueError("data bits must be 5..9")
    if parity not in ("N", "E", "O"):
        raise ValueError("parity must be N, E, or O")
    if stop_bits not in (1, 2):
        raise ValueError("stop bits must be 1 or 2")
    return UartFormat(data_bits=data_bits, parity=parity, stop_bits=stop_bits)


def parity_bit(value: int, data_bits: int, parity: str) -> int:
    if parity == "N":
        raise ValueError("no parity")
    ones = 0
    for i in range(data_bits):
        ones += (value >> i) & 1
    if parity == "E":
        return ones & 1  # make total even
    if parity == "O":
        return (ones + 1) & 1  # make total odd
    raise ValueError("unsupported parity")


def bit_time_us(baud: int) -> float:
    return 1_000_000.0 / float(baud)


def unescape_c_style(text: str) -> str:
    out: list[str] = []
    i = 0
    while i < len(text):
        ch = text[i]
        if ch != "\\":
            out.append(ch)
            i += 1
            continue

        i += 1
        if i >= len(text):
            out.append("\\")
            break

        esc = text[i]
        i += 1

        if esc == "n":
            out.append("\n")
        elif esc == "r":
            out.append("\r")
        elif esc == "t":
            out.append("\t")
        elif esc == "0":
            out.append("\x00")
        elif esc == "\\":
            out.append("\\")
        elif esc == "x":
            if i + 2 > len(text):
                out.append("\\x")
                continue
            hex_part = text[i : i + 2]
            if all(c in "0123456789abcdefABCDEF" for c in hex_part):
                out.append(chr(int(hex_part, 16)))
                i += 2
            else:
                out.append("\\x" + hex_part)
                i += 2
        else:
            out.append("\\" + esc)

    return "".join(out)


def render_frame(value: int, uart: UartFormat, bit_width: int, lsb_first: bool) -> str:
    bits: list[tuple[str, int]] = []
    bits.append(("Idle", 1))
    bits.append(("Start", 0))

    data_labels = [f"D{i}" for i in range(uart.data_bits)]
    if not lsb_first:
        data_labels = list(reversed(data_labels))

    for i, label in enumerate(data_labels):
        bit_index = i if lsb_first else (uart.data_bits - 1 - i)
        bits.append((label, (value >> bit_index) & 1))

    if uart.parity != "N":
        bits.append(("P", parity_bit(value, uart.data_bits, uart.parity)))

    for i in range(uart.stop_bits):
        bits.append((f"Stop{i + 1}" if uart.stop_bits > 1 else "Stop", 1))

    bits.append(("Idle", 1))

    def cell(text: str) -> str:
        return text.center(bit_width)

    def wave(level: int) -> str:
        return ("-" if level else "_") * bit_width

    bit_line = "Bit: " + "|".join(cell(name) for name, _ in bits)
    val_line = "Val: " + "|".join(cell(str(level)) for _, level in bits)
    tx_line = "TX : " + "|".join(wave(level) for _, level in bits)
    return "\n".join((bit_line, val_line, tx_line))


def label_byte(b: int) -> str:
    if b == 0x0D:
        return r"'\r'"
    if b == 0x0A:
        return r"'\n'"
    if b == 0x09:
        return r"'\t'"
    if 32 <= b <= 126 and b not in (0x27, 0x5C):  # ' and \
        return repr(chr(b))
    return f"0x{b:02X}"


def iter_bytes_from_text(text: str) -> Iterable[int]:
    return text.encode("latin1", errors="strict")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate ASCII UART timing diagram (idealized waveform)."
    )
    parser.add_argument("--baud", type=int, default=9600, help="UART baud rate")
    parser.add_argument(
        "--format", default="8N1", help="UART format like 8N1, 7E1, 8O2 (default: 8N1)"
    )
    parser.add_argument(
        "--bit-width",
        type=int,
        default=5,
        help="characters per bit cell (default: 5)",
    )
    parser.add_argument(
        "--msb-first",
        action="store_true",
        help="render data bits MSB-first (UART default is LSB-first)",
    )
    parser.add_argument(
        "--unescape",
        action="store_true",
        help=r"interpret C-style escapes in --text/--char (e.g. \r, \n, \t, \\ , \x0D)",
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--byte", help="single byte like 0x41 or 65")
    group.add_argument("--char", help="single character like A")
    group.add_argument("--text", help="ASCII text, render each character frame")

    args = parser.parse_args()

    uart = parse_uart_format(args.format)
    tbit = bit_time_us(args.baud)
    bits_per_char = 1 + uart.data_bits + (0 if uart.parity == "N" else 1) + uart.stop_bits
    tchar_ms = (tbit * bits_per_char) / 1000.0

    header = (
        f"UART {args.baud} bps {uart.data_bits}{uart.parity}{uart.stop_bits}  "
        f"Tbit≈{tbit:.2f}µs  1char≈{tchar_ms:.2f}ms  "
        f"({'MSB' if args.msb_first else 'LSB'} first)"
    )

    frames: list[tuple[str, int]] = []
    if args.byte is not None:
        s = args.byte.strip().lower()
        value = int(s, 0)
        if not (0 <= value <= 255):
            raise SystemExit("--byte must be 0..255")
        frames.append((f"0x{value:02X}", value))
    elif args.char is not None:
        if len(args.char) != 1:
            raise SystemExit("--char must be a single character")
        ch = unescape_c_style(args.char) if args.unescape else args.char
        if len(ch) != 1:
            raise SystemExit("--char must be a single character (after unescape)")
        value = ch.encode("latin1")[0]
        frames.append((label_byte(value), value))
    else:
        text = unescape_c_style(args.text) if args.unescape else args.text
        for b in iter_bytes_from_text(text):
            frames.append((label_byte(b), b))

    print(header)
    for idx, (label, value) in enumerate(frames):
        if idx:
            print()
        print(f"Frame {idx + 1}: {label} (0x{value:02X})")
        print(render_frame(value, uart, args.bit_width, lsb_first=not args.msb_first))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
