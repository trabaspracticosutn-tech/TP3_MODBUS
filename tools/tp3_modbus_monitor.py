#!/usr/bin/env python3
"""Monitor pasivo del bus Modbus RTU del TP3 en 8E1. No transmite solicitudes."""

import argparse
import sys
import time


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def candidate_lengths(data):
    if len(data) < 3 or not 1 <= data[0] <= 247:
        return []
    function = data[1]
    if function in (0x83, 0x84, 0x86, 0x90):
        return [5]
    if function == 0x06:
        return [8]
    if function in (0x03, 0x04):
        lengths = [8]
        byte_count = data[2]
        if 2 <= byte_count <= 250 and byte_count % 2 == 0:
            lengths.append(5 + byte_count)
        return lengths
    if function == 0x10:
        lengths = [8]
        if len(data) >= 7 and 2 <= data[6] <= 246 and data[6] % 2 == 0:
            lengths.append(9 + data[6])
        return lengths
    return []


class FrameReader:
    """Separa tramas por longitud y CRC, incluso si USB las agrupa o fragmenta."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 5:
            match = None
            for offset in range(len(self.buffer) - 4):
                remaining = self.buffer[offset:]
                for length in candidate_lengths(remaining):
                    if len(remaining) < length:
                        continue
                    frame = bytes(remaining[:length])
                    if crc16(frame[:-2]) == int.from_bytes(frame[-2:], "little"):
                        match = (offset + length, frame)
                        break
                if match is not None:
                    break
            if match is None:
                break
            consumed, frame = match
            del self.buffer[:consumed]
            frames.append(frame)
        # Una trama RTU tiene como maximo 256 bytes. Conservar un posible inicio.
        if len(self.buffer) > 512:
            del self.buffer[:-256]
        return frames


def word(data, offset):
    return int.from_bytes(data[offset:offset + 2], "big")


def describe_value(offset, value, base=40001):
    register = base + offset
    if offset == 0:
        return f"{register} ADC={value}/4095"
    if offset == 3:
        return f"{register} estado={value} (0x{value:04X})"
    if offset == 8:
        return f"{register} DAC2={value}/255"
    return f"{register}={value}"


class Decoder:
    def __init__(self):
        self.pending_reads = {}

    def describe(self, frame):
        address, function = frame[:2]
        prefix = f"Esclavo {address} | "
        if function & 0x80:
            self.pending_reads.pop((address, function & 0x7F), None)
            return prefix + f"EXCEPCION FC{function & 0x7F:02X}: codigo 0x{frame[2]:02X}"
        if function in (0x03, 0x04):
            base = 30001 if function == 0x04 else 40001
            if len(frame) == 8:
                start, quantity = word(frame, 2), word(frame, 4)
                self.pending_reads[(address, function)] = (start, quantity)
                return prefix + (f"CONSULTA FC{function:02X} | leer {quantity} registros "
                                 f"desde {base + start}")
            values = [word(frame, i) for i in range(3, len(frame) - 2, 2)]
            request = self.pending_reads.pop((address, function), None)
            if request is None:
                return prefix + f"RESPUESTA FC{function:02X} | valores={values} (consulta no observada)"
            start, quantity = request
            if len(values) != quantity:
                return prefix + f"RESPUESTA FC{function:02X} | cantidad inesperada: {values}"
            registers = "; ".join(describe_value(start + i, v, base) for i, v in enumerate(values))
            return prefix + f"RESPUESTA FC{function:02X} | {registers}"
        if function == 0x06:
            # La orden FC06 y su respuesta son identicas en el bus.
            return prefix + "FC06 ORDEN/ECO | " + describe_value(word(frame, 2), word(frame, 4))
        if function == 0x10:
            start, quantity = word(frame, 2), word(frame, 4)
            if len(frame) == 8:
                return prefix + f"CONFIRMACION FC10 | {quantity} registros desde {40001 + start}"
            values = [word(frame, i) for i in range(7, len(frame) - 2, 2)]
            return prefix + f"ORDEN FC10 | desde {40001 + start}: {values}"
        return prefix + frame.hex(" ").upper()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--hex", action="store_true", help="mostrar tambien los bytes de cada trama")
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        parser.exit(1, "Falta pyserial. En Ubuntu/Debian: sudo apt install python3-serial\n")

    port = serial.Serial()
    port.port = args.port
    port.baudrate = args.baud
    port.bytesize = serial.EIGHTBITS
    port.parity = serial.PARITY_EVEN
    port.stopbits = serial.STOPBITS_ONE
    port.timeout = 0.1
    port.xonxoff = False
    port.rtscts = False
    port.dsrdtr = False
    port.rts = False
    port.dtr = False
    port.exclusive = True
    reader, decoder = FrameReader(), Decoder()
    started = time.monotonic()
    last_warning = started
    bytes_without_frame = 0
    try:
        with port:
            print(f"Escuchando {args.port} | {args.baud} 8E1 | Ctrl+C para salir", flush=True)
            while True:
                received = port.read(max(1, min(port.in_waiting, 512)))
                bytes_without_frame += len(received)
                for frame in reader.feed(received):
                    elapsed = time.monotonic() - started
                    print(f"+{elapsed:8.3f}s | {decoder.describe(frame)} | CRC OK", flush=True)
                    if args.hex:
                        print("              HEX: " + frame.hex(" ").upper(), flush=True)
                    bytes_without_frame = 0
                    last_warning = time.monotonic()
                if bytes_without_frame and time.monotonic() - last_warning >= 5:
                    print("Recibo bytes, pero no encuentro tramas validas: revisar velocidad, paridad 8E1 y A/B.", flush=True)
                    bytes_without_frame = 0
                    last_warning = time.monotonic()
    except KeyboardInterrupt:
        print("\nMonitor finalizado.")
    except (serial.SerialException, OSError) as error:
        print(f"Error del puerto: {error}. Cerra minicom y verifica el dispositivo y los permisos.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
