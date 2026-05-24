#!/usr/bin/env python3
# diagnostic_lidar.py
# Запуск: python3 diagnostic_lidar.py

import serial
import time
import struct

PORT     = '/dev/ttyUSB0'
BAUDRATE = 128000

def hexdump(data: bytes, prefix=''):
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_part = ' '.join(f'{b:02X}' for b in chunk)
        asc_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        print(f"{prefix}{i:04X}  {hex_part:<48}  {asc_part}")

def main():
    print(f"Opening {PORT} @ {BAUDRATE}...")
    s = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=1.0,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    s.reset_input_buffer()
    s.reset_output_buffer()
    time.sleep(0.2)

    print("Sending scan command 0xA5 0x60...")
    s.write(bytes([0xA5, 0x60]))
    s.flush()
    time.sleep(0.5)

    # Читаем первые 200 байт и смотрим что пришло
    print("\n=== RAW FIRST 200 BYTES ===")
    raw = s.read(200)
    print(f"Got {len(raw)} bytes:")
    hexdump(raw)

    # Ищем все вхождения возможных заголовков
    print("\n=== SEARCHING FOR SYNC PATTERNS ===")
    patterns = [
        (0xAA, 0x55, "YDLidar data packet"),
        (0xA5, 0x5A, "YDLidar device info"),
        (0xAA, 0x55, "Alt header 1"),
        (0x55, 0xAA, "Alt header 2"),
    ]
    for b1, b2, name in patterns:
        positions = []
        for i in range(len(raw) - 1):
            if raw[i] == b1 and raw[i+1] == b2:
                positions.append(i)
        if positions:
            print(f"  {name} (0x{b1:02X} 0x{b2:02X}): found at positions {positions}")

    # Читаем ещё данных и смотрим структуру
    print("\n=== CONTINUOUS READ (5 seconds) ===")
    all_data = bytearray(raw)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        chunk = s.read(256)
        if chunk:
            all_data.extend(chunk)

    print(f"Total bytes collected: {len(all_data)}")

    # Анализируем частоту байт
    print("\n=== BYTE FREQUENCY (top 20) ===")
    freq = {}
    for b in all_data:
        freq[b] = freq.get(b, 0) + 1
    sorted_freq = sorted(freq.items(), key=lambda x: -x[1])[:20]
    for byte_val, count in sorted_freq:
        print(f"  0x{byte_val:02X} ({byte_val:3d}): {count:5d} times")

    # Ищем повторяющиеся паттерны заголовков
    print("\n=== PACKET STRUCTURE ANALYSIS ===")
    # Смотрим расстояния между вхождениями 0xAA 0x55
    positions_aa55 = []
    for i in range(len(all_data) - 1):
        if all_data[i] == 0xAA and all_data[i+1] == 0x55:
            positions_aa55.append(i)

    print(f"0xAA 0x55 found {len(positions_aa55)} times")
    if len(positions_aa55) > 1:
        diffs = [positions_aa55[i+1] - positions_aa55[i]
                 for i in range(min(20, len(positions_aa55)-1))]
        print(f"Distances between headers: {diffs}")

        # Показываем первые несколько "пакетов"
        print("\n=== FIRST 3 PACKETS CONTENT ===")
        for idx, pos in enumerate(positions_aa55[:3]):
            end = positions_aa55[idx+1] if idx+1 < len(positions_aa55) else pos+100
            pkt = all_data[pos:min(end, pos+80)]
            print(f"\nPacket {idx} at offset {pos} (size ~{end-pos}):")
            hexdump(bytes(pkt), prefix='  ')
            if len(pkt) >= 10:
                print(f"  Byte[2] = 0x{pkt[2]:02X} ({pkt[2]}) — packet_type или sample_count")
                print(f"  Byte[3] = 0x{pkt[3]:02X} ({pkt[3]}) — ?")
                print(f"  Byte[4] = 0x{pkt[4]:02X} ({pkt[4]}) — ?")
                print(f"  Byte[5] = 0x{pkt[5]:02X} ({pkt[5]}) — ?")

    # Сохраняем дамп для анализа
    dump_file = '/tmp/lidar_dump.bin'
    with open(dump_file, 'wb') as f:
        f.write(bytes(all_data))
    print(f"\n=== Raw dump saved to {dump_file} ===")
    print("Run: xxd /tmp/lidar_dump.bin | head -50")

    s.close()

if __name__ == '__main__':
    main()
