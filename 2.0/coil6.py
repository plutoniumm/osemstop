import statistics, time, serial

dac = [1, 3, 5, 7, 0, 2, 4, 6]                 
s = serial.Serial("/dev/cu.usbserial-1110", 115200, timeout=2)
time.sleep(2.5)                                
s.write(b"".join(b"SET %d 0.3\n" % c for c in range(8)) + b"ACK 0\nSTREAM\n")


def read(seconds, keep):                        
    end, rows = time.time() + seconds, []
    while time.time() < end:
        r = s.readline().decode(errors="replace").strip().split(",")
        
        if len(r) == 8 and all(x.isdigit() for x in r) and time.time() > end - keep:
            rows.append([int(x) for x in r])

    return [statistics.mean(c) for c in zip(*rows)]


read(30, 1)
for coil in (6, 7):
    print(
        "\n coil  volts |" + "".join("%6s" % ("A%d" % i) 
            for i in range(8))
    )
    print(" " + "-" * 61)

    prev = None
    for volts in (0.5, 0.1) * 3:
        s.write(b"SET %d %.1f\n" % (dac[coil], volts))
        now = read(6, 4.05)           
        
        cells = [
            "%6.0f" % n if prev is None else "%+6.0f" % (n - p)
                for n, p in zip(now, prev or now)
        ]
        
        print("    %d    %.1f |" % (coil, volts) + "".join(cells), flush=True)
        prev = now

    s.write(b"SET %d 0.3\n" % dac[coil])

s.write(b"STOP\n")
