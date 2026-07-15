#!/usr/bin/env python3
"""Recursive filename walker for an ext4 rootfs image via debugfs.

macOS helper: macOS can't mount ext4, and the R1 rootfs images are captured
live (block bitmaps fail checksum), so use debugfs in catastrophic mode (-c).
On Linux, just `mount -o ro,loop <partition> /mnt` and use plain find/grep.

Usage:
    python3 dbwalk.py /dev/diskNs7 /unitree/module/ai_sport
    python3 dbwalk.py /dev/diskNs7 /unitree | grep -iE '\\.onnx|\\.csv'

Setup (macOS):
    brew install e2fsprogs
    hdiutil attach -imagekey diskimage-class=CRawDiskImage -nomount rootfs.img
    # note the partition device it prints (e.g. /dev/disk4s7 for rootfs)
"""
import subprocess, sys, collections, shutil

DEBUGFS = shutil.which("debugfs") or "/opt/homebrew/opt/e2fsprogs/sbin/debugfs"
PRUNE = {"system_journal", "lost+found", "__pycache__", "proc", "sys"}


def ls(dev, path):
    out = subprocess.run([DEBUGFS, "-c", "-R", f"ls -l {path}", dev],
                         capture_output=True, text=True).stdout
    for line in out.splitlines():
        p = line.split()
        if len(p) < 9 or not p[0].isdigit():
            continue
        mode, name = p[1], p[-1]
        if name in (".", ".."):
            continue
        typ = "dir" if mode.startswith("4") else ("link" if mode.startswith("12") else "file")
        try:
            size = int(p[5])
        except ValueError:
            size = 0
        yield typ, name, size


def walk(dev, root, max_entries=300000):
    q, n = collections.deque([root]), 0
    while q:
        dpath = q.popleft()
        for typ, name, size in ls(dev, dpath):
            path = f"{dpath}/{name}".replace("//", "/")
            n += 1
            if n > max_entries:
                print(f"# stopped at {max_entries} entries", file=sys.stderr)
                return
            if typ == "dir":
                if name not in PRUNE:
                    q.append(path)
            else:
                print(f"{size}\t{path}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("usage: dbwalk.py <device> [start_path]")
    walk(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "/")
