#!/usr/bin/env python3
# cap.py SECS CMD...  -> run CMD for SECS seconds, capture stdout+stderr (bytes), then kill the process group.
import subprocess, sys, time, signal, os, select
secs = float(sys.argv[1]); cmd = sys.argv[2:]
p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0, preexec_fn=os.setsid)
start = time.time(); data = bytearray()
while time.time() - start < secs:
    if p.poll() is not None: break
    r,_,_ = select.select([p.stdout], [], [], 0.2)
    if r:
        chunk = os.read(p.stdout.fileno(), 65536)
        if not chunk: break
        data += chunk
try: os.killpg(os.getpgid(p.pid), signal.SIGTERM)
except Exception: pass
try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
except Exception: pass
sys.stdout.buffer.write(bytes(data)); sys.stdout.flush()
