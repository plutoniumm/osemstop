#!/usr/bin/env python3
"""scope.py -- live 8-channel oscilloscope in the browser. Raw counts, nothing else.

The bench scope only has four inputs and the rig has eight OSEMs. Two sources,
picked automatically:

    csv     tail the newest data/*_fast_lock.csv -- what a RUNNING controller
            writes, so all eight channels are watchable mid-damping-run
    serial  open the board and read its stream, when nothing else holds the port

It never competes for the port. A live run is detected from the mtime of the log
it is streaming, not by trying the port -- see bench_busy() for why a trial open
is not good enough on macOS.

    python3 scope.py                       # auto-detect, http://localhost:8770
    python3 scope.py --source csv
    python3 scope.py --port /dev/cu.usbmodem11101 --http 8771
"""

import argparse
import fcntl
import glob
import itertools
import json
import os
import re
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

N = 8
FULL = 1023                      # 10-bit ADC, full scale
RAIL_LO, RAIL_HI = 12, 1011      # the ladder's RAIL_LOW_COUNTS / RAIL_HIGH_COUNTS
MID = FULL / 2.0                 # 511.5; every OSEM rests above it (CLAUDE.md 13)
WIDEST_VIEW_S = 120.0            # the largest window the page offers
KEEP_S = WIDEST_VIEW_S + 5.0     # history held server-side, a little over it
CSV_GLOB = "data/*_fast_lock.csv"
LIVE_GLOB = "data/*.csv"         # any bench tool holding the port also streams to disk
LIVE_S = 3.0                     # controllers flush every 200 rows, i.e. ~0.5 s at 380 Hz


def newest(pattern):
    files = glob.glob(pattern)
    if not files:
        return None
    try:
        return max(files, key=os.path.getmtime)
    except OSError:
        return None


def bench_busy(within=LIVE_S):
    """The newest log in data/ if something is still writing it, else None.

    This, not a trial open, is how a live run is detected. macOS does NOT make a
    /dev/cu.* open exclusive and pyserial only flocks when asked, so a second
    reader CAN steal stream bytes -- measured 2026-08-17, two scope.py processes
    both read the same port at 380 Hz each. Every bench tool streams every raw
    sample to disk (CLAUDE.md standing practice), so a fresh mtime is the cheap
    check that touches nothing.
    """
    p = newest(LIVE_GLOB)
    try:
        return p if p and os.path.getmtime(p) > time.time() - within else None
    except OSError:
        return None


def counts_columns(header):
    """(time column, chN_counts columns in channel order) from a CSV header."""
    names = header.strip().split(",")
    cols = {}
    for i, name in enumerate(names):
        m = re.fullmatch(r"ch(\d+)_counts", name)
        if m:
            cols[int(m.group(1))] = i
    t_idx = names.index("time_s") if "time_s" in names else 0
    return t_idx, [cols[k] for k in sorted(cols)]


class Ring:
    """Timestamped rows of raw counts, plus a monotonic cursor so the page can ask
    for only what it has not already drawn."""

    def __init__(self, keep_s=KEEP_S):
        self.lock = threading.Lock()
        self.rows = deque()
        self.keep = keep_s
        self.cursor = 0
        self.mode = "starting"
        self.note = "looking for a source"
        self.nch = N

    def add(self, t, counts):
        with self.lock:
            self.rows.append((t, counts))
            self.cursor += 1
            cut = t - self.keep
            while self.rows and self.rows[0][0] < cut:
                self.rows.popleft()

    def set_mode(self, mode, note, nch=None):
        with self.lock:
            self.mode, self.note = mode, note
            if nch:
                self.nch = nch

    def clear(self):
        with self.lock:
            self.rows.clear()

    def since(self, cursor):
        with self.lock:
            first = self.cursor - len(self.rows)
            start = max(0, min(cursor, self.cursor) - first)
            rows = [[round(t, 4)] + c
                    for t, c in itertools.islice(self.rows, start, None)]
            return {"first": first, "cursor": self.cursor, "rows": rows,
                    "mode": self.mode, "note": self.note, "nch": self.nch,
                    "hz": self._hz()}

    def _hz(self):
        """Sample rate over the last two seconds of history. Caller holds the lock."""
        if len(self.rows) < 2:
            return 0.0
        t_end = self.rows[-1][0]
        n = 0
        for t, _ in reversed(self.rows):
            if t_end - t > 2.0:
                break
            n += 1
        span = t_end - self.rows[-n][0]
        return round(n / span, 1) if span > 0 else 0.0


class CsvSource(threading.Thread):
    """Follow the newest fast_lock CSV, switching if a new run starts."""

    daemon = True

    def __init__(self, ring, pattern=CSV_GLOB, path=None, back_bytes=1 << 23):
        super().__init__(name="csv")
        self.ring, self.pattern, self.fixed = ring, pattern, path
        self.back = back_bytes
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            path = self.fixed or newest(self.pattern)
            if path is None:
                self.ring.set_mode("csv", "no %s yet -- start a controller" % self.pattern)
                self.stop.wait(1.0)
                continue
            try:
                self._follow(path)
            except OSError as exc:
                self.ring.set_mode("csv", "%s: %s" % (os.path.basename(path), exc))
                self.stop.wait(1.0)

    def _follow(self, path):
        with open(path, "r", errors="replace") as f:
            header = f.readline()
            if not header.endswith("\n"):
                self.stop.wait(0.5)          # header still being written
                return
            t_idx, cols = counts_columns(header)
            if not cols:
                self.ring.set_mode("csv", "%s has no chN_counts columns"
                                   % os.path.basename(path))
                self.stop.wait(2.0)
                return
            self.ring.clear()
            self.ring.set_mode("csv", "tailing %s" % path, nch=len(cols))
            # Start part-way back so the page opens with a screenful of history;
            # the seek lands mid-row, so throw that row away.
            f.seek(max(0, os.path.getsize(path) - self.back))
            f.readline()
            buf = ""
            idle = 0.0
            while not self.stop.is_set():
                chunk = f.read(1 << 18)
                if not chunk:
                    idle += 0.05
                    if idle > 1.0:
                        live = os.path.getmtime(path) > time.time() - LIVE_S
                        self.ring.set_mode(
                            "csv", "tailing %s%s" % (path, "" if live else " (idle)"),
                            nch=len(cols))
                        if not self.fixed and newest(self.pattern) != path:
                            return                   # a newer run started
                        idle = 0.0
                    self.stop.wait(0.05)
                    continue
                idle = 0.0
                buf += chunk
                lines = buf.split("\n")
                buf = lines.pop()                    # trailing partial row
                for line in lines:
                    self._row(line, t_idx, cols)

    def _row(self, line, t_idx, cols):
        parts = line.split(",")
        if len(parts) <= cols[-1]:
            return                                   # short or torn row
        try:
            t = float(parts[t_idx])
            counts = [int(parts[c]) for c in cols]
        except ValueError:
            return
        self.ring.add(t, [v if 0 <= v <= FULL else None for v in counts])


class SerialSource(threading.Thread):
    """Read the board's own stream. Started only when bench_busy() found nobody."""

    daemon = True

    def __init__(self, ring, dac, note, nch=N):
        super().__init__(name="serial")
        self.ring, self.dac, self.note, self.nch = ring, dac, note, nch
        self.stop = threading.Event()

    def run(self):
        t0 = last = time.perf_counter()
        flowing = True
        try:
            while not self.stop.is_set():
                s = self.dac.read_sample(self.nch)
                now = time.perf_counter()
                if s is None:
                    # Opening the port resets the board, so anything else that
                    # touches it stops this stream dead without an exception.
                    if flowing and now - last > 2.0:
                        self.ring.set_mode("dead", "%s went silent -- board reset, "
                                           "unplugged, or another reader took it"
                                           % self.note)
                        flowing = False
                    time.sleep(0.001)
                    continue
                if not flowing:
                    self.ring.set_mode("serial", self.note)
                    flowing = True
                last = now
                self.ring.add(now - t0,
                              [v if 0 <= v <= FULL else None for v in s])
        except Exception as exc:                     # unplugged mid-run
            self.ring.set_mode("dead", "serial port died: %s" % exc)
        finally:
            self.close()

    def close(self):
        self.stop.set()
        try:
            self.dac.close()
        except Exception:
            pass


def find_port():
    """Best candidate port, ranked by `bench.rank` -- one owner for that ranking.

    This used to take the first device reporting any VID, which is NOT what
    bench.py picks: a known board VID outranks a generic USB serial device.
    """
    try:
        from serial.tools import list_ports

        import bench
    except ImportError:
        return None
    found = [p for p in list_ports.comports() if bench.rank(p) < 2]
    return min(found, key=bench.rank).device if found else None


def open_board(port):
    """(FastDAC, note) or (None, why-not). Never called while bench_busy()."""
    import stdlib
    if port is None:
        return None, "no USB serial device found"
    dac = None
    try:
        # stdlib.open_dac probes the baud first. Never skip that: at the wrong
        # rate the READY scan is 200 lines x 2 s = 400 s of silence.
        dac = stdlib.open_dac(port, log=lambda s: print("  " + s))
        baud = dac.ser.baudrate
        # Advisory, and it only keeps a SECOND scope.py off the wire -- the
        # controllers do not flock, which is why bench_busy() exists.
        fcntl.flock(dac.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        dac.start_stream()
        return dac, "%s at %d baud" % (port, baud)
    except SystemExit as exc:                        # resolve_baud found no READY
        why = str(exc).strip().splitlines()[0]
    except Exception as exc:
        why = "%s: %s" % (port, exc)
    if dac is not None:
        try:
            dac.close()
        except Exception:
            pass
    return None, why


# --------------------------------------------------------------------------
PAGE = r"""<!doctype html>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OSEM scope</title>
<style>
 :root{color-scheme:dark}
 *{box-sizing:border-box}
 body{margin:0;background:#0b0f14;color:#c8d3de;
      font:13px/1.4 ui-monospace,SFMono-Regular,Menlo,monospace}
 #bar{display:flex;gap:14px;align-items:center;flex-wrap:wrap;padding:8px 12px;
      border-bottom:1px solid #1e2936}
 #bar b{color:#eef4fa;font-weight:600}
 #mode{color:#8fa3b6}
 #mode.bad{color:#ff6b6b}
 select{background:#141c26;color:#c8d3de;border:1px solid #2a3a4c;padding:2px 4px;
        font:inherit}
 #wrap{padding:10px 12px}
 canvas{width:100%;height:64vh;min-height:320px;display:block;
        background:#070b0f;border:1px solid #1e2936}
 #leg{display:flex;gap:6px;flex-wrap:wrap;margin-top:8px}
 .ch{border:1px solid #2a3a4c;border-radius:3px;padding:3px 7px;cursor:pointer;
     user-select:none;min-width:104px}
 .ch.off{opacity:.35}
 .ch i{display:inline-block;width:9px;height:9px;margin-right:6px;border-radius:2px}
 .ch span{color:#eef4fa}
 .ch em{font-style:normal;color:#ff6b6b}
</style>
<div id="bar">
  <b>OSEM scope</b>
  <span id="mode">connecting</span>
  <span>window
    <select id="win">
      <option value="5">5 s</option>
      <option value="20" selected>20 s</option>
      <option value="60">60 s</option>
      <option value="120">120 s</option>
    </select></span>
  <span id="rate"></span>
</div>
<div id="wrap"><canvas id="c"></canvas><div id="leg"></div></div>
<script>
const {FULL, LO, HI, MID, KEEP} = __CONSTS__;   // filled in from the Python side
const COL=['#ff4d4d','#ff9f2e','#ffe14d','#4ade4a',
           '#35e0e0','#5aa0ff','#b07cff','#ff5ec4'];
const cv=document.getElementById('c'), ctx=cv.getContext('2d');
const legend=document.getElementById('leg');
let nch=8, on=[], ts=[], ys=[], cur=0, win=20, stale=0;

function reset(n){
  nch=n; ts=[]; ys=[];
  for(let i=0;i<nch;i++) ys.push([]);
  if(on.length!==nch) on=new Array(nch).fill(true);
  legend.innerHTML='';
  for(let i=0;i<nch;i++){
    const d=document.createElement('div');
    d.className='ch'; d.dataset.i=i;
    d.innerHTML='<i style="background:'+COL[i%8]+'"></i>a'+i+' <span>--</span>';
    d.onclick=()=>{on[i]=!on[i]; d.classList.toggle('off',!on[i]); draw();};
    legend.appendChild(d);
  }
}

function trim(){
  const cut=ts[ts.length-1]-KEEP;
  let k=0; while(k<ts.length && ts[k]<cut) k++;
  if(k){ ts.splice(0,k); for(const y of ys) y.splice(0,k); }
}

async function poll(){
  try{
    const r=await fetch('data?since='+cur,{cache:'no-store'});
    const j=await r.json();
    if(j.nch!==nch) reset(j.nch);
    if(j.first>cur && cur) { ts=[]; for(const y of ys) y.length=0; }   // fell behind
    cur=j.cursor;
    for(const row of j.rows){
      if(ts.length && row[0]<ts[ts.length-1]){       // new run: time restarted
        ts=[]; for(const y of ys) y.length=0;
      }
      ts.push(row[0]);
      for(let i=0;i<nch;i++) ys[i].push(row[i+1]);
    }
    if(ts.length) trim();
    const m=document.getElementById('mode');
    m.textContent=j.mode.toUpperCase()+' -- '+j.note;
    m.className=(j.mode==='dead'||!j.rows.length&&!ts.length)?'bad':'';
    document.getElementById('rate').textContent=
      j.hz?j.hz.toFixed(0)+' Hz  t='+(ts.length?ts[ts.length-1].toFixed(1):'0')+' s':'';
    stale=0;
  }catch(e){
    if(++stale>20) document.getElementById('mode').textContent='server not responding';
  }
  draw();
  setTimeout(poll,100);
}

function draw(){
  const dpr=window.devicePixelRatio||1;
  const w=Math.round(cv.clientWidth*dpr), h=Math.round(cv.clientHeight*dpr);
  if(cv.width!==w||cv.height!==h){ cv.width=w; cv.height=h; }
  ctx.fillStyle='#070b0f'; ctx.fillRect(0,0,w,h);
  const L=44*dpr, R=8*dpr, T=8*dpr, B=20*dpr;
  const px=v=>L+v*(w-L-R), py=v=>T+(1-v/FULL)*(h-T-B);

  ctx.lineWidth=dpr; ctx.font=(10*dpr)+'px ui-monospace,monospace';
  ctx.textBaseline='middle';
  for(const v of [0,256,768,FULL]){                       // plain gridlines
    ctx.strokeStyle='#141d27'; ctx.beginPath();
    ctx.moveTo(L,py(v)); ctx.lineTo(w-R,py(v)); ctx.stroke();
    ctx.fillStyle='#5b6b7c'; ctx.textAlign='right'; ctx.fillText(v,L-6*dpr,py(v));
  }
  for(const [v,c] of [[LO,'#7a2a2a'],[MID,'#2a4a5a'],[HI,'#7a2a2a']]){
    ctx.strokeStyle=c; ctx.setLineDash([5*dpr,4*dpr]); ctx.beginPath();
    ctx.moveTo(L,py(v)); ctx.lineTo(w-R,py(v)); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle=c; ctx.textAlign='right'; ctx.fillText(v,L-6*dpr,py(v));
  }

  const n=ts.length;
  if(!n) return;
  const t1=ts[n-1], t0=t1-win;
  let i0=n-1; while(i0>0 && ts[i0-1]>=t0) i0--;
  const span=Math.max(1,n-i0), cols=Math.max(1,Math.round((w-L-R)/dpr));
  const step=Math.max(1,Math.floor(span/cols));

  for(let ch=0;ch<nch;ch++){
    if(!on[ch]) continue;
    const y=ys[ch];
    ctx.strokeStyle=COL[ch%8]; ctx.beginPath();
    let open=false;
    // One vertical extent per pixel column when decimating, so a single-sample
    // rail touch is still visible.
    for(let i=i0;i<n;i+=step){
      let lo=Infinity, hi=-Infinity;
      for(let k=i;k<Math.min(i+step,n);k++){
        const v=y[k];
        if(v===null) continue;
        if(v<lo) lo=v; if(v>hi) hi=v;
      }
      if(lo===Infinity){ open=false; continue; }          // gap: torn samples
      const x=px((ts[i]-t0)/win);
      if(!open){ ctx.moveTo(x,py(lo)); open=true; }
      ctx.lineTo(x,py(lo));
      if(hi!==lo) ctx.lineTo(x,py(hi));
    }
    ctx.stroke();
  }

  ctx.fillStyle='#5b6b7c'; ctx.textAlign='center'; ctx.textBaseline='bottom';
  for(let k=0;k<=4;k++)
    ctx.fillText((t0+win*k/4).toFixed(1)+'s',px(k/4),h-4*dpr);

  const cells=legend.children;
  for(let ch=0;ch<nch && ch<cells.length;ch++){
    const v=ys[ch][n-1], s=cells[ch].querySelector('span');
    if(v===null){ s.innerHTML='<em>torn</em>'; continue; }
    s.textContent=String(v).padStart(4);
    s.style.color=(v<=LO||v>=HI)?'#ff6b6b':'#eef4fa';
  }
}

document.getElementById('win').onchange=e=>{win=+e.target.value; draw();};
addEventListener('resize',draw);
reset(8); poll();
</script>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    ring = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?")[0].rstrip("/") or "/"
        if path in ("/", "/index.html"):
            page = PAGE.replace("__CONSTS__", json.dumps(
                {"FULL": FULL, "LO": RAIL_LO, "HI": RAIL_HI, "MID": MID,
                 "KEEP": KEEP_S}))
            return self._send(200, page, "text/html; charset=utf-8")
        if path == "/data":
            since = 0
            m = re.search(r"since=(\d+)", self.path)
            if m:
                since = int(m.group(1))
            return self._send(200, json.dumps(self.ring.since(since)),
                              "application/json")
        self._send(404, "not found", "text/plain")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--http", type=int, default=8770, help="HTTP port (default 8770)")
    ap.add_argument("--port", default=os.environ.get("PORT") or None,
                    help="serial device (default: autodetect)")
    ap.add_argument("--source", choices=("auto", "serial", "csv"), default="auto")
    ap.add_argument("--file", default=None,
                    help="tail this CSV instead of the newest " + CSV_GLOB)
    args = ap.parse_args()

    ring = Ring()
    Handler.ring = ring
    ThreadingHTTPServer.allow_reuse_address = True
    try:
        srv = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    except OSError as exc:
        sys.exit("  Could not bind port %d: %s\n  Try: --http %d"
                 % (args.http, exc, args.http + 1))

    src = None
    want_serial = args.source == "serial" or (args.source == "auto" and not args.file)
    # The guard is unconditional, `--source serial` included: opening the port at
    # all sends STOP and ACK 0, which would stop a live controller's stream dead.
    busy = bench_busy()
    if want_serial and busy:
        print("  %s is being written right now -- something owns the board.\n"
              "  Not going near the port; tailing the run instead." % busy)
        if args.source == "serial":
            sys.exit("  --source serial refused while a run is live.")
        ring.set_mode("csv", "a run is live (%s) -- tailing its CSV, port untouched"
                      % os.path.basename(busy))
        want_serial = False
    if want_serial:
        port = args.port or find_port()
        print("  serial: trying %s" % (port or "(nothing found)"))
        dac, why = open_board(port)
        if dac is not None:
            print("  serial: %s" % why)
            src = SerialSource(ring, dac, why)
            ring.set_mode("serial", why)
        else:
            print("  serial: %s" % why)
            if args.source == "serial":
                sys.exit("  --source serial was asked for and the port is not usable.")
            print("  falling back to tailing %s" % (args.file or CSV_GLOB))
            ring.set_mode("csv", "port unavailable (%s) -- tailing the run's CSV" % why)
    if src is None:
        src = CsvSource(ring, path=args.file)
    src.start()

    print("\n  scope on http://localhost:%d   (Ctrl+C to stop)\n" % args.http,
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        src.stop.set()
        if isinstance(src, SerialSource):
            src.close()
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
