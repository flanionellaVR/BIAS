#!/usr/bin/env python3
"""Repeatedly poll the BIAS FlyTrack HTTP server for the fly ellipse + wing angles,
and report dropped/skipped frames, request latency, hangs, and the frame capture
times (capture_time) with the camera interval they imply.

Use this to reproduce/measure the high-rate polling slowdown, and to check the camera
timing that BIAS reports with each track.

Usage:
  python poll_http.py [--host 127.0.0.1] [--port 5010] [--cmd pop-back-track]
                      [--rate 0] [--duration 10] [--timeout 2] [--quiet]

  --rate 0   poll as fast as possible; otherwise a target Hz.
  --cmd      pop-back-track : newest entry, popped (what the colleague polls). With this,
                              frame gaps mix "polled too slow" and tracker drops.
             pop-front-track: oldest entry, popped -> drains the queue IN ORDER, so
                              frame gaps = genuine tracker-dropped input frames. Best for
                              measuring whether wing tracking causes drops.
             get-last-clear-track: newest entry, clears the queue.
  --timeout  per-request timeout (s); requests slower than this count as hangs.

Capture times:
  BIAS 0.79.0 and later add "capture_time" to every track: the capture time of the
  tracked frame in seconds since the capture started (the camera's image timestamp;
  for video input, frame index times frame interval). The per-poll line then shows it
  as ct=..., and the summary reports
  * whether capture_time was present (missing means an older BIAS build),
  * how many consecutive replies had a repeated or backwards capture_time. With
    pop-front-track and get-last-clear-track this should be 0 within one capture
    (restarting the capture resets it to about 0). pop-back-track pops only the newest
    track and leaves the older ones queued, so a poll that finds no newer track
    returns an older one and backwards steps are possible. These pairs are counted
    and left out of the two statistics below,
  * the capture_time interval between consecutive replies in ms (min, median, p95,
    p99, max). With pop-front-track the frame difference is mostly 1, so this is the
    camera timing itself,
  * the camera interval, (capture_time difference) / (frame difference) over
    consecutive replies whose frame difference is at least 1, as median, std, p5 and
    p95. The frame number is the grabber's counter, so a frame that the camera itself
    dropped makes the estimate of that pair too large.

Notes:
  * The server port is the one shown in the BIAS window's status / config (default 5010).
    If you don't get data, try a neighboring port (e.g. 5020) -- BIAS offsets the port by
    camera number.
  * Stdlib only (urllib/json) -- no numpy/requests needed.
"""
import argparse
import http.client
import json
import statistics
import time
import urllib.parse


def _pct(sorted_vals, p):
    # nearest-rank percentile of an already sorted, non-empty list
    return sorted_vals[min(len(sorted_vals) - 1, int(p * len(sorted_vals)))]


def _parse_ellipse(body):
    # response is a JSON array: [{success, message, value, command}], value = ellipse JSON str
    data = json.loads(body)
    item = data[0] if isinstance(data, list) and data else data
    val = item.get("value", "") if isinstance(item, dict) else ""
    if isinstance(item, dict) and item.get("success") and val:
        try:
            return json.loads(val) if isinstance(val, str) else val
        except (json.JSONDecodeError, TypeError):
            return None
    return None


class Poller:
    """Polls the BIAS HTTP server. In keep-alive mode it reuses a single TCP
    connection (sending keep-alive=1) so polling isn't capped by per-request
    connection setup; otherwise it opens and closes a connection per poll
    (like BIAS's default behavior, for comparison)."""

    def __init__(self, host, port, timeout, keep_alive):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.keep_alive = keep_alive
        self.conn = None

    def _connect(self):
        self.conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)

    def close(self):
        if self.conn is not None:
            try:
                self.conn.close()
            except OSError:
                pass
            self.conn = None

    def poll(self, plugin, cmd, last=False):
        params = {"plugin": plugin, "cmd": cmd}
        q = urllib.parse.quote(json.dumps(params))
        path = "/?plugin-cmd=%s" % q
        if self.keep_alive:
            # keep-alive=1 every poll; 0 on the final poll so the server closes cleanly.
            path += "&keep-alive=%d" % (0 if last else 1)
        hdr = {"Connection": "keep-alive" if (self.keep_alive and not last) else "close"}

        t0 = time.perf_counter()
        if self.conn is None:
            self._connect()
        try:
            self.conn.request("GET", path, headers=hdr)
            r = self.conn.getresponse()
            body = r.read().decode("utf-8", "replace")
        except (http.client.HTTPException, OSError):
            # stale/closed connection -- reconnect once and retry
            self.close()
            self._connect()
            self.conn.request("GET", path, headers=hdr)
            r = self.conn.getresponse()
            body = r.read().decode("utf-8", "replace")
        dt = time.perf_counter() - t0

        if not self.keep_alive or last:
            self.close()
        return dt, _parse_ellipse(body)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5010)
    ap.add_argument("--plugin", default="FlyTrack")
    ap.add_argument("--cmd", default="pop-back-track",
                    choices=["pop-back-track", "pop-front-track", "get-last-clear-track"])
    ap.add_argument("--rate", type=float, default=0.0, help="target poll Hz (0 = as fast as possible)")
    ap.add_argument("--duration", type=float, default=10.0, help="seconds to poll")
    ap.add_argument("--timeout", type=float, default=2.0, help="per-request timeout (s) = hang threshold")
    ap.add_argument("--quiet", action="store_true", help="suppress the per-poll lines (summary only)")
    ap.add_argument("--no-keep-alive", action="store_true",
                    help="open/close a new connection every poll (default: keep-alive, reuse one)")
    args = ap.parse_args()
    keep_alive = not args.no_keep_alive
    poller = Poller(args.host, args.port, args.timeout, keep_alive)

    period = 1.0 / args.rate if args.rate > 0 else 0.0
    latencies = []
    frames = []                 # frame numbers of non-empty responses, in poll order
    wing_of = {}                # frame -> (nwings, |anglel|+|angler|)
    cts = []                    # (frame, capture_time) of non-empty responses that have it
    n_poll = n_empty = n_hang = n_err = 0
    prev_fr = None

    print("polling http://%s:%d  cmd=%s  rate=%s  keep-alive=%s  for %.0fs ..."
          % (args.host, args.port, args.cmd, "max" if args.rate == 0 else args.rate,
             "on" if keep_alive else "off", args.duration))
    t_end = time.perf_counter() + args.duration
    next_t = time.perf_counter()
    while time.perf_counter() < t_end:
        if period:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(next_t - now)
            next_t += period
        n_poll += 1
        try:
            dt, ell = poller.poll(args.plugin, args.cmd)
            latencies.append(dt)
            if ell is None:
                n_empty += 1
                if not args.quiet:
                    print("[%5d] %6.1fms  <empty>" % (n_poll, 1e3 * dt))
            else:
                fr = int(ell.get("frame", -1))
                gap = (fr - prev_fr - 1) if (prev_fr is not None and fr > prev_fr) else 0
                prev_fr = fr
                frames.append(fr)
                wl = abs(float(ell.get("wing_anglel", 0.0)))
                wr = abs(float(ell.get("wing_angler", 0.0)))
                wing_of[fr] = (int(ell.get("nwings", 0)), wl + wr)
                ct = ell.get("capture_time")
                if ct is not None:
                    ct = float(ct)
                    cts.append((fr, ct))
                if not args.quiet:
                    ct_s = "ct=%.6f  " % ct if ct is not None else ""
                    print("[%5d] %6.1fms  frame=%-7d gap=%-4d  %sx=%7.1f y=%7.1f th=%+.3f  "
                          "nw=%d wl=%+.3f wr=%+.3f"
                          % (n_poll, 1e3 * dt, fr, gap, ct_s,
                             float(ell.get("x", 0)), float(ell.get("y", 0)), float(ell.get("theta", 0)),
                             int(ell.get("nwings", 0)),
                             float(ell.get("wing_anglel", 0)), float(ell.get("wing_angler", 0))))
        except (http.client.HTTPException, OSError) as e:
            is_to = isinstance(e, TimeoutError) or "timed out" in str(e).lower()
            n_hang += int(is_to)
            n_err += int(not is_to)
            if not args.quiet:
                print("[%5d] %s: %s" % (n_poll, "HANG" if is_to else "ERR", e))
        except Exception as e:
            n_err += 1
            if not args.quiet:
                print("[%5d] ERR: %s" % (n_poll, e))

    # Tell the server we're done so it closes the kept-open connection cleanly.
    if keep_alive:
        try:
            poller.poll(args.plugin, args.cmd, last=True)
        except (http.client.HTTPException, OSError):
            pass
    poller.close()

    # ---- summary ----
    print("\npolls=%d  with-data=%d  empty=%d  hangs(>%.1fs)=%d  errors=%d  achieved=%.0f Hz"
          % (n_poll, len(frames), n_empty, args.timeout, n_hang, n_err, n_poll / args.duration))
    if latencies:
        latencies.sort()
        print("latency ms: min=%.1f  p50=%.1f  p95=%.1f  p99=%.1f  max=%.1f"
              % (1e3 * latencies[0], 1e3 * _pct(latencies, .5), 1e3 * _pct(latencies, .95),
                 1e3 * _pct(latencies, .99), 1e3 * latencies[-1]))
    if len(frames) >= 2:
        gaps = [(a, b, b - a - 1) for a, b in zip(frames, frames[1:]) if b - a > 1]
        dropped = sum(g[2] for g in gaps)
        span = max(frames) - min(frames) + 1     # frames the tracker *could* have produced
        tracked = len(set(frames))
        nz = sum(1 for nw, w in wing_of.values() if nw > 0)
        print("frame range: %d..%d  (span %d)  tracked=%d  frames-with-wings=%d/%d"
              % (min(frames), max(frames), span, tracked, nz, len(wing_of)))
        drop_pct = 100.0 * dropped / span if span else 0.0
        maxrun = max((g[2] for g in gaps), default=0)
        if args.cmd == "pop-front-track":
            print(">>> DROPPED frames (tracker skipped): %d of %d  (%.1f%%)   max run dropped: %d"
                  % (dropped, span, drop_pct, maxrun))
            print("    (drain-in-order, so this is the true tracker drop count -- as long as you")
            print("     polled fast enough that the queue never overflowed)")
        elif args.cmd == "get-last-clear-track":
            print(">>> frames DROPPED by grab-newest-and-clear: %d of %d  (%.1f%%)   max run dropped: %d"
                  % (dropped, span, drop_pct, maxrun))
            print("    (each poll keeps the newest and clears the queue; this is every produced")
            print("     frame the consumer never saw = queue-cleared + tracker-dropped combined.")
            print("     pass -o on BIAS and diff against the trajectory file to split the two.)")
        else:
            print(">>> frames skipped between polls: %d  (newest-only; mixes drops + poll rate;"
                  % dropped)
            print("    use --cmd pop-front-track or get-last-clear-track for a drop count)")
        if gaps:
            gaps.sort(key=lambda g: -g[2])
            print("largest gaps (from -> to, skipped, nwings/|wing| at 'from'):")
            for a, b, sk in gaps[:8]:
                nw, w = wing_of.get(a, (0, 0.0))
                print("  %7d -> %-7d  skipped %-5d  nwings=%d  |wing|=%.3f" % (a, b, sk, nw, w))
    if frames:
        print_capture_time_summary(cts, len(frames), args.cmd)


def print_capture_time_summary(cts, n_with_data, cmd):
    """Summarize the (frame, capture_time) pairs of the non-empty replies, in poll order."""
    if not cts:
        print(">>> capture_time MISSING in all %d replies with data: this BIAS build does not"
              % n_with_data)
        print("    send capture times (older than 0.79.0)")
        return
    print("capture_time: present in %d of %d replies with data" % (len(cts), n_with_data))
    n_back = n_repeat = 0
    intervals = []              # ms, consecutive replies with increasing capture_time
    cam = []                    # ms per frame, of those with a frame difference >= 1
    for (fa, ta), (fb, tb) in zip(cts, cts[1:]):
        if tb < ta:
            n_back += 1
            continue
        if tb == ta:
            n_repeat += 1
            continue
        intervals.append(1e3 * (tb - ta))
        if fb - fa >= 1:
            cam.append(1e3 * (tb - ta) / (fb - fa))
    print("capture_time backwards: %d  repeated: %d" % (n_back, n_repeat))
    if cmd == "pop-back-track":
        print("    (possible with pop-back-track: it pops the newest track and leaves the older")
        print("     ones queued, and a later poll returns one of them when no newer track has")
        print("     arrived. Use pop-front-track for an in-order check.")
    else:
        print("    (should both be 0 within one capture.")
    print("     Those pairs are left out of the statistics below.)")
    if intervals:
        intervals.sort()
        print("capture_time interval between replies ms: min=%.3f  median=%.3f  p95=%.3f  "
              "p99=%.3f  max=%.3f  (n=%d)"
              % (intervals[0], statistics.median(intervals), _pct(intervals, .95),
                 _pct(intervals, .99), intervals[-1], len(intervals)))
    if cam:
        cam.sort()
        med = statistics.median(cam)
        sd = statistics.stdev(cam) if len(cam) >= 2 else 0.0
        print(">>> camera interval ms (capture_time diff / frame diff): median=%.3f  std=%.3f  "
              "p5=%.3f  p95=%.3f  (n=%d, median %.1f Hz)"
              % (med, sd, _pct(cam, .05), _pct(cam, .95), len(cam), 1e3 / med if med > 0 else 0.0))


if __name__ == "__main__":
    main()
