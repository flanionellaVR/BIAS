#!/usr/bin/env python3
"""
Poll BIAS's FlyTrack tracking endpoint as hard as possible for a while, and
report how many frames were dropped or otherwise irregular, plus request
latency and hangs.

This talks to BIAS directly over HTTP, the same way BIASBridge's
``BIASBridge._request`` does, but without any of the Bridge/Scene/Unity
machinery. It only needs ``requests``.

The run has two back-to-back phases:

  Phase 1 -- max speed: poll with no delay at all, to find the fastest rate
             this connection can sustain, and whether BIAS ever skips
             frames under that load.
  Phase 2 -- matched FPS: pace polling to BIAS's own reported camera FPS
             (falls back to 120 FPS if BIAS doesn't report one), matching
             how BIASBridge's ``run()`` polls in a real session.

Each phase gets its own report, since they test different things.

Keep-alive: by default every poll appends ``keep-alive=1`` so BIAS leaves the
TCP connection open and the next poll reuses it. Without that, BIAS closes
the connection after each response and the poll rate is capped by TCP setup
(~50 Hz in practice). ``--no-keep-alive`` reverts to one connection per poll,
for comparison with clients that don't opt in.

Which command to poll (``--cmd``):

  pop-back-track       newest entry, popped (what BIASBridge polls). Frame
                       gaps mix "polled too slowly" with real tracker drops.
  pop-front-track      oldest entry, popped. Drains the queue IN ORDER, so
                       frame gaps are genuine tracker-dropped frames (as
                       long as you poll fast enough that the queue never
                       overflows). Best for measuring whether tracking
                       itself drops frames.
  get-last-clear-track newest entry, clears the queue. Gaps are every
                       produced frame the consumer never saw: cleared plus
                       tracker-dropped combined.

How to read "frame": a poll that finds the queue empty is "nothing new yet",
not a drop. A drop is a jump in the frame number (frame 10, then frame 15):
BIAS moved on without us ever seeing frames 11-14. What a jump *means*
depends on ``--cmd`` as described above.

Usage
-----
    python tools/stress_test_bias.py --track-guid 23577160 --duration 300
    python tools/stress_test_bias.py --track-host http://127.0.0.1:5020 --duration 3600 --csv stress.csv
    python tools/stress_test_bias.py --track-host http://127.0.0.1:5010 --cmd pop-front-track --duration 20

``--parallel N`` (default 1) keeps up to N HTTP requests in flight at once
instead of one at a time, to check whether reading several packets
concurrently actually raises throughput -- or just makes BIAS's `frame`
counter jump around more, since every ``--cmd`` mutates shared state on
each call rather than just reading it.

With ``--csv stress.csv`` you get four files: a raw per-poll log and a
matching markdown report for each phase (``stress_phase1.csv``/``.md``,
``stress_phase2.csv``/``.md``).
"""

import argparse
import concurrent.futures
import csv
import json
import statistics
import time
from pathlib import Path

import requests

FALLBACK_FPS = 120.0

TRACK_COMMANDS = ("pop-back-track", "pop-front-track", "get-last-clear-track")

# BIAS's RtnStatus message when the FlyTrack ellipse queue has nothing to hand out.
QUEUE_EMPTY_MESSAGE = "Ellipse queue empty"


def find_camera(guid, timeout=0.5):
    """Same port-guessing scheme as BIASBridge's ``_find_camera``."""
    for port in range(5000, 5100, 10):
        host = f"http://127.0.0.1:{port}"
        try:
            response = requests.get(host, params="get-camera-guid", timeout=timeout)
            if response.ok:
                r_j = response.json()[0]
                if "value" in r_j and int(r_j["value"]) == guid:
                    return host
        except Exception:
            pass
    raise RuntimeError(f"Camera with ID {guid} not found")


def build_track_params(cmd, keep_alive, last=False):
    """
    Query parameters for one FlyTrack poll.

    ``keep-alive=1`` asks BIAS to leave the TCP connection open after the
    response (it only honors this for plugin-cmd requests). On the final
    poll ``last=True`` sends ``keep-alive=0`` so the server closes the
    connection cleanly instead of waiting for its idle timeout.
    """
    payload = {"plugin": "FlyTrack", "cmd": cmd}
    payload_json = json.dumps(payload).replace(" ", "")
    params = {"plugin-cmd": payload_json}
    if keep_alive:
        params["keep-alive"] = "0" if last else "1"
    return params


def get_fps(session, host, timeout):
    try:
        response = session.get(host, params="get-frames-per-sec", timeout=timeout)
        if not response.ok:
            return None
        response_json = response.json()
        if isinstance(response_json, list):
            response_json = response_json[0]
        if response_json.get("success") and "value" in response_json:
            return float(response_json["value"])
    except Exception:
        pass
    return None


def poll_once(session, host, params, timeout):
    """
    One HTTP round trip to a FlyTrack track command.

    Returns
    -------
    latency_s : float
        Wall-clock round-trip time for this request.
    val : dict or None
        Decoded ``value`` payload (frame/x/y/theta/timestamp), or None on
        error or when the queue was empty.
    error : str or None
        Short error tag, or None on success. ``"queue-empty"`` is not a
        failure: BIAS simply had nothing new for us yet.
    """
    t0 = time.perf_counter()
    try:
        response = session.get(host, params=params, timeout=timeout)
    except requests.RequestException as exc:
        return time.perf_counter() - t0, None, f"request-error:{type(exc).__name__}"
    latency = time.perf_counter() - t0

    if not response.ok:
        return latency, None, f"http-{response.status_code}"

    try:
        response_json = response.json()
    except json.JSONDecodeError:
        return latency, None, "bad-json"

    if isinstance(response_json, list):
        if not response_json:
            return latency, None, "empty-list"
        response_json = response_json[0]

    if not (response_json.get("success") and "value" in response_json):
        if response_json.get("message") == QUEUE_EMPTY_MESSAGE:
            return latency, None, "queue-empty"
        return latency, None, "not-success"

    val = response_json["value"]
    if isinstance(val, str):
        if len(val) == 0:
            val = {}
        else:
            try:
                val = json.loads(val)
            except json.JSONDecodeError:
                return latency, None, "bad-value-json"

    return latency, val, None


def percentile(data, pct):
    if not data:
        return 0.0
    data = sorted(data)
    idx = (pct / 100) * (len(data) - 1)
    lo = int(idx)
    if idx == lo:
        return data[lo]
    frac = idx - lo
    return data[lo] + (data[lo + 1] - data[lo]) * frac


def new_stats():
    return {
        "latencies": [],
        "poll_count": 0,
        "error_count": 0,
        "errors_by_type": {},
        "timeout_count": 0,
        "queue_empty_count": 0,
        "no_detection_count": 0,
        "duplicate_count": 0,
        "new_frame_count": 0,
        "dropped_frame_total": 0,
        "gap_sizes": [],
        "backwards_count": 0,
        "stall_count": 0,
        "max_stall_s": 0.0,
        "first_new_frame_num": None,
        "first_new_frame_local_t": None,
        "first_new_frame_bias_ts": None,
        "last_new_frame_num": None,
        "last_new_frame_local_t": None,
        "last_new_frame_bias_ts": None,
        "first_backward_frame_num": None,
        "first_backward_local_t": None,
        "last_backward_frame_num": None,
        "last_backward_local_t": None,
    }


def record_response(stats, carry, csv_writer, poll_idx, wall_time, now, latency, val, error, stall_threshold_s):
    """Classify one poll's response (new/repeat/empty/backward/error) and update stats + carry in place."""
    stats["latencies"].append(latency)
    frame = x = y = theta = bias_ts = gap = None
    is_new_frame = False

    if error == "queue-empty":
        stats["queue_empty_count"] += 1
    elif error:
        stats["error_count"] += 1
        stats["errors_by_type"][error] = stats["errors_by_type"].get(error, 0) + 1
        if error.startswith("request-error:") and "Timeout" in error:
            stats["timeout_count"] += 1
    elif val is None or "frame" not in val:
        stats["no_detection_count"] += 1
    else:
        frame = val["frame"]
        x, y, theta = val.get("x"), val.get("y"), val.get("theta")
        bias_ts = val.get("timestamp")

        last_frame = carry["last_frame"]
        if last_frame is not None and frame == last_frame:
            stats["duplicate_count"] += 1
        elif last_frame is not None and frame < last_frame:
            # Doesn't move carry["last_frame"] -- a backward blip must not
            # poison the gap calculation for the next real forward frame.
            stats["backwards_count"] += 1
            if stats["first_backward_frame_num"] is None:
                stats["first_backward_frame_num"] = frame
                stats["first_backward_local_t"] = now
            stats["last_backward_frame_num"] = frame
            stats["last_backward_local_t"] = now
        else:
            is_new_frame = True
            stats["new_frame_count"] += 1
            if last_frame is not None:
                gap = frame - last_frame - 1
                if gap > 0:
                    stats["dropped_frame_total"] += gap
                    stats["gap_sizes"].append(gap)
            if stall_threshold_s and carry["last_new_frame_time"] is not None:
                stall_s = now - carry["last_new_frame_time"]
                if stall_s > stall_threshold_s:
                    stats["stall_count"] += 1
                    stats["max_stall_s"] = max(stats["max_stall_s"], stall_s)
            carry["last_new_frame_time"] = now
            carry["last_frame"] = frame

            if stats["first_new_frame_num"] is None:
                stats["first_new_frame_num"] = frame
                stats["first_new_frame_local_t"] = now
                stats["first_new_frame_bias_ts"] = bias_ts
            stats["last_new_frame_num"] = frame
            stats["last_new_frame_local_t"] = now
            stats["last_new_frame_bias_ts"] = bias_ts

        if x == 0 and y == 0:
            stats["no_detection_count"] += 1

    if csv_writer:
        csv_writer.writerow(
            [
                poll_idx, f"{wall_time:.6f}", f"{latency * 1000:.3f}",
                frame, is_new_frame, gap, x, y, theta, bias_ts, error or "",
            ]
        )


def print_progress_line(phase_label, elapsed, stats, current_frame, progress_last_frame, progress_last_time, now):
    if (
        progress_last_frame is not None
        and current_frame is not None
        and now > progress_last_time
    ):
        counter_rate_str = f"{(current_frame - progress_last_frame) / (now - progress_last_time):.0f}/s"
    else:
        counter_rate_str = "n/a"
    print(
        f"[{phase_label} {elapsed:7.1f}s] polls={stats['poll_count']} "
        f"new_frames={stats['new_frame_count']} "
        f"dropped={stats['dropped_frame_total']} "
        f"dupes={stats['duplicate_count']} "
        f"empty={stats['queue_empty_count']} "
        f"backwards={stats['backwards_count']} "
        f"no_detect={stats['no_detection_count']} "
        f"errors={stats['error_count']} "
        f"poll_rate={stats['poll_count'] / elapsed:.1f}/s "
        f"frame_counter_rate={counter_rate_str}"
    )


def run_phase(
    host,
    cmd,
    keep_alive,
    phase_duration,
    pacing_interval,
    timeout,
    stall_threshold_s,
    progress_interval,
    phase_label,
    carry,
    csv_writer,
    parallel=1,
):
    """
    Poll BIAS for ``phase_duration`` seconds with up to ``parallel`` requests
    in flight at once (a true sliding window, not batches): as soon as a
    request completes, another takes its place, one at a time.

    ``pacing_interval`` is None for max-throughput: refill the window the
    instant a slot frees up, no delay at all.

    ``pacing_interval`` is a float number of seconds for the matched-FPS
    phase: a new request is due at every ``pacing_interval`` tick, still
    bounded by the ``parallel`` window. If earlier requests are still
    pending when a later tick is due (latency > pacing_interval), the new
    request still goes out on schedule as long as a slot is free -- e.g.
    with ``parallel=4`` we may still be waiting on frames 1-3 when frame 4's
    request is sent. If the window is full (depleted) when a tick is due,
    that request waits for a slot instead of being dropped, so we fall
    behind schedule rather than skip data.

    ``carry`` is a dict with ``last_frame``/``last_new_frame_time`` state
    that is threaded through across phases so frame-counter continuity is
    preserved at the phase boundary; it is mutated in place and returned.

    Each of the ``parallel`` slots gets its own ``requests.Session`` (a
    single Session isn't guaranteed thread-safe), reused across that slot's
    requests. With ``keep_alive`` the Session's pooled connection is actually
    reused, because BIAS answers ``Connection: keep-alive``; without it BIAS
    closes the socket after every response and the Session reconnects each
    time.
    """
    params = build_track_params(cmd, keep_alive)
    stats = new_stats()
    t_start = time.perf_counter()
    t_end = t_start + phase_duration
    last_progress = t_start
    progress_last_frame = carry["last_frame"]
    progress_last_time = t_start
    next_submit_time = t_start

    sessions = [requests.Session() for _ in range(parallel)]
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=parallel) as executor:
            in_flight = set()
            next_session = 0

            def submit_one():
                nonlocal next_session
                fut = executor.submit(poll_once, sessions[next_session % parallel], host, params, timeout)
                next_session += 1
                in_flight.add(fut)

            while True:
                now = time.perf_counter()
                still_running = now < t_end

                if still_running:
                    if pacing_interval is None:
                        while len(in_flight) < parallel:
                            submit_one()
                    else:
                        while len(in_flight) < parallel and next_submit_time <= now:
                            submit_one()
                            next_submit_time += pacing_interval

                if not in_flight:
                    if not still_running:
                        break
                    sleep_for = 0.001 if pacing_interval is None else max(0.0, min(next_submit_time - now, 0.005))
                    time.sleep(sleep_for)
                    continue

                wait_timeout = 0.02
                if still_running and pacing_interval is not None and len(in_flight) < parallel:
                    wait_timeout = max(0.0, min(wait_timeout, next_submit_time - now))
                done, _ = concurrent.futures.wait(
                    in_flight, timeout=wait_timeout, return_when=concurrent.futures.FIRST_COMPLETED
                )
                for fut in done:
                    in_flight.discard(fut)
                    latency, val, error = fut.result()
                    stats["poll_count"] += 1
                    wall_time = time.time()
                    completed_at = time.perf_counter()
                    record_response(
                        stats, carry, csv_writer, stats["poll_count"], wall_time, completed_at,
                        latency, val, error, stall_threshold_s,
                    )

                    if completed_at - last_progress >= progress_interval:
                        elapsed = completed_at - t_start
                        current_frame = carry["last_frame"]
                        print_progress_line(
                            phase_label, elapsed, stats, current_frame,
                            progress_last_frame, progress_last_time, completed_at,
                        )
                        progress_last_frame = current_frame
                        progress_last_time = completed_at
                        last_progress = completed_at

                if not still_running and not in_flight:
                    break
    finally:
        if keep_alive:
            # Tell BIAS we're done so it closes each kept-open connection now
            # rather than after its idle timeout. Not counted in the stats.
            close_params = build_track_params(cmd, keep_alive, last=True)
            for sess in sessions:
                try:
                    sess.get(host, params=close_params, timeout=timeout)
                except requests.RequestException:
                    pass
        for sess in sessions:
            sess.close()

    elapsed = time.perf_counter() - t_start
    return stats, elapsed


def build_markdown_report(
    title, host, cmd, keep_alive, camera_fps, pacing_description, stats, elapsed, expected_interval, parallel=1
):
    latencies_ms = [l * 1000 for l in stats["latencies"]]
    gaps = stats["gap_sizes"]
    lines = []
    lines.append(f"# BIAS Poll Stress Test Report -- {title}")
    lines.append("")
    lines.append("## Setup")
    lines.append(f"- **BIAS host:** {host}")
    lines.append(f"- **Command:** `{cmd}`")
    lines.append(f"- **HTTP keep-alive:** {'on (one connection reused)' if keep_alive else 'off (new connection per poll)'}")
    lines.append(f"- **BIAS-reported camera FPS:** {camera_fps}")
    lines.append(f"- **Polling mode:** {pacing_description}")
    lines.append(f"- **Concurrent connections:** {parallel}")
    lines.append(f"- **Duration:** {elapsed:.1f} s")
    lines.append("")
    if parallel > 1:
        lines.append(
            f"> This run used more than one connection at once. `{cmd}` "
            "changes BIAS's internal state on every call, so more connections may "
            "not mean more distinct frames read -- compare against a `--parallel 1` "
            "run on the same host before trusting a speed-up."
        )
        lines.append("")
    lines.append("## Speed")
    lines.append(f"- **Total polls:** {stats['poll_count']}")
    lines.append(f"- **Polls per second:** {stats['poll_count'] / elapsed:.1f}")
    if expected_interval:
        unique_rate = stats["new_frame_count"] / elapsed
        lines.append(
            f"- **New frames per second:** {unique_rate:.2f} "
            f"({100 * unique_rate / (1 / expected_interval):.1f}% of camera FPS)"
        )
    lines.append("")
    lines.append("## What We Read")
    lines.append("| Type of read | Count |")
    lines.append("|---|---|")
    lines.append(f"| New frame | {stats['new_frame_count']} |")
    lines.append(f"| Queue empty (BIAS had nothing new yet) | {stats['queue_empty_count']} |")
    lines.append(f"| Repeat (same frame number again) | {stats['duplicate_count']} |")
    lines.append(f"| Went backward (see below) | {stats['backwards_count']} |")
    lines.append(f"| Fly not found (x=0, y=0) | {stats['no_detection_count']} |")
    lines.append(f"| Error | {stats['error_count']} |")
    for err, count in sorted(stats["errors_by_type"].items(), key=lambda kv: -kv[1]):
        lines.append(f"| &nbsp;&nbsp;{err} | {count} |")
    if stats["timeout_count"]:
        lines.append(f"| &nbsp;&nbsp;of which hangs (request timed out) | {stats['timeout_count']} |")
    lines.append("")
    lines.append("## Dropped Frames")
    lines.append(
        "A drop is a jump in BIAS's frame number: we saw frame N, then next time "
        "frame N+5, so frames N+1..N+4 were never seen."
    )
    if cmd == "pop-front-track":
        lines.append(
            "`pop-front-track` drains the queue in order, so these are frames the "
            "tracker itself skipped, as long as we polled fast enough that the queue "
            "never overflowed."
        )
    elif cmd == "get-last-clear-track":
        lines.append(
            "`get-last-clear-track` keeps the newest entry and clears the rest, so this "
            "is every produced frame the consumer never saw: cleared-away plus "
            "tracker-dropped combined. Run BIAS with `-o` and diff against the "
            "trajectory file to split the two."
        )
    else:
        lines.append(
            "`pop-back-track` returns the newest entry only, so these mix frames left "
            "behind in the queue (polled too slowly) with real tracker drops. Use "
            "`--cmd pop-front-track` for a true drop count."
        )
    lines.append(f"- **Frames dropped:** {stats['dropped_frame_total']}")
    lines.append(f"- **Number of jumps:** {len(gaps)}")
    if gaps:
        lines.append(f"- **Biggest single jump:** {max(gaps)} frames")
        lines.append(f"- **Average jump size:** {statistics.mean(gaps):.2f} frames")
        drop_rate = stats["dropped_frame_total"] / max(
            stats["dropped_frame_total"] + stats["new_frame_count"], 1
        )
        lines.append(f"- **Share of frames dropped:** {100 * drop_rate:.3f}%")
    else:
        lines.append("- None -- every new frame followed the last one directly.")
    lines.append("")
    lines.append("## Frame Counter Going Backward")
    lines.append(
        "This checks whether `frame` really counts up by one per tracked pose. "
        "If the rate below is much higher than the camera FPS, it isn't -- and the "
        "\"Dropped Frames\" numbers above don't mean real data loss."
    )
    span = None
    forward_rate = None
    if stats["new_frame_count"] >= 2:
        span = stats["last_new_frame_num"] - stats["first_new_frame_num"]
        local_dt = stats["last_new_frame_local_t"] - stats["first_new_frame_local_t"]
        if local_dt > 0:
            forward_rate = span / local_dt
            lines.append(
                f"- **Forward rate:** {span} counts over {local_dt:.1f} s "
                f"= ~{forward_rate:.1f} counts/s"
            )
            if camera_fps:
                lines.append(f"- That's **{forward_rate / camera_fps:.1f}x** the reported camera FPS.")
    if span is None:
        lines.append("- Not enough distinct frames observed to check this.")
    if stats["backwards_count"]:
        lines.append(f"- The counter went backward on {stats['backwards_count']} reads.")
        backward_span = None
        backward_rate = None
        if (
            stats["first_backward_frame_num"] is not None
            and stats["last_backward_frame_num"] is not None
        ):
            backward_span = stats["last_backward_frame_num"] - stats["first_backward_frame_num"]
            backward_dt = stats["last_backward_local_t"] - stats["first_backward_local_t"]
            if backward_dt > 0:
                backward_rate = backward_span / backward_dt
                lines.append(
                    f"- Those backward reads count *down* steadily too: "
                    f"{backward_span} counts over {backward_dt:.1f} s = ~{backward_rate:.1f} counts/s."
                )
        if (
            forward_rate
            and backward_rate
            and abs(abs(backward_rate) / forward_rate - 1) < 0.5
        ):
            lines.append(
                "- **This is not random noise.** The backward reads fall about as "
                "fast and as steadily as the forward reads climb. That looks like BIAS "
                "is handing back two different counters/queues on alternating calls, "
                "not simply losing data. Since this shows up whether we poll flat-out "
                f"or paced to BIAS's own FPS, it points to BIAS's `{cmd}` "
                "handler, not to how this script polls."
            )
    lines.append("")
    lines.append("## Long Gaps With Nothing New")
    lines.append(f"- **Count:** {stats['stall_count']}")
    lines.append(f"- **Longest:** {stats['max_stall_s']:.3f} s")
    lines.append("")
    lines.append("## Request Latency (round trip per poll)")
    if latencies_ms:
        lines.append("| Stat | ms |")
        lines.append("|---|---|")
        lines.append(f"| mean | {statistics.mean(latencies_ms):.3f} |")
        lines.append(f"| median | {statistics.median(latencies_ms):.3f} |")
        lines.append(f"| p95 | {percentile(latencies_ms, 95):.3f} |")
        lines.append(f"| p99 | {percentile(latencies_ms, 99):.3f} |")
        lines.append(f"| max | {max(latencies_ms):.3f} |")
    return "\n".join(lines) + "\n"


def csv_path_for_phase(base_csv, phase_num):
    base = Path(base_csv)
    suffix = base.suffix or ".csv"
    return base.with_name(f"{base.stem}_phase{phase_num}{suffix}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--track-host", default=None, help='e.g. "http://127.0.0.1:5020"')
    parser.add_argument("--track-guid", type=int, default=None, help="camera GUID to auto-discover")
    parser.add_argument("--duration", type=float, default=300, help="total seconds to run, split evenly across both phases (default 300)")
    parser.add_argument(
        "--cmd",
        default="pop-back-track",
        choices=TRACK_COMMANDS,
        help="FlyTrack command to poll (default pop-back-track; see the module docstring for what each measures)",
    )
    parser.add_argument(
        "--no-keep-alive",
        action="store_true",
        help="open a new TCP connection for every poll instead of asking BIAS to keep one open (default: keep-alive on)",
    )
    parser.add_argument("--timeout", type=float, default=1.0, help="per-request HTTP timeout, s; slower requests count as hangs")
    parser.add_argument(
        "--stall-factor",
        type=float,
        default=5.0,
        help="report a stall when no new frame arrives for stall_factor / fps seconds",
    )
    parser.add_argument("--csv", default=None, help="optional base path; writes <base>_phase1.csv / <base>_phase2.csv (and matching .md reports)")
    parser.add_argument("--progress-interval", type=float, default=10.0, help="seconds between progress lines")
    parser.add_argument(
        "--parallel",
        type=int,
        default=1,
        help="number of HTTP requests to keep in flight at once (default 1 = one at a time)",
    )
    args = parser.parse_args()

    if not args.track_host and not args.track_guid:
        parser.error("Provide --track-host or --track-guid")
    if args.parallel < 1:
        parser.error("--parallel must be at least 1")

    keep_alive = not args.no_keep_alive
    session = requests.Session()
    host = args.track_host or find_camera(args.track_guid)
    print(f"Polling BIAS at {host}  cmd={args.cmd}  keep-alive={'on' if keep_alive else 'off'}")

    fps = get_fps(session, host, args.timeout)
    if fps:
        print(f"BIAS-reported camera FPS: {fps}")
        phase2_fps = fps
    else:
        print(f"BIAS did not report an FPS; phase 2 will pace to the fallback of {FALLBACK_FPS} FPS")
        phase2_fps = FALLBACK_FPS
    expected_interval = 1 / fps if fps else None

    stall_threshold_s = args.stall_factor * expected_interval if expected_interval else None

    half_duration = args.duration / 2
    carry = {"last_frame": None, "last_new_frame_time": None}

    parallel_suffix = f", {args.parallel} concurrent connections" if args.parallel > 1 else ""
    phases = [
        (1, "Phase 1: Max Throughput", None, f"as fast as possible (no pacing){parallel_suffix}"),
        (2, "Phase 2: Matched FPS", 1 / phase2_fps, f"paced to {phase2_fps:.2f} FPS{parallel_suffix}"),
    ]

    try:
        for phase_num, title, pacing_interval, pacing_description in phases:
            csv_file = None
            csv_writer = None
            csv_out_path = None
            if args.csv:
                csv_out_path = csv_path_for_phase(args.csv, phase_num)
                csv_file = open(csv_out_path, "w", newline="")
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(
                    [
                        "poll_idx", "wall_time", "latency_ms", "frame", "is_new_frame",
                        "frame_gap", "x", "y", "theta", "bias_timestamp", "error",
                    ]
                )

            print(f"\n--- Starting {title} ({pacing_description}) for {half_duration:.1f}s ---")
            try:
                stats, elapsed = run_phase(
                    host=host,
                    cmd=args.cmd,
                    keep_alive=keep_alive,
                    phase_duration=half_duration,
                    pacing_interval=pacing_interval,
                    timeout=args.timeout,
                    stall_threshold_s=stall_threshold_s,
                    progress_interval=args.progress_interval,
                    phase_label=f"phase{phase_num}",
                    carry=carry,
                    csv_writer=csv_writer,
                    parallel=args.parallel,
                )
            finally:
                if csv_file:
                    csv_file.close()

            report_md = build_markdown_report(
                title, host, args.cmd, keep_alive, fps, pacing_description, stats, elapsed,
                expected_interval, parallel=args.parallel,
            )
            print("\n" + report_md)

            if args.csv:
                md_path = csv_out_path.with_suffix(".md")
                md_path.write_text(report_md)
                print(f"Wrote {csv_out_path} and {md_path}")
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
