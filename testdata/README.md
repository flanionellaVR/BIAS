# FlyTrack smoke test data

This folder holds the input and the expected output of the FlyTrack smoke test. The test runs `test_gui --batch` on a short recorded video and compares the trajectory it writes against a reference trajectory from a known-good Windows run.

## Files

| File | What it is |
|---|---|
| `sample.avi` | Short recording of a single fly, a few MB, committed directly (no Git LFS). |
| `background.png` | Background image for `sample.avi`, as FlyTrack loads it in tracking mode. |
| `flytrack_config.json` | BIAS configuration for the run. All paths in it are relative (see below). |
| `trx_reference.json` | Trajectory from a known-good Windows run of `sample.avi` with this configuration. |
| `compare_trx.py` | Compares a trajectory against `trx_reference.json`. Python 3, standard library only. |

> **Status:** `sample.avi`, `background.png` and `trx_reference.json` are not committed yet. Until all three are here, CMake prints `FlyTrack smoke test: not registered` and `ctest` has no smoke test. Follow [Producing the test data](#producing-the-test-data) to add them.

## Provenance of `trx_reference.json`

Fill this in whenever the reference is regenerated:

| | |
|---|---|
| BIAS commit | _not produced yet_ |
| Operating system | _not produced yet_ |
| OpenCV version | _not produced yet_ |
| Qt version | _not produced yet_ |
| Source of `sample.avi` | _not produced yet_ |
| Command | `test_gui.exe --batch -i sample.avi -c flytrack_config.json -o trx_reference.json`, run from this folder |

## Running the test

The smoke test is registered when CMake is configured with `with_tests`, `with_qt_gui` and `with_video_backend` (all `ON` by default), the files above are present, and a Python 3 interpreter is found. CMake reports the outcome in a `FlyTrack smoke test:` status line.

From the build directory:

```bash
ctest -L smoke --output-on-failure              # Ninja or Makefiles
ctest -C Release -L smoke --output-on-failure   # Visual Studio (multi-config) builds
```

The test runs as three steps, so a failure shows which one broke:

| Test | Step |
|---|---|
| `flytrack_smoke_clean` | Removes the trajectory left by an earlier run, so a stale file can never pass. |
| `flytrack_smoke_run` | Runs `test_gui --batch -i sample.avi -c flytrack_config.json -o <build>/src/test/flytrack_smoke_trx.json` in this folder, with `QT_QPA_PLATFORM=offscreen` and a timeout. |
| `flytrack_smoke_compare` | Runs `compare_trx.py` on that trajectory and `trx_reference.json`. It only runs if the previous step passed. |

Two CMake cache variables tune the test:

| Variable | Default | Meaning |
|---|---|---|
| `flytrack_smoke_tolerance` | `0.01` | Largest allowed per-frame difference from the reference: pixels for `x`, `y`, `a`, `b`, and radians for `theta`. |
| `flytrack_smoke_timeout` | `300` | Seconds before `ctest` kills the `test_gui --batch` run. |

CMake checks for the data files only when it configures. After adding or removing files here, run `cmake` on the build directory again before `ctest`.

### Running the steps by hand

From this folder, with `test_gui` from your build:

```bash
QT_QPA_PLATFORM=offscreen test_gui --batch -i sample.avi -c flytrack_config.json -o trx.json
python compare_trx.py trx.json trx_reference.json --tolerance 0.01
```

In PowerShell, set the variable first with `$env:QT_QPA_PLATFORM = "offscreen"`. On Windows the variable is optional, since the desktop is always available.

`compare_trx.py` exits with `0` when the trajectories match and `1` when they differ or a file cannot be read. It checks:

- the number of frames, taken from the reference, not from the video: the image grabber skips its first frames at startup (`numStartUpSkip_` in `src/gui/image_grabber.cpp`), so a trajectory has fewer frames than the video;
- that the frame numbers agree, entry by entry;
- that `x`, `y`, `a`, `b` and `theta` of every frame lie within the tolerance. `theta` is compared modulo 2π, and a `nan` in both files counts as a match.

It ignores all other fields, including `timestamp` and the wing fields. On a mismatch it lists the first differing frames (`--max-report`, default 10). Use `--theta-tolerance` to give `theta` its own tolerance. On success it prints the largest difference per field, which helps when choosing a tolerance.

## Relative paths in `flytrack_config.json`

FlyTrack resolves the background image path (`plugin.config.bgEst.bgImageFilePath`) against the **working directory**, not against the folder of the configuration file. That is why the test runs `test_gui` from this folder, and why the configuration says `"background.png"` rather than an absolute path.

The configuration only sets the sections the test needs: the capture timer and the HTTP server are switched off, and the FlyTrack plugin is configured. BIAS keeps its defaults for every section that is missing (camera, logging, display, configuration file), so the file carries none of the machine-specific absolute paths that a configuration saved from the GUI contains. With `-o` on the command line, FlyTrack writes the trajectory even though logging is off, and no video is recorded.

The FlyTrack values in the file are the defaults, with the region of interest switched off (`"roiType": "NONE"`), so the whole frame is tracked. Adjust them to the sample video before producing the reference, as described below.

## Producing the test data

This needs a Windows machine with a working BIAS build of the commit you want as the baseline, including the `--batch` flag.

1. **Pick the video.** Take a recording of a single fly that FlyTrack tracks well, in a format OpenCV can read (AVI or MP4, not `.ufmf`). Cut it to a few seconds so that the file stays at a few MB. Copying the stream keeps the pixels exactly as recorded, but only cuts cleanly at a keyframe:

   ```powershell
   ffmpeg -i recording.avi -t 10 -c copy -an sample.avi
   ```

   If that file is too large, or does not start cleanly, re-encode it instead. H.264 decoding is bit-exact by specification, so other platforms decode the same frames:

   ```powershell
   ffmpeg -ss 0 -i recording.avi -t 10 -c:v libx264 -crf 18 -bf 0 -pix_fmt yuv420p -an sample.avi
   ```

   Check that BIAS opens the result (`test_gui.exe -i sample.avi`), then save it here as `sample.avi`.
2. **Make the background image.** If you cut with `-c copy`, you can reuse the background image of the original recording. After re-encoding, compute it from `sample.avi` instead, because the encoder shifts the grey levels slightly. To compute it in the BIAS GUI: load `sample.avi` with `-i`, enable FlyTrack, select **Compute Background Image**, choose this folder's `background.png` as **Bkgd Image Path** and run capture to the end. FlyTrack adds one frame to the median every **N. Frames Skip** frames (default 500), so lower it for a short clip, for example to 10 for a clip of 1000 frames, and make sure the fly moves enough that the median removes it. Save the result here as `background.png`.
3. **Tune the configuration.** In the GUI, with `sample.avi` loaded, set the FlyTrack parameters that suit this video (threshold, fly darker or brighter than background, region of interest). Copy the values into the `plugin.config` section of `flytrack_config.json`, keep `bgImageFilePath` as `"background.png"`, and keep `computeBgMode` as `false`.
4. **Produce the reference.** In a terminal in this folder:

   ```powershell
   <build>\Release\test_gui.exe --batch -i sample.avi -c flytrack_config.json -o trx_reference.json
   echo $LASTEXITCODE   # must be 0
   ```

   Check that the trajectory looks right, because the test can only be as good as its reference. For example, draw it onto the video with `python ..\src\plugin\flytrack\make_overlay_video.py sample.avi trx_reference.json overlay.mp4` and watch `overlay.mp4` (do not commit it).
5. **Record the provenance** in the table above: commit (`git rev-parse --short HEAD`), Windows version, OpenCV version (**Help → About** in BIAS), Qt version (`qmake -query QT_VERSION` of the Qt you built against), and where the video comes from.
6. **Register and run the test.** Re-run `cmake` in the build directory; it should print `FlyTrack smoke test: registered`. Then run `ctest -C Release -L smoke --output-on-failure` and check that it passes.
7. **Check that the test can fail.** Change one `x` value in `trx_reference.json` by more than the tolerance (for example by `1.0`), run `ctest` again and check that `flytrack_smoke_compare` fails and names that frame. Then undo the change.

Regenerate the reference (steps 4 to 7) whenever a change to FlyTrack is meant to change the trajectory, and say so in the pull request.

## Windows notes

- **OpenCV DLLs.** `test_gui.exe` only starts if the OpenCV DLLs are next to it or on `PATH`, including `opencv_videoio_ffmpeg*.dll`, without which no video can be opened (compare issue #17). With `with_tests` on, the build copies every `opencv_*.dll` from the folder of the OpenCV libraries it links against next to `test_gui.exe` (target `copy_opencv_dlls`). If CMake prints `No OpenCV DLLs found for the imported OpenCV targets`, put the OpenCV `bin` folder on `PATH` before running `ctest`.
- **Offscreen platform plugin.** The run step sets `QT_QPA_PLATFORM=offscreen`, so it also works without a desktop. If it fails with `could not find the Qt platform plugin "offscreen"`, copy `qoffscreen.dll` from `<Qt>\plugins\platforms` into the `platforms` folder next to `test_gui.exe`.
