"""Present both complete v2 recordings and two short, labeled simulation replays."""
from pathlib import Path
from fractions import Fraction
import hashlib
import json
import subprocess

from PIL import Image, ImageDraw, ImageFont

P = Path(__file__).resolve().parent
ROOT = P.parents[1]
BUILD = P / "build"
OUT = P / "SCONE_ICRA2027_Video.mp4"
BUILD.mkdir(exist_ok=True)
FONT = "/System/Library/Fonts/Supplemental/Arial.ttf"
BOLD = "/System/Library/Fonts/Supplemental/Arial Bold.ttf"
BG = "#101d29"
FPS = 30


def run(args):
    subprocess.run(args, check=True)


def probe(path):
    return json.loads(subprocess.check_output([
        "ffprobe", "-v", "error", "-show_streams", "-show_format", "-of", "json", str(path)
    ]))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def text(draw, xy, message, size, color="white", bold=False):
    font = ImageFont.truetype(BOLD if bold else FONT, size)
    box = draw.textbbox(xy, message, font=font)
    assert box[0] >= 0 and box[2] <= 1248 and box[3] <= 710, (message, box)
    draw.text(xy, message, font=font, fill=color)


segments = []


def clip(name, source, title, subtitle, footer, duration=None):
    source = ROOT / source
    meta = probe(source)
    stream = next(s for s in meta["streams"] if s["codec_type"] == "video")
    full = duration is None
    duration = float(stream["duration"]) if full else duration
    # Preserve every frame of each 30 fps physical source. Simulations are 25 fps.
    frames = int(stream["nb_frames"]) if full and Fraction(stream["r_frame_rate"]) == FPS else round(duration * FPS)
    overlay = Image.new("RGBA", (1280, 720), (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)
    d.rectangle((0, 0, 1280, 71), fill=BG)
    d.rectangle((0, 672, 1280, 720), fill=BG)
    d.rectangle((32, 16, 38, 57), fill="#64d2c0" if full and "v2" in name else "#e9b765")
    text(d, (52, 8), title, 29, bold=True)
    text(d, (52, 43), subtitle, 20, "#c1d1dd")
    text(d, (32, 686), footer, 21, "#c1d1dd")
    png = BUILD / f"{name}_overlay.png"
    overlay.save(png)
    target = BUILD / f"{name}.mp4"
    # Fit, never crop: hands, cables, landings, and the original field of view remain visible.
    vf = ("[0:v]setpts=PTS-STARTPTS,fps=30,"
          "scale=1280:600:force_original_aspect_ratio=decrease:force_divisible_by=2,"
          "setsar=1,pad=1280:720:(ow-iw)/2:72+(600-ih)/2:color=0x101d29[base];"
          "[base][1:v]overlay=0:0:shortest=1[out]")
    run(["ffmpeg", "-y", "-v", "error", "-i", str(source), "-loop", "1", "-i", str(png),
         "-filter_complex", vf, "-map", "[out]", "-frames:v", str(frames),
         "-c:v", "libx264", "-preset", "fast", "-crf", "18", "-threads", "4",
         "-pix_fmt", "yuv420p", "-r", str(FPS), "-an", "-map_metadata", "-1", str(target)])
    segments.append({"name": name, "file": str(target.relative_to(P)),
                     "kind": "physical_v2" if "v2" in name else "simulation",
                     "source": str(source.relative_to(ROOT)), "source_sha256": sha(source),
                     "source_start_s": 0, "source_end_s": duration, "complete_source": full,
                     "source_frames": int(stream["nb_frames"]), "output_frames": frames,
                     "playback_speed": 1, "spatial_crop": False,
                     "title": title, "subtitle": subtitle, "footer": footer})
    print("Rendered", name, flush=True)


def card(name, eyebrow, title, lines, seconds):
    im = Image.new("RGB", (1280, 720), BG)
    d = ImageDraw.Draw(im)
    d.rectangle((72, 147, 144, 153), fill="#64d2c0")
    text(d, (72, 93), eyebrow, 23, "#64d2c0", True)
    text(d, (72, 182), title, 42, bold=True)
    for i, line in enumerate(lines):
        text(d, (72, 285 + i * 67), line, 27, "#c1d1dd")
    png = BUILD / f"{name}.png"
    im.save(png)
    target = BUILD / f"{name}.mp4"
    run(["ffmpeg", "-y", "-v", "error", "-loop", "1", "-i", str(png),
         "-frames:v", str(seconds * FPS), "-r", str(FPS), "-c:v", "libx264",
         "-preset", "fast", "-crf", "18", "-threads", "4", "-pix_fmt", "yuv420p",
         "-an", str(target)])
    segments.append({"name": name, "kind": "explanation", "file": str(target.relative_to(P)),
                     "output_frames": seconds * FPS, "eyebrow": eyebrow, "title": title, "lines": lines})


clip("v2_floor", "archive/videos/SCONEv2.mp4",
     "SCONE v2 | Physical demonstration 1/2",
     "Floor motion and posture changes | Complete source clip | 1x",
     "Archival hardware footage; separate from the paper's controlled simulation experiments.")
clip("v2_stairs", "archive/videos/SCONEv2_stairs.mp4",
     "SCONE v2 | Physical demonstration 2/2",
     "Stair motion | Complete source clip | 1x",
     "Archival hardware footage; separate from the paper's controlled simulation experiments.")
card("simulation_intro", "SELECTED SIMULATION REPLAYS", "Two controls illustrated", [
     "1. Periodic flat-ground rolling / stepping",
     "2. Fixed-posture ascent of 150 mm stairs",
     "These are separate runs, not a continuous floor-to-stair sequence."
], 4)
clip("simulation_flat", "archive/ICRA_2027_revision4_20260913/media/joint_P.mp4",
     "Simulation 1/2 | Periodic rolling / stepping",
     "Flat-ground periodic reindexing | First 10 s of the recorded replay | 1x",
     "Illustrates the simpler flat policy evaluated in the manuscript.", duration=10)
clip("simulation_stairs", "archive/ICRA_2027_revision5_20260913/media/stair_45.mp4",
     "Simulation 2/2 | Fixed-posture stair ascent",
     "150 mm risers | 350 mm treads | Common distal phase | 2 ms physics step | 1x",
     "Starts after posture acquisition; includes rear-leg clearance and the supported halt.")
card("scope", "INTERPRETATION", "What the clips establish", [
     "The complete real recordings show the SCONE v2 hardware in motion.",
     "They do not validate the exact controllers used in the simulation study.",
     "Simulation limits: 200 mm transfer fails; wheel results depend on timestep."
], 7)

cursor = 0
for seg in segments:
    seg["output_start_frame"] = cursor
    seg["output_start_s"] = cursor / FPS
    cursor += seg["output_frames"]
    seg["output_end_s"] = cursor / FPS
listing = BUILD / "concat.txt"
listing.write_text("".join(f"file '{P / s['file']}'\n" for s in segments))
common = ["ffmpeg", "-y", "-v", "error", "-f", "concat", "-safe", "0", "-i", str(listing),
          "-c:v", "libx264", "-preset", "slow", "-b:v", "1450k", "-threads", "4",
          "-pix_fmt", "yuv420p", "-r", str(FPS), "-an", "-map_metadata", "-1",
          "-passlogfile", str(BUILD / "encode")]
run(common + ["-pass", "1", "-f", "null", "/dev/null"])
run(common + ["-pass", "2", "-movflags", "+faststart", str(OUT)])
meta = probe(OUT)
s = next(x for x in meta["streams"] if x["codec_type"] == "video")
assert int(s["nb_frames"]) == cursor
assert float(meta["format"]["duration"]) <= 180
assert int(meta["format"]["size"]) <= 20_000_000
assert s["height"] >= 480 and Fraction(s["r_frame_rate"]) >= 20 and s["field_order"] == "progressive"
manifest = {"edited_on": "2026-09-20", "fps": FPS, "output": OUT.name,
            "sha256": sha(OUT), "duration_s": float(meta["format"]["duration"]),
            "size_bytes": int(meta["format"]["size"]), "width": s["width"], "height": s["height"],
            "scan": s["field_order"], "segments": segments,
            "physical_v2_source_count": 2, "physical_duration_s": 69.6,
            "simulation_clip_count": 2,
            "removed_from_previous_edit": [
                {"source": "stair_47.mp4", "reason": "20 s closed-wheel comparison removed from short companion edit; result remains in the submitted paper."},
                {"source": "stair_75.mp4", "reason": "20 s incomplete 200 mm transfer removed from short companion edit; limitation remains on the closing card."}],
            "source_originals_modified": False, "submitted_paper_modified": False}
(P / "edit_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps({k:v for k,v in manifest.items() if k != "segments"}, indent=2), flush=True)
