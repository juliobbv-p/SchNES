#!/usr/bin/env python3
"""Build the lean static PGO releases for Apple silicon, Windows ARM64 and Windows x64."""
import argparse
from base64 import b64encode
from concurrent.futures import ThreadPoolExecutor
import ctypes
from datetime import datetime
from hashlib import sha256
from html import unescape
import json
import os
from pathlib import Path
import plistlib
import re
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
from urllib.parse import unquote
from zipfile import ZIP_DEFLATED, ZipFile


def release_readme(source):
    heading = "## Build and run\n"
    start = source.index(heading)
    content = start + len(heading)
    end = content + re.search(r"^#{2,3} ", source[content:], re.MULTILINE).start()
    source = (
        source[:start]
        + "## Run\n\n"
        + "Open `SchNES.app` on macOS or `SchNES.exe` on Windows and drag a ROM onto\n"
        + "the window. You can also drop a ROM during play to switch games.\n\n"
        + source[end:]
    )
    start = source.index("## Other build options\n")
    end = source.index("## Implementation details\n", start)
    source = source[:start] + source[end:]
    return source.replace("./build/SchNES --headless", "./SchNES.app/Contents/MacOS/SchNES --headless")


def render_html(markdown, asset_root):
    # macOS ships the GFM renderer, including the README's table extension
    cmark = ctypes.CDLL("/usr/lib/libcmark-gfm.dylib")
    pointer = ctypes.c_void_p
    default_lib = ctypes.CDLL(None)

    def bind(library, name, result, *args):
        function = getattr(library, name)
        function.restype = result
        function.argtypes = args
        return function

    bind(cmark, "cmark_gfm_core_extensions_ensure_registered", None)()
    find_extension = bind(cmark, "cmark_find_syntax_extension", pointer, ctypes.c_char_p)
    parser_free = bind(cmark, "cmark_parser_free", None, pointer)
    node_free = bind(cmark, "cmark_node_free", None, pointer)
    free = bind(default_lib, "free", None, pointer)
    parser = bind(cmark, "cmark_parser_new", pointer, ctypes.c_int)(0)
    document = output = None
    try:
        attach = bind(cmark, "cmark_parser_attach_syntax_extension", ctypes.c_int, pointer, pointer)
        table = find_extension(b"table")
        if not parser or not table or not attach(parser, table):
            raise RuntimeError("Could not initialize GitHub Markdown tables")
        data = markdown.encode("utf-8")
        bind(cmark, "cmark_parser_feed", None, pointer, ctypes.c_char_p, ctypes.c_size_t)(
            parser, data, len(data))
        document = bind(cmark, "cmark_parser_finish", pointer, pointer)(parser)
        extensions = bind(cmark, "cmark_parser_get_syntax_extensions", pointer, pointer)(parser)
        if document:
            output = bind(cmark, "cmark_render_html", pointer, pointer, ctypes.c_int, pointer)(
                document, 0, extensions)
        if not output:
            raise RuntimeError("Could not render the release README")
        body = ctypes.string_at(output).decode("utf-8")
    finally:
        free(output)
        if document:
            node_free(document)
        if parser:
            parser_free(parser)

    def inline_png(match):
        image = asset_root / unquote(unescape(match[1]))
        encoded = b64encode(image.read_bytes()).decode("ascii")
        return f'<img src="data:image/png;base64,{encoded}"'

    def split_mapper_table(match):
        header, table_body = match.groups()
        rows = re.findall(r"<tr>.*?</tr>", table_body, re.DOTALL)
        split_at = next(
            (i for i, row in enumerate(rows) if re.match(r"<tr>\s*<td\b[^>]*>\s*21\s*</td>", row)),
            (len(rows) + 1) // 2,
        )
        tables = "\n".join(
            f"<table>\n{header}\n<tbody>\n" + "\n".join(half) + "\n</tbody>\n</table>"
            for half in (rows[:split_at], rows[split_at:])
        )
        return f'<div class="mapper-tables">\n{tables}\n</div>'

    body = re.sub(r'<img src="(docs/screenshots/[^"\n]+\.png)"', inline_png, body)
    body = re.sub(
        r"(<h3>Hotkeys</h3>\s*<table>.*?</table>)\s*"
        r"(<h3>Options</h3>\s*<table>.*?</table>)",
        r'<div class="run-tables">\n<section>\n\1\n</section>\n<section>\n\2\n</section>\n</div>',
        body, count=1, flags=re.DOTALL)
    body = re.sub(
        r"<table>\s*(<thead>\s*<tr>\s*<th[^>]*>(?:Mapper )?ID</th>\s*"
        r"<th[^>]*>Mapper family</th>.*?</thead>)\s*<tbody>(.*?)</tbody>\s*</table>",
        split_mapper_table, body, count=1, flags=re.DOTALL)
    body = re.sub(
        r"<table>(?=\s*<thead>\s*<tr>\s*<th[^>]*>Game</th>\s*"
        r"<th[^>]*>Regular FPS</th>\s*<th[^>]*>Ultrafast FPS</th>\s*<th[^>]*>Interpreter FPS</th>)",
        '<table class="fps-comparison">', body, count=1)
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SchNES</title>
<style>
:root {{
    color-scheme: dark;
    --page: #0d1117;
    --surface: #161b22;
    --text: #e6edf3;
    --border: #30363d;
}}
body {{ margin: 0; font: 16px/1.6 system-ui, sans-serif; background: var(--page); color: var(--text); }}
main {{ max-width: 64rem; margin: 0 auto; padding: 1.5rem; }}
a {{ color: #79c0ff; }}
img {{ max-width: 100%; height: auto; }}
table {{ display: block; overflow-x: auto; border-collapse: collapse; margin: 1rem 0; }}
th, td {{ border: 1px solid var(--border); padding: .35rem .65rem; }}
th {{ background: var(--surface); }}
code {{ font-family: ui-monospace, monospace; }}
pre {{
    margin: 1rem 0 1.5rem;
    overflow-x: auto;
    padding: .85rem 1rem;
    border: 1px solid var(--border);
    border-inline-start: 3px solid #426581;
    border-radius: .45rem;
    background: #111a25;
    color: #d5e7f5;
    font-size: .875rem;
    line-height: 1.65;
    tab-size: 4;
}}
pre > code {{ font-size: inherit; }}

.run-tables, .mapper-tables {{
    display: grid;
    align-items: start;
    gap: 1.25rem;
}}
.run-tables {{ grid-template-columns: minmax(0, 3fr) minmax(0, 2fr); }}
.mapper-tables {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }}
.run-tables h3 {{ margin: 0; }}
.run-tables table {{ margin: .5rem 0 1rem; }}
.mapper-tables table {{ margin: 0; }}
@media (max-width: 48rem) {{
    .run-tables, .mapper-tables {{ grid-template-columns: minmax(0, 1fr); gap: .5rem; }}
}}

/* OSD mode colors, with a lighter Ultrafast orange for the README. */
.fps-comparison {{ font-variant-numeric: tabular-nums; }}
.fps-comparison :is(th, td):nth-child(2) {{ color: rgb(137, 227, 208); }}
.fps-comparison :is(th, td):nth-child(3) {{ color: rgb(255, 97, 38); }}
.fps-comparison :is(th, td):nth-child(4) {{ color: rgb(104, 232, 116); }}
.fps-comparison th:nth-child(n+2):nth-child(-n+5) {{
    background-color: color-mix(in srgb, currentColor 10%, var(--surface));
}}
.fps-comparison td:nth-child(n+2):nth-child(-n+5) {{
    background-color: color-mix(in srgb, currentColor 5%, transparent);
}}
</style>
</head>
<body>
<main>
{body}</main>
</body>
</html>
"""


def training_jobs(root):
    replays = root / "build/replay-inputs/active-gameplay"
    rows = json.loads((replays / "workloads.json").read_text(encoding="utf-8"))
    for row in rows:
        if not row.get("pgo", row["player_driven"]):
            continue
        rom = root / row["rom"]
        ultrafast = {**row, **row.get("ultrafast", {})}
        sources = [(rom, row["rom_sha256"]), (replays / row["input"], row["input_sha256"])]
        if "ultrafast" in row:
            sources.append((replays / ultrafast["input"], ultrafast["input_sha256"]))
        if "bios_sha256" in row:
            sources.append((rom.with_name("disksys.rom"), row["bios_sha256"]))
        for source, digest in sources:
            if sha256(source.read_bytes()).hexdigest() != digest:
                raise ValueError(f"Training replay input has changed: {source}")
        for mode, depth, weight in (("r", 0, 2), ("u", 0, 2), ("i", 0, 1),
                                    ("r", 1, 1), ("u", 1, 1)):
            scene = ultrafast if mode == "u" else row
            tape = replays / scene["input"]
            first = scene["start"]
            last = min(first + 3000, scene["end"])
            yield (rom, tape, str(first), str(last), mode, str(depth)), weight


def macos_release_target():
    active_sdk = Path(subprocess.check_output(["xcrun", "--show-sdk-path"], text=True).strip())

    def version_parts(value):
        return tuple(map(int, value.split(".")))

    sdks = {path.resolve() for path in active_sdk.parent.glob("MacOSX*.sdk")}
    sdks.add(active_sdk.resolve())
    candidates = []
    for sdk in sdks:
        settings = sdk / "SDKSettings.json"
        if not settings.is_file():
            continue
        metadata = json.loads(settings.read_text(encoding="utf-8"))
        target = metadata.get("SupportedTargets", {}).get("macosx", {})
        if minimum := target.get("MinimumDeploymentTarget"):
            candidates.append((max((11, 0), version_parts(minimum)), version_parts(metadata["Version"]), sdk))

    if not candidates:
        raise RuntimeError("Could not determine the deployment target from the installed macOS SDKs")
    minimum, _, sdk = min(candidates)
    return ".".join(map(str, minimum)), sdk


def build_pgo(root, directory, sdl_archive, jobs, profdata, platform_options,
              runner=(), windows=False, env=None):
    configure = ["cmake", "-S", root, "-B", directory, "-G", "Ninja",
                 "-DCMAKE_BUILD_TYPE=Release", f"-DSCHNES_SDL2_ARCHIVE={sdl_archive}",
                 *platform_options]
    build_targets = ["cmake", "--build", directory, "--parallel", "4", "--target"]
    subprocess.run([*configure, "--fresh", "-DSCHNES_PGO_GENERATE=ON", "-DSCHNES_PGO_PROFILE="],
                   check=True, env=env)
    subprocess.run([*build_targets, "schnes_profile"], check=True, env=env)
    trainer = [*runner, directory / ("schnes_profile.exe" if windows else "schnes_profile")]
    profile = directory / "gameplay.profdata"
    with TemporaryDirectory(prefix="training-", dir=directory) as temporary:
        scratch = Path(temporary)

        def train(job):
            number, (args, weight) = job
            raw = scratch / f"{number}.profraw"
            profile_env = env if windows else {**(env or os.environ), "LLVM_PROFILE_FILE": str(raw)}
            subprocess.run([*trainer, *args, raw], cwd=scratch, check=True, env=profile_env)
            if not raw.stat().st_size:
                raise ValueError(f"Empty training profile for {args}")
            return f"{weight},{raw}"

        # Start the runner and verify it can write a profile before queuing the corpus
        pending = iter(enumerate(jobs))
        profiles = [train(next(pending))]
        with ThreadPoolExecutor(max_workers=2 if windows else 3) as pool:
            profiles.extend(pool.map(train, pending))
        inputs = scratch / "profiles.txt"
        inputs.write_text("\n".join(profiles) + "\n", encoding="utf-8")
        merged = scratch / profile.name
        subprocess.run([profdata, "merge", "--failure-mode=any", f"--input-files={inputs}", "-o", merged],
                       check=True, env=env)
        os.replace(merged, profile)
    print(f"Merged {len(jobs)} gameplay profiles into {profile}", flush=True)

    subprocess.run([*configure, "-DSCHNES_PGO_GENERATE=OFF", f"-DSCHNES_PGO_PROFILE={profile}"],
                   check=True, env=env)
    subprocess.run([*build_targets, "SchNES"], check=True, env=env)


def stamp_macos_bundle(app):
    executable = app / "Contents/MacOS/SchNES"
    stamp = datetime.fromtimestamp(executable.stat().st_mtime).strftime("%m/%d/%Y %H:%M:%S")
    subprocess.run(["xcrun", "SetFile", "-d", stamp, "-m", stamp, app], check=True)


def publish(root, mac_app, windows_exe, showcase, windows_build_dir, windows_compat_dir, macos_minimum):
    if not (mac_app / "Contents/MacOS/SchNES").is_file():
        raise ValueError(f"Missing SchNES executable in {mac_app}")
    for source in (windows_exe, showcase):
        if source is not None and not source.is_file():
            raise FileNotFoundError(source)

    destination = root / "bin"
    destination.mkdir(exist_ok=True)
    readme = release_readme((root / "README.md").read_text(encoding="utf-8"))
    app_name = Path("SchNES.app")
    exe_name = Path("SchNES.exe")
    showcase_name = Path("SchNES Showcase.mp4")
    readme_name = Path("README.html")
    package_members = (app_name, exe_name, readme_name)
    if showcase is not None:
        package_members += (showcase_name,)
    targets = {name: destination / name for name in package_members}

    with TemporaryDirectory(prefix=".publish-", dir=destination) as directory:
        stage = Path(directory)
        staged_app = stage / app_name
        staged_exe = stage / exe_name
        shutil.copytree(mac_app, staged_app, symlinks=True)
        plist = staged_app / "Contents/Info.plist"
        with plist.open("rb") as source:
            metadata = plistlib.load(source)
        metadata["LSMinimumSystemVersion"] = macos_minimum
        with plist.open("wb") as target:
            plistlib.dump(metadata, target, sort_keys=False)
        subprocess.run(["/usr/bin/codesign", "--force", "--sign", "-", staged_app], check=True)
        subprocess.run(["/usr/bin/codesign", "--verify", "--strict", staged_app], check=True)
        stamp_macos_bundle(staged_app)
        shutil.copy2(windows_exe, staged_exe)
        if showcase is not None:
            shutil.copy2(showcase, stage / showcase_name)
        (stage / readme_name).write_text(render_html(readme, root), encoding="utf-8")

        archive = Path("SchNES.zip")
        with ZipFile(stage / archive, "w", compression=ZIP_DEFLATED) as package:
            for name in package_members:
                source = stage / name
                members = (source, *sorted(source.rglob("*"))) if source.is_dir() else (source,)
                for member in members:
                    package.write(member, member.relative_to(stage))
        targets[archive] = destination / archive

        generator = windows_compat_dir / "sdl-map.py"
        if generator.is_file():
            build = (windows_build_dir or windows_exe.resolve().parent).resolve()
            if not (build / "build.ninja").is_file():
                raise ValueError(f"Pass --windows-build-dir with the Ninja build for {windows_exe}")
            map_name = Path(sha256(staged_exe.read_bytes()).hexdigest() + ".sdlmap")
            subprocess.run([sys.executable, generator, staged_exe, "--ninja-dir", build,
                            "--output", stage / map_name], check=True, stdout=subprocess.DEVNULL)
            targets[map_name] = windows_compat_dir / "maps" / map_name

        legacy_readme = Path("README.md")
        targets[legacy_readme] = destination / legacy_readme
        prior = stage / "previous"
        installed = []
        backups = []
        try:
            for file, target in targets.items():
                target.parent.mkdir(parents=True, exist_ok=True)
                if target.exists() or target.is_symlink():
                    backup = prior / file
                    backup.parent.mkdir(parents=True, exist_ok=True)
                    os.replace(target, backup)
                    backups.append(file)
                if file != legacy_readme:
                    os.replace(stage / file, target)
                    installed.append(file)
        except BaseException:
            for file in reversed(installed):
                os.replace(targets[file], stage / file)
            for file in reversed(backups):
                os.replace(prior / file, targets[file])
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[1]
    parser.add_argument("showcase", nargs="?", type=Path, help="Optional showcase video to include in the release")
    parser.add_argument("--mingw-bin", type=Path, help="LLVM-MinGW bin directory if it is outside PATH")
    parser.add_argument("--windows-compat-dir", type=Path, default=root.parent / "schnes-win-compat",
                        help="Directory containing the Windows ARM64 test runner and SDL map tool")
    parser.add_argument("--windows-x64", action="store_true",
                        help="Build only the lean static PGO executable for Windows x64")
    parser.add_argument("--wine", type=Path, help="Wine executable for cross-host Windows x64 training")
    args = parser.parse_args()

    triple = "x86_64-w64-mingw32" if args.windows_x64 else "aarch64-w64-mingw32"
    suffix = ".exe" if os.name == "nt" else ""
    mingw = args.mingw_bin or (
        Path(compiler).parent if (compiler := shutil.which(f"{triple}-clang{suffix}")) else None)
    if mingw is None:
        parser.error("put LLVM-MinGW on PATH or pass --mingw-bin")
    mingw = mingw.resolve()
    compiler, windres = (mingw / f"{triple}-{tool}{suffix}" for tool in ("clang", "windres"))
    windows_profdata = mingw / f"llvm-profdata{suffix}"
    sdl = root / "build/SDL2-2.32.10.tar.gz"
    for path in (sdl, compiler, windres, windows_profdata):
        if not path.is_file():
            parser.error(f"missing {path}")

    builds = root / "build/publish"
    if args.windows_x64:
        windows = builds / "windows-x64"
        runner = []
        if os.name != "nt":
            wine = args.wine or shutil.which("wine64") or shutil.which("wine")
            if not wine or not Path(wine).is_file():
                parser.error("Windows x64 cross-host training needs Wine on PATH or --wine")
            runner = [sys.executable, root / "scripts/wine_runner.py", Path(wine).absolute()]
        toolchain = "WindowsX64.cmake"
    else:
        windows = builds / "windows-arm64"
        windows_compat_dir = args.windows_compat_dir.resolve()
        runner = [windows_compat_dir / "build/schnes-win"]
        for path in (*runner, args.showcase):
            if path is not None and not path.is_file():
                parser.error(f"missing {path}")
        toolchain = "WindowsArm64.cmake"

    jobs = list(training_jobs(root))
    if not jobs:
        parser.error("no gameplay replays for PGO training")
    if not args.windows_x64:
        profdata = subprocess.check_output(["xcrun", "-f", "llvm-profdata"], text=True).strip()
        mac = builds / "macos"
        macos_minimum, sdk = macos_release_target()
        print(f"Targeting macOS {macos_minimum} with {sdk.name}", flush=True)
        build_pgo(root, mac, sdl, jobs, profdata,
                  ["-DCMAKE_OSX_ARCHITECTURES=arm64",
                   f"-DCMAKE_OSX_DEPLOYMENT_TARGET={macos_minimum}", f"-DCMAKE_OSX_SYSROOT={sdk}"])

    options = ["--toolchain", root / "cmake" / toolchain,
               "-DCMAKE_C_FLAGS=-D_WIN32_WINNT=0x0601",
               f"-DCMAKE_C_COMPILER={compiler}", f"-DCMAKE_RC_COMPILER={windres}"]
    if runner:
        options.append("-DCMAKE_CROSSCOMPILING_EMULATOR=" + ";".join(map(str, runner)))
    build_pgo(root, windows, sdl, jobs, windows_profdata, options, runner=runner, windows=True,
              env={**os.environ, "PATH": f"{mingw}{os.pathsep}{os.environ['PATH']}"})
    if args.windows_x64:
        print(f"Built {windows / 'SchNES.exe'}", flush=True)
    else:
        publish(root, mac / "SchNES.app", windows / "SchNES.exe", args.showcase, windows,
                windows_compat_dir, macos_minimum)


if __name__ == "__main__":
    main()
