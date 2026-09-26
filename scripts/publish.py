#!/usr/bin/env python3
"""Batch-publish the workspace crates to crates.io.

Design notes, because a publish cannot be undone:

* It is a **dry run** unless --execute is given.
* Even with --execute it prints the plan and waits for the word `publish`.
* **A batch that stopped partway is resumable.** Crates whose current version is
  already on crates.io are skipped, visibly, and the rest go out. So simply
  re-running the same command finishes the job; there is no need to remember
  which ones succeeded. --strict turns that into an error instead, for when you
  want a forgotten version bump to be caught.
* crates.io rate-limits *new* crates (HTTP 429). That is expected on a large
  first release, so it is detected, reported with the retry time, and the exact
  resume command is printed.
* Ctrl-C during publishing is caught and reported the same way, rather than
  leaving you guessing what went out.
* **Versions are per crate**, so --bump only ever touches the crates you selected
  with --only/--exclude. Fixing one driver must not force a release of the other
  fourteen, so there is deliberately no workspace-wide version to bump.

Usage:
    scripts/publish.py                       # dry run: shows the plan
    scripts/publish.py --execute             # publish, after confirmation
    scripts/publish.py --execute --only mhz19,pmsx003
    scripts/publish.py --bump patch --only bme280 --execute
    scripts/publish.py --bump minor --exclude spl06 --execute
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

CRATES_IO = "https://crates.io/api/v1/crates"
USER_AGENT = "edrv-publish-script (+https://github.com/embedded-drivers/embedded-drivers)"
HTTP_TIMEOUT = 30

# crates.io rate-limits new crate creation, e.g.
#   status 429 Too Many Requests: You have published too many new crates in a
#   short period of time. Please try again after Sat, 26 Sep 2026 05:04:57 GMT
EXAMPLES = """examples:
  scripts/publish.py                       # dry run: shows the plan
  scripts/publish.py --execute             # publish, after confirmation
  scripts/publish.py --execute --only mhz19,pmsx003
  scripts/publish.py --bump patch --only bme280 --execute
  scripts/publish.py --bump minor --exclude spl06 --execute

--bump needs a scope: with per-crate versions there is no single version to bump,
so pass --only (or --exclude) naming the crates you are actually releasing."""

# crates.io validates these server-side, so `cargo publish --dry-run` does not
# catch them: a bad keyword or category only fails at upload time, wasting a
# request against the new-crate rate limit. Checked locally instead.
KEYWORD_RE = re.compile(r"^[A-Za-z0-9_-]+$")
KEYWORD_MAX_LEN = 20
KEYWORD_MAX_COUNT = 5

RATE_LIMITED = re.compile(r"\b429\b|too many new crates|rate limit", re.I)
RETRY_AFTER = re.compile(r"try again after (.+?)(?:\s+and see|\s*$)", re.I | re.M)


# ---------------------------------------------------------------------------
# terminal output
# ---------------------------------------------------------------------------
class Style:
    def __init__(self) -> None:
        on = sys.stdout.isatty()
        self.bold = "\033[1m" if on else ""
        self.dim = "\033[2m" if on else ""
        self.red = "\033[31m" if on else ""
        self.green = "\033[32m" if on else ""
        self.yellow = "\033[33m" if on else ""
        self.blue = "\033[34m" if on else ""
        self.reset = "\033[0m" if on else ""


S = Style()


def step(title: str) -> None:
    print(f"\n{S.blue}==>{S.reset} {S.bold}{title}{S.reset}")


def info(message: str = "") -> None:
    print(f"  {message}" if message else "")


def ok(message: str) -> None:
    print(f"  {S.green}✓{S.reset} {message}")


def warn(message: str) -> None:
    sys.stdout.flush()
    print(f"  {S.yellow}!{S.reset} {message}", flush=True)


def error(message: str) -> None:
    sys.stdout.flush()
    print(f"  {S.red}✗{S.reset} {message}", file=sys.stderr, flush=True)


class Abort(Exception):
    """Something the user has to fix; never a crash."""


# ---------------------------------------------------------------------------
# crates.io
# ---------------------------------------------------------------------------
def fetch_versions(name: str) -> tuple[bool, set[str]]:
    """Return (crate_exists, published_versions).

    Raises Abort if crates.io cannot be reached, so the caller never has to guess
    whether a version collision is real.
    """
    request = urllib.request.Request(f"{CRATES_IO}/{name}", headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT) as response:
            payload = json.load(response)
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            return False, set()
        raise Abort(f"crates.io returned HTTP {exc.code} while looking up {name}") from exc
    except Exception as exc:  # noqa: BLE001 - anything here means "do not publish blindly"
        raise Abort(f"could not reach crates.io for {name}: {exc}") from exc

    if "crate" not in payload:
        return False, set()
    return True, {v["num"] for v in payload.get("versions", []) if not v.get("yanked")}


# ---------------------------------------------------------------------------
# workspace
# ---------------------------------------------------------------------------
@dataclass
class Crate:
    name: str
    version: str
    directory: str
    exists: bool = False
    versions: set[str] = field(default_factory=set)
    keywords: list[str] = field(default_factory=list)
    categories: list[str] = field(default_factory=list)

    @property
    def status(self) -> str:
        """new | update | published"""
        if self.version in self.versions:
            return "published"
        return "update" if self.exists else "new"


def workspace_root() -> Path:
    here = Path(__file__).resolve().parent
    root = here.parent
    manifest = root / "Cargo.toml"
    if not manifest.is_file() or "[workspace]" not in manifest.read_text():
        raise Abort(f"{root} is not a cargo workspace root")
    return root


def workspace_crates(root: Path) -> list[Crate]:
    result = subprocess.run(
        ["cargo", "metadata", "--no-deps", "--format-version", "1"],
        cwd=root,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise Abort(f"cargo metadata failed:\n{result.stderr.strip()}")
    metadata = json.loads(result.stdout)
    base = Path(metadata["workspace_root"]).resolve()
    crates = []
    for package in metadata["packages"]:
        directory = Path(package["manifest_path"]).resolve().parent
        crates.append(
            Crate(
                name=package["name"],
                version=package["version"],
                directory=str(directory.relative_to(base)),
                keywords=list(package.get("keywords") or []),
                categories=list(package.get("categories") or []),
            )
        )
    return crates


def fetch_category_slugs() -> set[str]:
    """Every valid crates.io category slug. The endpoint is paginated, 10 a page."""
    slugs: set[str] = set()
    page = 1
    while True:
        request = urllib.request.Request(
            f"https://crates.io/api/v1/categories?page={page}",
            headers={"User-Agent": USER_AGENT},
        )
        try:
            with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT) as response:
                payload = json.load(response)
        except Exception as exc:  # noqa: BLE001
            raise Abort(f"could not fetch the crates.io category list: {exc}") from exc
        items = payload.get("categories", [])
        if not items:
            break
        slugs |= {item["slug"] for item in items}
        if len(slugs) >= payload.get("meta", {}).get("total", 0):
            break
        page += 1
    return slugs


def validate_metadata(crates: list[Crate], categories: set[str]) -> list[str]:
    """Return a list of things crates.io will reject. Empty means fine."""
    problems: list[str] = []
    for crate in crates:
        if len(crate.keywords) > KEYWORD_MAX_COUNT:
            problems.append(
                f"{crate.name}: {len(crate.keywords)} keywords, crates.io allows {KEYWORD_MAX_COUNT}"
            )
        for keyword in crate.keywords:
            if not KEYWORD_RE.match(keyword):
                problems.append(
                    f"{crate.name}: keyword {keyword!r} - only letters, digits, '-' and '_' are allowed"
                )
            elif len(keyword) > KEYWORD_MAX_LEN:
                problems.append(
                    f"{crate.name}: keyword {keyword!r} is longer than {KEYWORD_MAX_LEN} characters"
                )
        for category in crate.categories:
            if category not in categories:
                problems.append(f"{crate.name}: {category!r} is not a crates.io category")
    return problems


def select(crates: list[Crate], only: set[str], exclude: set[str]) -> list[Crate]:
    chosen = [c for c in crates if (not only or c.directory in only or c.name in only)]
    chosen = [c for c in chosen if c.directory not in exclude and c.name not in exclude]
    if not chosen:
        raise Abort("no crates selected (check --only/--exclude)")
    return chosen


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def run(
    args: list[str],
    root: Path,
    *,
    capture: bool = False,
    check: bool = False,
    quiet: bool = True,
) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        args,
        cwd=root,
        capture_output=capture,
        text=True,
        stdout=subprocess.DEVNULL if (quiet and not capture) else None,
    )
    if check and result.returncode != 0:
        raise Abort(f"{' '.join(args)} failed")
    return result


def git_is_clean(root: Path) -> bool:
    result = subprocess.run(
        ["git", "status", "--porcelain"], cwd=root, capture_output=True, text=True
    )
    return result.returncode != 0 or not result.stdout.strip()


def crate_list(names: list[str]) -> str:
    return ",".join(names)


def resume_hint(todo: list[str]) -> str:
    return f"scripts/publish.py --execute --only {crate_list(todo)}"


# ---------------------------------------------------------------------------
# steps
# ---------------------------------------------------------------------------
def preflight(root: Path, skip_checks: bool, allow_dirty: bool) -> None:
    step("Preflight")

    if not allow_dirty:
        if not git_is_clean(root):
            subprocess.run(["git", "status", "--short"], cwd=root)
            raise Abort("git tree is dirty - commit first, or pass --allow-dirty")
        ok("git tree clean")
    else:
        warn("skipping the dirty-tree check (--allow-dirty)")

    cargo_home = Path(os.environ.get("CARGO_HOME", Path.home() / ".cargo"))
    has_credentials = any(
        (cargo_home / name).exists() for name in ("credentials.toml", "credentials")
    ) or bool(os.environ.get("CARGO_REGISTRY_TOKEN"))
    if not has_credentials:
        warn("no cargo credentials found; run 'cargo login' before the real publish")

    if skip_checks:
        warn("skipping the build/test/clippy/fmt checks (--skip-checks)")
        return

    info("  running build / test / clippy / fmt")
    env = {**os.environ, "RUSTFLAGS": "-Dwarnings"}
    build = subprocess.run(["cargo", "build", "--all", "--quiet"], cwd=root, env=env)
    if build.returncode != 0:
        raise Abort("cargo build failed")
    test = subprocess.run(["cargo", "test", "--all", "--quiet"], cwd=root)
    if test.returncode != 0:
        raise Abort("cargo test failed")
    for args, label in (
        (["cargo", "+nightly", "fmt", "--all", "--check"], "formatting"),
        (["cargo", "+nightly", "clippy", "--all", "--all-targets", "--", "-D", "warnings"], "clippy"),
    ):
        if subprocess.run(args, cwd=root).returncode != 0:
            raise Abort(f"{label} check failed")
    ok("build, tests, fmt and clippy are clean")


def next_version(current: str, spec: str) -> str:
    if spec in {"patch", "minor", "major"}:
        core = re.split(r"[-+]", current, maxsplit=1)[0]
        parts = [int(p) for p in core.split(".")]
        while len(parts) < 3:
            parts.append(0)
        major, minor, patch = parts[:3]
        if spec == "patch":
            patch += 1
        elif spec == "minor":
            minor, patch = minor + 1, 0
        else:
            major, minor, patch = major + 1, 0, 0
        return f"{major}.{minor}.{patch}"
    if not re.fullmatch(r"\d+\.\d+\.\d+(?:[-+].*)?", spec):
        raise Abort(f"invalid --bump value: {spec} (use patch, minor, major or X.Y.Z)")
    return spec


def bump_and_commit(root: Path, crates: list[Crate], spec: str) -> None:
    """Bump each selected crate's own version, then commit.

    Versions are per crate: a fix in one driver must not force a release of all
    the others, so there is no workspace-wide version to bump and a blank
    `--bump` would have nothing meaningful to do. Hence the scope requirement.
    """
    if not crates:
        raise Abort("--bump selected no crates (check --only/--exclude)")

    step(f"Bumping {len(crates)} crate version(s) ({spec})")

    touched: list[Path] = []
    released: list[str] = []

    for crate in crates:
        manifest = root / crate.directory / "Cargo.toml"
        text = manifest.read_text()
        match = re.compile(r'^(version\s*=\s*")([^"]+)(")', re.M).search(text)
        if not match:
            raise Abort(f"no `version` in {manifest}")
        if match.group(2) != crate.version:
            raise Abort(
                f"{crate.name}: cargo metadata says {crate.version} but {manifest} "
                f"says {match.group(2)}; re-run cargo metadata first"
            )

        current = match.group(2)
        bumped = next_version(current, spec)
        if bumped == current:
            raise Abort(f"--bump {spec} did not change {crate.name} ({current})")

        manifest.write_text(text[: match.start(2)] + bumped + text[match.end(2) :])
        info(f"  {crate.name:<16} {current} -> {bumped}")
        touched.append(manifest)
        released.append(f"{crate.name} {bumped}")

    # Keep Cargo.lock in step, then commit only what we touched.
    run(["cargo", "update", "--workspace"], root)
    run(["git", "add", *[str(path.relative_to(root)) for path in touched]], root)
    if (root / "Cargo.lock").is_file():
        run(["git", "add", "Cargo.lock"], root)
    run(["git", "commit", "-m", "chore: release " + ", ".join(released)], root)
    ok(f"bumped and committed: {', '.join(released)}")


def report_publish_failure(crate: Crate, output: str, done: list[Crate], todo: list[Crate]) -> None:
    print()
    if RATE_LIMITED.search(output):
        match = RETRY_AFTER.search(output)
        error(f"{crate.name} hit the crates.io rate limit for new crates.")
        if match:
            info(f"  Retry after: {S.bold}{match.group(1).strip()}{S.reset}")
        info("  This is a normal limit on how many new crates you can create at once,")
        info("  not a problem with the crate itself.")
    else:
        error(f"cargo publish failed for {crate.name}:")
        for line in output.strip().splitlines()[-15:]:
            info(f"    {line}")

    print()
    if done:
        info(f"  {S.green}Published{S.reset}: " + ", ".join(f"{c.name} {c.version}" for c in done))
    else:
        info("  Nothing was published.")
    info("  Remaining: " + ", ".join(c.name for c in todo))
    print()
    info(f"  Resume with:  {S.bold}{resume_hint([c.name for c in todo])}{S.reset}")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="publish.py",
        description="Batch-publish the workspace crates to crates.io (dry run by default).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=EXAMPLES,
    )
    parser.add_argument("--execute", action="store_true", help="actually publish")
    parser.add_argument("--yes", "-y", action="store_true", help="skip the confirmation prompt")
    parser.add_argument("--strict", action="store_true",
                        help="fail if a version is already published, instead of skipping it")
    parser.add_argument("--bump", metavar="SPEC",
                        help="bump the selected crates' own versions first "
                             "(patch|minor|major|X.Y.Z); needs --only/--exclude")
    parser.add_argument("--only", metavar="A,B", default="", help="publish only these crates")
    parser.add_argument("--exclude", metavar="A,B", default="", help="skip these crates")
    parser.add_argument("--skip-checks", action="store_true", help="skip build/test/clippy/fmt")
    parser.add_argument("--allow-dirty", action="store_true", help="pass --allow-dirty to cargo")
    args = parser.parse_args(argv)

    def as_set(value: str) -> set[str]:
        return {item.strip() for item in value.split(",") if item.strip()}


    done: list[Crate] = []
    todo: list[Crate] = []

    try:
        root = workspace_root()

        # --bump must be scoped explicitly. Without this, a bare `--bump patch`
        # selects every crate and releases all fifteen for a one-driver fix -
        # which is the whole thing per-crate versions exist to prevent.
        if args.bump and not (args.only or args.exclude):
            raise Abort(
                "--bump needs a scope.\n"
                "  Versions are per crate, so name the crates you are releasing:\n"
                "    scripts/publish.py --bump patch --only bme280 --execute\n"
                "  A blanket bump would release all of them."
            )

        # Selection happens first: with per-crate versions, --bump has to know
        # which crates it is bumping.
        crates = select(workspace_crates(root), as_set(args.only), as_set(args.exclude))

        if args.bump:
            bump_and_commit(root, crates, args.bump)
            # the versions on disk just changed, so re-read them
            crates = select(workspace_crates(root), as_set(args.only), as_set(args.exclude))

        preflight(root, args.skip_checks, args.allow_dirty)

        step("Planning")
        for crate in crates:
            crate.exists, crate.versions = fetch_versions(crate.name)
            print(f"  {crate.name:<16} {crate.version:<8} {crate.directory:<10} {crate.status}")

        to_publish = [c for c in crates if c.status != "published"]
        skipped = [c for c in crates if c.status == "published"]

        if skipped and args.strict:
            raise Abort(
                f"{len(skipped)} crate(s) are already published at this version: "
                + ", ".join(c.name for c in skipped)
                + "\n  Bump those crates if you meant to re-release them, e.g."
                + " --bump patch --only " + ",".join(c.name for c in skipped)
            )

        if skipped:
            warn(f"{len(skipped)} crate(s) already published at this version - skipping:")
            for crate in skipped:
                info(f"    {S.dim}{crate.name} {crate.version}{S.reset}")
            info("  (pass --strict to treat that as an error instead)")

        if not to_publish:
            raise Abort(
                "nothing left to publish: every selected crate is already on crates.io.\n"
                "  Bump those crates and re-run, e.g."
                " --bump patch --only " + ",".join(c.name for c in crates) + " --execute"
            )

        step("Metadata")
        known_categories = fetch_category_slugs()
        problems = validate_metadata(to_publish, known_categories)
        if problems:
            for problem in problems:
                error(problem)
            raise Abort(
                "crates.io would reject this metadata (it validates keywords and "
                "categories server-side, so --dry-run cannot catch it)"
            )
        ok(f"keywords and categories are valid for {len(to_publish)} crate(s)")

        # From here on `todo` is meaningful, so an interrupt at any point can
        # report what is actually left.
        todo = list(to_publish)

        step("Dry run")
        for crate in to_publish:
            print(f"  {crate.name:<16} {crate.version} ... ", end="", flush=True)
            dry_run = ["cargo", "publish", "--dry-run", "--quiet"]
            if args.allow_dirty:
                dry_run.append("--allow-dirty")
            dry_run += ["-p", crate.name]
            result = run(dry_run, root, capture=True)
            if result.returncode == 0:
                print(f"{S.green}ok{S.reset}")
            else:
                print(f"{S.red}failed{S.reset}")
                for line in (result.stdout + result.stderr).strip().splitlines()[-15:]:
                    info(f"    {line}")
                raise Abort(f"dry run failed for {crate.name}")

        step("Plan")
        info(f"{len(to_publish)} crate(s) will be published to crates.io:")
        for crate in to_publish:
            print(f"    {S.bold}{crate.name} {crate.version}{S.reset}  ({crate.status})")
        if skipped:
            print()
            info(f"{S.dim}Skipped, already published:{S.reset}")
            for crate in skipped:
                info(f"    {S.dim}{crate.name} {crate.version}{S.reset}")
        print()
        info("  Once published, a version cannot be removed - only yanked.")
        print()

        if not args.execute:
            info(f"{S.yellow}Dry run only.{S.reset} Nothing was uploaded.")
            info(f"Re-run with {S.bold}--execute{S.reset} to publish for real.")
            return 0

        if not args.yes:
            try:
                answer = input(
                    f"Type {S.bold}publish{S.reset} to upload these "
                    f"{len(to_publish)} crate(s), anything else aborts: "
                )
            except EOFError:
                print()
                info("No input; aborted. Nothing was uploaded.")
                return 1
            if answer.strip() != "publish":
                info("Aborted; nothing was uploaded.")
                return 1

        step("Publishing")
        for index, crate in enumerate(to_publish):
            todo = to_publish[index:]
            print(f"  {S.bold}{crate.name:<16} {crate.version}{S.reset} ... ", end="", flush=True)
            command = ["cargo", "publish"]
            if args.allow_dirty:
                command.append("--allow-dirty")
            command += ["-p", crate.name]
            result = subprocess.run(command, cwd=root, capture_output=True, text=True)
            output = result.stdout + result.stderr
            (Path("/tmp") / f"edrv-publish-{crate.name}.log").write_text(output)

            if result.returncode != 0:
                print(f"{S.red}failed{S.reset}")
                report_publish_failure(crate, output, done, todo)
                return 1

            print(f"{S.green}ok{S.reset}")
            done.append(crate)
        todo = []

        step("Done")
        for crate in done:
            ok(f"{crate.name} {crate.version}")
        print()
        info("  The crates.io index takes a minute to catch up; docs.rs builds follow.")
        return 0

    except KeyboardInterrupt:
        print()
        error("Interrupted.")
        if done:
            info("  Published: " + ", ".join(f"{c.name} {c.version}" for c in done))
        else:
            info("  Nothing was published yet.")
        if todo:
            info("  Remaining: " + ", ".join(c.name for c in todo))
            info(f"  Resume with:  {S.bold}{resume_hint([c.name for c in todo])}{S.reset}")
        return 130
    except Abort as exc:
        error(str(exc))
        return 1


if __name__ == "__main__":
    sys.exit(main())
