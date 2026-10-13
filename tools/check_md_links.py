#!/usr/bin/env python3
"""Check that document references in the repo's markdown files exist.

  python3 tools/check_md_links.py [--repo PATH] [--verbose]

Checks every tracked ``*.md`` file (``git ls-files``; falls back to walking
the tree when git is unavailable):

1. Markdown links ``[text](target)`` and reference definitions
   ``[id]: target`` with a relative target are resolved against the file's
   directory and must exist.  URLs, ``mailto:`` and pure ``#anchor`` links are
   skipped.  This applies to every file, archived ones included.
2. Inline code spans that name a document, e.g. ```docs/STATUS.md``` or
   ```.agents/skills/...```, are resolved against the repo root and must
   exist.  A bare file name such as ```SPEC.md``` must match the name of some
   tracked file.  Files under ``docs/archive/`` are skipped for this check:
   their text is frozen and may mention pre-move paths on purpose.
3. Inline code spans under ``src/`` or ``tools/`` are reported as WARN only
   (they name code, and may name packages that are still being planned).

Fenced code blocks are ignored.  Exit code 1 when there is any ERROR.
"""
import argparse
import os
import re
import subprocess
import sys

ARCHIVE_PREFIX = 'docs/archive/'
LINK_RE = re.compile(r'(?<!\\)\[[^\]\n]*\]\(\s*<?([^)\s>]+)>?(?:\s+"[^"]*")?\s*\)')
REFDEF_RE = re.compile(r'^\s{0,3}\[[^\]]+\]:\s*<?(\S+?)>?(?:\s+"[^"]*")?\s*$')
CODE_RE = re.compile(r'(`+)([^`\n]+?)\1')
FENCE_RE = re.compile(r'^\s{0,3}(`{3,}|~{3,})')
DOC_PREFIXES = ('docs/', '.agents/')
CODE_PREFIXES = ('src/', 'tools/')
PLACEHOLDER = re.compile(r'[*<>{}$|]|\.\.\.')


def tracked_markdown(repo):
    try:
        out = subprocess.run(['git', '-C', repo, 'ls-files', '-z'],
                             capture_output=True, check=True).stdout
        files = [f for f in out.decode('utf-8').split('\0') if f]
    except (OSError, subprocess.CalledProcessError):
        files = []
        for root, dirs, names in os.walk(repo):
            dirs[:] = [d for d in dirs if d not in ('.git', 'build', 'install', 'log')]
            for n in names:
                files.append(os.path.relpath(os.path.join(root, n), repo).replace(os.sep, '/'))
    return sorted(files)


def strip_fences(text):
    """Return (line_number, line) pairs outside fenced code blocks."""
    out, fence = [], None
    for no, line in enumerate(text.splitlines(), 1):
        m = FENCE_RE.match(line)
        if fence is None and m:
            fence = m.group(1)[0] * 3
            continue
        if fence is not None:
            if line.strip().startswith(fence):
                fence = None
            continue
        out.append((no, line))
    return out


def is_external(target):
    return re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:', target) is not None or target.startswith('#')


def check(repo, verbose=False):
    files = tracked_markdown(repo)
    basenames = {os.path.basename(f) for f in files}
    md_files = [f for f in files if f.endswith('.md')]
    errors, warns, checked, frozen = [], [], 0, 0

    for rel in md_files:
        path = os.path.join(repo, rel)
        if not os.path.isfile(path):
            continue
        with open(path, encoding='utf-8') as stream:
            text = stream.read()
        base_dir = os.path.dirname(rel)
        archived = rel.startswith(ARCHIVE_PREFIX)
        for no, line in strip_fences(text):
            no_code = CODE_RE.sub('', line)
            targets = LINK_RE.findall(no_code)
            m = REFDEF_RE.match(no_code)
            if m:
                targets.append(m.group(1))
            for target in targets:
                if is_external(target):
                    continue
                target = target.split('#', 1)[0]
                if not target:
                    continue
                checked += 1
                resolved = os.path.normpath(os.path.join(repo, base_dir, target))
                if not os.path.exists(resolved):
                    errors.append(f'{rel}:{no}: link target missing: {target}')
                elif verbose:
                    print(f'ok   {rel}:{no}: {target}')

            for _, span in CODE_RE.findall(line):
                span = span.strip()
                if ' ' in span or PLACEHOLDER.search(span) or span.startswith(('/', '~')):
                    continue
                is_doc = span.startswith(DOC_PREFIXES)
                is_bare_md = '/' not in span and span.endswith('.md')
                is_code = span.startswith(CODE_PREFIXES)
                if not (is_doc or is_bare_md or is_code):
                    continue
                if archived:
                    frozen += 1
                    continue
                checked += 1
                if is_bare_md:
                    ok = span in basenames
                else:
                    ok = os.path.exists(os.path.join(repo, span.rstrip('/')))
                if ok:
                    if verbose:
                        print(f'ok   {rel}:{no}: `{span}`')
                elif is_code:
                    warns.append(f'{rel}:{no}: code path not found: {span}')
                else:
                    errors.append(f'{rel}:{no}: document reference missing: {span}')

    for w in warns:
        print(f'WARN  {w}')
    for e in errors:
        print(f'ERROR {e}')
    print(f'\n{len(md_files)} markdown files, {checked} references checked, '
          f'{frozen} inline references in {ARCHIVE_PREFIX} skipped (frozen), '
          f'{len(warns)} WARN, {len(errors)} ERROR')
    return 1 if errors else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    default_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument('--repo', default=default_repo, help='repository root')
    ap.add_argument('--verbose', action='store_true', help='also print references that are ok')
    args = ap.parse_args()
    return check(args.repo, args.verbose)


if __name__ == '__main__':
    sys.exit(main())
