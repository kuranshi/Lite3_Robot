#!/usr/bin/env python3
"""Scan only release additions, never print matching credential contents."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
CHECKS = {
    'absolute personal home': re.compile(r'/(?:home|Users)/[^\s/"\']+'),
    'private network address': re.compile(r'\b(?:192\.168\.\d{1,3}\.\d{1,3}|10\.\d{1,3}\.\d{1,3}\.\d{1,3})\b'),
    'private key material': re.compile(r'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----'),
    'GitHub token': re.compile(r'\b(?:ghp_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,})\b'),
}


def release_files():
    result = subprocess.run(['git', '-C', str(ROOT), 'ls-files', '-z', '--cached', '--others',
        '--exclude-standard', '--', 'recovery', 'README.md', '.gitignore', 'pytest.ini',
        '.github/workflows/recovery-tests.yml'], check=True, capture_output=True)
    return sorted({ROOT/p.decode() for p in result.stdout.split(b'\0') if p})


def findings(paths):
    problems = []
    for path in paths:
        name = path.name
        if name.endswith(('.local.json', '.jsonl', '.log', '.pem', '.key', '.pyc')) or name.startswith('id_'):
            problems.append((path, 'private file or runtime artifact'))
        text = path.read_text(encoding='utf8')
        for label, pattern in CHECKS.items():
            if pattern.search(text):
                problems.append((path, label))
    return problems


def main():
    files = release_files()
    problems = findings(files)
    for path, label in problems:
        print(str(path.relative_to(ROOT))+': '+label)
    print('Release files checked: '+str(len(files))+'; problems: '+str(len(problems)))
    return 1 if problems else 0


if __name__ == '__main__':
    raise SystemExit(main())
