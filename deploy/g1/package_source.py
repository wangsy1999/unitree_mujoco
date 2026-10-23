#!/usr/bin/env python3
"""Create a small source/model bundle to compile natively on the G1 development PC."""
import argparse
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[2]
FILES = [
    'LICENSE', 'policy.onnx',
    'example/cpp/reset_g1_29dof.cpp',
    'example/cpp/g1_hardware.h', 'example/cpp/g1_gamepad.h',
    'example/cpp/g1_locomotion_policy.h', 'example/cpp/g1_locomotion.yaml',
    'example/cpp/README_g1_hardware.md', 'example/cpp/README_g1_locomotion.md',
    'example/cpp/README_f710.md',
    'example/cpp/tests/g1_hardware_test.cpp', 'example/cpp/tests/g1_gamepad_test.cpp',
    'example/cpp/tests/g1_locomotion_policy_test.cpp',
    'deploy/g1/CMakeLists.txt', 'deploy/g1/BUILD_INFO.in',
    'deploy/g1/g1.sh', 'deploy/g1/README.md', 'deploy/g1/package_source.py',
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'deploy/g1/build/g1-dreamwaq-source.tar.gz')
    args = parser.parse_args()
    for name in FILES:
        if not (ROOT / name).is_file():
            parser.error(f'Missing deployment source: {name}')
    output = args.output.resolve()
    if output in [(ROOT / name).resolve() for name in FILES]:
        parser.error('Output must not overwrite a source file')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(output, 'w:gz') as archive:
        for name in FILES:
            archive.add(ROOT / name, arcname=f'g1-dreamwaq-source/{name}')
    print(f'{output} ({output.stat().st_size} bytes, {len(FILES)} files)')


if __name__ == '__main__':
    main()
