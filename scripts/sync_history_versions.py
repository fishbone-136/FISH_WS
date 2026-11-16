#!/usr/bin/env python3
"""Sync source-only historical workspaces; never modify the source directories."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat
from urllib.parse import quote


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOTS = (
    'config', 'go2_jiaohu', 'ros2_ws/src', 'ros2_ws/tools', 'scripts', 'test',
    'third_party/livox_sdk2_system_spdlog',
)
SKIP_DIRS = {
    '.git', '.aws', '.agents', '.codex', '.venv', 'venv', '__pycache__',
    '.pytest_cache', '.mypy_cache', '.ruff_cache', '.vscode', '.idea',
    'models', 'kws-model', 'build', 'install', 'log', 'logs', 'CMakeFiles',
}
SKIP_SUFFIXES = {
    '.pyc', '.pyo', '.o', '.a', '.so', '.whl', '.egg', '.log', '.pid',
    '.pt', '.pth', '.onnx', '.onnx_data', '.safetensors', '.tflite', '.bin',
    '.ckpt', '.npz', '.npy', '.pcd', '.bag', '.db3', '.mcap', '.pem', '.key',
    '.zip', '.tar', '.gz', '.tgz', '.7z', '.swp',
}
EXCLUDED_TREES = (
    Path('go2_jiaohu/third_party'), Path('ros2_ws/src/asset'),
    Path('ros2_ws/src/fast_lio/doc'), Path('ros2_ws/src/fast_lio/PCD'),
)
MAX_BYTES = 1024 * 1024
SECRET_PATTERNS = tuple(re.compile(pattern) for pattern in (
    rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
    rb'\bgh[pousr]_[A-Za-z0-9]{30,}\b',
    rb'\bgithub_pat_[A-Za-z0-9_]{30,}\b',
    rb'\bAKIA[0-9A-Z]{16}\b',
    rb'\bsk-[A-Za-z0-9_-]{24,}\b',
))


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def excluded_directory(relative: Path) -> bool:
    return any(
        part in SKIP_DIRS or part.startswith(('build_', 'install_', 'log_', '.venv-'))
        or part.endswith('.egg-info') for part in relative.parts
    ) or any(relative == tree or tree in relative.parents for tree in EXCLUDED_TREES)


def source_files(source: Path):
    for path in sorted(source.glob('*.md')):
        if path.name not in {'1.md', '123.md'} and not path.is_symlink():
            yield path
    for name in SOURCE_ROOTS:
        base = source / name
        if base.is_symlink() or not base.is_dir():
            continue
        for directory, dirs, files in os.walk(base, followlinks=False):
            directory = Path(directory)
            dirs[:] = sorted(child for child in dirs if not (
                (directory / child).is_symlink()
                or excluded_directory((directory / child).relative_to(source))
            ))
            for child in sorted(files):
                yield directory / child


def collect(source: Path, ignore_policy: bytes) -> tuple[dict, list]:
    files, oversized = {}, []
    for path in source_files(source):
        relative = path.relative_to(source)
        name = path.name
        if path.is_symlink() or not path.is_file():
            continue
        if (name in {'.git', '.gitignore', '.gitmodules', 'CMakeCache.txt', 'compile_commands.json', '.DS_Store'}
                or name == '.env' or (name.startswith('.env.') and name != '.env.example')
                or path.suffix.lower() in SKIP_SUFFIXES or '.so.' in name
                or name.startswith('wget-log') or name.endswith('~')
                or (Path('ros2_ws/src/fast_lio/Log') in relative.parents
                    and path.suffix.lower() in {'.txt', '.csv'})):
            continue
        if path.stat().st_size >= MAX_BYTES:
            oversized.append(relative.as_posix())
            continue
        data = path.read_bytes()
        if any(pattern.search(data) for pattern in SECRET_PATTERNS):
            raise ValueError(f'疑似凭据，已停止同步（未打印内容）: {source.name}/{relative}')
        mode = 0o755 if path.stat().st_mode & stat.S_IXUSR else 0o644
        files[relative.as_posix()] = (data, mode)
    files['.gitignore'] = (ignore_policy, 0o644)
    note = (
        f'# 历史源码快照：{source.name}\n\n'
        f'来源为上一级 `历史版本/{source.name}/`。目录名称按原样保留，'
        '不代表本次验证过该版本的功能或可运行性，也不据此推断版本先后顺序。\n\n'
        '原始代码、配置模板、测试和项目说明按文件内容保存；模型、环境、编译产物、'
        '地图、历史报告、备份、嵌套 Git 元数据与忽略规则及上游大体积演示资料未收录。'
        '每个快照统一使用主工程的 `.gitignore`，以免上游规则遗漏已收录源码。'
        '原目录中的文件不会被此同步过程修改或删除。\n\n'
        '资源需求以本版本的源码和 `.env.example` 为准；'
        '主工程的 [本地资源说明](../../docs/LOCAL_RESOURCES.md) 可作为部署参考。'
        '旧说明中指向已排除备份、报告或本机绝对路径的链接可能不可用。\n\n'
        '`SOURCE_MANIFEST.json` 记录所收录文件的 SHA-256、大小和执行权限。'
        '后续请修改原历史目录，再从主工程执行 `python3 scripts/sync_history_versions.py`。\n'
    )
    files['SNAPSHOT.md'] = (note.encode(), 0o644)
    return files, oversized


def safe_target(destination: Path, relative: str) -> Path:
    path = Path(relative)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError(f'非法清单路径: {relative}')
    target = destination / path
    for parent in (target, *target.parents):
        if parent.is_symlink():
            raise ValueError(f'目标路径不能包含符号链接: {parent}')
        if parent == destination:
            break
    return target


def history_index(manifests: list[dict]) -> str:
    lines = [
        '# 历史版本索引', '',
        '这里收录上一级“历史版本”目录中的工程源码。当前主工程位于仓库根目录；'
        '各历史版本分别保存在下面的子目录，原始目录不会被同步脚本修改。', '',
        '| 原目录名 | 源码及说明文件数（不含清单） | 文件内容合计 |',
        '| --- | ---: | ---: |',
    ]
    for manifest in manifests:
        name = manifest['version']
        lines.append(f"| [{name}]({quote(name)}/SNAPSHOT.md) | "
                     f"{manifest['file_count']} | {manifest['total_bytes'] / 1024**2:.2f} MiB |")
    lines.extend([
        '', '版本名称沿用原目录名，不据此推断先后顺序或保证对应功能已通过验证。'
        '每个目录的 `SOURCE_MANIFEST.json` 保存文件 SHA-256、大小和执行权限，'
        '可核对源码是否与同步来源一致。', '',
        '模型权重、虚拟环境、CUDA/PyTorch 运行库、编译产物、地图、报告、备份、'
        '嵌套 Git 元数据、上游忽略规则及大体积演示资料均未收录。'
        '每个快照使用主工程的 `.gitignore`。原项目说明中引用这些本地文件的链接可能失效。'
        '资源补齐方法见 [主工程本地资源说明](../docs/LOCAL_RESOURCES.md)，'
        '具体路径和依赖以历史版本自身的配置与源码为准。', '',
        '## 后续同步', '',
        '在主工程根目录执行：', '', '```bash',
        'python3 scripts/sync_history_versions.py --dry-run',
        'python3 scripts/sync_history_versions.py',
        'git add history', 'git commit -m "Sync historical source snapshots"',
        'git push', '```', '',
        '默认来源为 `../历史版本/`，也可用 `--source /实际路径/历史版本` 指定。'
        '脚本发现新的完整工程子目录后会一并收录，并自动更新本索引。', '',
        '同步会更新已有快照，并移除来源中已删除、且快照里没有被另行修改的受管理文件。'
        '未出现在旧清单里的文件和来源中缺失的整个版本目录会保留。'
        '如果快照文件有独立的本地改动，脚本会停止，防止覆盖；应先处理这些改动。'
        '脚本不会自动提交或推送。', '',
        '需要运行旧版本时，可将对应子目录复制到单独的工作区，'
        '补齐该版本的模型和环境后重新构建。本次同步只验证文件完整性，未运行历史版本的机器人程序。', '',
    ])
    return '\n'.join(lines)


def sync(source_root: Path, destination_root: Path, ignore_policy: bytes, dry_run=False):
    source_root = source_root.resolve(strict=True)
    if destination_root.is_symlink():
        raise ValueError('目标目录不能是符号链接')
    destination_root = destination_root.resolve()
    if (source_root == destination_root or source_root in destination_root.parents
            or destination_root in source_root.parents):
        raise ValueError('来源与目标目录不能相互包含')
    plans = []
    for source in sorted(source_root.iterdir()):
        if source.name.startswith('.') or source.is_symlink() or not source.is_dir():
            continue
        if not (source / 'go2_jiaohu').is_dir() or not (source / 'ros2_ws/src').is_dir():
            raise ValueError(f'不是预期的 Go2 工程目录: {source.name}')
        files, oversized = collect(source, ignore_policy)
        destination = destination_root / source.name
        manifest_path = safe_target(destination, 'SOURCE_MANIFEST.json')
        previous = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        previous_files = previous.get('files', {})
        metadata = {name: {'sha256': digest(data), 'bytes': len(data),
                           'mode': format(mode, 'o')}
                    for name, (data, mode) in sorted(files.items())}
        for name in set(previous_files) | set(files):
            target = safe_target(destination, name)
            if target.exists():
                current = digest(target.read_bytes())
                if current not in {previous_files.get(name, {}).get('sha256'),
                                   metadata.get(name, {}).get('sha256')}:
                    raise ValueError(f'目标快照存在本地修改，请先处理: {target}')
        manifest = {'version': source.name, 'source': f'历史版本/{source.name}',
                    'file_count': len(files), 'total_bytes': sum(len(v[0]) for v in files.values()),
                    'omitted_files_at_least_1_MiB': oversized, 'files': metadata}
        plans.append((destination, files, previous_files, manifest))

    if not plans:
        raise ValueError('没有找到可同步的历史工程')
    for destination, files, previous_files, manifest in plans:
        print(f"{destination.name}: {manifest['file_count']} 个文件，"
              f"{manifest['total_bytes'] / 1024**2:.2f} MiB"
              + ('（预览）' if dry_run else ''))
        if dry_run:
            continue
        for name in sorted(set(previous_files) - set(files)):
            target = safe_target(destination, name)
            if target.exists():
                target.unlink()
        for name, (data, mode) in sorted(files.items()):
            target = safe_target(destination, name)
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists() or target.read_bytes() != data:
                target.write_bytes(data)
            target.chmod(mode)
        text = json.dumps(manifest, ensure_ascii=False, indent=2) + '\n'
        (destination / 'SOURCE_MANIFEST.json').write_text(text, encoding='utf-8')
    manifests = [plan[3] for plan in plans]
    if not dry_run:
        safe_target(destination_root, 'README.md').write_text(
            history_index(manifests), encoding='utf-8')
    return manifests


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=PROJECT_ROOT.parent / '历史版本')
    parser.add_argument('--dry-run', action='store_true', help='只检查并统计，不写文件')
    args = parser.parse_args()
    try:
        sync(args.source, PROJECT_ROOT / 'history',
             (PROJECT_ROOT / '.gitignore').read_bytes(), args.dry_run)
    except (OSError, ValueError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
