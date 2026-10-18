#!/usr/bin/env bash
# Run the checks that matter before publishing this repository.

set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

failed=0

report_failure() {
  printf 'ERROR: %s\n' "$1" >&2
  failed=1
}

printf '[1/6] Checking patch whitespace...\n'
if ! git diff --check; then
  failed=1
fi

printf '[2/6] Checking candidate files for machine-local paths...\n'
developer_home="$(getent passwd "$(id -u)" | cut -d: -f6)"
while IFS= read -r -d '' path; do
  if grep -Iq . "$path" 2>/dev/null && \
     grep -Fn "$developer_home/" "$path"; then
    report_failure "machine-local path found in $path"
  fi
done < <(git ls-files -co --exclude-standard -z)

printf '[3/6] Checking candidate files for common secret formats...\n'
while IFS= read -r -d '' path; do
  if grep -Iq . "$path" 2>/dev/null && \
     grep -En 'BEGIN (RSA|OPENSSH|EC) PRIVATE KEY|ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|AKIA[0-9A-Z]{16}' "$path"; then
    report_failure "possible secret found in $path"
  fi
done < <(git ls-files -co --exclude-standard -z)

printf '[4/6] Checking for tracked runtime/build output...\n'
generated_pattern='(^|/)(build|install|log|CMakeFiles|__pycache__|\.pytest_cache|runs|shared_camera_data)(/|$)|\.(pyc|pyo|log|pid)$'
unexpected_generated="$({
  git ls-files | while IFS= read -r path; do
    [[ -e "$path" ]] || continue
    if grep -Eq "$generated_pattern" <<<"$path"; then
      printf '%s\n' "$path"
    fi
  done
} || true)"
if [[ -n "$unexpected_generated" ]]; then
  printf '%s\n' "$unexpected_generated" >&2
  report_failure 'generated files are tracked by Git'
fi

printf '[5/6] Checking GitHub file-size limit...\n'
while IFS= read -r -d '' path; do
  [[ -f "$path" ]] || continue
  size="$(stat -c '%s' "$path")"
  if (( size >= 100 * 1024 * 1024 )); then
    report_failure "$path is at least 100 MiB; use Git LFS or remove it"
  elif (( size >= 50 * 1024 * 1024 )); then
    printf 'WARNING: %s is %.1f MiB\n' "$path" "$(awk -v n="$size" 'BEGIN {print n / 1048576}')"
  fi
done < <(git ls-files -co --exclude-standard -z)

printf '[6/6] Verifying distributed ONNX models...\n'
printf '%s  %s\n' \
  'bd98dd453af0053a8b1cbafaf09327abb9665925871a6b5c4e15acbbc05258d6' \
  'models/piper_yolov8s_seg.onnx' \
  'db8b059f17566202870a89f88e835cad19e76b45cbda790cfd59500df5c16620' \
  'models/tabletop_yolov8s_seg_v2.onnx' | sha256sum --check --status || \
  report_failure 'an ONNX model is missing or has an unexpected checksum'

if (( failed != 0 )); then
  printf '\nRepository preflight failed.\n' >&2
  exit 1
fi

printf '\nRepository preflight passed. Current changes:\n'
git status --short
