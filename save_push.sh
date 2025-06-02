#!/bin/bash

cd "$(dirname "$0")"

# 첫 번째 인자를 커밋 메시지로 사용, 없으면 기본 메시지 사용
COMMIT_MSG="$*"

if [ -z "$COMMIT_MSG" ]; then
  COMMIT_MSG="자동 커밋: $(date '+%Y-%m-%d %H:%M:%S')"
fi

# 변경사항 있는 경우에만 커밋
if ! git diff --quiet || ! git diff --cached --quiet; then
  git add .
  git commit -m "$COMMIT_MSG"
  git push origin main
else
  echo "변경사항 없음, 푸시 생략"
fi
