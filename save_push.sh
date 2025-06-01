#!/bin/bash

# 현재 스크립트가 위치한 디렉토리로 이동
cd "$(dirname "$0")"

# 변경사항 Git에 반영
git add .
git commit -m "자동 커밋: $(date '+%Y-%m-%d %H:%M:%S')"
git push origin main
