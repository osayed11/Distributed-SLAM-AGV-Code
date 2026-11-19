#!/usr/bin/env bash
# UserPromptSubmit: surface board posts that arrived since the last check. Silent when none. Never fails.
cd "${CLAUDE_PROJECT_DIR:-$(git rev-parse --show-toplevel 2>/dev/null || echo .)}" 2>/dev/null || exit 0
out="$(tools/board unread 2>/dev/null)" || exit 0
[ -n "$out" ] && printf 'New board posts (issue #%s):\n%s\n' "${BOARD_ISSUE:-3}" "$out"
exit 0
