# Rules for every agent on this repo

Two people run this project, each with their own coding agent: Omar (`@osayed11`) and Riyaadh (`@Gani332`). The agents talk to each other on a GitHub issue, the board. There is no team lead: anything that changes the other person's work is agreed between Omar and Riyaadh, never decided by an agent.

## 1. The board

The board is the pinned "Agent message board" issue (#3), used through `tools/board` (needs only `gh`, logged in). Its issue number is the default in `tools/board`; `BOARD_ISSUE` overrides it.

- Start every post with `[<human>'s <agent>]`, e.g. `[Omar's Claude]`, `[Riyaadh's Codex]`.
- Keep posts short: what changed, which robots, files or configs, what you need, from whom.
- Mention by GitHub login: `@osayed11`, `@Gani332`, or `@all` for both.
- **A post mentions you or `@all`:** act on it and reply on the board.
- **A post doesn't mention you:** still read it. Reassess your own work only if it is affected (a robot you are using, a file you are editing, a config or launch argument you depend on). Do not reply just to acknowledge.
- **Never put secrets on the board:** no passwords, tokens or keys. The repo's collaborators can read it.

Commands:
- `tools/board unread`: new posts since you last checked, marked read; mentions are flagged.
- `tools/board read -n 20`: the last 20 posts.
- `tools/board post "[Name's Agent] text"`: post, or pipe the text on stdin.
- `tools/board wait`: blocks until the other person posts.
- `tools/board whoami`: your GitHub login.

Polling makes about one API call every 30 s.

**Staying in the loop:**
- **Claude Code:** the committed hooks in `.claude/settings.json` show unread posts at session start and on every prompt. Keep one background `tools/board wait` running, and re-arm it after each wake.
- **Codex (Riyaadh):** the committed hooks in `.codex/hooks.json` show unread posts at session start and on every prompt, using the same scripts. Run `/hooks` once to review and trust them; Codex asks again whenever they change. Codex cannot keep a background wait running, so also run `tools/board unread` before each commit and push, and before touching a robot.
- **Optional, for Codex to react to posts while idle:** run this in a terminal: `while posts="$(tools/board wait)"; do codex exec "Follow AGENTS.md. Handle these new board posts: $posts"; done`.

## 2. Robots: claim before you touch

The robots are shared hardware. A change made while the other person is mid-run can ruin their run or damage the robot.

- **Before SSH-changing a robot:** post the robot names and what you will do, e.g. `[Omar's Claude] taking agv105: external IMU wiring + overlay, reboot expected`. Changing means installing, editing files, rebooting, flashing, starting sessions, or deleting data.
- **When you are done:** post `releasing agv105` with its state.
- Read-only checks such as `uptime`, `df`, `git status` or `lsusb` need no claim.
- **Never touch a robot the other person has claimed**, not even to fix something of theirs. Report what you found on the board instead.
- `docs/FLEET_STATUS.md` is the single record of each robot's state. Update it in the same session as the change.
- **No robot motion without its human operator's explicit go.** Use only the MoCap-feedback drivers. The operator chooses the motion pattern and direction every time.

## 3. Code workflow

1. The working branch is `ros2-migration`. Small commits, and stage only your own files (never `git add -A`).
2. **Never push without your own human's OK.** Before pushing, `git fetch` and rebase. Never force-push.
3. **Ask before editing a file the other person has recently changed or announced.** Check on the board, and wait for them.
4. **After a push that changes something the other person depends on, post on the board.** Examples: launch arguments, configs, gate JSONs, the RealSense or firmware pins, setup scripts, topic names. Name the commit.

## 4. Robot safety rules (learned the hard way)

- **Camera processes:** stop them with SIGINT and wait. Never SIGKILL them; a killed camera node can leave the D455's depth stream unable to start.
- **Process matching:** `pgrep -f` and `ps | grep` match your own SSH command line. Match exact PIDs, or use `[x]`-style patterns.
- **Deleting data on SD cards:** delete one file at a time, with `sync; sleep` between. Root is mounted `discard`, and a bulk delete killed agv110's card. Keep 15 GB or more free; a 10-minute RGB-D bag is 5–7 GB.
- **USB quirk:** keep `usbcore.quirks=8086:0b5c:kn`. Without `n` the D455 does not work.
- **ROS domain:** export `ROS_LOCALHOST_ONLY=1` for robot-local measurements. The robots share a DDS domain.
- **Probe processes:** count them and stop them before every run, by PID.

The full operating procedure is the `agv-robot-ops` skill in `.claude/skills/`.

## 5. Who is working on what

Keep this current. Update it with a commit and a board post when work moves.

| Area | Owner | Notes |
|---|---|---|
| External IMU (Adafruit ISM330DHCX) bring-up: wiring, driver, capture, then calibration | `@osayed11` + `@Gani332` | agv105 first. Split the tasks on the board, and claim each one before starting it |
