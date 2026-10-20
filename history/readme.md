# History

This folder holds **session logs** for Serial Terminal. Each person who runs the app gets their own files here (they are not shared through git).

## What is tracked in git

The `history/` directory, this `readme.md`, and `.gitignore` are tracked so every clone has a place to write logs.

Individual `*.log` files are **not** tracked. `.gitignore` ignores everything in this folder except those two files, so serial traffic, credentials, and device output stay on the machine that captured them.

## How logging works

When the application starts, it:

1. Creates this directory if it is missing.
2. Deletes any `*.log` files older than **30 days**.
3. Opens a new log named from the session start time, for example:

   `session_2026-10-20_103112.log`

That one file records the whole time the app is open, including multiple connect/disconnect cycles. Closing the app ends the session. The next launch starts a new file.

Clearing the on-screen terminal does **not** stop or reset the session log. The live filter also does not affect what is written here.

## What each log contains

Lines are timestamped local time (`YYYY-MM-DD HH:MM:SS.mmm`) with a tag:

| Tag | Meaning |
|-----|---------|
| `SYS` | Session start/end and other application notes |
| `CONN` | Connect, disconnect, or loss of the port (user click or OS/driver error), including port, baud, data bits, parity, stop bits, flow control, and send line ending |
| `SIG` | RTS/DTR the user sets, and changes on CTS/DSR/RI/CD |
| `TX` | Text the user sends (send box or macros) |
| `RX` | Complete lines received from the device (ANSI color codes stripped) |

## Retention

On startup, logs whose session date (from the filename, or file modification time if the name does not parse) is more than a month old are deleted automatically.

## Manual copies

Use **Save log…** in the Terminal tab if you want a snapshot of the current on-screen buffer under another name. That is separate from this always-on session file.
