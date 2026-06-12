# Central Server — Rover Telemetry

Receives telemetry from ESP32-BOT, Pi CV pipeline, and ESP32-CTRL.  
Logs everything to SQLite. Serves a live polling dashboard.

## Setup

```bash
pip install -r requirements.txt
python app.py
```

Open **http://localhost:5000** for the dashboard.

## Run mock clients (no hardware needed)

```bash
# in a second terminal
python mock_clients.py

# against a different host
python mock_clients.py --server http://192.168.1.100:5000
```

Prints a status line every 5 s: `Posted N messages (bot: X, pi: Y, ctrl: Z)`.

## Endpoints

### Telemetry ingress (POST)

| Path | kind | source |
|------|------|--------|
| `/telemetry/pi-cmd`  | `pi_cmd`         | `pi`      |
| `/telemetry/esp-bot` | `bot_telemetry`  | `esp-bot` |
| `/telemetry/esp-ctrl`| `ctrl_telemetry` | `esp-ctrl`|

Body: JSON object. Returns `{"status":"ok","id":<row_id>}` on success,  
`{"error":"invalid json"}` (400) for bad input.

### Dashboard

| Path | Description |
|------|-------------|
| `GET /` | Dashboard UI |
| `GET /dashboard/view` | Recent log rows as JSON |

`/dashboard/view` query params:

| Param | Default | Notes |
|-------|---------|-------|
| `kind` | — | filter: `pi_cmd` \| `bot_telemetry` \| `ctrl_telemetry` |
| `limit` | 50 | max 500 |
| `since_id` | — | return only rows with `id > since_id` |

Response: `{"rows":[…], "latest_id":<int>, "counts":{<kind>:<int>}}`

## Database

SQLite file `log.db` is created automatically on first run (WAL mode enabled).

```bash
# quick inspection
sqlite3 log.db "SELECT id, kind, source, payload FROM log ORDER BY id DESC LIMIT 10;"
```

```bash
# Python one-liner
python -c "import sqlite3; [print(r) for r in sqlite3.connect('log.db').execute('SELECT id, kind, source, payload FROM log ORDER BY id DESC LIMIT 5')]"
```
