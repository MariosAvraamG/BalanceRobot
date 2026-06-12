import json
import os
import sqlite3

from flask import Flask, jsonify, request, send_from_directory

DB_PATH     = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log.db')
SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'schema.sql')

app = Flask(__name__, static_folder='static')


# ---------------------------------------------------------------------------
# Database
# ---------------------------------------------------------------------------

def get_db() -> sqlite3.Connection:
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def init_db() -> None:
    conn = get_db()
    with open(SCHEMA_PATH) as f:
        conn.executescript(f.read())
    # WAL persists in the DB file after first set; safe to call each startup.
    conn.execute('PRAGMA journal_mode=WAL')
    conn.commit()
    conn.close()
    print(f'Database ready: {DB_PATH}')


# ---------------------------------------------------------------------------
# CORS — permissive for local dev
# ---------------------------------------------------------------------------

@app.after_request
def add_cors(response):
    response.headers['Access-Control-Allow-Origin']  = '*'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    return response


@app.route('/telemetry/<path:sub>', methods=['OPTIONS'])
def telemetry_preflight(sub):
    return '', 204


# ---------------------------------------------------------------------------
# Telemetry ingress
# ---------------------------------------------------------------------------

def _ingest(kind: str, source: str):
    data = request.get_json(force=True, silent=True)
    if data is None or not isinstance(data, dict):
        return jsonify({'error': 'invalid json'}), 400
    try:
        conn = get_db()
        try:
            cur = conn.execute(
                'INSERT INTO log (kind, source, payload) VALUES (?, ?, ?)',
                (kind, source, json.dumps(data)),
            )
            conn.commit()
            row_id = cur.lastrowid
        finally:
            conn.close()
    except Exception as exc:
        return jsonify({'error': str(exc)}), 500
    return jsonify({'status': 'ok', 'id': row_id}), 200


@app.route('/telemetry/pi-cmd', methods=['POST'])
def telemetry_pi_cmd():
    return _ingest('pi_cmd', 'pi')


@app.route('/telemetry/esp-bot', methods=['POST'])
def telemetry_esp_bot():
    return _ingest('bot_telemetry', 'esp-bot')


@app.route('/telemetry/esp-ctrl', methods=['POST'])
def telemetry_esp_ctrl():
    return _ingest('ctrl_telemetry', 'esp-ctrl')


# ---------------------------------------------------------------------------
# Dashboard
# ---------------------------------------------------------------------------

@app.route('/')
def dashboard():
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/dashboard/view')
def dashboard_view():
    try:
        kind_filter = request.args.get('kind') or None
        try:
            limit = min(int(request.args.get('limit', 50)), 500)
        except (ValueError, TypeError):
            limit = 50
        since_id = request.args.get('since_id', type=int)

        conn = get_db()
        try:
            qry    = 'SELECT id, timestamp, kind, source, payload FROM log WHERE 1=1'
            params: list = []

            if kind_filter:
                qry += ' AND kind = ?'
                params.append(kind_filter)
            if since_id is not None:
                qry += ' AND id > ?'
                params.append(since_id)

            qry += ' ORDER BY id DESC LIMIT ?'
            params.append(limit)

            rows = [
                {
                    'id':        r['id'],
                    'timestamp': r['timestamp'],
                    'kind':      r['kind'],
                    'source':    r['source'],
                    'payload':   json.loads(r['payload']),
                }
                for r in conn.execute(qry, params).fetchall()
            ]

            latest    = conn.execute('SELECT COALESCE(MAX(id), 0) AS m FROM log').fetchone()
            latest_id = latest['m']

            counts_rows = conn.execute(
                "SELECT kind, COUNT(*) AS cnt FROM log "
                "WHERE timestamp >= datetime('now', '-60 seconds') GROUP BY kind"
            ).fetchall()
            counts = {r['kind']: r['cnt'] for r in counts_rows}

        finally:
            conn.close()

        return jsonify({'rows': rows, 'latest_id': latest_id, 'counts': counts})

    except Exception as exc:
        return jsonify({'error': str(exc)}), 500


if __name__ == '__main__':
    init_db()
    app.run(host='0.0.0.0', port=5001, debug=False)
