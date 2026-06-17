import json
import os
import sqlite3

from flask import Flask, jsonify, request

DB_PATH     = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log.db')
SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'schema.sql')

app = Flask(__name__, static_folder='static')

_latest_object = {'name': None}


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




@app.after_request
def add_cors(response):
    response.headers['Access-Control-Allow-Origin']  = '*'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    return response


@app.route('/telemetry/<path:sub>', methods=['OPTIONS'])
def telemetry_preflight(sub):
    return '', 204

def _ingest(kind: str, source: str):
    data = request.get_json(force=True, silent=True)
    if data is None or not isinstance(data, dict):
        return jsonify({'error': 'invalid json'}), 400
    if _latest_object['name'] and 'object' not in data:
        data['object'] = _latest_object['name']
    print(f'[{source}] fields: {list(data.keys())}')
    print(f'[{source}] values: {data}')
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
# Vision — object detection from Pi
# ---------------------------------------------------------------------------

@app.route('/vision/object', methods=['POST'])
def vision_object():
    data = request.get_json(force=True, silent=True)
    if not data or 'object' not in data:
        return jsonify({'error': 'missing object field'}), 400
    name = str(data['object']).strip()
    if name:
        _latest_object['name'] = name
        print(f'[vision] detected: {name}')
    return jsonify({'status': 'ok', 'object': name})

@app.route('/vision/object', methods=['DELETE'])
def vision_clear():
    _latest_object['name'] = None
    return jsonify({'status': 'ok'})

@app.route('/vision/latest')
def vision_latest():
    return jsonify({'object': _latest_object['name']})


@app.route('/')
def dashboard():
    html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'html.h')
    with open(html_path) as f:
        raw = f.read()
    start = raw.find('R"html(') + len('R"html(')
    end   = raw.rfind(')html"')
    return raw[start:end], 200, {'Content-Type': 'text/html; charset=utf-8'}


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
