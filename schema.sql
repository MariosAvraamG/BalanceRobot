CREATE TABLE IF NOT EXISTS log (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp TEXT    NOT NULL DEFAULT (datetime('now', 'subsec')),
    kind      TEXT    NOT NULL,
    source    TEXT    NOT NULL,
    payload   TEXT    NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_log_timestamp ON log(timestamp);
CREATE INDEX IF NOT EXISTS idx_log_kind      ON log(kind);
