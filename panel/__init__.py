"""Command panel for derpbot debug observation + teleop (#24).

Separate process that connects to an agent's debug bus (WS) and serves
a single-page web UI to browsers. Acts as a fan-out proxy: one bus
connection, N browser connections.

Usage:
    python3.12 -m panel --agent-url ws://localhost:8770 --bind 0.0.0.0:8080
"""