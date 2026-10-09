"""Fly the rocket and watch it in 3D.

    python tools/view.py               opens the viewer, flying rocket.cfg
    python tools/view.py other.cfg     uses a different config
    python tools/view.py --web         saves the normal flight and 20 random ones into docs/flights
                                       for the online viewer (GitHub Pages)

"Random flight" flies again with randomized wind, motors, mass and so on, the same way the
Monte Carlo does. Press Ctrl+C here to stop.
"""
import http.server
import json
import os
import random
import subprocess
import sys
import webbrowser

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(ROOT, "tvc_sim.exe" if os.name == "nt" else "tvc_sim")
VIEWER = os.path.join(ROOT, "docs", "index.html")
WEB_FLIGHTS = os.path.join(ROOT, "docs", "flights")
RANDOM_FLIGHTS = 20  # how many random flights the online viewer gets (it picks one at random)
args = [a for a in sys.argv[1:] if a != "--web"]
CONFIG = os.path.abspath(args[0] if args else os.path.join(ROOT, "rocket.cfg"))


def read_config(path):
    values = {}
    for line in open(path):
        line = line.split("#")[0]
        if "=" in line:
            name, value = line.split("=", 1)
            values[name.strip()] = value.strip()
    return values


# Fly once (seed None = the normal flight) and bundle up everything the viewer needs
def fly(seed=None):
    command = [SIM, CONFIG] + (["random", str(seed)] if seed is not None else [])
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    if result.returncode != 0:
        return {"error": result.stdout + result.stderr}

    # The printout comes in blocks split by blank lines:
    # rocket info, then (for random flights) what was randomized, then the results
    blocks = result.stdout.replace("Flight log written to flight.csv", "").strip().split("\n\n")
    conditions = blocks[1] if seed is not None else ""
    summary = "\n\n".join(blocks[2:] if seed is not None else blocks[1:])
    print("Random" if seed is not None else "Normal", "flight:", blocks[-1])

    cfg = read_config(CONFIG)
    dims = ["body_length", "body_diameter", "gimbal_position", "leg_span", "leg_drop"]
    return {
        "csv": open(os.path.join(ROOT, "flight.csv")).read(),
        "terrain": open(os.path.join(ROOT, "terrain.csv")).read(),
        "conditions": conditions,
        "summary": summary,
        "rocket": {name: float(cfg[name]) for name in dims},
    }


# The viewer asks for flights/normal.json and flights/random-N.json. Online those are saved
# files, here we fly them fresh.
class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.reply("text/html", open(VIEWER, "rb").read())
        elif self.path == "/flights/normal.json":
            self.reply("application/json", json.dumps(fly()).encode())
        elif self.path.startswith("/flights/random-"):
            self.reply("application/json", json.dumps(fly(random.randint(1, 999999))).encode())
        else:
            self.send_error(404)

    def reply(self, kind, body):
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass  # keep the terminal quiet


def save_web_flights():
    os.makedirs(WEB_FLIGHTS, exist_ok=True)
    flights = {"normal": fly()}
    for i in range(1, RANDOM_FLIGHTS + 1):
        flights[f"random-{i}"] = fly(i)
    for name, flight in flights.items():
        with open(os.path.join(WEB_FLIGHTS, name + ".json"), "w") as f:
            json.dump(flight, f, separators=(",", ":"))
    print("Saved", len(flights), "flights to", WEB_FLIGHTS)


def main():
    if "--web" in sys.argv:
        save_web_flights()
        return

    server = http.server.HTTPServer(("127.0.0.1", 0), Handler)  # port 0 = any free port
    url = f"http://127.0.0.1:{server.server_port}/"
    print("Viewer running at", url, "- press Ctrl+C to stop")
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopped")


if __name__ == "__main__":
    main()
