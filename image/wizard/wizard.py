#!/usr/bin/env python3
"""DroneCtl first-boot provisioning wizard (manual Part 8.4).

Zero-dependency (Python 3.9+ stdlib only) first-boot wizard that:
  * brings up a Wi-Fi AP (hostapd + dnsmasq) when --bring-up-ap is passed,
  * serves a tiny HTML form at http://<bind_ip>/ collecting drone_id,
    camera.type, fc.url, tether IPs, and an optional server endpoint,
  * on POST writes /etc/dronectl/config.yaml from a template, disables
    dronectl-wizard.service, and reboots when --apply is set.

Flags:
  --bring-up-ap   Actually start hostapd+dnsmasq (import-time safe: the AP is
                  only started when this flag is passed).
  --apply         Persist the config to /etc/dronectl/config.yaml, disable
                  the wizard unit, and reboot after a successful POST. Without
                  --apply the wizard still answers the form but only echoes the
                  generated config + recommended next steps (tests don't reboot
                  the dev box).
  --dry-run       Ignore form input; render the default config to stdout and
                  exit (does not start the AP or the HTTP server).
  --bind IP       IP to bind the HTTP server (default 10.0.0.1).
  --port N        HTTP port (default 80).
  --config PATH   Config target (default /etc/dronectl/config.yaml).
  --iface NAME    wlan interface for the AP (default wlan0).
  --ssid SSID     AP SSID (default dronectl-<hostname suffix>).
  --passphrase S  WPA2 passphrase (default dronectl-setup).
  --help          This help.

The wizard is shipped at /opt/dronectl/bin/wizard (the pi-gen stage installs
this file verbatim). It is import-clean: no side effects at import time.
"""

from __future__ import annotations

import argparse
import html
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ---------------------------------------------------------------------------
# Defaults / constants
# ---------------------------------------------------------------------------

DEFAULT_CONFIG_PATH = "/etc/dronectl/config.yaml"
DEFAULT_BIND_IP = "10.0.0.1"
DEFAULT_PORT = 80
DEFAULT_IFACE = "wlan0"
DEFAULT_PASSPHRASE = "dronectl-setup"
WIZARD_UNIT = "dronectl-wizard.service"
HOSTAPD_TMPL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hostapd.conf")
DNSMASQ_TMPL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dnsmasq.conf")

CAMERA_TYPES = ("depth_oakd", "realsense", "mono_rpicam")

DEFAULT_FORM = {
    "drone_id": "drone-01",
    "camera.type": "depth_oakd",
    "camera.width": "640",
    "camera.height": "480",
    "camera.fov_deg": "71",
    "grid.n": "96",
    "grid.voxel_size_m": "0.4",
    "grid.horizon_m": "30",
    "fc.url": "udp://:14540",
    "fc.max_velocity_ms": "5.0",
    "fc.max_accel_ms2": "2.0",
    "cfc.model": "/opt/dronectl/cfc_policy.onnx",
    "cfc.hidden_units": "48",
    "cfc.rate_hz": "20",
    "tether.iface": "eth0",
    "tether.self_ip": "10.8.0.2",
    "tether.ground_ip": "10.8.0.1",
    "failsafe.link_loss_action": "return_along_track",
    "failsafe.low_batt_pct": "20",
    "server.endpoint": "",
}


# ---------------------------------------------------------------------------
# Config rendering (pure; no I/O)
# ---------------------------------------------------------------------------

def render_config(form: dict) -> str:
    """Render the config.yaml text from a flat form dict.

    Stdlib only; a tiny hand-rolled serializer keeps us YAML-reader-free.
    """
    g = {k: form.get(k, v) for k, v in DEFAULT_FORM.items()}

    def s(key):
        return html.escape(g.get(key, "").strip(), quote=True)

    cam_type = g["camera.type"] if g["camera.type"] in CAMERA_TYPES else "depth_oakd"
    server_line = (
        f"server: {{ endpoint: {json.dumps(s('server.endpoint')) } }}"
        if s("server.endpoint") else "# server: { endpoint: \"\" }  # not configured"
    )

    return (
        "# /etc/dronectl/config.yaml — written by dronectl first-boot wizard.\n"
        f"drone_id: {json.dumps(s('drone_id'))}\n"
        "camera: "
        f"{{ type: {cam_type}, width: {int(g['camera.width']) or 640}, "
        f"height: {int(g['camera.height']) or 480}, fov_deg: {float(g['camera.fov_deg']) or 71} }}\n"
        "grid:   "
        f"{{ n: {int(g['grid.n']) or 96}, voxel_size_m: {float(g['grid.voxel_size_m']) or 0.4}, "
        f"horizon_m: {float(g['grid.horizon_m']) or 30} }}\n"
        "fc:     "
        f"{{ url: {json.dumps(s('fc.url'))}, max_velocity_ms: {float(g['fc.max_velocity_ms']) or 5.0}, "
        f"max_accel_ms2: {float(g['fc.max_accel_ms2']) or 2.0} }}\n"
        "cfc:    "
        f"{{ model: {json.dumps(s('cfc.model'))}, hidden_units: {int(g['cfc.hidden_units']) or 48}, "
        f"rate_hz: {int(g['cfc.rate_hz']) or 20} }}\n"
        "tether: "
        f"{{ iface: {json.dumps(s('tether.iface'))}, ground_ip: {json.dumps(s('tether.ground_ip'))}, "
        f"self_ip: {json.dumps(s('tether.self_ip'))} }}\n"
        "failsafe: "
        f"{{ link_loss_action: {json.dumps(s('failsafe.link_loss_action'))}, "
        f"low_batt_pct: {int(g['failsafe.low_batt_pct']) or 20} }}\n"
        f"{server_line}\n"
    )


# ---------------------------------------------------------------------------
# AP bring-up (only when --bring-up-ap)
# ---------------------------------------------------------------------------

def _fill_template(tmpl_path: str, values: dict) -> str:
    """Read a stub template, drop comment-only placeholder lines, fill ours."""
    if not os.path.exists(tmpl_path):
        return ""
    out_lines = []
    with open(tmpl_path, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if stripped.startswith("#") and "=" in stripped and not stripped.startswith("# "):
                # placeholder like "# interface=" -> fill
                key = stripped[1:].split("=", 1)[0].strip()
                if key in values and values[key] != "":
                    out_lines.append(f"{key}={values[key]}")
                continue
            if stripped.startswith("# fill at runtime by wizard"):
                continue
            out_lines.append(line.rstrip("\n"))
    return "\n".join(out_lines) + ("\n" if out_lines else "")


def bring_up_ap(iface: str, ssid: str, passphrase: str, ap_ip: str) -> list:
    """Start the Wi-Fi AP. Returns the list of subprocess return codes.

    Only called when --bring-up-ap is passed. Uses hostapd + dnsmasq configs
    materialized into /tmp/wizard_*.conf, then exec's them (background). The
    caller is responsible for tearing down; in production the wizard reboots.
    """
    steps = []

    hostapd_cfg = _fill_template(HOSTAPD_TMPL, {
        "interface": iface,
        "ssid": ssid,
        "channel": "6",
        "wpa_passphrase": passphrase,
    })
    dnsmasq_cfg = _fill_template(DNSMASQ_TMPL, {
        "interface": iface,
        "dhcp-range": f"{ap_ip.rsplit('.', 1)[0]}.10,{ap_ip.rsplit('.', 1)[0]}.50,12h",
    })

    with tempfile.NamedTemporaryFile("w", suffix=".conf", delete=False, prefix="wizard_hostapd_") as hf:
        hf.write(hostapd_cfg)
        hostapd_path = hf.name
    with tempfile.NamedTemporaryFile("w", suffix=".conf", delete=False, prefix="wizard_dnsmasq_") as df:
        df.write(dnsmasq_cfg)
        dnsmasq_path = df.name

    # Assign the AP IP to the interface (best-effort; ignore errors on dev box).
    for cmd in (
        ["ip", "link", "set", iface, "up"],
        ["ip", "addr", "add", f"{ap_ip}/24", "dev", iface],
        ["dnsmasq", "-C", dnsmasq_path, "--keep-in-foreground"],
        ["hostapd", "--config", hostapd_path],
    ):
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            steps.append(0)
        except FileNotFoundError:
            # tool not installed on dev machine -> graceful MISSING
            steps.append(127)
    return steps


# ---------------------------------------------------------------------------
# HTTP form
# ---------------------------------------------------------------------------

FORM_HTML = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>DroneCtl first-boot wizard</title>
<style>
body{{font:16px/1.5 system-ui,-apple-system,sans-serif;margin:2em auto;max-width:42em;color:#222}}
h1{{font-size:1.4em}} label{{display:block;margin:.6em 0 .15em;font-weight:600}}
input,select{{font:inherit;padding:.35em .5em;width:100%;box-sizing:border-box}}
.row{{display:flex;gap:1em}} .row>label{{flex:1}}
button{{margin-top:1.2em;padding:.6em 1.2em;font:inherit}}
.muted{{color:#666;font-size:.9em}}
</style></head><body>
<h1>DroneCtl setup</h1>
<form method="post" action="/">
<fieldset>
 <legend>Identity</legend>
 <label>drone_id <input name="drone_id" value="{drone_id}"></label>
</fieldset>
<fieldset>
 <legend>Camera</legend>
 <label>camera.type
  <select name="camera.type">{cam_opts}</select></label>
 <div class="row">
  <label>width <input name="camera.width" value="{camera.width}"></label>
  <label>height <input name="camera.height" value="{camera.height}"></label>
  <label>fov_deg <input name="camera.fov_deg" value="{camera.fov_deg}"></label>
 </div>
</fieldset>
<fieldset>
 <legend>Flight controller</legend>
 <label>fc.url <input name="fc.url" value="{fc.url}"></label>
 <div class="row">
  <label>max_velocity_ms <input name="fc.max_velocity_ms" value="{fc.max_velocity_ms}"></label>
  <label>max_accel_ms2 <input name="fc.max_accel_ms2" value="{fc.max_accel_ms2}"></label>
 </div>
</fieldset>
<fieldset>
 <legend>Tether (fiber)</legend>
 <div class="row">
  <label>tether.iface <input name="tether.iface" value="{tether.iface}"></label>
  <label>tether.self_ip <input name="tether.self_ip" value="{tether.self_ip}"></label>
  <label>tether.ground_ip <input name="tether.ground_ip" value="{tether.ground_ip}"></label>
 </div>
</fieldset>
<fieldset>
 <legend>Mission server (optional)</legend>
 <label>server.endpoint <input name="server.endpoint" value="{server.endpoint}" placeholder="https://ai.example.org"></label>
</fieldset>
<button type="submit">Save &amp; reboot</button>
<p class="muted">Grid/cfc/failsafe defaults are kept unless set in the raw form.</p>
</form></body></html>"""


def render_form_html(form: dict) -> str:
    opts = []
    for t in CAMERA_TYPES:
        sel = " selected" if t == form.get("camera.type") else ""
        opts.append(f'<option value="{t}"{sel}>{t}</option>')
    fields = dict(DEFAULT_FORM)
    fields.update(form)
    fields["cam_opts"] = "\n  ".join(opts)
    return FORM_HTML.format(**fields)


class WizardHandler(BaseHTTPRequestHandler):
    server_version = "dronectl-wizard/1.0"

    def log_message(self, fmt, *args):  # silence default stderr noise
        pass

    def do_GET(self):
        if self.path not in ("/", "/index.html"):
            self.send_error(404)
            return
        body = render_form_html(self.server.form_defaults).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0") or 0)
        raw = self.rfile.read(length) if length else b""
        form = self._parse_form(raw)
        cfg_text = render_config(form)
        self.server.last_config = cfg_text
        self.server.last_form = form

        applied = getattr(self.server, "apply", False)
        if applied:
            ok, msg = self.server.persist(cfg_text)
        else:
            ok, msg = True, "dry-run: not persisted (--apply not set)"

        body = self._result_html(ok, msg, cfg_text, form).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

        if applied and ok and getattr(self.server, "reboot", True):
            # disable ourselves then reboot
            self.server.schedule_finish()

    @staticmethod
    def _parse_form(raw: bytes) -> dict:
        from urllib.parse import parse_qs
        parsed = parse_qs(raw.decode("utf-8", "replace"))
        return {k: (v[0] if v else "") for k, v in parsed.items()}

    @staticmethod
    def _result_html(ok, msg, cfg_text, form):
        drone_id = html.escape(form.get("drone_id", ""))
        status = "Saved &amp; rebooting…" if ok else "Not saved"
        pre = html.escape(cfg_text)
        return (
            "<!doctype html><html><head><meta charset='utf-8'>"
            "<title>DroneCtl wizard — done</title>"
            "<style>body{font:16px/1.5 system-ui;max-width:42em;margin:2em auto}"
            "pre{background:#f4f4f4;padding:1em;white-space:pre-wrap}</style>"
            f"</head><body><h1>{status}</h1><p>{html.escape(msg)}</p>"
            f"<p>drone_id: <code>{drone_id}</code></p>"
            f"<h2>Generated config</h2><pre>{pre}</pre></body></html>"
        )


class WizardServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, handler, *, form_defaults, apply_flag, config_path, do_reboot):
        super().__init__(addr, handler)
        self.form_defaults = form_defaults
        self.apply = apply_flag
        self.config_path = config_path
        self.reboot = do_reboot
        self.last_config = ""
        self.last_form = {}
        self._finish_pending = False

    def persist(self, cfg_text: str):
        try:
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            with open(self.config_path, "w", encoding="utf-8") as f:
                f.write(cfg_text)
            return True, f"wrote {self.config_path}"
        except OSError as e:
            return False, f"write failed: {e}"

    def schedule_finish(self):
        self._finish_pending = True

    def finish_request(self, request, client_address):
        super().finish_request(request, client_address)
        if self._finish_pending:
            self._disable_and_reboot()

    @staticmethod
    def _disable_and_reboot():
        for cmd in (["systemctl", "disable", WIZARD_UNIT],
                    ["systemctl", "reboot"]):
            try:
                subprocess.run(cmd, check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except FileNotFoundError:
                pass


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _default_ssid() -> str:
    try:
        host = socket.gethostname() or "pi"
    except OSError:
        host = "pi"
    short = host.strip().lower().replace(" ", "-")[:16] or "pi"
    return f"dronectl-{short}"


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="wizard",
        description="DroneCtl first-boot provisioning wizard (manual Part 8.4).",
        add_help=True,
    )
    p.add_argument("--bring-up-ap", action="store_true",
                   help="Start the Wi-Fi AP (hostapd+dnsmasq). Default: do not.")
    p.add_argument("--apply", action="store_true",
                   help="Persist config + disable wizard unit + reboot on POST.")
    p.add_argument("--dry-run", action="store_true",
                   help="Render the default config to stdout and exit.")
    p.add_argument("--bind", default=DEFAULT_BIND_IP, metavar="IP",
                   help=f"HTTP bind IP (default {DEFAULT_BIND_IP}).")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, metavar="N",
                   help="HTTP port (default 80).")
    p.add_argument("--config", default=DEFAULT_CONFIG_PATH, metavar="PATH",
                   help=f"Config target (default {DEFAULT_CONFIG_PATH}).")
    p.add_argument("--iface", default=DEFAULT_IFACE, metavar="NAME",
                   help=f"wlan interface for the AP (default {DEFAULT_IFACE}).")
    p.add_argument("--ssid", default=_default_ssid(), metavar="SSID",
                   help="AP SSID (default dronectl-<host>).")
    p.add_argument("--passphrase", default=DEFAULT_PASSPHRASE, metavar="S",
                   help="WPA2 passphrase (default dronectl-setup).")
    p.add_argument("--no-reboot", action="store_true",
                   help="With --apply, persist + disable but skip the reboot.")
    return p


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)

    if args.dry_run:
        sys.stdout.write(render_config(dict(DEFAULT_FORM)))
        return 0

    if args.bring_up_ap:
        rc = bring_up_ap(args.iface, args.ssid, args.passphrase, args.bind)
        missing = [name for name, code in zip(("ip", "dnsmasq", "hostapd"), rc) if code == 127]
        if missing:
            print(f"WARNING: AP tool(s) missing on this host: {', '.join(missing)}",
                  file=sys.stderr)
        print(f"AP bring-up attempted on {args.iface} as {args.ssid} ({args.bind})",
              file=sys.stderr)

    server = WizardServer(
        (args.bind, args.port),
        WizardHandler,
        form_defaults=dict(DEFAULT_FORM),
        apply_flag=args.apply,
        config_path=args.config,
        do_reboot=not args.no_reboot,
    )
    print(f"dronectl wizard serving http://{args.bind}:{args.port}/ "
          f"(apply={args.apply}, config={args.config})", file=sys.stderr)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())