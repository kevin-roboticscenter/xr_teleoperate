# Preview browser checks

These checks exercise the static preview in a separate headless browser. They
never run the G1/H2 integration tests, connect to a robot or load its SDKs.

Start the preview with `python3 dashboard/serve.py`. In a second terminal, start
Chrome with a separate temporary browser profile:

```bash
google-chrome --headless --remote-debugging-address=127.0.0.1 \
  --remote-debugging-port=9224 --user-data-dir=/tmp/robot-dashboard-browser-check \
  about:blank
```

From the repository root:

```bash
python3 -m venv dashboard/.venv-checks
dashboard/.venv-checks/bin/pip install -r dashboard/checks/requirements.txt
dashboard/.venv-checks/bin/python dashboard/checks/browser_check.py
```

The script checks official image loading, robot selection, independent session state, Prepare/Start/Stop,
model-specific controls, camera lists, Add robot, and mobile overflow. It also
checks for browser exceptions and robot API requests. Its disposable preview
profiles live only in that browser profile. Screenshots and results are written
to ignored `dashboard/artifacts/`.

Use `--url`, `--cdp` or `--output` to override the preview URL, debug endpoint or
artifact directory. Bind the debug endpoint to loopback only.
