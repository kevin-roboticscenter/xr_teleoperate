"""Browser interaction and screenshots for the standalone UI preview only."""
import asyncio
import base64
import json
from pathlib import Path
from urllib.request import urlopen

import websockets

import argparse
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--url', default='http://127.0.0.1:8790/')
parser.add_argument('--cdp', default='http://127.0.0.1:9224')
parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'artifacts')
args = parser.parse_args()
ROOT = args.output.resolve()
ROOT.mkdir(parents=True, exist_ok=True)


async def main():
    with urlopen(args.cdp.rstrip('/') + '/json/list', timeout=5) as response:
        page = next(p for p in json.load(response) if p['type'] == 'page')
    events, checks = [], []
    async with websockets.connect(page['webSocketDebuggerUrl'], max_size=20_000_000) as ws:
        sequence = 0

        async def call(method, params=None):
            nonlocal sequence
            sequence += 1
            request_id = sequence
            await ws.send(json.dumps({'id': request_id, 'method': method, 'params': params or {}}))
            while True:
                result = json.loads(await asyncio.wait_for(ws.recv(), 12))
                if 'method' in result:
                    events.append(result)
                if result.get('id') == request_id:
                    if 'error' in result:
                        raise RuntimeError(result['error'])
                    return result.get('result', {})

        async def evaluate(expression):
            result = await call('Runtime.evaluate', {'expression': expression, 'returnByValue': True, 'awaitPromise': True})
            if 'exceptionDetails' in result:
                raise RuntimeError(result['exceptionDetails'])
            return result['result'].get('value')

        async def expect(label, expression):
            value = await evaluate(expression)
            assert value is True, (label, value)
            checks.append(label)
            print('PASS', label, flush=True)

        async def click(selector):
            await evaluate(f'document.querySelector({json.dumps(selector)}).click()')
            await asyncio.sleep(.12)

        async def screenshot(name):
            await evaluate("document.querySelector('#toast').classList.remove('visible')")
            await asyncio.sleep(.2)
            result = await call('Page.captureScreenshot', {'format': 'png', 'captureBeyondViewport': True})
            target = ROOT / 'screenshots' / name
            target.parent.mkdir(exist_ok=True)
            target.write_bytes(base64.b64decode(result['data']))

        await call('Page.enable')
        await call('Runtime.enable')
        await call('Network.enable')
        await call('Emulation.setDeviceMetricsOverride', {'width': 1440, 'height': 1040, 'deviceScaleFactor': 1, 'mobile': False})
        await call('Page.navigate', {'url': args.url})
        await asyncio.sleep(.5)
        await click('[data-action="reset"]')
        await expect('Robot picker contains H2 and G1', 'document.querySelectorAll("[data-robot-card]").length === 2')
        await evaluate('Promise.all(Array.from(document.querySelectorAll(".robot-photo"), image => image.decode()))')
        await expect('Official H2 and G1 product images load', 'document.querySelectorAll(".robot-photo").length === 2 && Array.from(document.querySelectorAll(".robot-photo")).every(image => image.complete && image.naturalWidth === 428 && image.naturalHeight === 404 && image.alt.includes("Unitree"))')
        await expect('Preview disclosure stays visible', 'document.querySelector(".preview-banner").textContent.includes("No robots are connected")')
        await screenshot('robots.png')

        await click('[data-action="open"][data-id="h2-main"]')
        await expect('H2 capabilities match its integration', 'document.querySelector("[data-setting=side]").options.length === 2 && !!document.querySelector("[data-setting=headFollow]")')
        await expect('Start waits for preview Prepare', 'document.querySelector("[data-action=start]").disabled')
        await click('[data-action="prepare"]')
        await expect('Prepare enables preview Start', '!document.querySelector("[data-action=start]").disabled')
        await screenshot('h2-operate.png')
        await click('[data-action="start"]')
        await expect('Preview Start changes H2 state', 'document.querySelector(".state-pill").textContent.includes("Following")')

        await evaluate('location.hash="/robots/g1-main/operate"')
        await asyncio.sleep(.12)
        await expect('G1 has independent session state', 'document.querySelector(".state-pill").textContent.includes("Not started") && robots.find(r=>r.id==="h2-main").phase === "following"')
        await expect('New G1 controls and BrainCo hardware are represented', 'document.querySelector("[data-setting=side]").options.length === 3 && !!document.querySelector("[data-setting=reference]") && !document.querySelector("[data-setting=headFollow]") && document.querySelector("main").textContent.includes("BrainCo Revo2")')
        await screenshot('g1-operate.png')
        await click('[data-action="prepare"]')
        await click('[data-action="start"]')
        await click('[data-action="stop"]')
        await expect('G1 Stop does not stop H2 preview state', 'robots.find(r=>r.id==="g1-main").phase === "idle" && robots.find(r=>r.id==="h2-main").phase === "following"')
        await click('[data-tab="cameras"]')
        await expect('G1 camera page has its own camera list', 'document.querySelectorAll(".camera-scene").length === 1')
        await evaluate('location.hash="/robots/h2-main/cameras"')
        await asyncio.sleep(.12)
        await expect('H2 camera page has head and both wrists', 'document.querySelectorAll(".camera-scene").length === 3')

        await click('[data-action="add"]')
        await evaluate('document.querySelector("#add-form [name=name]").value="G1 & Test lab"; document.querySelector("#new-model").value="g1"; document.querySelector("#add-form [name=address]").value="g1-test.local"; document.querySelector("#add-form").requestSubmit()')
        await asyncio.sleep(.15)
        await expect('Add robot creates a separate setup profile', 'robots.length === 3 && document.querySelector("h1").textContent === "G1 & Test lab" && document.querySelector("#connection-form [name=address]").value === "g1-test.local"')
        await click('[data-action="connect"]')
        await expect('Connection action remains a simulation', 'selected().connected && document.querySelector(".connection-status").textContent.includes("Ready")')

        await click('[data-action="reset"]')
        await call('Emulation.setDeviceMetricsOverride', {'width': 390, 'height': 844, 'deviceScaleFactor': 1, 'mobile': True})
        await asyncio.sleep(.2)
        await expect('Robot picker fits a narrow screen', 'document.documentElement.scrollWidth <= innerWidth')
        await screenshot('mobile-robots.png')
        await evaluate('location.hash="/robots/h2-main/operate"')
        await asyncio.sleep(.15)
        await expect('Operate fits a narrow screen', 'document.documentElement.scrollWidth <= innerWidth')
        await screenshot('mobile-operate.png')

        errors = [e for e in events if e['method'] == 'Runtime.exceptionThrown']
        api_requests = [e['params']['request']['url'] for e in events if e['method'] == 'Network.requestWillBeSent' and '/api/' in e['params']['request']['url']]
        assert not errors, errors
        assert not api_requests, api_requests
        print('PASS no browser exceptions or robot API requests', flush=True)
        await call('Emulation.setDeviceMetricsOverride', {'width': 1440, 'height': 1040, 'deviceScaleFactor': 1, 'mobile': False})
        await click('[data-action="reset"]')
        summary = {'checks': checks, 'browser_errors': len(errors), 'robot_api_requests': api_requests, 'screenshots': [p.name for p in (ROOT/'screenshots').glob('*.png')]}
        (ROOT/'preview-checks.json').write_text(json.dumps(summary, indent=2)+'\n')


asyncio.run(main())
