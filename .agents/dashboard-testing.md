<!-- reference — read before changing src/kb_dashboard/kb_dashboard/index.html -->
# Testing the dashboard

The dashboard is one self-contained file, `src/kb_dashboard/kb_dashboard/index.html`: markup, the
whole stylesheet and all the JavaScript. There is no build step and no bundler, so it can be tested
on a laptop with no Orin, no ROS and no ESP32 attached. Do that before deploying — the round trip
through the kart is minutes, and most dashboard bugs are visible in a browser in seconds.

## Run it with no hardware

```bash
cd src/kb_dashboard/kb_dashboard && python3 -m http.server 8765
# then open http://localhost:8765/index.html?demo=1
```

`?demo=1` is a no-backend mode built into the page: telemetry is generated locally, and `wsSend()`
applies mission, state, compressor and PID changes to local variables instead of sending them. The
PID branch even mimics the firmware's clamping, so typing 99 into Kp and watching "In force" come
back at the clamp is testable offline.

The console will fill with `WebSocket connection to 'ws://localhost:8765/' failed`, once every
reconnect attempt. A static file server cannot answer a WebSocket upgrade; this is expected in demo
mode and is not a bug. Any *other* console error is real.

Kill the server when finished: `pkill -f "http.server 8765"`.

## Check the syntax before you open the page

```bash
node --input-type=module -e "
import {readFileSync} from 'fs';
const s = readFileSync('src/kb_dashboard/kb_dashboard/index.html','utf8');
const m = s.match(/<script>([\s\S]*)<\/script>/);
try { new Function(m[1]); console.log('JS parses OK'); } catch(e) { console.log('PARSE ERROR:', e.message); }
"
```

This matters more than it looks, because **the entire race-skin stylesheet lives inside a JavaScript
template literal**. A CSS edit can therefore be a JavaScript syntax error, and the failure mode is
the whole skin failing to apply — a blank or unstyled page — with one parse error in the console.
Two characters are forbidden anywhere in that stylesheet, comments included:

- **Backtick.** It ends the template literal mid-rule.
- **Backslash followed by a digit.** `content:'\25B8'` is a normal CSS unicode escape but parses in a
  template literal as an octal escape: *"Octal escape sequences are not allowed in template
  strings."* Paste the literal character instead.

## Driving the UI from a test harness

**The left nav is a drag wheel, not a row of buttons.** Calling `.click()` on a `.race-tab` does
nothing at all — the handler is a pointerdown/pointerup pair on the rail, and a tap is resolved by
where the pointer landed rather than by the event target. To switch page:

```js
const sleep = ms => new Promise(r => setTimeout(r, ms));
const go = async name => {
  const rail = document.getElementById('rcRail');
  const b = document.querySelector(`.race-tab[data-page="${name}"]`).getBoundingClientRect();
  const o = { bubbles: true, pointerId: 1, isPrimary: true,
              clientX: rail.getBoundingClientRect().left + 10, clientY: b.top + b.height / 2 };
  rail.dispatchEvent(new PointerEvent('pointerdown', o));
  rail.dispatchEvent(new PointerEvent('pointerup', o));
  await sleep(300);
};
await go('mission');   // telemetry | mission | vision | system | battery | ebs
```

**The `await` is load-bearing, not politeness.** The wheel animates the switch over ~220 ms, and the
tab rectangles move while it does — so a second `go()` fired immediately reads a rect that has slid
out from under it and lands on the wrong page. Without the wait, a loop over all six pages selects
`telemetry` six times, which is easy to mistake for the snippet working.

The same wait applies to anything that measures an element on a freshly opened page: during the
animation the rect can still be zero-sized, and the joystick's `getAxes` treats a zero-sized pad as
"centred" and returns 0 — a test that drags the pad then reads 0.00 has measured the animation, not
a bug.

Other entry points:

- Mission buttons (`.mis-btn`) are ordinary buttons and respond to `.click()`.
- The driving pad only exists in mission `remote_control` — select **Remote** first.
- Portrait orientation shows a "rotate the phone — landscape only" screen. Only test landscape.

## What to check, and at what size

Any change to a shared class, to the `--fs-*` type scale, or to anything in `:root` reaches all six
pages. Look at all six — telemetry, mission, vision, system, battery, ebs — at **1280×800** (laptop)
and **844×390** (phone in landscape, which is how it is read at the kart). The phone size is the one
that finds bugs: panes there are short enough that a block which merely looks generous on a laptop
squeezes a neighbour to nothing.

## Verifying what would be sent

In demo mode nothing reaches a server, so to assert on the wire payload, stub the sender:

```js
const sent = []; const real = window.wsSend; window.wsSend = o => sent.push(JSON.stringify(o));
// ...drive the UI...
window.wsSend = real;   // always restore
```

## Playwright MCP gotchas

- **Screenshots can only be written inside the repo, and a bare filename lands in its root.** An
  absolute path outside the repo is refused outright, and `filename: "shot.png"` resolves against
  the repo root — where it is *not* gitignored and will show up in `git status` as an untracked
  file for someone else to clean up. Always write the prefix yourself:
  `filename: ".playwright-mcp/shot.png"`. That directory is gitignored. Check `git status` before
  finishing either way.
- **A screenshot and a measurement can disagree.** The screenshot renders at whatever viewport size
  was requested, but `window.innerHeight` and every `getBoundingClientRect()` report the *real*
  browser window. So a screenshot taken "at 390px tall" may show a layout that no measurement in the
  same session agrees with, and any `vh`-based rule resolves against the real window. Trust the
  rects; treat the screenshot as indicative for anything viewport-relative.
- **A stuck browser looks exactly like a broken page.** If a real click times out at "performing
  click action" and nothing happens, confirm the browser is delivering input at all before blaming
  the code:

  ```js
  window.__evlog = [];
  ['pointerdown','mousedown','click'].forEach(t =>
    window.addEventListener(t, e => window.__evlog.push(t + ' ' + e.isTrusted), true));
  ```

  An empty log after a click means no event reached the page: restart the browser and retest. A
  synthetic `.click()` succeeding while a real click hangs is the signature of this, not of a bug.

## Layout traps in the race skin

- **`.ctrl-btn` sets `flex:1`.** That is right for `.ctrl-row`, which is a row. Dropped into a
  column pane, the same rule makes the button grow *vertically* until it swallows the pane. Override
  with a two-class selector (`.rc-joypane .pid-open`), because a single-class override loses to
  `.ctrl-btn` on source order.
- **`.rc-joypane` is `overflow:hidden`,** unlike the `.rc-mpanel` panes it is built on, which scroll.
  Content that does not fit is clipped and unreachable rather than scrollable, so anything added to
  that pane must fit at 844×390 or be collapsible.
- **Live values change width.** A label/value row that is allowed to wrap will put the value beside
  the label at one reading and under it at the next, so the card visibly twitches several times a
  second as the number counts. Give such rows a fixed layout rather than a wrapping one.
- **Font sizes come from the `--fs-*` scale** defined in `:root` next to the colours (xs/sm/md/lg/xl).
  Use the nearest step instead of a px value. The two deliberate exceptions — the speed hero and the
  steering angle, both sized from the viewport — say so in a comment where they are defined.
- **Dial text is drawn on canvas, not styled by CSS.** Its sizes come from `RC_DIAL_FS`, as fractions
  of the dial radius; the `--fs-*` tokens do not reach it.
