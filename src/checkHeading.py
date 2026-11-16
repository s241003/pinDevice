"""[時刻, 相対角, heading, bearing] のリストから、コンパス上の針で表示するHTMLを出力する。

    python compass_view.py log.txt              # → compass_view.html
    python compass_view.py log.txt -o out.html
    python compass_view.py --demo               # ダミーデータで動作確認

入力の形式 (1行に1組。行頭の [ や行末の ], は付けても付けなくてもよい):
    ["12:34:56", 30.0, 100.0, 130.0],
    [12:34:56, 30.0, 100.0, 130.0],
  時刻 ... HH:MM:SS
  相対角 ... bearing - heading (実際にユーザが進むべき方向)
  heading ... BNO055の方位
  bearing ... いま位置しているエッジの方位 (vincenty_inverse の azimuth1)
角度はすべて度で、0°=北、時計回りに増加。
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

# 時刻, 相対角, heading, bearing を拾う (時刻の引用符や、前後の [ ] , の有無は問わない)
ROW = re.compile(
    r'\[(\d{2}):(\d{2}):(\d{2})["\']?\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\],'
)


def parse_rows(text):
    """テキストから [時刻文字列, 相対角, heading, bearing, 通算秒] のリストを作る"""
    rows, day_offset, prev = [], 0, None
    for m in ROW.finditer(text):
        h, mi, s = int(m.group(1)), int(m.group(2)), int(m.group(3))
        sec = h * 3600 + mi * 60 + s
        if prev is not None and sec + day_offset < prev - 3600:    # 日付をまたいだ (23:59→00:00)
            day_offset += 86400
        t = sec + day_offset
        prev = t
        rows.append([f"{h:02d}:{mi:02d}:{s:02d}",
                     float(m.group(4)), float(m.group(5)), float(m.group(6)), t])
    return rows


def check_rows(rows, text):
    """入力の読み取り漏れと、相対角が bearing - heading と合わない行を知らせる"""
    n_lines = sum(1 for ln in text.splitlines() if re.search(r"\[\d{2}:\d{2}:\d{2}", ln))
    if n_lines > len(rows):
        print(f"警告: 時刻を含む行が {n_lines} 行ありますが、読み取れたのは {len(rows)} 行です "
              f"(角度が None や欠けている行は飛ばしています)", file=sys.stderr)
    bad = 0
    for _, rel, head, bear, _ in rows:
        diff = ((bear - head) - rel + 540.0) % 360.0 - 180.0          # 360°の周期を考慮した差
        if abs(diff) > 1.0:
            bad += 1
    if bad:
        print(f"警告: {bad} 行で、相対角が (bearing - heading) と1°以上ずれています", file=sys.stderr)


def demo_text():
    """ダミー: ユーザが右へ旋回しながら、進むべき方位に向いていく様子"""
    lines = []
    for i in range(90):
        sec = 12 * 3600 + 34 * 60 + i
        bearing = 90.0 if i < 45 else 10.0
        heading = (350 + i * 2.0) % 360 if i < 45 else (80 - (i - 45) * 1.6) % 360
        rel = ((bearing - heading + 180) % 360) - 180
        lines.append(f'["{sec // 3600:02d}:{sec % 3600 // 60:02d}:{sec % 60:02d}", '
                     f'{rel:.1f}, {heading:.1f}, {bearing:.1f}],')
    return "\n".join(lines)


def build_html(rows):
    return TEMPLATE.replace("__DATA__", json.dumps(rows, ensure_ascii=False))


def main():
    ap = argparse.ArgumentParser(description="コンパスの針でheading/bearing/相対角を表示するHTMLを作る")
    ap.add_argument("input", nargs="?", help="入力ファイル ([時刻, 相対角, heading, bearing] を1行ずつ)")
    ap.add_argument("-o", "--output", default="compass_view.html", help="出力HTML (既定: compass_view.html)")
    ap.add_argument("--demo", action="store_true", help="ダミーデータで出力する")
    args = ap.parse_args()

    if args.demo:
        text = demo_text()
    elif args.input:
        text = Path(args.input).read_text(encoding="utf-8")
    else:
        ap.error("入力ファイルか --demo を指定してください")

    rows = parse_rows(text)
    if not rows:
        sys.exit("エラー: 読み取れる行がありません。形式: [\"HH:MM:SS\", 相対角, heading, bearing]")
    check_rows(rows, text)

    Path(args.output).write_text(build_html(rows), encoding="utf-8")
    print(f"{len(rows)} 件を {args.output} に出力しました (ブラウザで開いてください)")


TEMPLATE = r"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>コンパス針ビューア</title>
<style>
  :root { --bg:#fff; --fg:#1a1a1a; --muted:#666; --ring:#888; --panel:#f3f3f6;
          --rel:#e63946; --head:#1d6fe0; --bear:#2a9d4a; }
  @media (prefers-color-scheme: dark) {
    :root { --bg:#16171a; --fg:#eee; --muted:#9a9aa2; --ring:#777; --panel:#23252a;
            --rel:#ff6b78; --head:#5aa2ff; --bear:#4cc76b; }
  }
  body { margin:0; padding:12px; background:var(--bg); color:var(--fg);
         font-family:system-ui,sans-serif; display:flex; flex-direction:column; align-items:center; }
  #time { font-size:34px; font-weight:600; font-variant-numeric:tabular-nums; }
  #idx { color:var(--muted); font-size:13px; }
  svg { width:min(92vw,520px); height:auto; }
  #readout { display:flex; flex-wrap:wrap; gap:8px; justify-content:center; margin:6px 0; }
  .chip { background:var(--panel); border-radius:8px; padding:6px 10px; min-width:150px; }
  .chip .name { font-size:12px; color:var(--muted); }
  .chip .name i { display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:5px; }
  .chip .val { font-size:20px; font-variant-numeric:tabular-nums; }
  #controls { display:flex; flex-wrap:wrap; gap:8px; align-items:center; justify-content:center; margin:8px 0; }
  button, select { font-size:15px; padding:6px 12px; }
  #slider { width:min(92vw,520px); }
  #note { color:var(--muted); font-size:12px; margin-top:6px; text-align:center; }
</style>
</head>
<body>
<div id="time">--:--:--</div>
<div id="idx"></div>
<svg id="dial" viewBox="0 0 400 400"></svg>
<div id="readout">
  <div class="chip"><div class="name"><i style="background:var(--rel)"></i>相対角 (bearing − heading)</div><div class="val" id="v-rel">-</div></div>
  <div class="chip"><div class="name"><i style="background:var(--head)"></i>heading (BNO055)</div><div class="val" id="v-head">-</div></div>
  <div class="chip"><div class="name"><i style="background:var(--bear)"></i>bearing (エッジの方位)</div><div class="val" id="v-bear">-</div></div>
</div>
<input type="range" id="slider" min="0" value="0">
<div id="controls">
  <button id="prev">◀ 前へ</button>
  <button id="play">▶ 再生</button>
  <button id="next">次へ ▶</button>
  <label>速度 <select id="speed">
    <option value="0.5">0.5×</option><option value="1" selected>1×</option>
    <option value="2">2×</option><option value="5">5×</option><option value="10">10×</option>
  </select></label>
</div>
<div id="note">0° = 北 (上)、時計回りに増加。 最も長く太い針が相対角。 キー操作: ← → で1件ずつ、スペースで再生/停止</div>

<script>
const DATA = __DATA__;      // [["HH:MM:SS", 相対角, heading, bearing, 通算秒], ...]
const N = DATA.length;
const NS = "http://www.w3.org/2000/svg";
const svg = document.getElementById("dial");
const C = 200;

function el(tag, attrs, parent) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  (parent || svg).appendChild(e);
  return e;
}

// ---- 文字盤 (0°=北が真上に固定) ----
el("circle", { cx: C, cy: C, r: 150, fill: "none", style: "stroke:var(--ring)", "stroke-width": 2 });
for (let d = 0; d < 360; d += 10) {
  const major = d % 30 === 0, r1 = major ? 134 : 142, a = d * Math.PI / 180;
  el("line", { x1: C + r1 * Math.sin(a), y1: C - r1 * Math.cos(a),
               x2: C + 150 * Math.sin(a), y2: C - 150 * Math.cos(a),
               style: "stroke:var(--ring)", "stroke-width": major ? 2 : 1 });
}
const CARD = { 0: "N", 90: "E", 180: "S", 270: "W" };
for (let d = 0; d < 360; d += 30) {
  const a = d * Math.PI / 180, r = 169, card = CARD[d];
  const t = el("text", { x: C + r * Math.sin(a), y: C - r * Math.cos(a),
                         "text-anchor": "middle", "dominant-baseline": "central",
                         style: "fill:var(--fg)", "font-size": card ? 20 : 13,
                         "font-weight": card ? 700 : 400 });
  t.textContent = card ? card : String(d);
}

// ---- 針 (作った順に重なる: 長い相対角が一番下、heading が一番上) ----
function makeNeedle(len, w, tail, colorVar) {
  const g = el("g", { transform: `translate(${C},${C}) rotate(0)` });
  el("polygon", { points: `0,${-len} ${w},0 0,${tail} ${-w},0`,
                  style: `fill:var(${colorVar});stroke:var(--bg);stroke-width:1.5;opacity:.95` }, g);
  return g;
}
const needleRel  = makeNeedle(142, 12, 34, "--rel");     // 最大
const needleBear = makeNeedle(76, 7, 18, "--bear");
const needleHead = makeNeedle(106, 8, 22, "--head");
el("circle", { cx: C, cy: C, r: 6, style: "fill:var(--fg)" });

function rotate(g, deg) { g.setAttribute("transform", `translate(${C},${C}) rotate(${deg})`); }
const norm360 = x => ((x % 360) + 360) % 360;
const norm180 = x => ((x + 180) % 360 + 360) % 360 - 180;
function fmtRel(x) {
  const v = norm180(x);
  if (Math.abs(v) < 0.05) return "0.0°";
  return (v > 0 ? "右 " : "左 ") + Math.abs(v).toFixed(1) + "°";
}

// ---- 表示と再生 ----
const slider = document.getElementById("slider");
slider.max = N - 1;
let idx = 0, playing = false, vt = DATA[0][4], lastTs = null;

function render() {
  const [t, rel, head, bear] = DATA[idx];
  document.getElementById("time").textContent = t;
  document.getElementById("idx").textContent = (idx + 1) + " / " + N;
  rotate(needleRel, rel);
  rotate(needleHead, head);
  rotate(needleBear, bear);
  document.getElementById("v-rel").textContent = fmtRel(rel);
  document.getElementById("v-head").textContent = norm360(head).toFixed(1) + "°";
  document.getElementById("v-bear").textContent = norm360(bear).toFixed(1) + "°";
  slider.value = idx;
}

function setIdx(i) {
  idx = Math.max(0, Math.min(N - 1, i));
  vt = DATA[idx][4];
  render();
}

function setPlaying(p) {
  playing = p;
  document.getElementById("play").textContent = p ? "⏸ 停止" : "▶ 再生";
  if (p) {
    if (idx >= N - 1) setIdx(0);
    lastTs = null;
    requestAnimationFrame(loop);
  }
}

function loop(ts) {
  if (!playing) return;
  const speed = parseFloat(document.getElementById("speed").value);
  if (lastTs !== null) vt += (ts - lastTs) / 1000 * speed;      // 記録の時刻どおりの速さで進める
  lastTs = ts;
  while (idx < N - 1 && DATA[idx + 1][4] <= vt) idx++;
  if (vt >= DATA[N - 1][4]) { idx = N - 1; render(); setPlaying(false); return; }
  render();
  requestAnimationFrame(loop);
}

slider.addEventListener("input", () => setIdx(parseInt(slider.value, 10)));
document.getElementById("prev").addEventListener("click", () => { setPlaying(false); setIdx(idx - 1); });
document.getElementById("next").addEventListener("click", () => { setPlaying(false); setIdx(idx + 1); });
document.getElementById("play").addEventListener("click", () => setPlaying(!playing));
document.addEventListener("keydown", e => {
  if (e.key === "ArrowLeft") { setPlaying(false); setIdx(idx - 1); }
  else if (e.key === "ArrowRight") { setPlaying(false); setIdx(idx + 1); }
  else if (e.key === " ") { e.preventDefault(); setPlaying(!playing); }
});

render();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()