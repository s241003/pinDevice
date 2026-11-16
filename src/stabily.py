"""BNO055 の方位が揺れる原因を調べる測定ツール (userDirection.py と同じフォルダに置いて、ラズパイで実行する)。

  1) 起動のたびに方位がずれる問題:  センサーを毎回初期化し直し、同じ向きで静止させたまま測る
        python headingStabilityTest.py repeat --runs 5 --duration 30

  2) ソレノイドの動作が方位に影響するか:  ソレノイドを1つずつ・全部、挙上させて、方位と磁場(µT)の変化を測る
        python headingStabilityTest.py solenoid --duration 8

測定中は、センサーを同じ向きのまま、動かさないでください (机の上に固定するなど)。
結果は画面に表示され、repeat は CSV (heading_repeat_日時.csv) にも保存されます。
"""
import argparse
import csv
import datetime
import math
import sys
import time

import userDirection as ud


def read_mag(reader):
    """磁気センサーの値 (x, y, z) [µT]。読めなければ None"""
    try:
        b = reader.sensor.magnetic
        if b is None or any(v is None for v in b):
            return None
        return tuple(float(v) for v in b)
    except (RuntimeError, ValueError, OSError):
        reader._uart.reset_input_buffer()
        return None


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def mean_vec(vs):
    return tuple(sum(v[i] for v in vs) / len(vs) for i in range(3))


def collect(reader, duration, interval=0.1):
    """duration 秒のあいだ読み続ける。 [(t, heading, cal, mag)]"""
    out, t0 = [], time.monotonic()
    while time.monotonic() - t0 < duration:
        h = reader._read_heading_once(retries=2)
        cal = reader._read_cal()
        m = read_mag(reader)
        if h is not None:
            out.append((time.monotonic() - t0, h, cal, m))
        time.sleep(interval)
    return out


# ---------------------------------------------------------------
# 1) 起動のたびにずれるか
# ---------------------------------------------------------------
def cmd_repeat(args):
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"heading_repeat_{stamp}.csv"
    runs = []
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["run", "t_s", "heading_deg", "sys", "gyro", "acc", "mag", "Bx_uT", "By_uT", "Bz_uT"])
        for r in range(1, args.runs + 1):
            print(f"\n=== 測定 {r}/{args.runs}: センサーを初期化して {args.duration:.0f} 秒測ります (動かさないでください) ===")
            reader = ud.HeadingReader(mode=args.mode, settle_timeout=0)      # 待たずに、起動直後の様子も見る
            applied = reader.calibration_applied
            data = collect(reader, args.duration)
            reader.close()
            time.sleep(1.0)
            if len(data) < 10:
                print("  読み取れた回数が少なすぎます。配線とUARTを確認してください")
                continue
            for t, h, cal, m in data:
                c = cal if cal else (None,) * 4
                mm = m if m else (None,) * 3
                w.writerow([r, f"{t:.2f}", f"{h:.2f}", *c, *mm])

            def at(sec):
                """sec 秒付近 (前後0.5秒) の円周平均"""
                near = [h for t, h, _, _ in data if abs(t - sec) <= 0.5]
                return ud.circular_mean_std(near)[0] if near else None

            fin = [d for d in data if d[0] >= args.duration - args.final]
            fmean, fstd = ud.circular_mean_std([d[1] for d in fin])
            mags = [norm(d[3]) for d in fin if d[3]]
            t_mag3 = next((t for t, _, cal, _ in data if cal and cal[3] == 3), None)
            res = dict(run=r, applied=applied, fmean=fmean, fstd=fstd,
                       cps={s: at(s) for s in (2, 5, 10, 20) if s < args.duration},
                       Bmean=sum(mags) / len(mags) if mags else None, t_mag3=t_mag3,
                       cal_end=data[-1][2])
            runs.append(res)
            cp = "  ".join(f"{s}秒:{v:6.1f}°" for s, v in res["cps"].items() if v is not None)
            print(f"  校正値の適用: {'成功' if applied else '失敗 (校正が効いていません)'}   mag校正が3になるまで: "
                  f"{'-' if t_mag3 is None else f'{t_mag3:.1f}秒'}   終了時の校正: {res['cal_end']}")
            print(f"  経過ごとの方位: {cp}")
            print(f"  最後の{args.final:.0f}秒: 平均 {fmean:.2f}°  ばらつき {fstd:.2f}°"
                  + (f"   磁場の大きさ |B| = {res['Bmean']:.1f} µT" if res['Bmean'] else ""))

    if len(runs) < 2:
        print("\n複数回の測定ができなかったので、比較はできません")
        return
    means = [x["fmean"] for x in runs]
    m_all, s_all = ud.circular_mean_std(means)
    spread = max(abs(ud.angle_diff(a, b)) for a in means for b in means)
    print("\n================ まとめ ================")
    print(f"各回の最終方位: " + ", ".join(f"{m:.1f}°" for m in means))
    print(f"回ごとのずれ (起動のたびのずれ): 円周標準偏差 {s_all:.2f}°  /  最大の差 {spread:.2f}°")
    print(f"同じ回の中のばらつき (平均): {sum(x['fstd'] for x in runs) / len(runs):.2f}°")
    n_fail = sum(1 for x in runs if not x["applied"])
    if n_fail:
        print(f"→ 校正値の適用に失敗した回が {n_fail} 回あります。まず、これを直してください (userDirection.py のログを確認)")
    if spread > 2.0:
        print("→ 起動のたびに2°以上ずれています。校正値が効いていない、磁気センサーの収束待ちが足りない、"
              "または磁気の乱れが原因の可能性があります。solenoid の測定で、乱れの有無を調べてください")
    elif spread > 0.5:
        print("→ 起動のたびのずれは0.5〜2°です。データシートの、磁気センサーの方位精度 ±2.5° の範囲内です")
    else:
        print("→ 起動のたびのずれは0.5°以下です")
    print(f"生データ → {csv_path}")


# ---------------------------------------------------------------
# 2) ソレノイドが方位に影響するか
# ---------------------------------------------------------------
def cmd_solenoid(args):
    from solenoid import Display            # gpiozero が必要 (ラズパイで実行)

    disp = Display()
    reader = ud.HeadingReader(mode=args.mode, settle_timeout=args.settle)
    conds = ([("全部OFF (基準)", set())] + [(f"ソレノイド{i}のみ", {i}) for i in range(1, 7)]
             + [("全部ON", {1, 2, 3, 4, 5, 6}), ("全部OFF (最後)", set())])
    rows = []
    try:
        for name, s in conds:
            disp.set(s, "solid")
            time.sleep(args.wait)                       # 挙上して、落ち着くまで待つ
            data = collect(reader, args.duration)
            hs = [d[1] for d in data]
            ms = [d[3] for d in data if d[3]]
            if len(hs) < 5 or not ms:
                print(f"  {name}: 読み取れませんでした")
                continue
            hm, hstd = ud.circular_mean_std(hs)
            rows.append((name, hm, hstd, mean_vec(ms)))
    finally:
        disp.set(set(), "solid")

    if len(rows) < 2:
        print("測定できませんでした")
        return
    base_h, base_B = rows[0][1], rows[0][3]
    print("\n条件               方位[°]  基準との差[°]  ばらつき[°]   磁場の変化 |ΔB|[µT]   |B|[µT]")
    flagged = []
    for name, hm, hstd, B in rows:
        dh = ud.angle_diff(hm, base_h)
        dB = norm(tuple(B[i] - base_B[i] for i in range(3)))
        mark = ""
        if abs(dh) >= args.flag_deg or dB >= args.flag_ut:
            mark = "  ← 影響あり"
            flagged.append(name)
        print(f"{name:<18} {hm:7.1f} {dh:+12.2f} {hstd:12.2f} {dB:18.2f} {norm(B):10.1f}{mark}")
    last_dh = ud.angle_diff(rows[-1][1], base_h)
    print(f"\n基準と最後 (どちらも全部OFF) の差: {last_dh:+.2f}°  ← これが、ソレノイド以外の要因 (ドリフトなど) の目安")
    if flagged:
        print("→ ソレノイドの動作で、方位や磁場が変わっています: " + "、".join(flagged))
        print("   対策: ソレノイドと電流の流れる配線を、BNO055から離す / 電源の往復の線をより合わせる / "
              "センサーの位置を変える / (ソレノイドがONの間は方位を信用しない)")
    else:
        print("→ ソレノイドの動作による、明らかな方位・磁場の変化は見られませんでした")


def main():
    ap = argparse.ArgumentParser(description="BNO055の方位の安定性を測る")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p1 = sub.add_parser("repeat", help="センサーを初期化し直して、起動のたびのずれを測る")
    p1.add_argument("--runs", type=int, default=5, help="繰り返す回数 (既定 5)")
    p1.add_argument("--duration", type=float, default=30.0, help="1回あたりの測定秒数 (既定 30)")
    p1.add_argument("--final", type=float, default=5.0, help="最後のこの秒数を、その回の値とする (既定 5)")
    p1.add_argument("--mode", choices=["ndof", "fmc_off"], default="ndof")
    p1.set_defaults(func=cmd_repeat)

    p2 = sub.add_parser("solenoid", help="ソレノイドの動作が方位・磁場に影響するかを測る")
    p2.add_argument("--duration", type=float, default=8.0, help="条件ごとの測定秒数 (既定 8)")
    p2.add_argument("--wait", type=float, default=1.5, help="挙上してから測り始めるまでの待ち秒数 (既定 1.5)")
    p2.add_argument("--settle", type=float, default=15.0, help="最初に方位が落ち着くまで待つ最大秒数 (既定 15)")
    p2.add_argument("--flag-deg", type=float, default=1.0, help="これ以上方位が変われば「影響あり」 (既定 1.0°)")
    p2.add_argument("--flag-ut", type=float, default=1.0, help="これ以上磁場が変われば「影響あり」 (既定 1.0 µT)")
    p2.add_argument("--mode", choices=["ndof", "fmc_off"], default="ndof")
    p2.set_defaults(func=cmd_solenoid)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()