"""
Построение графиков по логу симуляции (sim_log.csv).
Запускать после simulation.py.
"""

import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_log.csv")

def load():
    with open(LOG_FILE, newline="") as f:
        rows = list(csv.DictReader(f))
    return [{k: float(v) for k, v in r.items()} for r in rows]

def main():
    if not os.path.exists(LOG_FILE):
        print(f"Нет лога: {LOG_FILE}\nСначала запустите simulation.py")
        return

    rows = load()
    t    = np.array([r["t"]   for r in rows])
    ex   = np.array([r["ex"]  for r in rows])
    ez   = np.array([r["ez"]  for r in rows])
    tx   = np.array([r["tx"]  for r in rows])
    tz   = np.array([r["tz"]  for r in rows])
    err  = np.array([r["err"] for r in rows])
    q1   = np.array([r["q1"]  for r in rows])
    q2   = np.array([r["q2"]  for r in rows])
    q1t  = np.array([r["q1t"] for r in rows])
    q2t  = np.array([r["q2t"] for r in rows])
    tidx = np.array([r["tidx"] for r in rows])

    # Моменты смены цели
    changes = [t[i] for i in range(1, len(rows)) if rows[i]["tidx"] != rows[i-1]["tidx"]]

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(14, 9), facecolor="#0d0d0d")
    fig.suptitle("Двухзвенный маятник — IK + POSITION_CONTROL",
                 fontsize=14, color="#e0e0e0", fontweight="bold")

    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.5, wspace=0.35,
                           left=0.08, right=0.97, top=0.92, bottom=0.06)

    axes = [fig.add_subplot(gs[i, j]) for i in range(3) for j in range(2)]
    ax_traj, ax_err, ax_x, ax_z, ax_q1, ax_q2 = axes

    for ax in axes:
        ax.set_facecolor("#161616")
        for s in ax.spines.values(): s.set_edgecolor("#333")
        ax.tick_params(colors="#888", labelsize=8)
        ax.title.set_color("#ccc")
        ax.xaxis.label.set_color("#aaa")
        ax.yaxis.label.set_color("#aaa")
        ax.grid(True, color="#2a2a2a", linewidth=0.6)

    cmap = plt.cm.tab10
    n_tgt = int(tidx.max()) + 1

    # 1. Траектория EEF
    ax_traj.set_title("Траектория EEF в плоскости X–Z")
    ax_traj.set_xlabel("X (м)"); ax_traj.set_ylabel("Z (м)")
    for ti in range(n_tgt):
        mask = tidx == ti
        if mask.sum() < 2: continue
        ax_traj.plot(ex[mask], ez[mask], color=cmap(ti/10), lw=1.2,
                     label=f"Цель {ti+1}", alpha=0.85)
    seen = set()
    for r in rows:
        ti = int(r["tidx"])
        if ti not in seen:
            ax_traj.scatter(r["tx"], r["tz"], s=100, color=cmap(ti/10), zorder=5, marker="*")
            seen.add(ti)
    ax_traj.legend(fontsize=7, facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc")

    # 2. Ошибка
    ax_err.set_title("Ошибка позиционирования ||e||")
    ax_err.set_xlabel("Время (с)"); ax_err.set_ylabel("м")
    ax_err.plot(t, err, color="#ff6b6b", lw=1.2)
    ax_err.axhline(0.01, color="#aaa", lw=0.8, ls="--", label="1 см")
    for c in changes: ax_err.axvline(c, color="#555", lw=0.8, ls=":")
    ax_err.legend(fontsize=7, facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc")

    # 3. X(t)
    ax_x.set_title("X эффектора"); ax_x.set_xlabel("Время (с)"); ax_x.set_ylabel("X (м)")
    ax_x.plot(t, ex, "#4fc3f7", lw=1.2, label="EEF X")
    ax_x.plot(t, tx, "#4fc3f7", lw=0.8, ls="--", alpha=0.45, label="Цель X")
    for c in changes: ax_x.axvline(c, color="#555", lw=0.8, ls=":")
    ax_x.legend(fontsize=7, facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc")

    # 4. Z(t)
    ax_z.set_title("Z эффектора"); ax_z.set_xlabel("Время (с)"); ax_z.set_ylabel("Z (м)")
    ax_z.plot(t, ez, "#aed581", lw=1.2, label="EEF Z")
    ax_z.plot(t, tz, "#aed581", lw=0.8, ls="--", alpha=0.45, label="Цель Z")
    for c in changes: ax_z.axvline(c, color="#555", lw=0.8, ls=":")
    ax_z.legend(fontsize=7, facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc")

    # 5. q1
    ax_q1.set_title("Угол q₁ (joint_1)"); ax_q1.set_xlabel("Время (с)"); ax_q1.set_ylabel("рад")
    ax_q1.plot(t, q1,  "#ff9800", lw=1.2, label="q1 факт")
    ax_q1.plot(t, q1t, "#ff9800", lw=0.8, ls="--", alpha=0.45, label="q1 IK цель")
    for c in changes: ax_q1.axvline(c, color="#555", lw=0.8, ls=":")
    ax_q1.legend(fontsize=7, facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc")

    # 6. q2
    ax_q2.set_title("Угол q₂ (joint_2)"); ax_q2.set_xlabel("Время (с)"); ax_q2.set_ylabel("рад")
    ax_q2.plot(t, q2,  "#ce93d8", lw=1.2, label="q2 факт")
    ax_q2.plot(t, q2t, "#ce93d8", lw=0.8, ls="--", alpha=0.45, label="q2 IK цель")
    for c in changes: ax_q2.axvline(c, color="#555", lw=0.8, ls=":")
    ax_q2.legend(fontsize=7, facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results.png")
    plt.savefig(out, dpi=150, facecolor="#0d0d0d")
    print(f"График сохранён: {out}")
    plt.show()

if __name__ == "__main__":
    main()
