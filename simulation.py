"""
Тема 5: переход двухзвенного маятника в заданное декартово положение
через p.calculateInverseKinematics в режиме POSITION_CONTROL.

Основан на коде с занятий (mip2025-main/two-link/pendulum.py).

Структура маятника (из two-link.urdf.xml):
  joint index 0 — world_to_base (fixed)  → пропускаем
  joint index 1 — joint_1  (revolute)    ← управляемый
  joint index 2 — joint_j1_j2 (fixed)   → пропускаем
  joint index 3 — joint_2  (revolute)    ← управляемый
  joint index 4 — joint_eef (fixed)      → link_eef (link index 4)

Управление:
  Кнопки в GUI PyBullet — переключение целей и сброс
  Ctrl+C — выход
"""

import pybullet as p
import pybullet_data
import time
import numpy as np
import csv
import os

# ── Параметры ─────────────────────────────────────────────────────────
URDF_PATH  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "two-link.urdf.xml")
DT         = 1 / 240          # шаг физики
G          = 10.0
L          = 0.8
H          = 2.0              # высота крепления (из URDF: world_to_base z=2)

# Индексы джоинтов (из URDF)
J1         = 1                # joint_1 (revolute)
J2         = 3                # joint_2 (revolute)
EEF_LINK   = 4                # link_eef

# Параметры POSITION_CONTROL
KP         = 0.3
KD         = 1.0
F_MAX      = 50.0

# Целевые декартовы точки (x, y, z) в мировых координатах
TARGETS = [
    ( 0.5,  0.0, 1.0),   # цель 1
    (-0.5,  0.0, 1.0),   # цель 2
    ( 0.0,  0.0, 0.6),   # цель 3
    ( 0.8,  0.0, 1.5),   # цель 4
    (-0.8,  0.0, 1.5),   # цель 5
]

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_log.csv")

# ─────────────────────────────────────────────────────────────────────

def main():
    # ── Инициализация ─────────────────────────────────────────────────
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -G)
    p.setTimeStep(DT)
    p.resetDebugVisualizerCamera(
        cameraDistance=3.5, cameraYaw=0, cameraPitch=-20,
        cameraTargetPosition=[0, 0, 1.0]
    )

    p.loadURDF("plane.urdf")
    robot = p.loadURDF(URDF_PATH, useFixedBase=True)

    # Убираем демпфирование (как в коде с занятий)
    p.changeDynamics(robot, J1, linearDamping=0, angularDamping=0)
    p.changeDynamics(robot, J2, linearDamping=0, angularDamping=0)

    # Начальное положение — небольшое смещение, чтобы не была вырожденная конфигурация
    th0 = 0.3
    p.setJointMotorControl2(robot, J1, p.POSITION_CONTROL, targetPosition=th0)
    p.setJointMotorControl2(robot, J2, p.POSITION_CONTROL, targetPosition=0.0)
    for _ in range(500):
        p.stepSimulation()

    # ── GUI кнопки ────────────────────────────────────────────────────
    btn_next  = p.addUserDebugParameter(">> Следующая цель", 1, 0, 1)
    btn_reset = p.addUserDebugParameter(">> Сброс",          1, 0, 1)
    prev_next  = p.readUserDebugParameter(btn_next)
    prev_reset = p.readUserDebugParameter(btn_reset)

    target_idx = 0
    target_pos = TARGETS[target_idx]
    text_id    = None

    print("=" * 55)
    print("  Двухзвенный маятник — POSITION_CONTROL + IK")
    print(f"  Начальная цель: {target_pos}")
    print("  Ctrl+C — выход")
    print("=" * 55)

    # ── Лог ──────────────────────────────────────────────────────────
    log_rows = []
    t_sim    = 0.0

    def update_target_label():
        nonlocal text_id
        if text_id is not None:
            p.removeUserDebugItem(text_id)
        text_id = p.addUserDebugText(
            f"Цель {target_idx+1}: ({target_pos[0]:.2f}, {target_pos[2]:.2f})",
            [target_pos[0], target_pos[1], target_pos[2] + 0.15],
            textColorRGB=[1, 0.3, 0.3], textSize=1.5
        )

    update_target_label()

    try:
        while True:
            # ── Кнопки ───────────────────────────────────────────────
            cur_next  = p.readUserDebugParameter(btn_next)
            cur_reset = p.readUserDebugParameter(btn_reset)

            if cur_next != prev_next:
                prev_next  = cur_next
                target_idx = (target_idx + 1) % len(TARGETS)
                target_pos = TARGETS[target_idx]
                update_target_label()
                print(f"  Цель #{target_idx+1}: {target_pos}")

            if cur_reset != prev_reset:
                prev_reset = cur_reset
                p.resetJointState(robot, J1, th0, 0)
                p.resetJointState(robot, J2, 0.0, 0)
                print("  Сброс.")

            # ── IK ───────────────────────────────────────────────────
            ik = p.calculateInverseKinematics(
                robot, EEF_LINK, target_pos,
                maxNumIterations=200,
                residualThreshold=1e-5
            )
            # ik возвращает решение для всех нефиксированных DoF
            # нефиксированных джоинтов ровно 2: J1 и J2
            q1_target = ik[0]
            q2_target = ik[1]

            # ── POSITION_CONTROL ─────────────────────────────────────
            p.setJointMotorControl2(
                robot, J1, p.POSITION_CONTROL,
                targetPosition=q1_target,
                positionGain=KP, velocityGain=KD, force=F_MAX
            )
            p.setJointMotorControl2(
                robot, J2, p.POSITION_CONTROL,
                targetPosition=q2_target,
                positionGain=KP, velocityGain=KD, force=F_MAX
            )

            # ── Шаг физики ───────────────────────────────────────────
            p.stepSimulation()

            # ── Лог ──────────────────────────────────────────────────
            q1  = p.getJointState(robot, J1)[0]
            q2  = p.getJointState(robot, J2)[0]
            eef = p.getLinkState(robot, EEF_LINK, computeForwardKinematics=True)[4]
            err = float(np.linalg.norm(np.array(eef) - np.array(target_pos)))

            log_rows.append({
                "t":      round(t_sim, 4),
                "tidx":   target_idx,
                "tx": target_pos[0], "ty": target_pos[1], "tz": target_pos[2],
                "ex": round(eef[0], 5), "ey": round(eef[1], 5), "ez": round(eef[2], 5),
                "err":    round(err, 5),
                "q1":     round(q1, 5),
                "q2":     round(q2, 5),
                "q1t":    round(q1_target, 5),
                "q2t":    round(q2_target, 5),
            })

            t_sim += DT
            time.sleep(DT)

    except KeyboardInterrupt:
        print("\n  Симуляция завершена.")
    finally:
        if log_rows:
            with open(LOG_FILE, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=log_rows[0].keys())
                w.writeheader()
                w.writerows(log_rows)
            print(f"  Лог: {LOG_FILE}")
        p.disconnect()


if __name__ == "__main__":
    main()
