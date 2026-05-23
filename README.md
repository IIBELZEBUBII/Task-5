# Тема 5 — Двухзвенный маятник: переход в декартово положение (IK + POSITION_CONTROL)

## Файлы

| Файл | Описание |
|---|---|
| `two-link.urdf.xml` | URDF маятника |
| `simulation.py` | Основная симуляция |
| `plot_results.py` | Построение графиков по логу |
| `sim_log.csv` | Лог |

## Установка зависимостей

```bash
pip install pybullet numpy matplotlib
```

## Запуск

```bash
# 1. Симуляция (открывает окно PyBullet)
python simulation.py

# 2. После выхода (Ctrl+C) — графики
python plot_results.py
```

## Как работает

1. **URDF** — `two-link.urdf.xml` маятник. Маятник подвешен в точке (0,0,2), звенья длиной 0.8 м.

2. **IK** — на каждом шаге вызывается `p.calculateInverseKinematics(robot, EEF_LINK, target_pos)`.  
   PyBullet решает задачу и возвращает целевые углы `q1_target, q2_target`.

3. **POSITION_CONTROL** — встроенный ПД-регулятор PyBullet отрабатывает эти углы:
   ```python
   p.setJointMotorControl2(robot, J1, p.POSITION_CONTROL,
       targetPosition=q1_target, positionGain=0.3, velocityGain=1.0, force=50)
   ```

4. **Переключение целей** — кнопка "Следующая цель" в GUI PyBullet (или Ctrl+C → перезапуск).

## Индексы джоинтов (important!)

```
index 0 — world_to_base  (fixed)   ← пропускаем
index 1 — joint_1        (revolute) ← J1
index 2 — joint_j1_j2   (fixed)   ← пропускаем
index 3 — joint_2        (revolute) ← J2
index 4 — joint_eef      (fixed)   ← link_eef (EEF)
```

## Целевые точки

Заданы в мировых координатах (x, y, z):
```python
TARGETS = [
    ( 0.5,  0.0, 1.0),
    (-0.5,  0.0, 1.0),
    ( 0.0,  0.0, 0.6),
    ( 0.8,  0.0, 1.5),
    (-0.8,  0.0, 1.5),
]
```
Можно добавить свои или изменить через переменную `TARGETS` в `simulation.py`.
