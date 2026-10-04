import csv
import math
import numpy as np

def generate_square(filename, center_x, center_y, center_z, size_m, plane='XY', step=0.02):
    """
    Генерирует CSV для квадрата.
    ПОМНИМ: Вперед = -Y. Вправо = +X. Влево = -X.
    """
    waypoints = []
    half = size_m / 2.0

    print(f"Генерация {filename} (Центр: X={center_x}, Y={center_y}, Z={center_z})")

    if plane == 'XY':
        # Горизонтальный квадрат (на плоскости стола)
        # Идем по часовой стрелке
        corners = [
            (center_x - half, center_y + half, center_z), # Левый ближний (к роботу)
            (center_x + half, center_y + half, center_z), # Правый ближний
            (center_x + half, center_y - half, center_z), # Правый дальний (сильнее в минус по Y)
            (center_x - half, center_y - half, center_z), # Левый дальний
            (center_x - half, center_y + half, center_z)  # Возврат
        ]
    elif plane == 'XZ':
        # Вертикальный квадрат (стена перед роботом)
        # Y не меняется
        corners = [
            (center_x - half, center_y, center_z + half), # Левый верхний
            (center_x + half, center_y, center_z + half), # Правый верхний
            (center_x + half, center_y, center_z - half), # Правый нижний
            (center_x - half, center_y, center_z - half), # Левый нижний
            (center_x - half, center_y, center_z + half)  # Возврат
        ]
    else:
        return

    # Разбиваем отрезки между углами на мелкие шаги (waypoints)
    for i in range(len(corners) - 1):
        start = corners[i]
        end = corners[i+1]
        
        dist = math.hypot(end[0] - start[0], math.hypot(end[1] - start[1], end[2] - start[2]))
        num_steps = max(int(dist / step), 1)
        
        for s in range(num_steps):
            t = s / num_steps
            px = start[0] + (end[0] - start[0]) * t
            py = start[1] + (end[1] - start[1]) * t
            pz = start[2] + (end[2] - start[2]) * t
            
            # Добавляем точку. Ориентацию (Pitch, Yaw, Roll) пока ставим нулями.
            # Если нужно будет, ИИ или ты сможешь задать здесь конкретный наклон кисти.
            waypoints.append([px, py, pz, 0.0, 0.0, 0.0])

    # Добавляем последнюю точку (замыкание)
    waypoints.append([corners[-1][0], corners[-1][1], corners[-1][2], 0.0, 0.0, 0.0])

    # Сохраняем в файл
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])
        writer.writerows(waypoints)
        
    print(f"✅ Сохранено точек: {len(waypoints)} в {filename}\n")

if __name__ == '__main__':
    # ТЕСТ 1: Горизонтальный квадрат на высоте 10см (Z=0.1)
    # Центр находится в 30см прямо перед роботом (Y = -0.30)
    generate_square('square_horizontal.csv', center_x=0.0, center_y=-0.30, center_z=0.1, size_m=0.15, plane='XY')

    # ТЕСТ 2: Вертикальный квадрат на высоте 20см (Z=0.2)
    # Центр находится в 35см прямо перед роботом (Y = -0.35)
    generate_square('square_vertical.csv', center_x=0.0, center_y=-0.35, center_z=0.2, size_m=0.15, plane='XZ')