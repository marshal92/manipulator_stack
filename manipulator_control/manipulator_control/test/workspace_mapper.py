import numpy as np
import matplotlib.pyplot as plt
import csv

class ExactRobotWorkspace:
    def __init__(self):
        self.L1 = 0.274  # Длина плеча
        self.L2 = 0.256  # Длина от локтя до кончика
        
        # Лимиты базы: ±90 градусов (π/2 радиан)
        self.q0_min = -np.pi / 2
        self.q0_max = np.pi / 2

    def is_reachable(self, x, y, z):
        """Проверяет точку (x, y, z) по точным геометрическим условиям"""
        
        # 1. Проверка вращения базы (угол в плоскости XY)
        q0 = np.arctan2(y, x)
        if not (self.q0_min <= q0 <= self.q0_max):
            return False
            
        r_xy = np.hypot(x, y)
        dist_origin = np.hypot(r_xy, z)
        
        # 2. Внешний радиус (максимальный вылет)
        if dist_origin > (self.L1 + self.L2):
            return False
            
        # 3. Внутренняя мертвая зона у основания
        if dist_origin < abs(self.L1 - self.L2):
            return False
            
        # 4. Нижняя зона (Z < 0): ограничена радиусом локтя из точки (L1, 0, 0)
        if z < 0:
            dist_elbow = np.hypot(r_xy - self.L1, z)
            if dist_elbow > self.L2:
                return False
                
        # 5. Левая слепая зона (вырезаем шар радиусом 0.25 вокруг точки X = -0.274, Y = 0, Z = 0)
        dist_blind = np.hypot(x - (-self.L1), np.hypot(y, z))
        if dist_blind < 0.25:
            return False

        return True

def generate_and_save_workspace(filename='exact_workspace_3d.csv', step=0.02):
    ws = ExactRobotWorkspace()
    print("Генерация 3D воркспейса по точной геометрии...")
    
    valid_points = []
    
    # Сетка пространства
    x_vals = np.arange(0.0, 0.55, step)
    y_vals = np.arange(-0.55, 0.55, step)
    z_vals = np.arange(-0.25, 0.55, step)

    for x in x_vals:
        for y in y_vals:
            for z in z_vals:
                if ws.is_reachable(x, y, z):
                    valid_points.append([x, y, z])

    # Сохраняем в CSV для использования вашими шаблонами и ИИ
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['X', 'Y', 'Z'])
        writer.writerows(valid_points)
        
    print(f"Готово! Сохранено точек: {len(valid_points)} в файл {filename}")
    return valid_points

if __name__ == '__main__':
    points = generate_and_save_workspace()
    
    # Быстрая визуализация результатов
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    fig = plt.figure(figsize=(14, 5))

    # Вид сверху
    ax1 = fig.add_subplot(121)
    ax1.scatter(xs, ys, c=zs, cmap='viridis', s=2, alpha=0.6)
    ax1.set_title('Вид сверху (X-Y): Сектор ±90° со слепой зоной')
    ax1.set_xlabel('X (м)')
    ax1.set_ylabel('Y (м)')
    ax1.plot(0, 0, 'ro', label='База')
    ax1.grid(True)
    ax1.axis('equal')
    ax1.legend()

    # Вид сбоку
    ax2 = fig.add_subplot(122)
    ax2.scatter(xs, zs, c=np.abs(ys), cmap='viridis', s=2, alpha=0.6)
    ax2.set_title('Вид сбоку (X-Z): Профиль с учетом работы внизу')
    ax2.set_xlabel('X (м)')
    ax2.set_ylabel('Z (м)')
    ax2.plot(0, 0, 'ro', label='База')
    ax2.grid(True)

    plt.tight_layout()
    plt.show()