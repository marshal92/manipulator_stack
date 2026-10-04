#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

filename = 'absolute_workspace_2.csv'

try:
    print(f"Чтение файла {filename}...")
    df = pd.read_csv(filename)
    
    df['Radius'] = np.sqrt(df['X']**2 + df['Y']**2)
    
    print("\n=== АНАЛИЗ ФИЗИЧЕСКИХ ЛИМИТОВ ===")
    print(f"Всего точек проанализировано: {len(df)}")
    print(f"Максимальный вылет кисти (радиус): {df['Radius'].max():.2f} м")
    print(f"Ось X (Вперед/Назад): от {df['X'].min():.2f} м до {df['X'].max():.2f} м")
    print(f"Ось Y (Влево/Вправо): от {df['Y'].min():.2f} м до {df['Y'].max():.2f} м")
    print(f"Ось Z (Высота подъема): от {df['Z'].min():.2f} м до {df['Z'].max():.2f} м")
    print("=================================\n")

    # Только 2 надежных 2D графика
    fig = plt.figure(figsize=(12, 6))

    # Вид сверху (Плоскость X-Y)
    ax1 = fig.add_subplot(121)
    # Цвет точек зависит от их высоты Z
    scatter1 = ax1.scatter(df['X'], df['Y'], c=df['Z'], cmap='viridis', s=3, alpha=0.6)
    ax1.set_title('Вид сверху (Ось X-Y)')
    ax1.set_xlabel('Вперед/Назад: X (м)')
    ax1.set_ylabel('Влево/Вправо: Y (м)')
    ax1.grid(True)
    ax1.plot(0, 0, 'ro', markersize=8, label='База робота')
    fig.colorbar(scatter1, ax=ax1, label='Высота (Z)')
    ax1.legend()

    # Вид сбоку (Плоскость X-Z)
    ax2 = fig.add_subplot(122)
    # Цвет точек зависит от отклонения по Y
    scatter2 = ax2.scatter(df['X'], df['Z'], c=np.abs(df['Y']), cmap='viridis', s=3, alpha=0.6)
    ax2.set_title('Вид сбоку (Ось X-Z)')
    ax2.set_xlabel('Вперед/Назад: X (м)')
    ax2.set_ylabel('Высота: Z (м)')
    ax2.grid(True)
    ax2.plot(0, 0, 'ro', markersize=8, label='База робота')
    fig.colorbar(scatter2, ax=ax2, label='Отклонение по Y')
    ax2.legend()

    plt.tight_layout()
    plt.show()

except Exception as e:
    print(f"Произошла ошибка: {e}")