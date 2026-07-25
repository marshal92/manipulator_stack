#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
import csv

class WorkspaceMapper(Node):
    def __init__(self):
        super().__init__('workspace_mapper')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Ждем сервис /compute_ik (симуляция должна быть запущена)...')

    def check_point(self, x, y, z):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.pose_stamped.header.frame_id = "arm_base_link"
        
        req.ik_request.pose_stamped.pose.position.x = float(x)
        req.ik_request.pose_stamped.pose.position.y = float(y)
        req.ik_request.pose_stamped.pose.position.z = float(z)
        req.ik_request.pose_stamped.pose.orientation.w = 1.0 
        
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 50000000 # 50 мс на попытку

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result().error_code.val == 1

def main(args=None):
    rclpy.init(args=args)
    mapper = WorkspaceMapper()
    
    mapper.get_logger().info('Начинаем сканирование пространства...')
    
    reachable_points = []

    # Диапазон сканирования в метрах. Шаг 5 см.
    # Диапазон [-0.6, 0.6] превращаем в целые шаги, чтобы избежать кривых float
    x_steps = range(-60, 61, 5)
    y_steps = range(-60, 61, 5)
    z_steps = range(0, 81, 5)

    total_points = len(x_steps) * len(y_steps) * len(z_steps)
    checked = 0

    for z_cm in z_steps:
        for x_cm in x_steps:
            for y_cm in y_steps:
                checked += 1
                if checked % 200 == 0:
                    print(f"Проверено: {checked}/{total_points}")
                
                # Переводим сантиметры обратно в метры
                x, y, z = x_cm / 100.0, y_cm / 100.0, z_cm / 100.0
                
                if mapper.check_point(x, y, z):
                    reachable_points.append([x, y, z])

    mapper.get_logger().info(f'Сканирование завершено. Найдено точек: {len(reachable_points)}')

    # Сохраняем в CSV
    filename = 'reachable_workspace.csv'
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['X', 'Y', 'Z'])
        writer.writerows(reachable_points)
        
    mapper.get_logger().info(f'Координаты успешно сохранены в файл: {filename}')

    mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()