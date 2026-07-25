from moveit_configs_utils import MoveItConfigsBuilder
import pprint

print("--- НАЧИНАЕМ ДЕБАГ MOVEIT CONFIGS ---")
try:
    # Имитируем то, что делает наш лаунч-файл
    config = MoveItConfigsBuilder("manipulator").planning_pipelines("ompl").to_dict()
    
    print("\n1. Ключи, которые удалось загрузить:")
    print(list(config.keys()))
    
    print("\n2. Содержимое planning_pipelines:")
    print(config.get('planning_pipelines', 'КЛЮЧ ОТСУТСТВУЕТ!'))
    
    print("\n3. Содержимое OMPL:")
    if 'ompl' in config:
        print("OMPL конфиг НАЙДЕН (первые 100 символов):")
        print(str(config['ompl'])[:100] + "...")
    else:
        print("OMPL конфиг ОТСУТСТВУЕТ В СЛОВАРЕ!")

except Exception as e:
    print(f"\nКРИТИЧЕСКАЯ ОШИБКА БИЛДЕРА: {e}")