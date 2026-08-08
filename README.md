# 1. Переход в рабочую папку
cd ~/robot_mapper_ws

# 2. Полная пересборка (обязательно, так как добавили новые ноды)
colcon build --symlink-install

# 3. Обновление переменных среды
source install/setup.bash

# 4. Запуск всей системы (один файл поднимает всё)
ros2 launch robot_mapper robot_autonomous.launch.py



# 📱 Переключиться на точку доступа телефона:
sudo ~/wifi_switch.sh hotspot

# 🏠 Переключиться на домашний роутер:
sudo ~/wifi_switch.sh home

bluetoothctl

power on
agent on
default-agent
scan on
