# 🤖 Robot Mapper — Автономный робот-картограф на ROS 2

Мобильный робот с лидаром YDLIDAR X4, камерой Raspberry Pi Camera V2 и автономной навигацией на базе Raspberry Pi 4B.

## 📋 Характеристики

### Железо
- **Контроллер**: Raspberry Pi 4B 4GB (Ubuntu 22.04 + ROS 2 Humble)
- **Лидар**: YDLIDAR X4 (360°, 10 Гц, до 10 м, подключение через UART/GPIO)
- **Камера**: Raspberry Pi Camera NoIR V2 (IMX219, 8 МП, CSI)
- **Привод**: 4× TT-мотора с редукторами, 2× L298N (GPIO управление)
- **Питание**: 18650 (моторы) + Power Bank PD 22.5W (Raspberry Pi)

### Возможности
- ✅ Построение 2D-карты в реальном времени (SLAM Toolbox без одометрии)
- ✅ Ручное управление через веб-интерфейс (стрелки/кнопки)
- ✅ Автономное исследование помещения (auto_explorer)
- ✅ Видеотрансляция в Foxglove Studio
- ✅ Бесшовное переключение ручной/авто режимов (cmd_vel_mux)

## 🚀 Быстрый старт

### Установка зависимостей
```bash
sudo apt update
sudo apt install -y python3-pip v4l-utils fswebcam libcap-dev git
pip3 install --user flask flask-cors opencv-python
