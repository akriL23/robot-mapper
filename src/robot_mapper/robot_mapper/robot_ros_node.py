#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import struct
import threading
import traceback
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster

# --- MOCK GPIO ---
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

    class MockPWM:
        def __init__(self, pin, freq): self.pin, self.freq = pin, freq
        def start(self, duty): pass
        def stop(self): pass
        def ChangeDutyCycle(self, duty): pass
        def ChangeFrequency(self, freq): pass

    class MockGPIO:
        BCM = "BCM"; OUT = "OUT"; LOW = 0; HIGH = 1
        def setwarnings(self, f): pass
        def setmode(self, m): pass
        def setup(self, p, m): pass
        def output(self, p, v): pass
        def cleanup(self): pass
        def PWM(self, p, f): return MockPWM(p, f)
        def getmode(self): return None

    GPIO = MockGPIO()


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def safe_gpio_setmode(mode):
    if GPIO.getmode() is None:
        GPIO.setwarnings(False)
        GPIO.setmode(mode)


class YDLidarX4:
    """
    Парсер протокола YDLidar X4 (UART, CP2102).

    Подтверждённый формат пакета (из реального дампа):
    ┌───────┬───────┬──────┬──────┬─────────────┬─────────────┬─────────────┬──────────────────────────┐
    │ 0xAA  │ 0x55  │  CT  │ LSN  │  FSA 2байта │  LSA 2байта │  CS  2байта │  данные: N*2 байт        │
    └───────┴───────┴──────┴──────┴─────────────┴─────────────┴─────────────┴──────────────────────────┘

    CT  — тип пакета (bit0 = 1 → начало нового оборота)
    LSN — кодированное количество точек:
            если LSN >= 0x28 (40): реальное_кол-во = LSN // 2
            если LSN <  0x28 (40): реальное_кол-во = LSN
    FSA — начальный угол: (FSA >> 1) / 64.0 = градусы
    LSA — конечный  угол: (LSA >> 1) / 64.0 = градусы
    CS  — контрольная сумма uint16 LE
    Si  — дистанции uint16 LE: dist_mm = Si / 4.0, Si==0 → нет цели

    Примеры из дампа:
      CT=0x40 LSN=0x50(80) → 40 точек, пакет=90 байт  (90=10+80)
      CT=0x30 LSN=0x48(72) → 36 точек, пакет=82 байт  (в дампе следующий через 90 — padding?)
      CT=0xCC LSN=0x07(7)  →  7 точек, пакет=24 байт  (24=10+14) ✓
      CT=0xF3 LSN=0x01(1)  →  1 точка,  пакет=12 байт  (12=10+2)  ✓

    Безопасная стратегия: размер пакета = 10 + real_count * 2,
    верификация по следующему маркеру AA 55.
    """

    SYNC1       = 0xAA
    SYNC2       = 0x55
    HEADER_SIZE = 10

    def __init__(self, motor_pin=17, serial_port='/dev/ttyUSB0', baudrate=128000):
        self.motor_pin   = motor_pin
        self.serial_port = serial_port
        self.baudrate    = baudrate

        self.serial_conn = None
        self.pwm         = None
        self.running     = False
        self.thread      = None
        self.lock        = threading.RLock()
        self.init_lock   = threading.Lock()
        self.initialized = False

        # 360 значений в метрах, inf = нет препятствия
        self.current_scan = [float('inf')] * 360
        # Накопительный буфер одного оборота
        self._scan_buf    = {}

        # Статистика
        self.stat_ok     = 0
        self.stat_resync = 0
        self.stat_bytes  = 0

    # ------------------------------------------------------------------ #
    #  Декодирование LSN → реальное количество точек                      #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _decode_lsn(lsn_byte: int) -> int:
        """
        LSN >= 0x28 (40): реальное кол-во = lsn_byte // 2
        LSN <  0x28 (40): реальное кол-во = lsn_byte
        Граница 0x28 выбрана потому что максимум точек в пакете = 40,
        а при lsn_byte=0x50=80 → 80//2=40, при lsn_byte=7 → 7.
        """
        if lsn_byte >= 0x28:
            return lsn_byte // 2
        return lsn_byte

    # ------------------------------------------------------------------ #
    #  Запуск / остановка                                                  #
    # ------------------------------------------------------------------ #

    def start(self, motor_speed=55):
        with self.init_lock:
            if self.initialized:
                return
            self.initialized = True

        print(f"[Lidar] GPIO={GPIO_AVAILABLE}", flush=True)
        print(f"[Lidar] Motor pin={self.motor_pin} speed={motor_speed}%", flush=True)

        safe_gpio_setmode(GPIO.BCM)
        GPIO.setup(self.motor_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.motor_pin, 1000)
        self.pwm.start(motor_speed)

        print("[Lidar] Motor spin-up 2s...", flush=True)
        time.sleep(2.0)

        print(f"[Lidar] Opening {self.serial_port} @ {self.baudrate}", flush=True)
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()

            self.serial_conn = serial.Serial(
                port      = self.serial_port,
                baudrate  = self.baudrate,
                timeout   = 0.05,
                parity    = serial.PARITY_NONE,
                stopbits  = serial.STOPBITS_ONE,
                bytesize  = serial.EIGHTBITS,
                exclusive = True
            )
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            time.sleep(0.1)

            # Команда начала сканирования
            self.serial_conn.write(bytes([0xA5, 0x60]))
            self.serial_conn.flush()
            print("[Lidar] Scan command sent (0xA5 0x60)", flush=True)

            # Ответный заголовок устройства (7 байт: A5 5A 05 00 00 40 81)
            time.sleep(0.3)
            resp = self.serial_conn.read(7)
            print(f"[Lidar] Device header: {resp.hex() if resp else 'none'}", flush=True)

            if len(resp) >= 2 and resp[0] == 0xA5 and resp[1] == 0x5A:
                print("[Lidar] Header OK", flush=True)
            else:
                print("[Lidar] WARNING: unexpected header, continuing", flush=True)

            # Сбрасываем мусор после заголовка
            time.sleep(0.1)
            self.serial_conn.reset_input_buffer()

        except Exception as e:
            print(f"[Lidar] Serial error: {e}", flush=True)
            raise

        with self.lock:
            self.running = True

        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print("[Lidar] Read thread started", flush=True)

    def stop(self):
        with self.lock:
            self.running = False

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=3.0)

        if self.serial_conn:
            try:
                if self.serial_conn.is_open:
                    self.serial_conn.write(bytes([0xA5, 0x65]))
                    self.serial_conn.flush()
                    time.sleep(0.1)
                    self.serial_conn.close()
            except Exception as e:
                print(f"[Lidar] Stop error: {e}", flush=True)
            self.serial_conn = None

        if self.pwm:
            try:
                self.pwm.stop()
            except Exception:
                pass
            self.pwm = None

    # ------------------------------------------------------------------ #
    #  Поток чтения                                                        #
    # ------------------------------------------------------------------ #

    def _read_loop(self):
        buf = bytearray()

        while True:
            with self.lock:
                if not self.running:
                    break
            try:
                if not self.serial_conn or not self.serial_conn.is_open:
                    time.sleep(0.1)
                    continue

                chunk = self.serial_conn.read(512)
                if not chunk:
                    continue

                buf.extend(chunk)
                self.stat_bytes += len(chunk)

                self._process(buf)

                if len(buf) > 8192:
                    print(f"[Lidar] Buffer overflow {len(buf)}B, clearing", flush=True)
                    buf.clear()

            except serial.SerialException as e:
                print(f"[Lidar] SerialException: {e}", flush=True)
                time.sleep(0.5)
            except Exception as e:
                print(f"[Lidar] Error: {e}", flush=True)
                traceback.print_exc()
                time.sleep(0.1)

    # ------------------------------------------------------------------ #
    #  Парсинг буфера                                                      #
    # ------------------------------------------------------------------ #

    def _process(self, buf: bytearray):
        """
        Извлекает все полные пакеты из буфера.
        Буфер модифицируется на месте (обработанные байты удаляются).
        """
        while len(buf) >= self.HEADER_SIZE:

            # Шаг 1: Ищем маркер AA 55
            pos = -1
            for i in range(len(buf) - 1):
                if buf[i] == self.SYNC1 and buf[i + 1] == self.SYNC2:
                    pos = i
                    break

            if pos == -1:
                # Маркер не найден — сохраняем последний байт на случай
                # если он является первым байтом следующего маркера
                last = buf[-1]
                buf.clear()
                buf.append(last)
                return

            # Удаляем мусор до маркера
            if pos > 0:
                self.stat_resync += 1
                del buf[:pos]

            # Шаг 2: Ждём полного заголовка
            if len(buf) < self.HEADER_SIZE:
                return

            # Шаг 3: Читаем заголовок
            ct       = buf[2]
            lsn_byte = buf[3]
            fsa      = struct.unpack_from('<H', buf, 4)[0]
            lsa      = struct.unpack_from('<H', buf, 6)[0]
            cs_recv  = struct.unpack_from('<H', buf, 8)[0]

            # Декодируем реальное количество точек
            n_points = self._decode_lsn(lsn_byte)

            # Минимальная валидация
            if n_points == 0 or n_points > 80:
                self.stat_resync += 1
                del buf[:2]
                continue

            # Шаг 4: Ждём полного пакета
            pkt_size = self.HEADER_SIZE + n_points * 2
            if len(buf) < pkt_size:
                return

            # Шаг 5: Верификация — следующий пакет должен начинаться с AA 55
            # (только если у нас достаточно данных для проверки)
            if len(buf) >= pkt_size + 2:
                nb0 = buf[pkt_size]
                nb1 = buf[pkt_size + 1]
                if nb0 != self.SYNC1 or nb1 != self.SYNC2:
                    # Неверное смещение — сдвигаемся на 2 байта (пропускаем маркер)
                    self.stat_resync += 1
                    del buf[:2]
                    continue

            # Шаг 6: Извлекаем и обрабатываем пакет
            pkt = bytes(buf[:pkt_size])
            del buf[:pkt_size]

            self.stat_ok += 1
            self._parse_packet(ct, n_points, fsa, lsa, pkt[self.HEADER_SIZE:])

            # Периодическая статистика
            if self.stat_ok % 200 == 0:
                pts = sum(1 for d in self.current_scan if d != float('inf'))
                print(
                    f"[Lidar] ok={self.stat_ok} "
                    f"resync={self.stat_resync} "
                    f"points={pts}/360 "
                    f"rx={self.stat_bytes}B",
                    flush=True
                )

    # ------------------------------------------------------------------ #
    #  Парсинг одного пакета                                               #
    # ------------------------------------------------------------------ #

    def _parse_packet(self, ct: int, n_points: int,
                      fsa: int, lsa: int, data: bytes):
        """
        Декодирует углы и дистанции одного пакета.

        Углы:
            start_deg = (fsa >> 1) / 64.0
            end_deg   = (lsa >> 1) / 64.0
        Угловой шаг учитывает переход через 0°/360°.

        Дистанция:
            dist_mm = uint16_LE / 4.0
            dist_m  = dist_mm / 1000.0
            raw == 0 → нет цели → float('inf')

        CT bit0 == 1 → начало нового оборота → сохраняем накопленный скан.
        """
        start_deg = (fsa >> 1) / 64.0
        end_deg   = (lsa >> 1) / 64.0

        # Угловой шаг с учётом перехода через 360°
        if n_points > 1:
            span = end_deg - start_deg
            if span < 0:
                span += 360.0
            step = span / (n_points - 1)
        else:
            step = 0.0

        # Начало нового оборота → применяем накопленный буфер
        if ct & 0x01:
            self._flush_scan()

        for i in range(n_points):
            off = i * 2
            if off + 1 >= len(data):
                break

            raw_dist = struct.unpack_from('<H', data, off)[0]
            dist_mm  = raw_dist / 4.0
            dist_m   = dist_mm / 1000.0

            angle = (start_deg + step * i) % 360.0
            idx   = int(round(angle)) % 360

            if raw_dist == 0:
                self._scan_buf[idx] = float('inf')
            elif 0.08 <= dist_m <= 16.0:
                self._scan_buf[idx] = dist_m

    def _flush_scan(self):
        """Копирует накопленный буфер оборота в current_scan."""
        if not self._scan_buf:
            return
        with self.lock:
            for idx, val in self._scan_buf.items():
                self.current_scan[idx] = val
        self._scan_buf.clear()

    def get_ranges_ros(self):
        """Возвращает копию текущего скана (360 значений в метрах)."""
        with self.lock:
            return self.current_scan.copy()


# ====================================================================== #
#  RobotController                                                         #
# ====================================================================== #

class RobotController:
    def __init__(self):
        self.motors = {
            "BACK_LEFT":   {"ENA": 12, "IN1": 5,  "IN2": 6},
            "BACK_RIGHT":  {"ENB": 13, "IN3": 20, "IN4": 21},
            "FRONT_RIGHT": {"ENA": 18, "IN1": 23, "IN2": 24},
            "FRONT_LEFT":  {"ENB": 19, "IN3": 25, "IN4": 8},
        }
        self.lock          = threading.RLock()
        self.pwm           = {}
        self.pwm_frequency = 500
        safe_gpio_setmode(GPIO.BCM)
        self._setup_gpio()

    def _setup_gpio(self):
        for name, pins in self.motors.items():
            for pin_name, pin_num in pins.items():
                GPIO.setup(pin_num, GPIO.OUT)
                GPIO.output(pin_num, GPIO.LOW)
                if pin_name in ("ENA", "ENB"):
                    p = GPIO.PWM(pin_num, self.pwm_frequency)
                    p.start(0)
                    self.pwm[name] = p

    def _set_motor(self, name, direction, speed):
        pins  = self.motors[name]
        speed = max(0, min(100, int(abs(speed))))
        a, b  = ((pins["IN1"], pins["IN2"]) if "IN1" in pins
                  else (pins["IN3"], pins["IN4"]))
        if direction > 0:
            GPIO.output(a, GPIO.HIGH); GPIO.output(b, GPIO.LOW)
        elif direction < 0:
            GPIO.output(a, GPIO.LOW);  GPIO.output(b, GPIO.HIGH)
        else:
            GPIO.output(a, GPIO.LOW);  GPIO.output(b, GPIO.LOW)
        self.pwm[name].ChangeDutyCycle(speed)

    def drive(self, linear, angular):
        with self.lock:
            lin = max(-0.30, min(0.30, linear))
            ang = max(-1.50, min(1.50, angular))
            ln  = lin / 0.30
            an  = ang / 1.50
            l   = (ln - an) * 100.0
            r   = (ln + an) * 100.0
            ld  = 1 if l >  5 else (-1 if l < -5 else 0)
            rd  = 1 if r >  5 else (-1 if r < -5 else 0)
            for n in ["BACK_LEFT", "FRONT_LEFT"]:
                self._set_motor(n, ld, l)
            for n in ["BACK_RIGHT", "FRONT_RIGHT"]:
                self._set_motor(n, rd, r)

    def stop(self):
        with self.lock:
            for m in self.motors:
                self._set_motor(m, 0, 0)

    def cleanup(self):
        self.stop()
        for p in self.pwm.values():
            try: p.stop()
            except Exception: pass


# ====================================================================== #
#  ROS-нода                                                                #
# ====================================================================== #

class RobotRosNode(Node):
    def __init__(self):
        super().__init__('robot_mapper_node')

        self.declare_parameter('lidar_motor_pin',   17)
        self.declare_parameter('lidar_port',        '/dev/ttyUSB0')
        self.declare_parameter('lidar_baudrate',    128000)
        self.declare_parameter('lidar_motor_speed', 55)
        self.declare_parameter('base_frame',        'base_link')
        self.declare_parameter('laser_frame',       'laser')
        self.declare_parameter('odom_frame',        'odom')

        mp   = self.get_parameter('lidar_motor_pin').value
        port = self.get_parameter('lidar_port').value
        baud = self.get_parameter('lidar_baudrate').value
        self.mspd = self.get_parameter('lidar_motor_speed').value
        self.bf   = self.get_parameter('base_frame').value
        self.lf   = self.get_parameter('laser_frame').value
        self.of   = self.get_parameter('odom_frame').value

        self.robot = RobotController()
        self.lidar = YDLidarX4(mp, port, baud)

        threading.Thread(target=self._init_lidar, daemon=True).start()

        self.scan_pub = self.create_publisher(LaserScan,   '/scan',       10)
        self.odom_pub = self.create_publisher(Odometry,    '/odom',       10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.path_pub = self.create_publisher(Path,        '/robot_path', 10)
        self.cmd_sub  = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.tf_br = TransformBroadcaster(self)

        self.x = self.y = self.theta = 0.0
        self.vx = self.vth = 0.0
        self.last_cmd   = time.time()
        self.last_odom  = self.get_clock().now()
        self.path_msg   = Path()
        self.path_msg.header.frame_id = self.of
        self._no_data_t = 0.0

        self.create_timer(0.10, self.pub_scan)
        self.create_timer(0.05, self.pub_odom)
        self.create_timer(0.10, self.pub_laser_tf)
        self.create_timer(0.10, self.watchdog)
        self.get_logger().info('robot_mapper_node ready')

    def _init_lidar(self):
        try:
            self.lidar.start(self.mspd)
            self.get_logger().info('Lidar started OK')
        except Exception as e:
            self.get_logger().error(
                f'Lidar start failed: {e}\n{traceback.format_exc()}')

    def cmd_vel_cb(self, msg: Twist):
        self.vx  = msg.linear.x
        self.vth = msg.angular.z
        self.last_cmd = time.time()
        self.robot.drive(self.vx, self.vth)

    def watchdog(self):
        if time.time() - self.last_cmd > 0.7:
            self.vx = self.vth = 0.0
            self.robot.stop()

    def pub_scan(self):
        try:
            raw = self.lidar.get_ranges_ros()
            if len(raw) < 360:
                return

            pts = sum(1 for d in raw if d != float('inf'))
            if pts == 0 and time.time() - self._no_data_t > 5.0:
                self.get_logger().warn('No lidar points — check wiring/motor')
                self._no_data_t = time.time()

            # Поправка угла монтажа лидара (меняйте если скан перевёрнут)
            offset = 180
            rmax   = 12.0

            ranges = []
            for deg in range(360):
                d = raw[(deg + offset) % 360]
                if d == float('inf') or d > rmax:
                    ranges.append(float('inf'))
                elif d < 0.08:
                    ranges.append(float('nan'))
                else:
                    ranges.append(float(d))

            msg = LaserScan()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self.lf
            msg.angle_min       = -math.pi
            msg.angle_max       =  math.pi - math.radians(1.0)
            msg.angle_increment =  math.radians(1.0)
            msg.time_increment  = 0.0
            msg.scan_time       = 0.1
            msg.range_min       = 0.08
            msg.range_max       = rmax
            msg.ranges          = ranges
            msg.intensities     = []
            self.scan_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'pub_scan error: {e}')

    def pub_odom(self):
        try:
            now = self.get_clock().now()
            dt  = (now - self.last_odom).nanoseconds / 1e9
            self.last_odom = now

            self.x     += self.vx * math.cos(self.theta) * dt
            self.y     += self.vx * math.sin(self.theta) * dt
            self.theta += self.vth * dt

            q = yaw_to_quaternion(self.theta)

            odom = Odometry()
            odom.header.stamp          = now.to_msg()
            odom.header.frame_id       = self.of
            odom.child_frame_id        = self.bf
            odom.pose.pose.position.x  = self.x
            odom.pose.pose.position.y  = self.y
            odom.pose.pose.orientation = q
            odom.twist.twist.linear.x  = self.vx
            odom.twist.twist.angular.z = self.vth
            self.odom_pub.publish(odom)

            pose = PoseStamped()
            pose.header.stamp     = now.to_msg()
            pose.header.frame_id  = self.of
            pose.pose.position.x  = self.x
            pose.pose.position.y  = self.y
            pose.pose.orientation = q
            self.pose_pub.publish(pose)

            self.path_msg.header.stamp = now.to_msg()
            self.path_msg.poses.append(pose)
            if len(self.path_msg.poses) > 2000:
                self.path_msg.poses = self.path_msg.poses[-2000:]
            self.path_pub.publish(self.path_msg)

            tf = TransformStamped()
            tf.header.stamp            = now.to_msg()
            tf.header.frame_id         = self.of
            tf.child_frame_id          = self.bf
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation      = q
            self.tf_br.sendTransform(tf)

        except Exception as e:
            self.get_logger().warn(f'pub_odom error: {e}')

    def pub_laser_tf(self):
        try:
            tf = TransformStamped()
            tf.header.stamp            = self.get_clock().now().to_msg()
            tf.header.frame_id         = self.bf
            tf.child_frame_id          = self.lf
            tf.transform.translation.z = 0.12
            tf.transform.rotation.w    = 1.0
            self.tf_br.sendTransform(tf)
        except Exception:
            pass

    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        try:
            self.robot.stop()
            self.lidar.stop()
            self.robot.cleanup()
        except Exception as e:
            self.get_logger().warn(f'Cleanup: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
