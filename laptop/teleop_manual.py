#!/usr/bin/env python3
"""Kontrol manual tank: keyboard (WASD / panah) lewat jendela pygame dan stik lewat /joy.

Pencampuran gaya tank per track: stik atau tombol diagonal menghasilkan
gerak melengkung (maju atau mundur). Hasil per track dikonversi balik ke
v dan w memakai rumus yang sama dengan nano_bridge.py, sehingga PWM
kiri dan kanan di Pi persis sesuai rasio yang dihitung di sini.

Menerbitkan /cmd_vel_manual hanya saat ada input. Saat input berhenti,
satu perintah nol dikirim lalu berhenti menerbitkan, sehingga twist_mux
menyerahkan kendali ke Nav2.
"""
import math
import threading
import time

import pygame
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

# ===== HARUS SAMA DENGAN nano_bridge.py =====
VMAX = 0.30                # max_speed
SEP = 0.17                 # wheel_separation

# ===== KECEPATAN DAN BELOKAN =====
SPEED_LIMIT = 0.8          # batas rasio track (0..1), turunkan bila terlalu cepat
SLOW_SCALE = 0.5           # pengali saat mode pelan
TURN_GAIN = 0.89            # kekuatan belok saat bergerak (1.0 = tajam, 0.3 = landai)
TRACK_DEADBAND = 0.08      # rasio track di bawah ini dianggap berhenti
MIN_TRACK_RATIO = 0.30     # rasio minimal track yang bergerak (sekitar PWM 172)
REVERSE_STEER_FLIP = True  # cerminkan arah belok saat mundur (seperti mobil)
RATE_HZ = 20

# ===== PEMETAAN STIK (cek dengan jendela diagnostik) =====
AXIS_LINEAR = 1           # stik kiri vertikal (atas = positif di paket joy)
AXIS_ANGULAR = 0          # stik kiri horizontal (kiri = positif di paket joy)
SCALE_LINEAR = 1.0        # ubah ke -1.0 bila arah maju terbalik
SCALE_ANGULAR = 1.0       # ubah ke -1.0 bila arah belok terbalik
DEADZONE = 0.20           # radius deadzone (0 sampai 1) dari titik tengah
EXPO = 0.30               # 0 = linear, makin besar makin halus di kecepatan rendah
REQUIRE_DEADMAN = False
DEADMAN_BUTTON = 4        # dipakai bila REQUIRE_DEADMAN = True
SLOW_BUTTON = 5           # tahan untuk mode pelan (misalnya R1)
JOY_TIMEOUT = 1.0         # detik, data /joy lebih tua dari ini diabaikan

# ===== D-PAD (opsional, None = mati) =====
DPAD_AXIS_X = None        # contoh: 4 atau 6 (kiri = positif)
DPAD_AXIS_Y = None        # contoh: 5 atau 7 (atas = positif)
DPAD_SCALE = 0.6


def track(t):
    """Rasio satu track: di bawah deadband berhenti, selain itu minimal MIN_TRACK_RATIO."""
    if abs(t) < TRACK_DEADBAND:
        return 0.0
    mag = max(abs(t), MIN_TRACK_RATIO)
    return mag if t > 0 else -mag


def drive(x, y, slow):
    """x: belok (kiri positif), y: maju (atas positif), kira-kira -1..1.

    Mengembalikan (v, w, kiri, kanan), kiri dan kanan berupa rasio track -1..1.
    """
    if x == 0.0 and y == 0.0:
        return 0.0, 0.0, 0.0, 0.0

    # belok lebih landai saat ada gerak maju atau mundur, penuh saat putar di tempat
    g = 1.0 - (1.0 - TURN_GAIN) * min(1.0, abs(y) / 0.4)
    steer = x * g

    # saat mundur, cerminkan arah belok agar sesuai arah gerak stik
    if REVERSE_STEER_FLIP and y < 0:
        w_y = min(1.0, max(0.0, abs(y) - 0.15) / 0.35)
        steer *= 1.0 - 2.0 * w_y

    left = y - steer
    right = y + steer
    m = max(1.0, abs(left), abs(right))
    scale = slow * SPEED_LIMIT
    left = track(left / m * scale)
    right = track(right / m * scale)

    v = (left + right) / 2.0 * VMAX
    w = (right - left) * VMAX / SEP
    return v, w, left, right


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_manual')
        self.pub = self.create_publisher(Twist, 'cmd_vel_manual', 10)
        self.create_subscription(Joy, 'joy', self.cb_joy, 10)
        self.joy = None
        self.joy_t = 0.0
        self.active = False
        self.disp = (0.0, 0.0)   # posisi stik mentah (x kiri positif, y atas positif)

    def cb_joy(self, msg):
        self.joy = msg
        self.joy_t = time.monotonic()

    @staticmethod
    def shape(x, y):
        """Deadzone radial lalu kurva halus."""
        mag = math.hypot(x, y)
        if mag < DEADZONE:
            return 0.0, 0.0
        scaled = (min(mag, 1.0) - DEADZONE) / (1.0 - DEADZONE)
        scaled = (1.0 - EXPO) * scaled + EXPO * scaled ** 3
        return x / mag * scaled, y / mag * scaled

    def stick_cmd(self):
        if self.joy is None or time.monotonic() - self.joy_t > JOY_TIMEOUT:
            self.disp = (0.0, 0.0)
            return 0.0, 0.0, 0.0, 0.0
        axes, btn = self.joy.axes, self.joy.buttons

        def axis(i):
            return axes[i] if (i is not None and 0 <= i < len(axes)) else 0.0

        rx = axis(AXIS_ANGULAR) * SCALE_ANGULAR
        ry = axis(AXIS_LINEAR) * SCALE_LINEAR
        self.disp = (rx, ry)

        if REQUIRE_DEADMAN and not (DEADMAN_BUTTON < len(btn) and btn[DEADMAN_BUTTON]):
            return 0.0, 0.0, 0.0, 0.0

        x, y = self.shape(rx, ry)

        if x == 0.0 and y == 0.0 and DPAD_AXIS_X is not None and DPAD_AXIS_Y is not None:
            dx = axis(DPAD_AXIS_X)
            dy = axis(DPAD_AXIS_Y)
            dmag = math.hypot(dx, dy)
            if dmag > 0.5:
                x = dx / dmag * DPAD_SCALE
                y = dy / dmag * DPAD_SCALE

        slow = SLOW_SCALE if (SLOW_BUTTON < len(btn) and btn[SLOW_BUTTON]) else 1.0
        return drive(x, y, slow)

    @staticmethod
    def key_cmd(keys):
        fwd = keys[pygame.K_w] or keys[pygame.K_UP]
        back = keys[pygame.K_s] or keys[pygame.K_DOWN]
        left = keys[pygame.K_a] or keys[pygame.K_LEFT]
        right = keys[pygame.K_d] or keys[pygame.K_RIGHT]
        slow = SLOW_SCALE if (keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]) else 1.0
        dy = float(int(bool(fwd)) - int(bool(back)))
        dx = float(int(bool(left)) - int(bool(right)))
        return drive(dx, dy, slow)

    def publish(self, v, w):
        if v != 0.0 or w != 0.0:
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            self.pub.publish(msg)
            self.active = True
        elif self.active:
            self.pub.publish(Twist())
            self.active = False


def main():
    rclpy.init()
    node = Teleop()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    pygame.display.init()
    pygame.font.init()
    screen = pygame.display.set_mode((500, 360))
    pygame.display.set_caption('Kontrol manual tank')
    font = pygame.font.SysFont(None, 24)
    clock = pygame.time.Clock()
    cx, cy, rad = 400, 110, 70

    running = True
    try:
        while running and rclpy.ok():
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE:
                    running = False

            v, w, kiri, kanan = node.stick_cmd()
            src = 'stik'
            if v == 0.0 and w == 0.0:
                v, w, kiri, kanan = node.key_cmd(pygame.key.get_pressed())
                src = 'keyboard'
            node.publish(v, w)

            screen.fill((25, 28, 34))

            # lingkaran posisi stik (kiri positif di data, jadi layar dibalik)
            pygame.draw.circle(screen, (70, 75, 85), (cx, cy), rad, 2)
            pygame.draw.circle(screen, (50, 54, 62), (cx, cy), int(rad * DEADZONE), 1)
            pygame.draw.line(screen, (50, 54, 62), (cx - rad, cy), (cx + rad, cy), 1)
            pygame.draw.line(screen, (50, 54, 62), (cx, cy - rad), (cx, cy + rad), 1)
            sx, sy = node.disp
            sx = max(-1.0, min(1.0, sx))
            sy = max(-1.0, min(1.0, sy))
            pygame.draw.circle(screen, (120, 220, 140),
                               (int(cx - sx * rad), int(cy - sy * rad)), 7)

            lines = [
                'W/S atau panah: maju/mundur',
                'A/D atau panah: putar di tempat',
                'W+A / W+D / S+A / S+D: melengkung',
                'Shift: pelan   Esc: keluar',
                'Stik kiri: analog ke segala arah',
                'Fokuskan jendela untuk keyboard',
            ]
            for i, t in enumerate(lines):
                screen.blit(font.render(t, True, (220, 220, 225)), (14, 14 + i * 26))

            joy = node.joy
            if joy is not None and time.monotonic() - node.joy_t <= JOY_TIMEOUT:
                axes_txt = 'axes: ' + ' '.join('%d:%+.2f' % (i, a) for i, a in enumerate(joy.axes[:8]))
                pressed = [str(i) for i, b in enumerate(joy.buttons) if b]
                btn_txt = 'tombol: ' + (' '.join(pressed) if pressed else '-')
            else:
                axes_txt = 'axes: (tidak ada data /joy)'
                btn_txt = 'tombol: -'
            screen.blit(font.render(axes_txt, True, (150, 170, 210)), (14, 214))
            screen.blit(font.render(btn_txt, True, (150, 170, 210)), (14, 240))

            active = (v != 0.0 or w != 0.0)
            color = (120, 220, 140) if active else (220, 220, 225)
            status = 'Sumber: %s   v=%.2f   w=%.2f' % (src if active else '-', v, w)
            screen.blit(font.render(status, True, color), (14, 285))
            track_txt = 'track kiri=%+.2f  kanan=%+.2f' % (kiri, kanan)
            screen.blit(font.render(track_txt, True, color), (14, 312))

            pygame.display.flip()
            clock.tick(RATE_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        pygame.quit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
