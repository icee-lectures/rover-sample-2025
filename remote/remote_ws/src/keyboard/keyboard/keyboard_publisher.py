#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import pygame
import sys
import os

# pygame キーコードから Windows 仮想キーコードへのマッピング
PYGAME_TO_VK = {
    pygame.K_ESCAPE: 0x1B,  # VK_ESCAPE
    pygame.K_0: 0x30, pygame.K_1: 0x31, pygame.K_2: 0x32, pygame.K_3: 0x33, pygame.K_4: 0x34,
    pygame.K_5: 0x35, pygame.K_6: 0x36, pygame.K_7: 0x37, pygame.K_8: 0x38, pygame.K_9: 0x39,
    pygame.K_a: 0x41, pygame.K_b: 0x42, pygame.K_c: 0x43, pygame.K_d: 0x44, pygame.K_e: 0x45,
    pygame.K_f: 0x46, pygame.K_g: 0x47, pygame.K_h: 0x48, pygame.K_i: 0x49, pygame.K_j: 0x4A,
    pygame.K_k: 0x4B, pygame.K_l: 0x4C, pygame.K_m: 0x4D, pygame.K_n: 0x4E, pygame.K_o: 0x4F,
    pygame.K_p: 0x50, pygame.K_q: 0x51, pygame.K_r: 0x52, pygame.K_s: 0x53, pygame.K_t: 0x54,
    pygame.K_u: 0x55, pygame.K_v: 0x56, pygame.K_w: 0x57, pygame.K_x: 0x58, pygame.K_y: 0x59,
    pygame.K_z: 0x5A,
    pygame.K_SPACE: 0x20,  # VK_SPACE
    pygame.K_RETURN: 0x0D,  # VK_RETURN
    pygame.K_BACKSPACE: 0x08,  # VK_BACK
    pygame.K_TAB: 0x09,  # VK_TAB
    pygame.K_LSHIFT: 0xA0,  # VK_LSHIFT
    pygame.K_RSHIFT: 0xA1,  # VK_RSHIFT
    pygame.K_LCTRL: 0xA2,  # VK_LCONTROL
    pygame.K_RCTRL: 0xA3,  # VK_RCONTROL
    pygame.K_LALT: 0xA4,  # VK_LMENU
    pygame.K_RALT: 0xA5,  # VK_RMENU
    pygame.K_UP: 0x26,  # VK_UP
    pygame.K_DOWN: 0x28,  # VK_DOWN
    pygame.K_LEFT: 0x25,  # VK_LEFT
    pygame.K_RIGHT: 0x27,  # VK_RIGHT
    pygame.K_HOME: 0x24,  # VK_HOME
    pygame.K_END: 0x23,  # VK_END
    pygame.K_PAGEUP: 0x21,  # VK_PRIOR
    pygame.K_PAGEDOWN: 0x22,  # VK_NEXT
    pygame.K_DELETE: 0x2E,  # VK_DELETE
    pygame.K_INSERT: 0x2D,  # VK_INSERT
    pygame.K_F1: 0x70, pygame.K_F2: 0x71, pygame.K_F3: 0x72, pygame.K_F4: 0x73,
    pygame.K_F5: 0x74, pygame.K_F6: 0x75, pygame.K_F7: 0x76, pygame.K_F8: 0x77,
    pygame.K_F9: 0x78, pygame.K_F10: 0x79, pygame.K_F11: 0x7A, pygame.K_F12: 0x7B,
    pygame.K_COMMA: 0xBC,  # VK_OEM_COMMA
    pygame.K_PERIOD: 0xBE,  # VK_OEM_PERIOD
    pygame.K_SLASH: 0xBF,  # VK_OEM_2
    pygame.K_SEMICOLON: 0xBB,  # VK_OEM_PLUS (JIS)
    pygame.K_QUOTE: 0xBA,  # VK_OEM_1 (JIS)
    pygame.K_LEFTBRACKET: 0xDB,  # VK_OEM_4
    pygame.K_RIGHTBRACKET: 0xDD,  # VK_OEM_6
    pygame.K_BACKSLASH: 0xDC,  # VK_OEM_5
    pygame.K_EQUALS: 0xBD,  # VK_OEM_MINUS (JIS)
    pygame.K_MINUS: 0xBD,  # VK_OEM_MINUS
    pygame.K_BACKQUOTE: 0xC0,  # VK_OEM_3
    pygame.K_CAPSLOCK: 0x14,  # VK_CAPITAL
}

# キーボード レイアウト定義（行列形式）
KEYBOARD_LAYOUT = [
    [
        (pygame.K_ESCAPE, "ESC", 15),
        None,
        (pygame.K_F1, "F1", 13), (pygame.K_F2, "F2", 13), (pygame.K_F3, "F3", 13), (pygame.K_F4, "F4", 13),
        None,
        (pygame.K_F5, "F5", 13), (pygame.K_F6, "F6", 13), (pygame.K_F7, "F7", 13), (pygame.K_F8, "F8", 13),
        None,
        (pygame.K_F9, "F9", 13), (pygame.K_F10, "F10", 13), (pygame.K_F11, "F11", 13), (pygame.K_F12, "F12", 13),
    ],
    [
        (pygame.K_BACKQUOTE, "`", 15), (pygame.K_1, "1", 15), (pygame.K_2, "2", 15), (pygame.K_3, "3", 15), 
        (pygame.K_4, "4", 15), (pygame.K_5, "5", 15), (pygame.K_6, "6", 15), (pygame.K_7, "7", 15), 
        (pygame.K_8, "8", 15), (pygame.K_9, "9", 15), (pygame.K_0, "0", 15), 
        (pygame.K_MINUS, "-", 15), (pygame.K_EQUALS, "=", 15), (pygame.K_BACKSPACE, "BACK", 30),
    ],
    [
        (pygame.K_TAB, "TAB", 22), (pygame.K_q, "Q", 15), (pygame.K_w, "W", 15), (pygame.K_e, "E", 15), 
        (pygame.K_r, "R", 15), (pygame.K_t, "T", 15), (pygame.K_y, "Y", 15), (pygame.K_u, "U", 15), 
        (pygame.K_i, "I", 15), (pygame.K_o, "O", 15), (pygame.K_p, "P", 15), 
        (pygame.K_LEFTBRACKET, "[", 15), (pygame.K_RETURN, "ENTER", 32),
    ],
    [
        (pygame.K_CAPSLOCK, "CAPS", 25), (pygame.K_a, "A", 15), (pygame.K_s, "S", 15), (pygame.K_d, "D", 15), 
        (pygame.K_f, "F", 15), (pygame.K_g, "G", 15), (pygame.K_h, "H", 15), (pygame.K_j, "J", 15), 
        (pygame.K_k, "K", 15), (pygame.K_l, "L", 15), (pygame.K_SEMICOLON, ";", 15), (pygame.K_COLON, ":", 15),
        (pygame.K_RIGHTBRACKET, "]", 15), 
    ],
    [
        (pygame.K_LSHIFT, "SHIFT", 30), (pygame.K_z, "Z", 15), (pygame.K_x, "X", 15), (pygame.K_c, "C", 15), 
        (pygame.K_v, "V", 15), (pygame.K_b, "B", 15), (pygame.K_n, "N", 15), (pygame.K_m, "M", 15), 
        (pygame.K_COMMA, ",", 15), (pygame.K_PERIOD, ".", 15), (pygame.K_SLASH, "/", 15), (pygame.K_BACKSLASH, "\\", 15),
        (pygame.K_RSHIFT, "SHIFT", 30), None, None, (pygame.K_UP, "UP", 15),
    ],
    [
        (pygame.K_LCTRL, "CTRL", 25), (pygame.K_LALT, "ALT", 25), (pygame.K_SPACE, "SPACE", 142), 
        (pygame.K_RALT, "ALT", 25), (pygame.K_RCTRL, "CTRL", 25), None,
        (pygame.K_LEFT, "LEFT", 15), (pygame.K_DOWN, "DOWN", 15), (pygame.K_RIGHT, "RIGHT", 15),
    ],
]

# 色定義
COLOR_BG = (40, 40, 40)
COLOR_KEY_NORMAL = (100, 100, 100)
COLOR_KEY_PRESSED = (255, 100, 100)
COLOR_TEXT = (255, 255, 255)
COLOR_TEXT_PRESSED = (0, 0, 0)


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        
        # パブリッシャーの作成
        self.publisher_ = self.create_publisher(UInt8MultiArray, '/keyboard', 10)
        
        # 50Hz でパブリッシュするタイマー
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        
        # pygame の初期化
        pygame.init()
        
        # ディスプレイを作成
        self.screen = pygame.display.set_mode((330, 115))
        pygame.display.set_caption("ROS2 Keyboard Publisher")
        
        # クロック オブジェクトを作成
        self.clock = pygame.time.Clock()
        
        # 押下されているキーのセット
        self.pressed_keys_set = set()
        
        self.get_logger().info('Keyboard Publisher ノードが起動しました。')
        self.get_logger().info('トピック: /keyboard')
        
    def process_pygame_events(self):
        """pygame イベントを処理"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info('ウィンドウが閉じられました。')
                rclpy.shutdown()
                return False
            elif event.type == pygame.KEYDOWN:
                self.pressed_keys_set.add(event.key)
            elif event.type == pygame.KEYUP:
                self.pressed_keys_set.discard(event.key)
        return True
        
    def draw_keyboard(self):
        """キーボード レイアウトを描画"""
        self.screen.fill(COLOR_BG)
        
        x, y = 5, 5
        row_height = 18
        key_spacing = 2
        
        for row_idx, row in enumerate(KEYBOARD_LAYOUT):
            x = 5
            for item in row:
                if item is None:
                    # スペース（ギャップ）
                    x += 18
                    continue
                
                key_code, label, width = item
                is_pressed = key_code in self.pressed_keys_set
                
                # キーの背景色
                bg_color = COLOR_KEY_PRESSED if is_pressed else COLOR_KEY_NORMAL
                
                # キーを描画
                pygame.draw.rect(self.screen, bg_color, (x, y, width, 15), border_radius=2)
                pygame.draw.rect(self.screen, (200, 200, 200), (x, y, width, 15), 1, border_radius=2)
                
                x += width + key_spacing
            
            y += row_height
        
        pygame.display.flip()
        
    def timer_callback(self):
        """タイマー コールバック: キー状態をパブリッシュ"""
        # pygame イベントを処理
        if not self.process_pygame_events():
            return
        
        # キーボードを描画
        self.draw_keyboard()
        
        # 押下キーを Windows 仮想キーコードに変換
        vk_codes = []
        for key in sorted(self.pressed_keys_set):
            vk = PYGAME_TO_VK.get(key)
            if vk:
                vk_codes.append(vk)
        
        # メッセージを作成してパブリッシュ
        msg = UInt8MultiArray()
        msg.data = vk_codes
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        keyboard_publisher = KeyboardPublisher()
        rclpy.spin(keyboard_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        os._exit(0)


if __name__ == '__main__':
    main()
