from picamera2 import Picamera2
import cv2
import numpy as np
import threading
lock = threading.Lock()


class Camera:
    def __init__(self):
        #カメラの初期化と設定とスタート
        self.picam2 = Picamera2()
        self.height = 480
        self.width = 480
        self.config = self.picam2.create_video_configuration(
            main={"format": 'XRGB8888', "size": (self.height, self.width)},
            controls={
                "FrameDurationLimits": (8333, 8333),
                "ExposureTime": 8000
            }
        )
        self.picam2.configure(self.config)

        # 蛍光ピンクのHSV範囲を定義
        self.lower_pink = np.array([140, 150, 50])  # H: 約140度から
        self.upper_pink = np.array([180, 255, 255])  # H: 約170度まで
        #カメラをスタート
        self.picam2.start()

    def take_pic(self):
        image = self.picam2.capture_array()
        return image

    
    def show_video(self, image):
        cv2.imshow("Live", image)
        cv2.waitKey(1)
    
    def find_ball(self, image):
        # HSV色空間に変換
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 色範囲に基づくマスクを作成
        mask = cv2.inRange(image_hsv, self.lower_pink, self.upper_pink)
        # 輪郭を見つける
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # 最大の輪郭を見つける
            largest_contour = max(contours, key=cv2.contourArea)
            # 最小外接円を取得
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            area = cv2.contourArea(largest_contour)  # 面積を計算
            if area > 200:  # ノイズを無視するための閾値
                # 円を画像に描画
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                self.show_video(image)
                d = radius*2
                h = 10000/d
                # 中心を修正
                x -= self.height / 2
                y -= self.width / 2
                x, y = -y, x
                return int(x), int(y), int(area)  # 画像と座標、面積を返す
        self.show_video(image)
        return -1, -1, 0  # ボールが検出されなかった場合


    def clean_up_cam(self):
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()
