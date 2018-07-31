import time

import imutils
import queue

try:
    from picamera.array import PiYUVArray
    from picamera import PiCamera
    import RPi.GPIO as GPIO
except Exception as e:
    pass

import cv2              #OpenCV
import numpy as np      

class MetalDetect:
    def __init__(self, parent=None):
        self.camera = PiCamera()
        try:
            self.resolution_width = 2592 # 2592
            self.resolution_height = 256 #256 
            self.camera.resolution = (self.resolution_width, self.resolution_height)
            # self.camera.shutter_speed = 5000
            # self.camera.iso = 200
            # self.image = PiYUVArray(self.camera, self.camera.resolution)
            self.img = None

            time.sleep(3)

            self.ioinput = 29
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.ioinput, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
            GPIO.add_event_detect(self.ioinput, GPIO.BOTH, bouncetime = 2000)
            # GPIO.add_event_callback(16, self.captureThree).

            self.capture_TimeCost = 0.0
            self.algorithm_TimeCost = 0.0
            self.Total_TimeCost = 0.0
        except Exception as e:
            print(e)


    def __del__(self):
        self.image.seek(0)
        self.image.truncate(0)
        self.camera.close()
        cv2.destroyAllWindows()
        print('destroyAll')

    def captureThree(self):
        pass


    def captureImages(self):
        """
        捕获图像
        :return:
        """
        try:
            self.image = PiYUVArray(self.camera, self.camera.resolution)
            try:
                for i in range(50):
                    
                    time_cap = time.time()
                    self.camera.capture(self.image, format="yuv", use_video_port=True)
                    print('capture timeCost:        '+ str(round((time.time() - time_cap),4)))

                    time_truncate = time.time()
                    '''流的后续处理'''
                    self.image.truncate(0)
                    self.image.seek(0)
                    
                    print('time_truncate timeCost:  '+ str(round((time.time() - time_truncate),4)))

                    time_array= time.time()
                    img = self.image.array
                    print(img.shape)
                    print('array timeCost:          '+ str(round((time.time() - time_array),4)))

                    time_resize = time.time()
                    img = imutils.resize(self.image.array, width=self.resolution_width, height=self.resolution_height)
                    print('resize timeCost:         '+ str(round((time.time() - time_resize),4)))
                    self.capture_TimeCost = round((time.time() - time_cap),4)

                    timeStart = time.time()
                    # self.detectImage_binary(img)
                    print('TotalCost:***************'+ str(round((time.time() - time_cap),4)))
                    self.algorithm_TimeCost = round((time.time() - timeStart),4)
                    self.Total_TimeCost = round((time.time() - time_cap),4)
                    self.timecaclulate(i)
                self.camera.close()
                cv2.destroyAllWindows()
            except Exception as e:
                print(e)
                self.image.seek(0)
                self.image.truncate(0)
        except Exception as e:
            print(e)


    ''' 计算总时间，捕获时间，处理时间比例'''
    def timecaclulate(self, i):
        print('*****************************************')
        print('num: ' + str(i))
        print('Capture_TimeCost  :' + str(round((self.capture_TimeCost / self.Total_TimeCost*100),3)) + '%')
        print('Algorithm_TimeCost:' + str(round((self.algorithm_TimeCost / self.Total_TimeCost*100), 3)) + '%')
        print('*****************************************')
        pass

    ''' 无检测，显示保存'''
    def detectImage_ShowSave(self, i, image):
        cv2.imshow('img',image)                       # 测试图片采集
        cv2.imwrite("test" + str(i) + ".jpg", image)   # 保存测试样本
        cv2.waitKey(30)                             # 界面延时显示

    ''' 阈值化检测'''
    def detectImage_binary(self, image):
        time1 = time.time()
        dest = cv2.cvtColor(image, cv2.COLOR_YUV420p2GRAY,1)
        # dest = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        print('cvtColor timeCost:       '+ str(round((time.time() - time1),4)))

        time2 = time.time()
        blur = cv2.GaussianBlur(dest ,(5,1),0)
        print('GaussianBlur timeCost:   '+ str(round((time.time() - time2),4)))

        time3 = time.time()
        ret,binary_frame = cv2.threshold( blur, 200, 255, cv2.THRESH_BINARY)
        print('threshold timeCost:      '+ str(round((time.time() - time3),4)))

        time4 = time.time()
        cv2.imshow("binary_frame",binary_frame)
        cv2.waitKey(30)
        print('Show timeCost:           '+ str(round((time.time() - time4),4)))

    '''滤波阈值检测'''
    def detectImage(self,image):
        dest = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(dest ,(5,1),0)
        g_kernel = cv2.getGaborKernel((41, 41), 19.04, 0, 34, 0, 0, ktype=cv2.CV_32F)
        gabor = cv2.filter2D(blur, cv2.CV_8UC3, g_kernel)
        ret,threshold = cv2.threshold(gabor, 60, 255,cv2.THRESH_BINARY)
        kernel = np.ones((5,5), np.uint8) 
        dilation = cv2.dilate(threshold, kernel,iterations = 1) 

        cv2.imshow("gabor",gabor)
        cv2.imshow("dilation",dilation)
        cv2.imshow("grayFrame", dest)

        cv2.waitKey(30)

if __name__ == '__main__':
    metaldetect = MetalDetect()
    metaldetect.captureImages()
