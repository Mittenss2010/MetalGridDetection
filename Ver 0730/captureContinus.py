
# import time
# import picamera
# with picamera.PiCamera() as camera:
#     camera.start_preview()
#     try:
#         for i, filename in enumerate(
#                 camera.capture_continuous('image{counter:02d}.jpg')):
#             print(filename)
#             time.sleep(1)
#             if i == 59:
#                 break
#     finally:
#         camera.stop_preview()

import time
import imutils
import queue
import math

try:
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    import picamera
    import picamera.array

    import RPi.GPIO as GPIO
except RuntimeError:
     print("导入 RPi.GPIO 时出现错误！这可能由于没有超级用户权限造成的。您可以使用 'sudo' 来运行您的脚本。")

import cv2              #OpenCV
import numpy as np      #数学计算库

class MetalDetect:
    def __init__(self, parent=None):
        self.camera = PiCamera()
        try:
            self.resolution_width = 2592
            self.resolution_height = 256 
            self.camera.resolution = (self.resolution_width, self.resolution_height)
            self.camera.shutter_speed = 5000
            # self.camera.iso = 200
            # self.image = PiRGBArray(self.camera, size=(self.resolution_width, self.resolution_height))
            self.img = None

            time.sleep(2)

            # self.ioinput = 29
            # GPIO.setmode( GPIO.BCM )
            # GPIO.setwarnings(False)
            # GPIO.setup(self.ioinput, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
            # GPIO.add_event_detect(self.ioinput, GPIO.BOTH, bouncetime = 2000)
            # GPIO.add_event_callback(16, self.captureThree).

            self.warningLightPin = 33  # 报警灯
            self.delaytime = 0.2

            self.capture_TimeCost = 0.0
            self.algorithm_TimeCost = 0.0
            self.Total_TimeCost = 0.0
        except Exception as e:
            print(e)

    
    def captureThree(self):
        pass


    def captureImages(self):
        """
        捕获图像
        :return:
        """
        with picamera.array.PiRGBArray(self.camera, self.camera.resolution) as stream:
            try:
                time_cap = time.time()
                enumerate
                
                for i, frame in enumerate(self.camera.capture_continuous(stream, format='bgr', resize=self.camera.resolution, use_video_port=True)): 

                    print('capture timeCost:        '+ str(round((time.time() - time_cap),4)))

                    time_truncate = time.time()
                    '''流的后续处理'''
                    stream.seek(0)
                    stream.truncate(0)
                    print('time_truncate timeCost:  '+ str(round((time.time() - time_truncate),4)))

                    '''图像数组化'''
                    time_array= time.time()
                    img = frame.array
                    print('array timeCost:          '+ str(round((time.time() - time_array),4)))

                    '''缩放'''
                    time_resize = time.time()
                    img = imutils.resize(frame.array, width=self.resolution_width, height=self.resolution_height)
                    print('缩放 resize timeCost:         '+ str(round((time.time() - time_resize),4)))
                    self.capture_TimeCost = round((time.time() - time_cap),4)

                    '''binary 阈值检测'''
                    timeStart = time.time()
                    self.detectImage_ShowColib(img, i)
                    print('TotalCost:***************'+ str(round((time.time() - time_cap),4)))
                    self.algorithm_TimeCost = round((time.time() - timeStart),4)
                    self.Total_TimeCost = round((time.time() - time_cap),4)
                    self.timecaclulate(i)
                    time_cap = time.time()

                self.camera.close()
                cv2.destroyAllWindows()
            except Exception as e:
                print(e)
                stream.seek(0)
                stream.truncate(0)


    ''' 计算总时间，捕获时间，处理时间比例'''
    def timecaclulate(self, i):
        print('*****************************************')
        print('num: ' + str(i))
        print('Capture_TimeCost  :' + str(round((self.capture_TimeCost / self.Total_TimeCost*100),3)) + '%')
        print('Algorithm_TimeCost:' + str(round((self.algorithm_TimeCost / self.Total_TimeCost*100), 3)) + '%')
        print('*****************************************')
        pass


    ''' 无检测，显示保存'''
    def detectImage_ShowSave(self, image, i):
        cv2.imshow('img',image)                        # 测试图片采集
        cv2.imwrite("test" + str(i) + ".jpg", image)   # 保存测试样本
        cv2.waitKey(30)                                # 界面延时显示

    ''' 无检测，显示校准'''
    def detectImage_ShowColib(self, image, i):
        image = cv2.line(image,(0, math.ceil(self.resolution_height/2)),(self.resolution_width, math.ceil(self.resolution_height/2)),(255,255,0),5)
        cv2.imshow('img', image)                        # 测试图片采集
        cv2.waitKey(30)                                # 界面延时显示

    ''' 阈值化检测'''
    def detectImage_binary(self, image, i):
        time1 = time.time()
        dest = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        print('cvtColor timeCost:       '+ str(round((time.time() - time1),4)))

        time2 = time.time()
        blur = cv2.GaussianBlur(dest ,(5,1),0)
        print('GaussianBlur timeCost:   '+ str(round((time.time() - time2),4)))

        time3 = time.time()
        ret,binary_frame = cv2.threshold( blur, 200, 255, cv2.THRESH_BINARY)
        print('threshold timeCost:      '+ str(round((time.time() - time3),4)))


        image, contours, hierarchy = cv2.findContours(binary_frame, 
                                                      cv2.RETR_TREE,
                                                      cv2.CHAIN_APPROX_SIMPLE)

        # 获取目标详细信息              
        if len(contours) > 0:
            # 报警
            self.warningLight()
            # 轮廓搜索完毕，展示最大面积区域
            try:
                areaList = []                               # 面积 list
                for c in contours:                          # 遍历轮廓集合
                    area = cv2.contourArea(c)               # 获取面积
                    areaList.append(area)                   # 加入面积数字列表
                idx = areaList.index(max(areaList))         # list找最大，并获取其下标 index

                moment = cv2.moments(contours[idx])               # 质心
                center = (int(moment["m10"] / moment["m00"]),     # 质心计算公式
                        int(moment["m01"] / moment["m00"]))

                x,y,w,h = center[0],center[1],100,100           # 连续赋值

                cv2.circle(dest, center,50, (0,255,0), 2)      # 画圆
            
    #         # 展示坐标数字
    #         str_pos = 'pos:'+ str(x) +','+ str(y)           
    #         cv2.putText(frame, str_pos, (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 1)
            except Exception as e:
                dest = dest
#  #   frame = cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
#  #   mask = cv2.drawContours(mask, contours, -1,(0,255,0), 3)

        cv2.imshow('dest' ,dest)

        time4 = time.time()
        cv2.imshow("binary_frame",binary_frame)

        k = cv2.waitKey(30) & 0xFF
        # if k == 27:
        #     break
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

    '''滤波阈值检测'''
    def warningLight(self):

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.warningLightPin, GPIO.OUT)

        for i in range(3):
            GPIO.output(self.warningLightPin, True)
            time.sleep(self.delaytime)
            GPIO.output(self.warningLightPin, False)
            time.sleep(self.delaytime)

        GPIO.cleanup()

if __name__ == '__main__':
    metaldetect = MetalDetect()
    metaldetect.captureImages()
