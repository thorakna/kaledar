from picamera.array import PiRGBArray
from picamera import PiCamera

try:
    import cv2
except Exception as e:
    print("Warning: OpenCV not installed. To use motion detection, make sure you've properly configured OpenCV.")

import time
import thread
import threading
import atexit
import sys
import termios
import contextlib
import Adafruit_PCA9685

import imutils
import RPi.GPIO as GPIO


camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(320, 240))

# allow the camera to warmup
time.sleep(0.1)
@contextlib.contextmanager
def raw_mode(file):
    """
    Magic function that allows key presses.
    :param file:
    :return:
    """
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)


class VideoUtils(object):
    """
    Helper functions for video utilities.
    """
    @staticmethod
    def live_video(camera_port=0):
        video_capture = cv2.VideoCapture(camera_port)

        while True:
            # Capture frame-by-frame
            ret, frame = video_capture.read()
            cv2.line(frame, (300, 220), (315, 235),(0,0,255), 2)
            cv2.line(frame, (340, 220), (325, 235),(0,0,255), 2)
            cv2.line(frame, (340, 260), (325, 245),(0,0,255), 2)
            cv2.line(frame, (300, 260), (315, 245),(0,0,255), 2)

            # Display the resulting frame
            cv2.imshow('VIDEO', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything is done, release the capture
        video_capture.release()
        cv2.destroyAllWindows()

    @staticmethod
    def find_motion(callback, calltwo, camera_port=0, show_video=True):

        #time.sleep(0.25)

        # initialize the first frame in the video stream
        firstFrame = None
        tempFrame = None
        count = 0

        # loop over the frames of the video
        for framemm in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		    # grab the raw NumPy array representing the image, then initialize the timestamp
		    # and occupied/unoccupied text
            frame = framemm.array
            rawCapture.truncate(0)
            rawCapture.seek(0)
		    # clear the stream in preparation for the next frame
		    

            # if the frame could not be grabbed, then we have reached the end
            # of the video
            #if not grabbed:
            #    break

            # resize the frame, convert it to grayscale, and blur it
            frame = imutils.resize(frame, width=320)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)
            # if the first frame is None, initialize it
            if firstFrame is None:
                print "Kalibrasyon icin bekleniyor..."
                if tempFrame is None:
                    tempFrame = gray
                    continue
                else:
                    delta = cv2.absdiff(tempFrame, gray)
                    tempFrame = gray
                    tst = cv2.threshold(delta, 5, 255, cv2.THRESH_BINARY)[1]
                    tst = cv2.dilate(tst, None, iterations=2)
                    if count > 30:
                        print "Tamamlandi.\n Hareket bekleniyor."
                        if not cv2.countNonZero(tst) > 0:
                            firstFrame = gray
                        else:
                            continue
                    else:
                        count += 1
                        continue

            # compute the absolute difference between the current frame and
            # first frame
            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

            # dilate the thresholded image to fill in holes, then find contours
            # on thresholded image
            thresh = cv2.dilate(thresh, None, iterations=5)
            c = VideoUtils.get_best_contour(thresh.copy(), 3500)
            if c is not None:
                # compute the bounding box for the contour, draw it on the frame,
                # and update the text
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                callback(c, frame)

            # show the frame and record if the user presses a key
            if show_video:
                cv2.imshow("VIDEO", frame)
                key = cv2.waitKey(1) & 0xFF

                # if the `q` key is pressed, break from the lop
                if key == ord("q"):
                    break
                elif key == ord("\n"):
				    calltwo()
					
        # cleanup the camera and close any open windows
        cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        im, contours, hierarchy = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt


class Turret(object):
    """
    Class used for turret control.
    """
    def __init__(self):
        GPIO.setwarnings(False)
        self.MOTOR_X_DERECE = 100
        self.MOTOR_Y_DERECE = 220
        self.ILK = 0
        self.ATES_SAYISI = 0
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        #Turret.goservoy(self, self.MOTOR_Y_DERECE)
        #Turret.goservox(self, self.MOTOR_Y_DERECE)

    def motion_detection(self, show_video=True):
        """
        Uses the camera to move the turret. OpenCV ust be configured to use this.
        :return:
        """
        VideoUtils.find_motion(self.__move_axis, self.ates, show_video=show_video)

    def __move_axis(self, contour, frame):
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)

        # find height
        target_d_x = 433 - ((126*(x + w / 2)) / v_w)
        target_d_y = 580 - ((230*(y + (h / 2))) / v_h)

        print "x kordinati: %s, y kordinati: %s" % (str(target_d_x), str(target_d_y))

        # move x
        Turret.goservox(self, target_d_x)

        # move y
        if(self.ILK == 0):
			Turret.goservoy(self, target_d_y)
			self.ILK = 1
        
        if(abs(self.MOTOR_Y_DERECE - target_d_y) < 30):
			fark = abs(self.MOTOR_Y_DERECE - target_d_y)
			print "Y Ekseni arasindaki fark: %s" % (fark)
			Turret.goservoy(self, target_d_y)
        else:
			print "Y EKSENI GOZ ARDI EDILDI!!"
        
        
        #time.sleep(0.1)
        
    def interactive(self):
        print 'Interaktif mod aktif edildi!'
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch or ch == "q":
						break

                    if ch == "s":
                        if (self.MOTOR_Y_DERECE > 220):
                            self.MOTOR_Y_DERECE = self.MOTOR_Y_DERECE - 2
                            Turret.goservoy(self, self.MOTOR_Y_DERECE)
                            print self.MOTOR_Y_DERECE
                    elif ch == "w":
                        if (self.MOTOR_Y_DERECE < 600):
                            self.MOTOR_Y_DERECE = self.MOTOR_Y_DERECE + 2
                            Turret.goservoy(self, self.MOTOR_Y_DERECE)
                            print self.MOTOR_Y_DERECE
                    elif ch == "a":
                        if (self.MOTOR_X_DERECE < 600):
                            self.MOTOR_X_DERECE = self.MOTOR_X_DERECE + 3
                            Turret.goservox(self, self.MOTOR_X_DERECE)
                            print self.MOTOR_X_DERECE
                    elif ch == "d":
                        if (self.MOTOR_X_DERECE > 100):
                            self.MOTOR_X_DERECE = self.MOTOR_X_DERECE - 3
                            Turret.goservox(self, self.MOTOR_X_DERECE)
                            print self.MOTOR_X_DERECE
                    elif ch == "\n":
                        Turret.ates(self)

            except (KeyboardInterrupt, EOFError):
                pass
                
    def goservox(self, angle):
        self.MOTOR_X_DERECE = angle
        self.pwm.set_pwm(8, 1, angle)
        
    def goservoy(self, anglem):
        self.MOTOR_Y_DERECE = anglem
        self.pwm.set_pwm(4, 1, anglem)
        
    def ates(self):
        self.ATES_SAYISI = self.ATES_SAYISI + 1
        print 'Ates Edildi! Atis Sayisi: ',self.ATES_SAYISI
        self.pwm.set_pwm(0, 0, 120)
        time.sleep(0.2)
        self.pwm.set_pwm(0, 0, 160)
        
        

if __name__ == "__main__":
    t = Turret()
    
    user_input = raw_input("Bir mod secin: (1) Harekete duyarli, (2) Manuel\n")

    if user_input == "1":
        t.motion_detection(show_video=True)
    elif user_input == "2":
        thread.start_new_thread(VideoUtils.live_video, ())
        t.interactive()
