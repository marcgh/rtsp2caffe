# pylint: disable=C0111, C0103, E1101, W0603, C0411, W0702, W0201, C0301, W0602

import numpy as np
import cv2
import threading
import Queue
#import caffe

print cv2.__version__

BUF_SIZE = 5
q = Queue.Queue(BUF_SIZE)
qdisp = Queue.Queue(BUF_SIZE)
alive = True
skipcount = 0

class ProducerThread(threading.Thread):
    # This class will prodce frames to the Q
    # When the Q is full, frames are skipped.
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
        super(ProducerThread, self).__init__()
        self.target = target
        self.name = name
        self.cap = cv2.VideoCapture("rtsp://192.168.0.102:554/h264")
        self.ret = False

    def run(self):
        global skipcount
        while alive:
            self.ret, frame = self.cap.read()
            try:
                q.put(frame, block=False)
            except:
                print 'processing skip'
            try:
                qdisp.put(('scaled', frame, 0.4), block=False)
            except:
                print 'display skip #', skipcount
                skipcount += 1

        self.cap.release()
        return


class DisplayThread(threading.Thread):
    # This class will display frames
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
        super(DisplayThread, self).__init__()
        self.target = target
        self.name = name
        return

    def run(self):
        global qdisp
        while alive:
            title, frame, ratio = qdisp.get(block=True)
            frame = cv2.resize(frame, (0, 0), fx=ratio, fy=ratio)
            cv2.imshow(title, frame)
            cv2.waitKey(1)
        return


class ConsumerThread(threading.Thread):
    # processing thread
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
        super(ConsumerThread, self).__init__()
        self.target = target
        self.name = name
        self.firstframe = True
        return

    def run(self):
        while alive:
            frame = q.get(block=True)
            frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
            if self.firstframe:
                self.hsv = np.zeros_like(frame)
                self.hsv[..., 1] = 255
                self.old_frame = frame
                self.old_gray = cv2.cvtColor(self.old_frame, cv2.COLOR_BGR2GRAY)
                self.firstframe = False
            else:
                self.frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                self.flow = cv2.calcOpticalFlowFarneback(self.old_gray, self.frame_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
                self.mag, self.ang = cv2.cartToPolar(self.flow[..., 0], self.flow[..., 1])
                self.hsv[..., 0] = self.ang*180/np.pi/2
                self.hsv[..., 2] = cv2.normalize(self.mag, None, 0, 255, cv2.NORM_MINMAX)
                self.bgr = cv2.cvtColor(self.hsv, cv2.COLOR_HSV2BGR)
                try:
                    qdisp.put(('process', self.bgr, 1.0), block=False)
                except:
                    print 'display skip from process'
        return

if __name__ == '__main__':
    p = ProducerThread(name='producer')
    c = ConsumerThread(name='consumer')
    d = DisplayThread(name='display')
    p.daemon = True
    c.daemon = True
    d.daemon = True
    d.start()
    p.start()
    c.start()

    raw_input("")

    alive = False
    p.join()
    c.join()
    d.join()

