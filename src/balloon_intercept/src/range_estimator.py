#!/usr/bin/env python
import cv2
import threading
import numpy

class RangeEstimator(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.est_radius = 0.75
        
    def __call__(self, pixels):
        self.pixels = pixels
    
    def run(self):
        while True:
            try:
                print("test")
            except:
                continue

def main():
    estimator = RangeEstimator()
    estimator.start()
    estimator.join()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        pass
