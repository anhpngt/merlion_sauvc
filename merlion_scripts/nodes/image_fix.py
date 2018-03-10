import numpy as np
import cv2
import matplotlib.pyplot as plt

if __name__ == '__main__':
  img1 = cv2.imread('/home/echo/Desktop/3.png')
  img2 = cv2.imread('/home/echo/Desktop/4.png')

  y_ = img1.flatten()
  x_ = img2.flatten()
  
  plt.scatter(x_, y_, s=1)
  plt.show()