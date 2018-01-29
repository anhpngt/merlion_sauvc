import cv2
import numpy as np 
import matplotlib.pyplot as plt

def get_start_end(hist):
    print(len(hist))
    diff=hist[1:len(hist)]-hist[0:len(hist)-1]
    # diff=np.sort(diff)
    start=np.argmax(diff)
    # print(diff[0], diff[1])

    # plt.plot(hist)
    # plt.show()

    return start, 0


img=cv2.imread("/home/rm/Pictures/image3.png")
# cv2.imshow("img", img)
# cv2.waitKey(0)
res=img.copy()
print(img.shape)

blur = cv2.GaussianBlur(img,(5, 5),0)
grey=cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)


hsv=cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
adap_thres = cv2.adaptiveThreshold(hsv[:, :, 2],255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,2)

# cv2.imshow("img", blur)
# cv2.waitKey(0)

# define the list of boundaries
boundaries = [
    # ([0, 0, 0], [150, 100, 100]),
    ([100, 20, 20], [150, 180, 100])
]


for (lower, upper) in boundaries:

    #threshold img
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    mask = cv2.inRange(blur, lower, upper)
    output = cv2.bitwise_and(img, img, mask = mask)


   #get vertical lines
    histogram_x = np.sum(mask, axis=0)

    pointfour = np.int(0.4*histogram_x.shape[0])
    pointsix=np.int(0.6*histogram_x.shape[0])
    leftx_base = np.argmax(histogram_x[:pointfour]) #get the peak from first half(left)
    rightx_base = np.argmax(histogram_x[pointsix:]) + pointsix #get peak from secondhalf(right)

    font = cv2.FONT_HERSHEY_SIMPLEX
    color=(0, 255, 100)

    left_val=np.sum(histogram_x[leftx_base-20:leftx_base+20])
    right_val=np.sum(histogram_x[rightx_base-20:rightx_base+20])

    thres=100

    if left_val>thres:
        res=cv2.line(res, (leftx_base, 0), (leftx_base, img.shape[0]), (255, 0, 0), 2, 8, 0)
        cv2.putText(res, str(left_val), (leftx_base, 15), font, 0.4, color, 1, cv2.LINE_AA)
        mask=mask.astype(np.float64)
        histogram_left = np.sum(mask[:, leftx_base-20:leftx_base+20], axis=1)
        # start, end=get_start_end(histogram_left)
        # cv2.circle(res, (int(leftx_base), int(start)), 10, color, -1)


    if right_val>thres:
        res=cv2.line(res, (rightx_base, 0), (rightx_base, img.shape[0]), (255, 0, 0), 2, 8, 0)
        cv2.putText(res, str(right_val), (rightx_base, 15), font, 0.4, color, 1, cv2.LINE_AA)
        mask=mask.astype(np.float64)
        # histogram_right = np.sum(mask[rightx_base-20:rightx_base+20, :], axis=1)
        # start, end=get_start_end(histogram_right)


    # show the images
    # cv2.imshow("mask", res)
    cv2.imshow("img", np.hstack([output, res]))
    cv2.waitKey(0)

#find vertical rectangle