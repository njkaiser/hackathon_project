import cv2
import numpy as np

def trackbar(image, h, s, v):
    # create trackbars for color change
    cv2.createTrackbar('H',image,128,255,void,h)
    cv2.createTrackbar('S',image,128,255,void,s)
    cv2.createTrackbar('V',image,128,255,void,v)

    # create switch for ON/OFF functionality
    # switch = '0 : OFF \n1 : ON'
    # cv2.createTrackbar(switch, 'image',0,1,nothing)

    while(1):
        cv2.imshow('HSV trackbar window',image)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

        # get current positions of trackbars
        h = cv2.getTrackbarPos('H',image)
        s = cv2.getTrackbarPos('S',image)
        v = cv2.getTrackbarPos('V',image)

    cv2.destroyAllWindows()

    return [h, s, v]

if __name__ == "__main__":
    trackbar(image)
