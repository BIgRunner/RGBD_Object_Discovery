import cv2

saliency = cv2.imread("saliency/0100128-color_saliency.png", -1)
_,good = cv2.threshold(saliency, 128, 255, cv2.THRESH_BINARY)
cv2.imshow("here", good)
cv2.waitKey()
