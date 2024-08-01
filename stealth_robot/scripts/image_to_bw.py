import cv2

img_name = "floorplan_ROS"
image = cv2.imread('/home/ralph/Pictures/'+img_name+'.pgm')

ret, bw_img = cv2.threshold(image, 220, 255, cv2.THRESH_BINARY)
bw_img = cv2.cvtColor(bw_img, cv2.COLOR_BGR2GRAY)
cv2.imshow("BW image", bw_img)
cv2.imwrite('/home/ralph/Pictures/'+img_name+'_binary.pgm', bw_img)
cv2.waitKey(0)
cv2.destroyAllWindows()




