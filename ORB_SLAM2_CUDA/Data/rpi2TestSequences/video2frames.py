import cv2
import os

cap = cv2.VideoCapture("60secTest.mp4")

timeFile = open("rgb.txt", "w")
stamp = 1305031473.0
step = (1/30)

count =0
while(cap.isOpened()):
	ret, frame = cap.read()
	print(frame, ret)
	if ret:
		cv2.imwrite("rgb/"+"%d.jpg" % count, frame)
		timeFile.write(str(stamp) + " " + "rgb/" + str(count) + ".jpg\n")
                stamp += step
		count += 1
	else:
		break

timeFile.close()
cap.release()
cv2.destroyAllWindows()

