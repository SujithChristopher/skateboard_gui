import cv2

cap = cv2.VideoCapture("aruco_original.mp4")
count = 0
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        cv2.imwrite(f"images//{count}.jpg", frame)
        count += 1
        print(count)
    else:
        break
print("finished")
