import cv2

cap = cv2.VideoCapture("..//video_recordings//phonecam_60fps.mp4")
# cap = cv2.VideoCapture(r"C:\Users\CMC\Documents\openposelibs\pose\skateboard_gui\video_recordings\phonecam_60fps.mp4")
count = 0
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        cv2.imwrite(f".//images//{count}.jpg", frame)
        count += 1
        print(count)
    else:
        break
print("finished")
