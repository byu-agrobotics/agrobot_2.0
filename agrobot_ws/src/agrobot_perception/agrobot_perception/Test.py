import cv2

for i in range(36):  # You listed up to /dev/video35
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera index {i} is available")
        ret, frame = cap.read()
        if ret:
            print(f"  Got a frame from /dev/video{i}")
        cap.release()