import cv2
import os

CAMERA_INDEX = 8  # Change to your camera device index

# Apply camera settings - update controls after checking your camera
os.system(
    f"v4l2-ctl -d /dev/video{CAMERA_INDEX} -c "
    "exposure_auto=1,"
    "white_balance_temperature_auto=0,"
    "white_balance_temperature=3300,"
    "exposure_absolute=500"
)

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print(f"Error: Cannot open camera {CAMERA_INDEX}")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("Egg Camera Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
