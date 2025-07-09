import cv2
import os

CAMERA_INDEX = 0  # Change to your camera index if needed

# Apply camera settings using v4l2-ctl
os.system(
    f"v4l2-ctl -d {CAMERA_INDEX} -c "
    "auto_exposure=1,"
    "exposure_dynamic_framerate=0,"
    "white_balance_automatic=0,"
    "white_balance_temperature=3300,"
    "exposure_time_absolute=500"
)

# Open the camera
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print(f"Error: Cannot open camera {CAMERA_INDEX}")
    exit()

# Live display loop
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("Egg Camera Feed", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
