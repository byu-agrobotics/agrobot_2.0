import cv2
from flask import Flask, Response
import os

CAMERA_INDEX = 8  # Change to your camera device index

# Apply camera settings - update controls after checking your camera
os.system(
    f"v4l2-ctl -d /dev/video{CAMERA_INDEX} -c "
    "white_balance_temperature_auto=0,"
    "white_balance_temperature=3300,"
    "exposure_absolute=500"
)

camera = cv2.VideoCapture(CAMERA_INDEX)

app = Flask(__name__)
# camera = cv2.VideoCapture(0)  # Or use your GStreamer pipeline if needed

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Convert to JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # Yield frame with proper multipart headers
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video')
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)



# import cv2
# import os

# CAMERA_INDEX = 8  # Change to your camera device index

# # Apply camera settings - update controls after checking your camera
# os.system(
#     f"v4l2-ctl -d /dev/video{CAMERA_INDEX} -c "
#     "white_balance_temperature_auto=0,"
#     "white_balance_temperature=3300,"
#     "exposure_absolute=500"
# )

# cap = cv2.VideoCapture(CAMERA_INDEX)

# if not cap.isOpened():
#     print(f"Error: Cannot open camera {CAMERA_INDEX}")
#     exit()

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("Failed to grab frame")
#         break

#     cv2.imshow("Egg Camera Feed", frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()
