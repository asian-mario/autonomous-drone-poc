import cv2
import os
import numpy as np
import face_recognition
import glob
import time
from flask import Flask, Response, render_template_string, send_file, jsonify

app = Flask(__name__)

ESP32_STREAM_URL = "<ip address>:81/stream"

# GLOBAL CACHES
known_face_encodings = []
known_face_names = []
reference_images = {}  # Stores reference images for display
FRAME_SKIP = 5

# --- Load Faces ---
enrollment_files = glob.glob("./data/*.jpg")

if not enrollment_files:
    print("❌ No enrollment images found.")
else:
    for file in enrollment_files:
        try:
            raw_name = os.path.basename(file).rsplit(".", 1)[0]  # Remove extension
            name = " ".join(raw_name.split("_")[:-1])  # Extract base name
            
            image = face_recognition.load_image_file(file)
            encodings = face_recognition.face_encodings(image)

            if encodings:
                known_face_encodings.append(encodings[0])  # Store first encoding
                known_face_names.append(name)
                reference_images[name] = file  # Store reference image path
                print(f"✅ Enrolled {name} from {file}")

        except Exception as e:
            print(f"❌ Error loading {file}: {e}")

print(f"🚀 Enrolled {len(known_face_encodings)} faces for {len(set(known_face_names))} people.")

# FPS Tracker
frame_count = 0
last_time = time.perf_counter()  # High-precision timer
detected_face = {"name": "Unknown", "ref_img": ""}  # Stores last detected face info

# Timer for detecting and recognizing faces
face_detected_timer = None  # Stores start time when a face is detected

def gen_frames():
    global frame_count, last_time, detected_face, face_detected_timer

    cap = cv2.VideoCapture(ESP32_STREAM_URL, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("❌ Error: Cannot open ESP32-CAM stream")
        return

    while True:
        frame_count += 1
        success, frame = cap.read()
        if not success:
            continue

        small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        rgb_small_frame = small_frame[:, :, ::-1]

        face_locations = face_recognition.face_locations(rgb_small_frame, model="cnn")
        face_encodings = []

        for loc in face_locations:
            try:
                if not isinstance(loc, tuple) or len(loc) != 4 or not all(isinstance(x, int) for x in loc):
                    continue

                frame_uint8 = np.array(rgb_small_frame, dtype=np.uint8)
                encodings = face_recognition.face_encodings(frame_uint8, [loc], num_jitters=1)

                if encodings:
                    face_encodings.append(encodings[0])

            except Exception as e:
                print(f"❌ Face encoding error: {e}")

        tolerance = 0.5
        face_names = []
        recognized = False  # Flag to check if a known face is found

        for face_encoding in face_encodings:
            if not known_face_encodings:
                face_names.append("Unknown")
                continue

            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)

            if face_distances[best_match_index] < tolerance:
                name = known_face_names[best_match_index]
                detected_face["name"] = name
                detected_face["ref_img"] = reference_images.get(name, "")
                recognized = True  # Found a known face
                print(f"✅ Recognized: {name}")
            else:
                name = "Unknown"

            face_names.append(name)

        # ✅ Start Timer when any face is detected
        if face_locations and face_detected_timer is None:
            face_detected_timer = time.perf_counter()  # High-precision start time
            print("⏳ Timer started: Face detected")

        # ✅ Stop Timer when a known face is recognized
        if recognized and face_detected_timer is not None:
            elapsed_time = (time.perf_counter() - face_detected_timer) * 1000  # Convert to milliseconds
            print(f"⏱️ Recognition time: {elapsed_time:.3f} ms")  # Print time with millisecond precision
            face_detected_timer = None  # Reset timer

        # Draw bounding boxes and labels
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            top *= 2
            right *= 2
            bottom *= 2
            left *= 2

            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 255, 0), cv2.FILLED)
            cv2.putText(frame, name, (left + 6, bottom - 6),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1)

        if frame_count % 10 == 0:
            fps = 10 / (time.perf_counter() - last_time)
            last_time = time.perf_counter()
            print(f"🎯 FPS: {fps:.2f}")

        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return render_template_string('''
        <html>
        <head>
            <title>ESP32-CAM Face Recognition</title>
            <script>
                function updateFaceData() {
                    fetch("/detected_face").then(response => response.json()).then(data => {
                        document.getElementById("detected_name").innerText = data.name;
                        let img = document.getElementById("detected_image");
                        if (data.ref_img) {
                            img.src = "/reference_image?_=" + new Date().getTime();
                            img.style.display = "block";
                        } else {
                            img.style.display = "none";
                        }
                    });
                }
                setInterval(updateFaceData, 1000);
            </script>
        </head>
        <body>
            <h1>ESP32-CAM Face Recognition</h1>
            <img src="{{ url_for('video_feed') }}" width="640" height="480">
            <h2>Last Detected Face: <span id="detected_name">Unknown</span></h2>
            <img id="detected_image" src="" width="200" style="display:none;">
        </body>
        </html>
    ''')

@app.route('/detected_face')
def detected_face_data():
    return jsonify(detected_face)

@app.route('/reference_image')
def reference_image():
    if detected_face["ref_img"]:
        return send_file(detected_face["ref_img"], mimetype='image/jpeg')
    return "", 404

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
