import cv2
import numpy as np
import serial
import time

# ================= USER MODE =================
MODE = "TRIANGLE"     # "SQUARE" or "TRIANGLE" or "CIRCLE"
# ============================================

# ================= CAPTURE MODE ==============
AUTO_CAPTURE = False
WARMUP_FRAMES = 15
CAMERA_INDEX = 2
# ============================================

# ================= SERIAL SETTINGS ============
SERIAL_PORT = "COM7"
BAUD_RATE = 9600
SERIAL_SEND_DELAY = 0.05   # small delay between messages
ENABLE_SERIAL = True
# ============================================


# --------------------------------------------------
# Serial init
# --------------------------------------------------
ser = None
if ENABLE_SERIAL:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) 
        time.sleep(2)
        print(f"[SERIAL] Connected to {SERIAL_PORT} @ {BAUD_RATE}")
    except Exception as e:
        print("[SERIAL] Could not open serial port:", e)
        ser = None


def send_line(line: str):
    """Send a raw line (already formatted) ending with \\n."""
    if ser is None:
        return
    ser.write((line.strip() + "\n").encode())
    print("[SERIAL] Sent:", line.strip())
    time.sleep(SERIAL_SEND_DELAY)

'''
def py_to_arduino_coords(px, py):
   
    minX = -28.0   # LEFT edge (measure this)
    maxX = -5.0    # RIGHT edge (measure this)

    minY = 3.0     # BOTTOM edge (measure this)
    maxY = 16.0   # TOP edge (measure this)

    ax = minX + (px / 640.0) * (maxX - minX)
    ay = maxY - (py / 360.0) * (maxY - minY)

    return ax, ay
'''
    # Base bounds (measured physically)
minX = -28.5
maxX = -4.5

minY = 5.0
maxY = 18.0

# Perspective tuning factors (start at zero)
x_perspective = 0   # tune this
y_skew = 0.5        # tune this

def py_to_arduino_coords(px, py):

    y_ratio = py / 360.0

    adj_minX = minX + x_perspective * y_ratio
    adj_maxX = maxX - x_perspective * y_ratio

    ax = adj_minX + (px / 640.0) * (adj_maxX - adj_minX)

    ay = maxY - (py / 360.0) * (maxY - minY)


    # ---- CORRECTED Y SKEW ----
    x_norm = (px - 320.0) / 320.0
    ay -= y_skew * x_norm
    ax = ax + 1
    ay = ay - 2.5
    return ax, ay
    

# --------------------------------------------------
# Helpers
# --------------------------------------------------
def contour_center(cnt):
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy


def is_square_like(approx):
    if len(approx) != 4:
        return False

    pts = approx.reshape(4, 2)
    for i in range(4):
        p0 = pts[i]
        p1 = pts[(i + 1) % 4]
        p2 = pts[(i - 1) % 4]
        v1 = p1 - p0
        v2 = p2 - p0
        denom = (np.linalg.norm(v1) * np.linalg.norm(v2))
        if denom == 0:
            return False
        cos_angle = abs(np.dot(v1, v2) / denom)
        if cos_angle > 0.35:
            return False

    x, y, w, h = cv2.boundingRect(approx)
    if h == 0:
        return False
    ar = w / float(h)
    return 0.80 < ar < 1.20


def is_triangle(cnt):
    peri = cv2.arcLength(cnt, True)
    for eps in [0.02, 0.03, 0.04, 0.05]:
        approx = cv2.approxPolyDP(cnt, eps * peri, True)
        if len(approx) == 3:
            return True
    return False


def is_circle(cnt):
    area = cv2.contourArea(cnt)
    if area < 800:
        return False
    peri = cv2.arcLength(cnt, True)
    if peri == 0:
        return False
    circularity = 4 * np.pi * area / (peri * peri)
    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
    return circularity > 0.82 and len(approx) > 7


# --------------------------------------------------
# 1. Capture image
# --------------------------------------------------
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    raise RuntimeError(f"Could not open webcam (VideoCapture({CAMERA_INDEX})).")

cv2.namedWindow("Live Feed", cv2.WINDOW_NORMAL)

frame = None
frame_count = 0

print("Webcam started.")
if AUTO_CAPTURE:
    print(f"Auto-capturing after {WARMUP_FRAMES} warmup frames...")
else:
    print("Click the Live Feed window, then press SPACE to capture, or ESC to quit.")

while True:
    ret, live = cap.read()
    if not ret:
        continue

    frame_count += 1
    cv2.imshow("Live Feed", live)

    key = cv2.waitKey(1) & 0xFF

    if cv2.getWindowProperty("Live Feed", cv2.WND_PROP_VISIBLE) < 1:
        cap.release()
        cv2.destroyAllWindows()
        raise SystemExit("Window closed without capturing.")

    if AUTO_CAPTURE:
        if frame_count >= WARMUP_FRAMES:
            frame = live.copy()
            break
    else:
        if key == 27:
            cap.release()
            cv2.destroyAllWindows()
            raise SystemExit("Exited without capturing.")
        if key == 32:
            frame = live.copy()
            break

cap.release()
cv2.destroyWindow("Live Feed")

img = frame


# --------------------------------------------------
# 2. Preprocessing
# --------------------------------------------------
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = cv2.bilateralFilter(gray, 9, 75, 75)

thresh = cv2.adaptiveThreshold(
    gray, 255,
    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
    cv2.THRESH_BINARY_INV,
    31,
    5
)

kernel = np.ones((3, 3), np.uint8)
thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

cv2.imshow("THRESH (Debug)", thresh)


# --------------------------------------------------
# 3. Find contours
# --------------------------------------------------
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


# --------------------------------------------------
# 4. Detection + collect centers (DON'T send inside loop)
# --------------------------------------------------
total_objects = 0
correct_objects = 0
wrong_centers_px = []     # (cx,cy) in pixels
wrong_centers_axay = []   # (ax,ay) converted for Arduino

for cnt in contours:
    area = cv2.contourArea(cnt)
    if area < 800:
        continue

    x0, y0, w0, h0 = cv2.boundingRect(cnt)
    if w0 < 25 or h0 < 25:
        continue

    total_objects += 1

    epsilon = 0.03 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)

    x, y, w, h = cv2.boundingRect(approx)

    if MODE == "SQUARE":
        ok = is_square_like(approx)
        ok_label = "SQUARE OK"
    elif MODE == "TRIANGLE":
        ok = is_triangle(cnt)
        ok_label = "TRIANGLE OK"
    elif MODE == "CIRCLE":
        ok = is_circle(cnt)
        ok_label = "CIRCLE OK"
    else:
        raise ValueError("Invalid MODE")

    if ok:
        correct_objects += 1
        label = ok_label
        color = (0, 255, 0)
    else:
        label = "WRONG"
        color = (0, 0, 255)

        center = contour_center(cnt)
        if center is not None:
            cx, cy = center
            wrong_centers_px.append((cx, cy))

            ax, ay = py_to_arduino_coords(cx, cy)
            wrong_centers_axay.append((ax, ay))

            # draw center on image
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                img, f"({cx},{cy})",
                (cx - 70, cy + 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 2
            )

            print(f"WRONG center PX=({cx},{cy})  ->  AXAY=({ax:.2f},{ay:.2f})")

    cv2.drawContours(img, [approx], -1, color, 2)
    cv2.putText(
        img, label,
        (x, max(20, y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6, color, 2
    )


# --------------------------------------------------
# 5. Summary + send ALL centers to Arduino
# --------------------------------------------------
percent = (correct_objects / total_objects * 100) if total_objects > 0 else 0.0
summary = f"TOTAL: {total_objects}   CORRECT: {correct_objects}   SCORE: {percent:.1f}%"
print(summary)

cv2.putText(img, summary, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4)
cv2.putText(img, summary, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

#if wrong_centers_axay:
#    print("All WRONG centers (AX,AY):", [(round(a,2), round(b,2)) for a,b in wrong_centers_axay])
#
#    # Send batch to Arduino:
#    # BEGIN
#    # C,x,y
#    # C,x,y
#    # END
#    # START
#    send_line("BEGIN")
#    for ax, ay in wrong_centers_axay:
#        send_line(f"C,{ax:.2f},{ay:.2f}")
#    send_line("END")
#    send_line("START")  # tell Arduino to draw circles now
#else:
#    print("No WRONG shapes detected.")

if wrong_centers_axay:
    print("All WRONG centers (AX,AY):", [(round(a,2), round(b,2)) for a,b in wrong_centers_axay])

    # Always compute mark
    mark_text = f"{correct_objects}/{total_objects}"
    print("MARK:", mark_text)

    # Send batch (even if empty)
    send_line("BEGIN")

    for ax, ay in wrong_centers_axay:
        send_line(f"C,{ax:.2f},{ay:.2f}")

    send_line("END")

    # 🔥 Send ONLY correct count as integer
    send_line(f"CORRECT,{correct_objects}")

    send_line("START")
else:
    print("No WRONG shapes detected.")

 

# --------------------------------------------------
# 6. Show output
# --------------------------------------------------

print(frame.shape)

cv2.imshow("Shape Check", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

if ser is not None:
    ser.close()