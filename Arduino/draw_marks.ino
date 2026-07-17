#include <Servo.h>

Servo servo1;
Servo servo2;
Servo penServo;

int correctCount = 0;

// ===== Arm lengths (cm) =====
float l1 = 14.0;
float l2 = 20.0;

// ===== Pen angles =====
const int PEN_UP   = 35;
const int PEN_DOWN = 90;

// ===== Circle settings =====
const float RADIUS_CM = 2.5;
const float CIRCLE_STEP_DEG = 2.0;   // smaller = smoother circle (try 1.0 if needed)

// ===== DEFAULT HOME =====
const float HOME_X = -2.0;
const float HOME_Y = 25.0;

// ===== Store received centers =====
const int MAX_CENTERS = 80;
float centersX[MAX_CENTERS];
float centersY[MAX_CENTERS];
int centerCount = 0;

bool receiving = false;
bool startDrawing = false;
String lineBuf = "";

// ===== Motion smoothing =====
float curA1 = 90.0;                 // current servo1 angle (deg)
float curA2 = 90.0;                 // current servo2 angle (deg)
const int SMOOTH_STEPS = 10;        // higher = smoother but slower
const int STEP_DELAY_MS = 15;       // per smooth step

// Convert radians to degrees
float rad2deg(float r) { return r * 180.0 / PI; }

// Pen helpers
void penUp() {
  penServo.write(PEN_UP);
  delay(200);
}
void penDown() {
  penServo.write(PEN_DOWN);
  delay(200);
}

// Inverse Kinematics
bool inverseKinematics(float x, float y, float &theta1, float &theta2) {
  float dist = sqrt(x*x + y*y);
  if (dist > (l1 + l2) || dist < abs(l1 - l2)) return false;

  float cos_theta2 = (x*x + y*y - l1*l1 - l2*l2) / (2 * l1 * l2);
  cos_theta2 = constrain(cos_theta2, -1.0, 1.0);

  theta2 = acos(cos_theta2);

  float k1 = l1 + l2 * cos(theta2);
  float k2 = l2 * sin(theta2);
  theta1 = atan2(y, x) - atan2(k2, k1);

  return true;
}

// Move ONLY arm (no pen control inside)
bool moveToXY(float x, float y) {
  float theta1, theta2;
  if (!inverseKinematics(x, y, theta1, theta2)) return false;

  // keep float until write to reduce quantization artifacts
  float a1f = rad2deg(theta1);
  float a2f = rad2deg(theta2);

  a1f = constrain(a1f, 0.0, 180.0);
  a2f = constrain(a2f, 0.0, 180.0);

  // Smooth interpolation
  for (int i = 1; i <= SMOOTH_STEPS; i++) {
    float t = (float)i / (float)SMOOTH_STEPS;

    float a1 = curA1 + (a1f - curA1) * t;
    float a2 = curA2 + (a2f - curA2) * t;

    servo1.write((int)(a1 + 0.5));
    servo2.write((int)(180.0 - a2 + 0.5));
    delay(STEP_DELAY_MS);
  }

  curA1 = a1f;
  curA2 = a2f;
  return true;
}

// Go HOME safely
void goHome() {
  penUp();
  moveToXY(HOME_X, HOME_Y);
  delay(400);
}

// Draw a full circle (pen drops only AFTER reaching start point)
void drawCircle(float cx, float cy, float r) {
  // 1) find any reachable start point with pen UP
  bool found = false;
  float startDeg = 0.0;

  penUp();
  for (float a = 0; a < 360.0; a += CIRCLE_STEP_DEG) {
    float rad = a * PI / 180.0;
    if (moveToXY(cx + r*cos(rad), cy + r*sin(rad))) {
      startDeg = a;
      found = true;
      break;
    }
  }

  if (!found) {
    penUp();
    return;
  }

  // 2) now drop pen (we are already at the start point)
  delay(500);
  penDown();

  // 3) draw circle
  for (float a = startDeg + CIRCLE_STEP_DEG; a <= startDeg + 360.0; a += CIRCLE_STEP_DEG) {
    float rad = a * PI / 180.0;
    moveToXY(cx + r*cos(rad), cy + r*sin(rad));
  }

  // 4) lift pen after finishing
  penUp();
}

// Function to draw parallel lines
void drawParallelLines(float x, float y, float l, int n) {
  float lineSpacing = 2;  // Reduced spacing between lines (adjust for tighter lines)
  int delayTime = 1000;    // Delay time to make the drawing slower (adjust as needed)

  for (int i = 0; i < n; i++) {
    // Move the pen to the start of the line without drawing
    moveToXY(x, y + i * lineSpacing);  // Move to the start of each line at different y positions
    delay(100);
    penDown();  // Lower the pen before drawing
    moveToXY(x + l, y + i * lineSpacing);  // Draw the line of length 'l'
    penUp();  // Lift the pen after drawing
    delay(delayTime);  // Add delay to make the drawing slower
  }
}

// Serial protocol
void handleLine(String ln) {
  ln.trim();
  if (ln == "BEGIN") { centerCount = 0; receiving = true; return; }
  if (ln == "END")   { receiving = false; return; }
  if (ln == "START") { startDrawing = (centerCount > 0); return; }

  if (receiving && ln.startsWith("C,")) {
    int c1 = ln.indexOf(',');
    int c2 = ln.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > 0 && centerCount < MAX_CENTERS) {
      centersX[centerCount] = ln.substring(c1+1, c2).toFloat();
      centersY[centerCount] = ln.substring(c2+1).toFloat();
      centerCount++;
    }
  }

  if (ln.startsWith("CORRECT,")) {
  correctCount = ln.substring(8).toInt();
  return;
}
}

void setup() {
  servo1.attach(11);
  servo2.attach(6);
  penServo.attach(3);

  Serial.begin(9600);

  // Set initial pose guess (optional)
  servo1.write((int)(curA1 + 0.5));
  servo2.write((int)(180.0 - curA2 + 0.5));
  penUp();
  delay(300);

  // HOME ON BOOT
  goHome();

  Serial.println("READY");
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      handleLine(lineBuf);
      lineBuf = "";
    } else if (ch != '\r') {
      lineBuf += ch;
    }
  }

  if (startDrawing) {
    startDrawing = false;

    // HOME before starting drawing batch
    goHome();

    // Draw all circles
   for (int i = 0; i < centerCount; i++) {
     goHome();  // HOME between circles
     drawCircle(centersX[i], centersY[i], RADIUS_CM);  // Draw the circle
   }

    // After all circles are drawn, draw parallel lines
    drawParallelLines(-10, 10, 3, correctCount);  // Draw 5 parallel lines, each of length 50

    // Final HOME after drawing
    goHome();

    Serial.println("DONE");
    while (1);  // Stop the program after drawing is done
  }
}