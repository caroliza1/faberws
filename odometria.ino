#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "FastIMU.h"
#include <math.h>

// =========================
// CONFIGURACIÓN (DEBUG)
// =========================
#define DEBUG_MAG 1   // 1 -> imprime mag crudo y calibrado

// =========================
// CONFIGURACIÓN PCA9685
// =========================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO1 0
#define SERVO2 1

#define SERVOMIN1 150
#define SERVOMAX1 600
#define SERVOMIN2 310
#define SERVOMAX2 340

float alpha = 0.9;        // filtro exponencial para servos
float servo1_f = 0, servo2_f = 0;

// =========================
// CONFIGURACIÓN ESC
// =========================
#define ESC_PIN     14
#define ESC_CHANNEL 0

// <<< valores fijos del ESC
const int ESC_STOP  = 1000;   // motor parado
const int ESC_LENTO = 1995;   // <-- AJUSTA AQUÍ la velocidad lenta fija (1100, 1150, 1200...)
int escSignal = ESC_STOP;     // valor actual que se envía al ESC

unsigned long lastMotorTime = 0;
bool motorOn = false;

// =========================
// IMU (MPU9250 + FastIMU)
// =========================
#define IMU_ADDRESS 0x68

MPU9250 imu(Wire);
calData calib = {0};
AccelData a;
GyroData g;
MagData m;

// =========================
// TIEMPO
// =========================
unsigned long lastTime = 0;
float dt = 0.0f;

// =========================
// ORIENTACIÓN (rad)
// =========================
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;
bool  yawInitialized = false;

// =========================
// ODOMETRÍA (marco mundial)
// =========================
float ax_w = 0, ay_w = 0, az_w = 0; // aceleraciones lineales (m/s^2)
float vx   = 0, vy   = 0, vz   = 0; // velocidades (m/s)
float px   = 0, py   = 0, pz   = 0; // posiciones (m)

// detección de quietud
const float STILL_ACC_THRESHOLD  = 0.20f; // m/s^2
const float STILL_GYRO_THRESHOLD = 2.0f;  // deg/s aprox.
unsigned long stillStart = 0;

// =========================
// CALIBRACIÓN MANUAL MAG
// =========================
float mx_offset = 0.0f, my_offset = 0.0f, mz_offset = 0.0f;

// =========================
// ESCALA DEL ACELERÓMETRO
// =========================
float G_RAW     = 1.0f;      // valor medio de |a| cuando está quieto
const float G_SI = 9.81f;    // 1 g en m/s^2
float ACC_SCALE = 9.81f;     // factor para pasar de crudo a m/s^2

const float ACC_DEADBAND_G = 0.05f; // 0.05 g
float ACC_DEADBAND = 0.0f;         // en m/s^2, se calcula en setup

// =========================
// FUNCIONES AUXILIARES
// =========================
void writeESC(int us) {
  int duty = map(us, 0, 20000, 0, 65535);
  ledcWrite(ESC_CHANNEL, duty);
}

void resetOdometry() {
  vx = vy = vz = 0.0f;
  px = py = pz = 0.0f;
  Serial.println("🔄 Odometría reiniciada (pos y vel = 0).");
}

bool anyNaN() {
  return isnan(px) || isnan(py) || isnan(pz) ||
         isnan(vx) || isnan(vy) || isnan(vz) ||
         isnan(ax_w) || isnan(ay_w) || isnan(az_w) ||
         isnan(roll) || isnan(pitch) || isnan(yaw);
}

// Estima automáticamente si el acelerómetro está en g o en m/s^2
void estimateAccelScale() {
  Serial.println("\n[IMU] Estimando escala del acelerómetro...");
  Serial.println("     Deja la medusa QUIETA y HORIZONTAL durante ~1 segundo.");
  delay(1000);

  const int N = 200;
  float sumNorm = 0.0f;

  for (int i = 0; i < N; i++) {
    imu.update();
    imu.getAccel(&a);
    float ax = a.accelX;
    float ay = a.accelY;
    float az = a.accelZ;
    float n = sqrtf(ax * ax + ay * ay + az * az);
    sumNorm += n;
    delay(5);
  }

  G_RAW = sumNorm / N;
  if (G_RAW < 0.1f || isnan(G_RAW)) {
    G_RAW = 1.0f; // por si algo sale muy raro
  }

  ACC_SCALE   = G_SI / G_RAW;
  ACC_DEADBAND = ACC_DEADBAND_G * G_SI;

  Serial.print("     |a| quieto ≈ "); Serial.println(G_RAW, 4);
  Serial.print("     ACC_SCALE (m/s^2 por unidad) ≈ "); Serial.println(ACC_SCALE, 4);
  Serial.print("     Deadband acel. ≈ "); Serial.print(ACC_DEADBAND, 3); Serial.println(" m/s^2");
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(2000);

  // PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);

  // ESC
  ledcSetup(ESC_CHANNEL, 50, 16);
  ledcAttachPin(ESC_PIN, ESC_CHANNEL);
  writeESC(ESC_STOP);   // <<< señal mínima al iniciar
  delay(2000);

  // IMU
  Serial.println("\n=== MPU9250 + FastIMU + ODOMETRÍA ===");
  Wire.begin(21, 22);
  Wire.setClock(400000);

  calib.valid = false;   // usamos IMU sin calibración interna
  int err = imu.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("❌ Error iniciando MPU9250: ");
    Serial.println(err);
    while (1) delay(1000);
  }

  Serial.println("✔ MPU9250 detectado");
  Serial.println("   (sin calibrar con FastIMU; usamos datos crudos + offsets manuales)");

  // --- Calibración MANUAL MAG: min/max para obtener offset ---
  Serial.println("➡ MUEVE la medusa en TODAS las direcciones (ochos en el aire)...");
  Serial.println("   Manténla moviéndose ~15 segundos para calibrar el magnetómetro.");
  delay(1500);

  float mx_min =  1e9, mx_max = -1e9;
  float my_min =  1e9, my_max = -1e9;
  float mz_min =  1e9, mz_max = -1e9;

  unsigned long t0 = millis();
  while (millis() - t0 < 15000) {  // ~15 s de movimiento
    imu.update();
    imu.getMag(&m);

    float mx = m.magX;
    float my = m.magY;
    float mz = m.magZ;

    if (!isnan(mx) && !isnan(my) && !isnan(mz)) {
      if (mx < mx_min) mx_min = mx;
      if (mx > mx_max) mx_max = mx;
      if (my < my_min) my_min = my;
      if (my > my_max) my_max = my;
      if (mz < mz_min) mz_min = mz;
      if (mz > mz_max) mz_max = mz;
    }

    delay(20);
  }

  mx_offset = (mx_min + mx_max) * 0.5f;
  my_offset = (my_min + my_max) * 0.5f;
  mz_offset = (mz_min + mz_max) * 0.5f;

  Serial.println("✔ Magnetómetro calibrado (offsets por min/max):");
  Serial.print("   mx_offset = "); Serial.println(mx_offset);
  Serial.print("   my_offset = "); Serial.println(my_offset);
  Serial.print("   mz_offset = "); Serial.println(mz_offset);

  // --- Estimar escala de acelerómetro ---
  estimateAccelScale();

  // Odometría a cero
  resetOdometry();
  lastTime = micros();

  Serial.println("Control de servos listo.");
  Serial.println("Comandos por monitor serie:");
  Serial.println("  1,angulo   -> Servo1 en grados (0–180)");
  Serial.println("  2,angulo   -> Servo2 en grados (0–180)");
  Serial.println("  pwm,valor  -> PWM motor (900–2000 us)");
  Serial.println("  zero       -> Reset odometría (pos y vel a 0)");
}

// =========================
// LOOP PRINCIPAL
// =========================
void loop() {
  unsigned long nowMs = millis();

  // --------------------------------
  // Control automático del motor
  // --------------------------------
  if (!motorOn && nowMs - lastMotorTime >= 20000) {
    motorOn = true;
    lastMotorTime = nowMs;
    Serial.println("⚡ Motor ENCENDIDO (10s)");
    escSignal = ESC_LENTO;        // <<< velocidad lenta fija
    writeESC(escSignal);
  }
  if (motorOn && nowMs - lastMotorTime >= 10000) {
    motorOn = false;
    lastMotorTime = nowMs;
    Serial.println("⛔ Motor APAGADO (20s)");
    escSignal = ESC_STOP;         // <<< motor parado
    writeESC(escSignal);
  }

  // --------------------------------
  // Δt en segundos
  // --------------------------------
  unsigned long nowMicros = micros();
  dt = (nowMicros - lastTime) / 1e6;
  lastTime = nowMicros;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;  // evita locuras

  // --------------------------------
  // Lectura IMU
  // --------------------------------
  imu.update();
  imu.getAccel(&a);
  imu.getGyro(&g);
  imu.getMag(&m);

  // magnetómetro calibrado (quitamos offset)
  float mx = m.magX - mx_offset;
  float my = m.magY - my_offset;
  float mz = m.magZ - mz_offset;

#if DEBUG_MAG
  Serial.print("MAG crudo: ");
  Serial.print(m.magX); Serial.print(", ");
  Serial.print(m.magY); Serial.print(", ");
  Serial.print(m.magZ);
  Serial.print("  | MAG cal: ");
  Serial.print(mx); Serial.print(", ");
  Serial.print(my); Serial.print(", ");
  Serial.println(mz);
#endif

  // Aceleraciones crudas
  float ax_raw = a.accelX;
  float ay_raw = a.accelY;
  float az_raw = a.accelZ;

  // Pasar a m/s^2 usando la escala estimada
  float ax = ax_raw * ACC_SCALE;
  float ay = ay_raw * ACC_SCALE;
  float az = az_raw * ACC_SCALE;

  // --------------------------------
  // Orientación roll/pitch a partir del acelerómetro
  // --------------------------------
  pitch = atan2(ay, az);
  roll  = atan2(-ax, sqrt(ay * ay + az * az));

  // --------------------------------
  // Yaw con magnetómetro (con compensación de inclinación)
  // --------------------------------
  float Bnorm = sqrt(mx * mx + my * my + mz * mz);

  // Declinación magnética (ejemplo Bogotá)
  float declination = -4.0 * PI / 180.0;

  if (Bnorm > 1e-6 && !isnan(Bnorm)) {
    float mx2 = mx * cos(pitch) + mz * sin(pitch);
    float my2 = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    float yawMag = atan2(my2, mx2) + declination;

    if (yawMag > PI)  yawMag -= 2 * PI;
    if (yawMag < -PI) yawMag += 2 * PI;

    if (!isnan(yawMag)) {
      yaw = yawMag;
      yawInitialized = true;
    }
  } else {
    if (!yawInitialized) yaw = 0.0f;
  }

  if (isnan(yaw)) {
    yaw = 0.0f;
    yawInitialized = false;
  }

  // --------------------------------
  // Matriz de rotación (body -> world)
  // --------------------------------
  float cr = cos(roll),  sr = sin(roll);
  float cp = cos(pitch), sp = sin(pitch);
  float cy = cos(yaw),   sy = sin(yaw);

  float R11 = cp * cy;
  float R12 = cp * sy;
  float R13 = -sp;

  float R21 = sr * sp * cy - cr * sy;
  float R22 = sr * sp * sy + cr * cy;
  float R23 = sr * cp;

  float R31 = cr * sp * cy + sr * sy;
  float R32 = cr * sp * sy - sr * cy;
  float R33 = cr * cp;

  // --------------------------------
  // Aceleraciones en marco mundial
  // --------------------------------
  float ax_world = R11 * ax + R12 * ay + R13 * az;
  float ay_world = R21 * ax + R22 * ay + R23 * az;
  float az_world = R31 * ax + R32 * ay + R33 * az;

  // Quitar gravedad en Z mundial
  const float G = 9.81f;
  az_world -= G;

  // Pequeño deadband para no integrar ruido
  if (fabs(ax_world) < ACC_DEADBAND) ax_world = 0.0f;
  if (fabs(ay_world) < ACC_DEADBAND) ay_world = 0.0f;
  if (fabs(az_world) < ACC_DEADBAND) az_world = 0.0f;

  ax_w = ax_world;
  ay_w = ay_world;
  az_w = az_world;

  // --------------------------------
  // Detección de quietud
  // --------------------------------
  bool accStill =
    fabs(ax_w) < STILL_ACC_THRESHOLD &&
    fabs(ay_w) < STILL_ACC_THRESHOLD &&
    fabs(az_w) < STILL_ACC_THRESHOLD;

  bool gyroStill =
    fabs(g.gyroX) < STILL_GYRO_THRESHOLD &&
    fabs(g.gyroY) < STILL_GYRO_THRESHOLD &&
    fabs(g.gyroZ) < STILL_GYRO_THRESHOLD;

  bool isStill = accStill && gyroStill;

  if (isStill) {
    if (stillStart == 0) stillStart = nowMs;
  } else {
    stillStart = 0;
  }

  // --------------------------------
  // Integración para velocidad y posición
  // --------------------------------
  vx += ax_w * dt;
  vy += ay_w * dt;
  vz += az_w * dt;

  // Si lleva un rato quieto, forzamos velocidades a 0 (baja deriva)
  if (isStill && (nowMs - stillStart > 500)) { // >0.5s quieto
    vx = vy = vz = 0.0f;
  }

  px += vx * dt;
  py += vy * dt;
  pz += vz * dt;

  if (anyNaN()) {
    Serial.println("⚠ Se detectó NaN, reseteando odometría y yaw.");
    yaw = 0.0f;
    yawInitialized = false;
    resetOdometry();
  }

  // --------------------------------
  // Control automático servos (nivelado)
  // --------------------------------
  int pulse1 = map(pitch * 180 / PI, -90, 90, SERVOMIN1, SERVOMAX1);
  int pulse2 = map(roll  * 180 / PI, -90, 90, SERVOMIN2, SERVOMAX2);
  pulse1 = constrain(pulse1, SERVOMIN1, SERVOMAX1);
  pulse2 = constrain(pulse2, SERVOMIN2, SERVOMAX2);

  servo1_f = alpha * servo1_f + (1 - alpha) * pulse1;
  servo2_f = alpha * servo2_f + (1 - alpha) * pulse2;

  pwm.setPWM(SERVO1, 0, (int)servo1_f);
  pwm.setPWM(SERVO2, 0, (int)servo2_f);

  // --------------------------------
  // Control manual (servos / motor / reset odometría)
  // (por si alguna vez sí tienes acceso al monitor serie)
  // --------------------------------
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int commaIndex = input.indexOf(',');

    if (commaIndex == -1) {
      if (input == "zero" || input == "reset") {
        resetOdometry();
      }
    } else {
      String cmd   = input.substring(0, commaIndex);
      int value    = input.substring(commaIndex + 1).toInt();

      if (cmd == "1") {
        value = constrain(value, 0, 180);
        pwm.setPWM(SERVO1, 0, map(value, 0, 180, SERVOMIN1, SERVOMAX1));
      } else if (cmd == "2") {
        value = constrain(value, 0, 180);
        pwm.setPWM(SERVO2, 0, map(value, 0, 180, SERVOMIN2, SERVOMAX2));
      } else if (cmd == "pwm") {
        value = constrain(value, 900, 2000);
        escSignal = value;
        writeESC(escSignal);
      }
    }
  }

  // --------------------------------
  // Mostrar resultados
  // --------------------------------
  Serial.print("ROLL: ");  Serial.print(roll * 180 / PI, 2);
  Serial.print(" | PITCH: "); Serial.print(pitch * 180 / PI, 2);
  Serial.print(" | YAW: ");   Serial.print(yaw * 180 / PI, 2);

  Serial.print(" | POS (m): X="); Serial.print(px, 3);
  Serial.print(" Y=");             Serial.print(py, 3);
  Serial.print(" Z=");             Serial.print(pz, 3);

  Serial.print(" | VEL (m/s): VX="); Serial.print(vx, 3);
  Serial.print(" VY=");             Serial.print(vy, 3);
  Serial.print(" VZ=");             Serial.print(vz, 3);

  Serial.print(" | PWM Motor: "); Serial.println(escSignal);

  delay(50);
}