#include <Arduino.h>
#include <FastAccelStepper.h>
#include <M5Unified.h>
#include <Module_Stepmotor.h>

// Module_Stepmotor - M5Core2 pin configuration
#define X_DIR_PIN G14
#define X_STEP_PIN G13
#define Y_DIR_PIN G19
#define Y_STEP_PIN G27
#define Z_DIR_PIN G0
#define Z_STEP_PIN G2

#define FULL_STEP_PER_REV 200
#define MICRO_STEPS 32

const uint32_t speedInHz = 2000 * MICRO_STEPS;   // steps/s
const int32_t acceleration = 500 * MICRO_STEPS;  // steps/s^2

const struct {
  uint8_t stepPin;
  uint8_t dirPin;
} StepperPins[3] = {
    {X_STEP_PIN, X_DIR_PIN},
    {Y_STEP_PIN, Y_DIR_PIN},
    {Z_STEP_PIN, Z_DIR_PIN},
};

// Ratio of speedInHz to target/current speed (-1.0f~1.0f)
struct SpeedRatio {
  float target;
  float current;
} speeds[3] = {
    {0.0f, 0.0f},  // X axis
    {0.0f, 0.0f},  // Y axis
    {0.0f, 0.0f},  // Z axis
};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *steppers[3] = {nullptr, nullptr, nullptr};

static Module_Stepmotor driver;

bool motorEnabled = false;  // flag to check if the motor is enabled

void drawMeter(const SpeedRatio *speeds);

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);

  engine.init();
  steppers[0] = engine.stepperConnectToPin(StepperPins[0].stepPin);  // X axis
  steppers[1] = engine.stepperConnectToPin(StepperPins[1].stepPin);  // Y axis
  steppers[2] = engine.stepperConnectToPin(StepperPins[2].stepPin);  // Z axis

  for (FastAccelStepper *&stepper : steppers) {
    size_t i = stepper - steppers[0];
    Serial.printf("stepper[%d]: %p\n", i, stepper);
    if (stepper != nullptr) {
      stepper->setDirectionPin(StepperPins[i].dirPin);
      stepper->setAutoEnable(true);
      stepper->setSpeedInHz(0);
      stepper->setAcceleration(acceleration);
    }
  }

  // Initialize Wire and driver (G21: SDA, G22: SCL)
  Wire.begin(G21, G22, 400000UL);
  driver.init(Wire);

  // Reset motor drivers
  driver.resetMotor(0, 0);
  driver.resetMotor(1, 0);
  driver.resetMotor(2, 0);

  // Disable power supply to motors
  driver.enableMotor(0);
}

void loop() {
  M5.update();

  // Check the scrolling of the touch panel and set the target speed
  if (M5.Touch.isEnabled()) {
    auto touch = M5.Touch.getDetail();
    if (touch.isPressed()) {
      if ((0 <= touch.y) && (touch.y < M5.Display.height() / 3)) {
        speeds[0].target += float(touch.deltaX()) / 250;
        speeds[0].target = constrain(speeds[0].target, -1.0f, 1.0f);
      } else if ((M5.Display.height() / 3 <= touch.y) && (touch.y < M5.Display.height() * 2 / 3)) {
        speeds[1].target += float(touch.deltaX()) / 250;
        speeds[1].target = constrain(speeds[1].target, -1.0f, 1.0f);
      } else if ((M5.Display.height() * 2 / 3 <= touch.y) && (touch.y < M5.Display.height())) {
        speeds[2].target += float(touch.deltaX()) / 250;
        speeds[2].target = constrain(speeds[2].target, -1.0f, 1.0f);
      }
    }
  }

  // Button events (click) to set target speed
  if (M5.BtnB.wasClicked() || !motorEnabled) {
    for (SpeedRatio &speed : speeds) {
      speed.target = 0.0f;  // set speed ratio to 0.0f
    }
  } else if (M5.BtnA.wasClicked()) {
    for (SpeedRatio &speed : speeds) {
      speed.target = -1.0f;  // set speed ratio to -1.0f
    }
  } else if (M5.BtnC.wasClicked()) {
    for (SpeedRatio &speed : speeds) {
      speed.target = 1.0f;  // set speed ratio to 1.0f
    }
  }

  // Button event (hold) to alternate motor enable/disable
  if (M5.BtnB.wasHold()) {
    if (motorEnabled) {
      for (FastAccelStepper *&stepper : steppers) {
        stepper->stopMove();  // stop all motors
      }
      driver.enableMotor(0);  // disable motor
      motorEnabled = false;
    } else {
      driver.enableMotor(1);  // enable motor
      motorEnabled = true;
    }
  }

  // Update stepper motors based on target speeds
  for (FastAccelStepper *&stepper : steppers) {
    size_t i = stepper - steppers[0];
    stepper->setSpeedInHz(abs(speeds[i].target) * speedInHz);

    if (speeds[i].target > 0) {
      stepper->runForward();
    } else if (speeds[i].target < 0) {
      stepper->runBackward();
    } else {
      stepper->stopMove();
    }

    // Update current speed
    speeds[i].current = (float(stepper->getCurrentSpeedInMilliHz(true)) / speedInHz) / 1000;
  }

  // Draw the speed meter on the display
  drawMeter(speeds);
}

void drawMeter(const SpeedRatio *speeds) {
  // Declare sprites
  static LGFX_Sprite sprite[3] = {LGFX_Sprite(&M5.Display), LGFX_Sprite(&M5.Display), LGFX_Sprite(&M5.Display)};

  // Create, initialize, draw and push sprite
  for (LGFX_Sprite &s : sprite) {
    s.createSprite(M5.Display.width(), M5.Display.height() / 3);
    s.setFont(&fonts::FreeMonoBold9pt7b);

    size_t i = &s - &sprite[0];                                                          // index of sprite
    float target = speeds[i].target;                                                     // target speed ratio
    float current = speeds[i].current;                                                   // current speed ratio
    float current_rps = (current * float(speedInHz) / MICRO_STEPS) / FULL_STEP_PER_REV;  // current speed in revolutions per second
    float current_rpm = current_rps * 60.0f;                                             // current speed in revolutions per minute
    float target_rps = (target * float(speedInHz) / MICRO_STEPS) / FULL_STEP_PER_REV;    // target speed in revolutions per second
    float target_rpm = target_rps * 60.0f;                                               // target speed in revolutions per minute

    // Center line
    s.drawFastVLine(s.width() / 2, 0, s.height(), TFT_DARKGRAY);

    char str[16];  // buffer for string

    // Print motor info
    snprintf(str, sizeof(str), " Motor %c ", i == 0 ? 'X' : (i == 1 ? 'Y' : 'Z'));
    s.setTextDatum(top_left);
    s.setTextColor(TFT_WHITE, TFT_BLACK);
    s.drawString(str, 0, 0);
    snprintf(str, sizeof(str), " %s ", motorEnabled ? "ENABLE" : "DISABLE");
    s.setTextDatum(top_right);
    s.setTextColor(motorEnabled ? TFT_GREEN : TFT_RED, TFT_BLACK);
    s.drawString(str, s.width(), 0);

    // Target speed line and text
    snprintf(str, sizeof(str), " %5.1f rpm ", target_rpm);
    s.setTextDatum(top_center);
    if (target < 0) {
      float absTarget = -target;
      s.fillRect(s.width() / 2 - 150 * absTarget, s.height() / 3, 150 * absTarget + 1, s.height() / 3 + 1, TFT_RED);
      s.setTextColor(TFT_RED, TFT_BLACK);
      s.drawString(str, s.width() / 2, s.height() * 2 / 3);
    } else {
      s.fillRect(s.width() / 2 + 1, s.height() / 3, 150 * target, s.height() / 3 + 1, TFT_BLUE);
      s.setTextColor(TFT_BLUE, TFT_BLACK);
      s.drawString(str, s.width() / 2, s.height() * 2 / 3);
    }

    // Current speed line
    s.drawLine(s.width() / 2 + 150 * current, s.height() / 3, s.width() / 2 + 150 * current, s.height() * 2 / 3 - 1, TFT_GREEN);

    // Current speed text
    s.setTextDatum(bottom_center);
    s.setTextColor(TFT_GREEN, TFT_BLACK);
    snprintf(str, sizeof(str), " %5.1f rpm ", current_rpm);
    s.drawString(str, s.width() / 2, s.height() / 3);

    // Push sprite to display
    s.pushSprite(0, i * (M5.Display.height() / 3));
  }
}