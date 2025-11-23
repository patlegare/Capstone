int joyXpin = A0;
int joyYpin = A1;
int buttonPin = 2;

int xVal;
int yVal;

// Motor driver pins
int ENApin = 4;
int IN1pin = 5;
int IN2pin = 6;
int IN3pin = 7;
int IN4pin = 8;
int ENBpin = 9;

// Toggle state: true = motors allowed to run, false = motors off
bool motorsEnabled = true;

// For button toggle (debounce)
bool lastButtonState = HIGH;      // because of INPUT_PULLUP
unsigned long lastToggleTime = 0;
const unsigned long DEBOUNCE_MS = 150; // adjust if needed


// Helper: stop both motors
void stopMotors() {
  analogWrite(ENApin, 0);
  analogWrite(ENBpin, 0);

  digitalWrite(IN1pin, LOW);
  digitalWrite(IN2pin, LOW);
  digitalWrite(IN3pin, LOW);
  digitalWrite(IN4pin, LOW);
}

void setup() {
  pinMode(joyXpin, INPUT);
  pinMode(joyYpin, INPUT);

  // Joystick button uses internal pull-up
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(ENApin, OUTPUT);
  pinMode(IN1pin, OUTPUT);
  pinMode(IN2pin, OUTPUT);
  pinMode(IN3pin, OUTPUT);
  pinMode(IN4pin, OUTPUT);
  pinMode(ENBpin, OUTPUT);

  stopMotors();
}

void loop() {
  //Read and process button as TOGGLE
  bool currentButtonState = digitalRead(buttonPin);  // HIGH = not pressed, LOW = pressed

  // Detect new press (HIGH -> LOW) with simple debounce
  if (currentButtonState != lastButtonState) {
    // state changed; check if it stayed changed for a bit
    if (currentButtonState == LOW && (millis() - lastToggleTime) > DEBOUNCE_MS) {
      motorsEnabled = !motorsEnabled;  // flip ON/OFF
      lastToggleTime = millis();
    }
    lastButtonState = currentButtonState;
  }

  // If motors are disabled, force them off and ignore joystick
  if (!motorsEnabled) {
    stopMotors();
    return;
  }

  // Normal joystick driving 
  xVal = analogRead(joyXpin);
  yVal = analogRead(joyYpin);

  // Joystick Up (forward)
  if (yVal < 485) {
    int motorSpeed = map(yVal, 485, 0, 70, 255);
    motorSpeed = constrain(motorSpeed, 0, 255);

    digitalWrite(IN1pin, HIGH);
    digitalWrite(IN2pin, LOW);
    digitalWrite(IN3pin, HIGH);
    digitalWrite(IN4pin, LOW);

    analogWrite(ENApin, motorSpeed);
    analogWrite(ENBpin, motorSpeed);
  }
  // Joystick Down (reverse)
  else if (yVal > 495) {
    int motorSpeed = map(yVal, 495, 1023, 70, 255);
    motorSpeed = constrain(motorSpeed, 0, 255);

    digitalWrite(IN1pin, LOW);
    digitalWrite(IN2pin, HIGH);
    digitalWrite(IN3pin, LOW);
    digitalWrite(IN4pin, HIGH);

    analogWrite(ENApin, motorSpeed);
    analogWrite(ENBpin, motorSpeed);
  }
  // Joystick Left (spin/turn left)
  else if (xVal < 505) {
    int motorSpeed = map(xVal, 505, 0, 70, 255);
    motorSpeed = constrain(motorSpeed, 0, 255);

    // left motor off, right forward
    digitalWrite(IN1pin, LOW);
    digitalWrite(IN2pin, LOW);
    digitalWrite(IN3pin, HIGH);
    digitalWrite(IN4pin, LOW);

    analogWrite(ENApin, 0);
    analogWrite(ENBpin, motorSpeed);
  }
  // Joystick Right (spin/turn right)
  else if (xVal > 515) {
    int motorSpeed = map(xVal, 515, 1023, 70, 255);
    motorSpeed = constrain(motorSpeed, 0, 255);

    // right motor off, left forward
    digitalWrite(IN1pin, HIGH);
    digitalWrite(IN2pin, LOW);
    digitalWrite(IN3pin, LOW);
    digitalWrite(IN4pin, LOW);

    analogWrite(ENApin, motorSpeed);
    analogWrite(ENBpin, 0);
  }
  // Joystick Deadzone (no movement)
  else {
    stopMotors();
  }
}

