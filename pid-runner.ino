// Define motor control pins
#define rightMotorF 27      // Right motor forward pin
#define rightMotorB 14      // Right motor backward pin
#define rightMotorPWM 12    // Right motor PWM pin (ENA)
#define leftMotorF 25      // Left motor forward pin
#define leftMotorB 33      // Left motor backward pin
#define leftMotorPWM 32    // Left motor PWM pin (ENB)
#define stby 26

const int numSensors = 8;
int irSensors[numSensors] = {19,18,5,17,16,4,2,15};

// PID parameters
float kp = 25;    // Proportional gain 25
float ki = 1;     // Integral gain 1
float kd = 20;    // Derivative gain 20

int baseSpeed = 200; // Base speed for motors (0-255)

// Threshold for sensor detection (tune based on your environment)
// const int threshold = 500; // For analog sensors

// PID variables
long integral = 0;
int previousError = 0;

void setup() {
  // Initialize motor pins as outputs
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby, OUTPUT);

  
  for (int i = 0; i < numSensors; i++) {
    pinMode(irSensors[i], INPUT);
  }
  digitalWrite(stby, HIGH);

  Serial.begin(9600);
}

void loop() {
  // Read sensor values
  int sensorStates[numSensors];
  for (int i = 0; i < numSensors; i++) {
    // If using digital sensors:
    sensorStates[i] = digitalRead(irSensors[i]);
    // If using analog sensors, uncomment the following lines and comment out the above line:
    // int sensorValue = analogRead(irSensors[i]);
    // sensorStates[i] = (sensorValue > threshold) ? HIGH : LOW;
  }

  int error = calculateError(sensorStates);

  int motorSpeedDifference = calculatePID(error);

  int leftSpeed = baseSpeed + motorSpeedDifference;
  int rightSpeed = baseSpeed - motorSpeedDifference;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  driveMotors(leftSpeed, rightSpeed);

  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | PID: ");
  Serial.print(motorSpeedDifference);
  Serial.print(" | Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);

  delay(1); 
}

int calculateError(int sensorStates[]) {
  int weights[numSensors] = {-1000, -1000, -500, 0, 0, 500, 1000, 1000};
  
  long weightedSum = 0;
  int activeSensors = 0;

  for (int i = 0; i < numSensors; i++) {
    if (sensorStates[i] == HIGH) { 
      weightedSum += weights[i];
      activeSensors++;
    }
  }

  if (activeSensors == 0) {
    return previousError;
  }

  int error = weightedSum / activeSensors;

  return error;
}

int calculatePID(int error) {

  integral += error;

  integral = constrain(integral, -1000, 1000); // Adjust limits as necessary

  int derivative = error - previousError;

  float PID = (kp * error) + (ki * integral) + (kd * derivative);
  
  previousError = error;

  return (int) PID;
}

void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, leftSpeed);
  } else {
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, -leftSpeed);
  }

  // Right Motor
  if (rightSpeed > 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, rightSpeed);
  } else {
    digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, HIGH);
    analogWrite(rightMotorPWM, -rightSpeed);
  }
}
