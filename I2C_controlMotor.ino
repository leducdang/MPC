#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>

Servo myServo;
// v = RPM × 0.00356(m/s)
float v_set = 0;
float v_cur = 0;

// tham số điều khiển động cơ
volatile long encoder_ticks = 0;
const int encoderA = 2;  // Interrupt pin
const int encoderB = 4;
const int pwm = 5;
double setpoint = 0.0;   // RPM mục tiêu
double input = 10;          // RPM thực tế
double output = 0;         // PWM output
double dt = 0;
float rpm;

// PID(Kp, Ki, Kd, Direction)
PID myPID(&input, &output, &setpoint, 20, 8, 0.5, DIRECT);
unsigned long last_time = 0;

// biến dữ liệu I2C
float f1 = 0.0;
float f2 = 0.0;
uint8_t buffer[9];
uint8_t index = 0, flag = 0;
// float sensorValue = 42.42;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(0x08);  // Địa chỉ I2C
  Wire.onReceive(receiveData);
  Wire.onRequest(sendFloat);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), readEncoder, RISING);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);

  myServo.attach(9); // Chân tín hiệu servo nối vào D5
  myServo.write(110);
}

unsigned long now = 0;
void loop() {
  // put your main code here, to run repeatedly:
  // myPID.Compute();
  
  if(flag)
  {
  // int pos = map(f2,-0.7,0.7,65,160);
  int pos = (int)( (f2 + 0.7) / (1.4) * (160 - 65) + 65 );
  myServo.write(pos);
  flag = 0;
  }

 now  = millis();
  if (now - last_time >= 100) { // tính mỗi 100ms
    // noInterrupts();
    long ticks = encoder_ticks;
    // long ticks = 1349;    
    encoder_ticks = 0;
    dt = now - last_time;
    // interrupts();

//     // Giả sử encoder 2000 xung/vòng
    rpm = (ticks / 1856.0) * 60.0 * (1000.0 / dt);   //HOẶC 1848
    input = (double)rpm;
    last_time = now;
    setpoint = f1*0.1/0.00356 + input;
    
//     // Tính PID
    myPID.Compute();
    analogWrite(pwm, (int)output);
  }
}
// unsigned long time_dt = millis()
void receiveData(int numBytes) {
  if (numBytes == 9) {
    for (int i = 0; i < 9; i++) {
      buffer[i] = Wire.read();
    }

    memcpy(&f1, &buffer[1], 4);
    memcpy(&f2, &buffer[5], 4);
    flag = 1;
  } else {
    // Đọc hết để tránh treo I2C nếu số byte sai
    while (Wire.available()) Wire.read();
    // Serial.println("Nhận sai số byte!");
  }
}

void sendFloat() {
  // Ép float thành byte array
  byte floatBytes[4];
  v_cur = input*0.00356; 
  memcpy(floatBytes, &v_cur, 4);
  Wire.write(floatBytes, 4);  // Gửi 4 byte
  // index = 0;
}

void readEncoder() {
  int b = digitalRead(encoderB);
  if (b == LOW)
    encoder_ticks--;
  else
    encoder_ticks++;
}




