#include "FastAccelStepper.h"

#define ACCEL 1500  // higher is faster
#define SPEED 200  // lower is faster


void pinmode_PullUP(int pin) {
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

bool readBtn(int pin){
  return ! digitalRead(pin);
  
}

class blinds {
  private:
    int32_t bot = 0;
    int32_t top = 100;
    int btn_up_pin;
    int btn_down_pin;
    int btn_set_stop_pin;
    FastAccelStepper *stepper;
    char *name;
    int prev_btn_dir = 0;
    unsigned long prev_btn_change_t;
  
  public:
    int32_t pos;
    int32_t dst;
    
    blinds(int btn_up_pin, int btn_down_pin, int btn_set_stop_pin, char *name);
    void begin(FastAccelStepper *stepper);
    
    void poll();  
};

blinds::blinds(int btn_up_pin, int btn_down_pin, int btn_set_stop_pin, char *name){
  this->btn_up_pin = btn_up_pin;
  this->btn_down_pin = btn_down_pin;
  this->btn_set_stop_pin = btn_set_stop_pin;  
  this->name = name;
  
}

void blinds::begin(FastAccelStepper *stepper) {
  pinmode_PullUP(btn_up_pin);
  pinmode_PullUP(btn_down_pin);
  pinmode_PullUP(btn_set_stop_pin);
  this->stepper = stepper;
  Serial.println((uint16_t) stepper);
 
}





void blinds::poll() {
  if (readBtn(btn_set_stop_pin)) {
    stepper->stopMove();
    
    if (readBtn(btn_up_pin)) {
      top = stepper->getCurrentPosition();
      Serial.print("New TOP pos: ");
      Serial.println(top);
      delay(100);
    }
    
    if (readBtn(btn_down_pin)) {
      bot = stepper->getCurrentPosition();
      Serial.print("New BOT pos: ");
      Serial.println(bot);
      delay(100);
    }

    return;
  }
  
  int btn_dir = 0;
  if (readBtn(btn_up_pin))    btn_dir = +1;
  if (readBtn(btn_down_pin))  btn_dir = -1;

  if (btn_dir != prev_btn_dir) {
    Serial.print(name);
    Serial.print(" btn: ");
    Serial.println(btn_dir);
    switch (btn_dir){
      case 0:
        if (millis() - prev_btn_change_t < 500) { // short press (fast release) - runs to end
          dst = prev_btn_dir > 0 ? top : bot;
          stepper->moveTo(dst);
          Serial.print("Move to: ");
          Serial.println(dst);
        }
        else {
          stepper->stopMove();    
          Serial.println("STOP");
          
        }
      break;
      
      case -1:
        stepper->runBackward();
        Serial.println("DOWN");
      break;
      
      case +1:
        stepper->runForward();
        Serial.println("UP");
      break;
    }
    
    prev_btn_dir = btn_dir;
    prev_btn_change_t = millis();
    delay(50);
  }
    
}

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *left_stepper = NULL;
FastAccelStepper *right_stepper = NULL;

blinds left( A0, A1, A4, "left");
blinds right(A2, A3, A4, "right");



void setup() {
  Serial.begin(115200);
  Serial.println("initializing...");
  engine.init();
  
  left_stepper = engine.stepperConnectToPin(9);
  if (left_stepper) {
    left_stepper->setDirectionPin(5);
    left_stepper->setEnablePin(6);
    left_stepper->setAutoEnable(true);

    left_stepper->setSpeedInUs(SPEED);
    left_stepper->setAcceleration(ACCEL);
    Serial.println("Left motor is ready");
  }
  left.begin(left_stepper);
  
  right_stepper = engine.stepperConnectToPin(10);
  if (right_stepper) {
    right_stepper->setDirectionPin(8);
    right_stepper->setEnablePin(7);
    right_stepper->setAutoEnable(true);

    right_stepper->setSpeedInUs(SPEED);
    right_stepper->setAcceleration(ACCEL);
    Serial.println("Right motor is ready");
  }
  right.begin(right_stepper);
}

void loop() {
  left.poll();
  right.poll();
}
