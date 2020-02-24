#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <VL53L0X.h>
#include <Servo.h>

// OLED display I2C address
#define OLED_ADDR   0x3C
#define OLED_RESET  -1        // no reset pin

#define XSHUT_LTS 13
#define XSHUT_LBS 12
#define XSHUT_RBS 4
#define XSHUT_RTS 5

// TOF sensors I2C addresses
#define TOF_LT_ADDR    0x29      // this addr is set by default for each VL53L0X sensor
#define TOF_LB_ADDR    0x2A
#define TOF_RT_ADDR    0x2B
#define TOF_RB_ADDR    0x2C

#define SYSTEM_INTERRUPT_CLEAR       0x0B
#define RESULT_INTERRUPT_STATUS      0x13
#define RESULT_RANGE_STATUS          0x14

#define STEERING_CONTROL 9
#define SPEED_CONTROL    10

#define MAX_LEFT               115
#define MAX_RIGHT              72
#define NEUTRAL                90
#define MAX_REVERSE            0
#define REVERSE_SPEED          45
#define NO_TURN_DIFF           10
#define MINIMUM_SPEED          102   //102
#define REVERSE_CNT_LIMIT      7
#define SPEED_CNT_LIMIT        1
#define TIMER_LOAD_VALUE       10  //10 150

#define REVERSE_TURN_DIF       100
#define DECISION_DISTANCE      1300  // time to make a decision on how to go next 
#define CRITICAL_DISTANCE      200   // the car shall be stopped
#define OVERTAKE_DISTANCE      250
#define STOP_REVERSE_DISTANCE  400   // on this distance, forward-moving can be resumed 

// reset pin not used on 4-pin OLED module
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
//Adafruit_SSD1306 display(OLED_RESET);

// create vlx sensor objects
VL53L0X lt_sensor;
VL53L0X lb_sensor;
VL53L0X rt_sensor;
VL53L0X rb_sensor;

Servo ESC;       // create servo object to control the ESC
Servo STEERING;  // create servo object to control steering rack

enum StateMachine {
    DECISION_MAKING = 0,
    CORRECTION_NEEDED = 1,
    TURN_NEEDED = 2,  
    OBSTACLE_DETECTION = 3,
    REVERSE_MOVING = 4,
};


// utility function prototypes
void displayShowTofs(int val1, int val2, int val3, int val4);
void keep_moving(int current_state);

void setup() {
    // put all sensors under RESET
    pinMode(XSHUT_LTS, OUTPUT);
    pinMode(XSHUT_LBS, OUTPUT);
    pinMode(XSHUT_RBS, OUTPUT);

    // join I2C bus as a master
    Wire.begin();

    // enable VL53L0X one after the other and set their I2C address 
    rt_sensor.setAddress(TOF_RT_ADDR);
    pinMode(XSHUT_RBS, INPUT);
    delay(10);
    rb_sensor.setAddress(TOF_RB_ADDR);
    pinMode(XSHUT_LBS, INPUT);
    delay(10);
    lb_sensor.setAddress(TOF_LB_ADDR);
    pinMode(XSHUT_LTS, INPUT);
    delay(10);

    // initialize sensors
    lb_sensor.init();
    rb_sensor.init();
    rt_sensor.init();
    lt_sensor.init();

    // set timeout for sensors
    lb_sensor.setTimeout(1000);
    rb_sensor.setTimeout(1000);
    rt_sensor.setTimeout(1000);
    lt_sensor.setTimeout(1000);

    // start each sensor to work in continuous mode
    lb_sensor.startContinuous();
    rb_sensor.startContinuous();
    rt_sensor.startContinuous();
    lt_sensor.startContinuous(); 

    // attach the ESC on pin 10
    ESC.attach(SPEED_CONTROL, 600, 2250); // (pin, min pulse width, max pulse width in microseconds) 600, 2250
    delay(1500);

    // attach the steering servo on pin 9 and set it in neutral position
    STEERING.attach(STEERING_CONTROL);
    STEERING.write(NEUTRAL);
    delay(5000);

    // configure Timer2: CTC mode without using of OC2 pins and interrupts, prescaller is 1024
    TCCR2A |= 0x02;
    TCCR2B |= 0x07;
    TCNT2 = 0x00;
    OCR2A = TIMER_LOAD_VALUE; //0xF3;
    

    // initialize and clear display
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.clearDisplay();
    //display.setRotation(2);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.display();
}


int lts, lbs, rts, rbs;
int delta;
int turnAngle;
int speedCounter, reverseCounter;
int direct_sensors_ready, oblique_sensors_ready;
int current_state = DECISION_MAKING;
int double_click_needed = 0;
int trottle = NEUTRAL;
int reverse_timeout = 0;
void loop() {
    displayShowTofs(lts, lbs, rts, rbs); 
    
    if (0 != (lt_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
        lts = lt_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
        lt_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
        direct_sensors_ready |= 0x01;
    }

    if (0 != (rt_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
        rts = rt_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
        rt_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
        direct_sensors_ready |= 0x02;
    }

    if (0 != (lb_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
        lbs = lb_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
        lb_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
        oblique_sensors_ready = 0x01;
    }

    if (0 != (rb_sensor.readReg(RESULT_INTERRUPT_STATUS) & 0x07)) {
        rbs = rb_sensor.readReg16Bit(RESULT_RANGE_STATUS + 10);
        rb_sensor.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
        oblique_sensors_ready = 0x02;
    }

    // state machine
    switch(current_state) {
        case DECISION_MAKING:
            if (lts < DECISION_DISTANCE || rts < DECISION_DISTANCE) {
                if (lts < CRITICAL_DISTANCE || rts < CRITICAL_DISTANCE) {
                    if (lbs > OVERTAKE_DISTANCE && rbs > OVERTAKE_DISTANCE){
                        // probably met an obstacle - try to go around it
                        current_state = OBSTACLE_DETECTION;
                    } else {
                        // seems that met a wall - need to stop and move backward
                        current_state = REVERSE_MOVING;
                        double_click_needed = 1;
                    }
                } else {
                    // a turn or a correction is needed
                    current_state = TURN_NEEDED;
                }                
            } else {
                // turn steering into neutral state
                STEERING.write(NEUTRAL);
            }
            break;
        case  TURN_NEEDED:
            delta = (lbs - rbs) / 10 + NO_TURN_DIFF;
            if ((delta < 0) || (delta > NO_TURN_DIFF << 1)) {
                if (delta > 0) {
                    delta = delta - (NO_TURN_DIFF << 1);
                }
     
                turnAngle = NEUTRAL + delta;
                if (turnAngle > MAX_LEFT){
                    turnAngle = MAX_LEFT;
                } else if (turnAngle < MAX_RIGHT) {
                    turnAngle = MAX_RIGHT;
                }
                STEERING.write(turnAngle);       
            }

            current_state = DECISION_MAKING;
            break;
        case  OBSTACLE_DETECTION:   // 20-30 mm diff between top and bottom at the wall
            if (lbs >= rbs) {
                STEERING.write(MAX_LEFT);
            } else {
                STEERING.write(MAX_RIGHT);
            }

            current_state = DECISION_MAKING;
            break;   
        case  REVERSE_MOVING:
            if ((lts > STOP_REVERSE_DISTANCE && rts > STOP_REVERSE_DISTANCE) || reverse_timeout) {
                current_state = DECISION_MAKING;
                reverseCounter = 0;
                reverse_timeout = 0;
            } else {
                // turn steering in right direction - где меньше нижний датчик, туда и выворачиваем колеса
                delta = lbs - rbs;
                if (delta < 0) {
                    if (delta < -REVERSE_TURN_DIF) {
                        turnAngle = NEUTRAL - delta;
                    }
                } else {
                    if (delta > REVERSE_TURN_DIF){
                        turnAngle = NEUTRAL - delta;
                    }
                }

                if (turnAngle > MAX_LEFT){
                    turnAngle = MAX_LEFT;
                } else if (turnAngle < MAX_RIGHT) {
                    turnAngle = MAX_RIGHT;
                }
                STEERING.write(turnAngle);
                
                // move backward - need to decrease speed
                ESC.write(REVERSE_SPEED);                
            }
            break;    
    }

    keep_moving(current_state);
}


void keep_moving(int current_state) {
    if (REVERSE_MOVING == current_state) {
        // make preparation for "double click"
        if (double_click_needed){
            ESC.write(REVERSE_SPEED);
            delay(200);
            ESC.write(NEUTRAL);
            delay(200);
            double_click_needed = 0;
        }

        // control reverse time
        if (reverseCounter < REVERSE_CNT_LIMIT) {
            reverseCounter++;
        } else {
            reverse_timeout = 1;
        }
    } else {
        if (TIFR2 & 0x02) {
            if (NEUTRAL == trottle) {
                delay(2);
                trottle = MINIMUM_SPEED;   
            } else {
                if (speedCounter < SPEED_CNT_LIMIT) {
                    speedCounter++;
                    
                } else {
                    trottle = NEUTRAL;
                    speedCounter = 0;
                    
                }
            }


//        if (MINIMUM_SPEED == trottle) {
//                trottle = NEUTRAL;   
//            } else {
//                if (speedCounter < SPEED_CNT_LIMIT) {
//                    speedCounter++;
//                } else {
//                    trottle = MINIMUM_SPEED;
//                    speedCounter = 0;
//                }
//            }

      
            TIFR2 |= 0x02;  // clear bit
        }
        ESC.write(trottle);
    }
}

void displayShowTofs(int val1, int val2, int val3, int val4){
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(val1);

    display.setCursor(0, 32);
    display.print(val2);
    
    display.setCursor(64, 0);
    display.print(val3);
        
    display.setCursor(64, 32);
    display.print(val4);
        
    display.display();
}
