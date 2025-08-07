#include <AFMotor.h>
#include <EEPROM.h>

/**
 * Motor Pin Setups
 */
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

/**
 * Initialize variables
 */
bool first_run = true;

// Take 3 measurements at each power/speed level
int number_readings = 3;
int speed_settings[5] = {80, 100, 120, 140, 160};

// Counters to track measurements and experiments
struct Trackers {
  int reset_num;
  int count_reading;
  int count_experiment;
  int experiment_number;
};
Trackers my_trackers;

// To detect new sketch programmed
const int MAGIC_ADDR = 4 * sizeof(int);
const int MAGIC_VALUE = 1048;

// Do not increment counters on first two resets after program new sketch
int run_on_reset = 2;

/**
 * Default Functions
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  int magic_val;
  EEPROM.get(MAGIC_ADDR, magic_val);

  if (magic_val != MAGIC_VALUE) {
    // New sketch just programmed
    EEPROM.put(0, my_trackers);
    EEPROM.put(MAGIC_ADDR, MAGIC_VALUE);
    Serial.println("new sketch");
  }
  else {
    // read values from persistent memory
    // first read will initialize counters to 0
    EEPROM.get(0, my_trackers);

    if (my_trackers.reset_num >= run_on_reset){
      my_trackers.count_reading++;
      if (my_trackers.count_reading > number_readings){
        // We have taken 3 measurements for a power level
        my_trackers.count_reading = 0;
        my_trackers.count_experiment++;
        
        if (my_trackers.count_experiment >= 5){
          // We have all the measurements for linear velocity
          my_trackers.count_experiment = 0;
          my_trackers.experiment_number++;
    
          if (my_trackers.experiment_number >= 2){
            // We have all the measurements for angular velocity
            // Reset experiment
            my_trackers.experiment_number = 0;
          }
        }
      }
    }
    else {
      my_trackers.reset_num++;
    }
  
    EEPROM.put(0, my_trackers); 
  }

  // Used to reset EEPROM magic addr
//  EEPROM.put(MAGIC_ADDR, 0);
//  EEPROM.get(MAGIC_ADDR, magic_val);
//  Serial.println("magic_val: " + String(magic_val));                                                                                                                                                                      

  Serial.println("my_trackers.reset_num: ");
  Serial.println(my_trackers.reset_num);
  Serial.println("my_trackers.count_reading: ");
  Serial.println(my_trackers.count_reading);
  Serial.println("my_trackers.count_experiment: ");
  Serial.println(my_trackers.count_experiment);
  Serial.println("my_trackers.experiment_number: ");
  Serial.println(my_trackers.experiment_number);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (first_run) {
    first_run = false;

    // Linear velocity experiments
    if (my_trackers.experiment_number == 0){
      int set_speed = speed_settings[my_trackers.count_experiment];
      // Drive forwards for 1 second
      forward(set_speed);                                                                                                                                                   
      delay(1000);  
      stop_motors();
      delay(100);
    }
    // Angular velocity experiments
    else if (my_trackers.experiment_number == 1){
      int set_speed = speed_settings[my_trackers.count_experiment];
      // Drive forwards for 1 second
      turn_left(set_speed);                                                                                                                                                   
      delay(1000);  
      stop_motors();
      delay(100);
    }
    else {
      // Do nothing
      turn_right(80);                                                                                                                                                   
      delay(1000);  
      stop_motors();
      delay(100);
    }
  }
}


/**
 * Custom Functions
 */
void forward(int speed) {
  motor1.setSpeed(speed);
  motor1.run(FORWARD);
  motor2.setSpeed(speed);
  motor2.run(FORWARD);
  motor3.setSpeed(speed);
  motor3.run(FORWARD);
  motor4.setSpeed(speed);
  motor4.run(FORWARD);
}

void turn_left(int speed) {
  motor3.setSpeed(speed);
  motor3.run(FORWARD);
  motor4.setSpeed(speed);
  motor4.run(FORWARD);
}

void turn_right(int speed) {
  motor1.setSpeed(speed);
  motor1.run(FORWARD);
  motor2.setSpeed(speed);
  motor2.run(FORWARD);
}

void stop_motors() {
  motor1.setSpeed(0);
  motor1.run(BRAKE);
  motor2.setSpeed(0);
  motor2.run(BRAKE);
  motor3.setSpeed(0);
  motor3.run(BRAKE);
  motor4.setSpeed(0);
  motor4.run(BRAKE);
}
