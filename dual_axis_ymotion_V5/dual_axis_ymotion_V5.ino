/*==============================================================
DUAL AXIS ELECTROSPINNING RIG
This code operates two stepper motors to actuate spindle rotation and y axis movement.
switches 2 heating element outputs to control temperature.

INPUT values

Spindle rpm
Spindle Diameter
Spindle surface speed (rotation)
Spindle Surface speed (Y axis - lateral motion)
? Spindle surface distance from needle
Enclosure temp
enclosure humidity
spindle temperature

OUTPUT values

Spindle Stepper speed
Spindle Stepper direction
Y- axis Stepper speed
Y- axis Stepper direction

Humidity control fan
Spindle Heater
Enclosure heater
==============================================================*/
#include <TimerOne.h>
#include <TimerThree.h>

/*==============================================================
Definitions
==============================================================*/
//
#define EG_SPINDLE 1.0 
#define EG_Y 1.0
//OUTPUTS
#define S_STEP 13
#define S_DIR 12
#define ENABLE 11

#define Y_STEP 10 
#define Y_DIR 9
#define Y_LIM 2

#define X_LIM 3
#define X_STEP 7
#define X_DIR 6

#define EN_HEAT 8
#define S_HEAT

#define DHT 5
#define DS18B20 4

//INPUTS
#define spindle_steps_rev 200
#define Y_steps_rev 200
#define y_accel 50
#define s_accel 50\

#define spindle_microsteps = 0; //set to the microstepping value on your stepper motor drivers (for gecko drivers this has to be calculated by measuring your system and using ratios)
#define y_microsteps = 0;
#define x_microsteps = 0;
/*==============================================================
//Variables
==============================================================*/
boolean Y_LIMIT = false;
boolean Y_FREEZE = false;
boolean y_move = false;

long debouncing_time = 30; //Debouncing Time in Milliseconds
long y_step_count=0;// tracking y axis movement actual step count will be half this number rounded down
volatile unsigned long last_micros;


float current_y_pos=0;

boolean current_y_dir=false;
float current_y_sps=100.0; // current spindle speed in stepper motor steps per second (sps)
unsigned long target_y_sps=0; //global vairbale for storage of target spindle speed prior to updating the speed.
int y_step_period=1000000/current_y_sps;
float y_micro_rat = 669.28; //this value incorporates the microstepping and the mechanical gear ratio into one number. obtain it by setting it to a random value and measuring the resulting motion then multiplying this value by the ration between input speed and output speed.

float current_spindle_sps=100.0; // current spindle speed in stepper motor steps per second (sps)
unsigned long target_spindle_sps=0; //global vairbale for storage of target spindle speed prior to updating the speed.
int spindle_step_period=1000000/current_spindle_sps;
float spindle_micro_rat = 669.28; //this value incorporates the microstepping and the mechanical gear ratio into one number. obtain it by setting it to a random value and measuring the resulting motion then multiplying this value by the ration between input speed and output speed.
/*==============================================================
//SETUP
==============================================================*/

void setup() {
  for (int pin = 6; pin<14; pin++){
    pinMode(pin, OUTPUT);
  }
  pinMode(Y_LIM, INPUT);
  pinMode(X_LIM, INPUT);
  
  //attachInterrupt(digitalPinToInterrupt(Y_LIM), y_axis_limit_triggered, FALLING);
  
  Serial.begin(115200);
  Serial.println("READY");
}
/*==============================================================
// MAIN
==============================================================*/

void loop() {

  if(Serial.available()){
    receive_command();
  }

}

/*==============================================================
FUNCTIONS
==============================================================*/
void receive_command(){
  String command = Serial.readString();

   if (command.substring(0,command.indexOf(":")) == "SS") { //(SS = spindle speed) check if the characters from start of string until ":" 
      int spindle_rpm = command.substring(command.indexOf(":") + 1).toInt(); //capture the characters after the ":" and convert to an integer
      Serial.print("SPINDLE_RPM:   ");
      Serial.println(spindle_rpm);
      
      target_spindle_sps = spindle_rps(0, spindle_rpm, 0); // calculate spindle steps per second required to attain the rpm.
      Serial.print("SPINDLE_SPS:    ");
      Serial.println(target_spindle_sps);
      
    } 
   if (command.substring(0,command.indexOf(":")) == "YS") { //YS = Y_START (start position of y axis)
      float zero_pos = command.substring(command.indexOf(":") + 1).toFloat();
      Serial.print("ZERO_POS:   ");
      Serial.println(zero_pos);
    } 
    if (command.substring(0,command.indexOf(":")) == "YE") {// YE = Y_END (end position of y axis)
      float max_pos = command.substring(command.indexOf(":") + 1).toFloat();
      Serial.print("MAX_POS:   ");
      Serial.println(max_pos);
    } 
     if (command.substring(0,command.indexOf(":")) == "YV") {//YV = Y_VELOCITY (speed y axis moves between start and end positions
      float y_speed = command.substring(command.indexOf(":") + 1).toFloat();
      Serial.print("Y_SPEED:   ");
      Serial.println(y_speed);
    } 
     if (command.substring(0,command.indexOf(":")) == "START") { //enable stepper motors and begin motion
      //START
      Serial.println("STARTING SYSTEM");
      //enable steppers
      enable_steppers();
      //home Y_axis and move to start position

      //start spindle rotation
      init_timers();
      update_spindle(target_spindle_sps);
    } 
     if (command.substring(0,command.indexOf(":")) == "STOP") { //enable stepper motors and begin motion
      //START
      Serial.println("STOPPING SYSTEM");
      //enable steppers
      //home Y_axis and move to start position
      update_spindle(0);
    }
  
}
//==============================================================
int y_rps(float surf_speed, float diameter){
  
  float dist_per_rev = 2*PI*(diameter/2); 
  float rps = surf_speed/dist_per_rev;
  int y_steps_sec = int(rps * Y_steps_rev);
  
  return(y_steps_sec);
}
//==============================================================
  //change spindle step frequency here
//==============================================================
void update_spindle(float spindle_sps){

    
    while(spindle_sps > current_spindle_sps){// accelerate to target speed
      
        spindle_step_period = spindle_step_period - 1;
        Timer1.setPeriod(spindle_step_period);            //set step period in microseconds
        current_spindle_sps=(1000000/spindle_step_period); //calculate current steps per second from step period
        delayMicroseconds(current_spindle_sps/5);//this is accel rate
    }
    
      while(spindle_sps < current_spindle_sps){//deccelerate to target speed
        
        spindle_step_period = spindle_step_period + 1;
        Timer1.setPeriod(spindle_step_period);
        current_spindle_sps=(1000000/spindle_step_period);
        delayMicroseconds(current_spindle_sps/5);//this is accel rate
      
      }

}
//==============================================================
int spindle_rps(float surf_speed, int rpm, float diameter){
  unsigned long spindle_sps=0;
      
      
  if(surf_speed>0){
    
    float dist_per_rev = PI*diameter; 
    float rps = surf_speed/dist_per_rev;
    spindle_sps = float(rps * spindle_steps_rev * spindle_micro_rat);
  }
  else if(rpm>0){
    Serial.println(rpm);
    spindle_sps= long((rpm/60.0)*spindle_micro_rat);
  }
  else{
    return spindle_sps=0;
  }
  
  return (spindle_sps);
  
}

//==============================================================
void update_y_axis(int y_direction, long y_sps ){
  
    if(digitalRead(Y_DIR)!= y_direction){ //if the direction needs to change then decelerate and accelerate in the opposite direction
      //===deccelerate down to 1 step per second
         while(1000000 < current_y_sps){
        
          y_step_period = y_step_period + 1;
          Timer3.setPeriod(y_step_period);
          current_y_sps=(1000000/y_step_period);
          delayMicroseconds(current_y_sps/5);//this is accel rate
      
        }
      //===change direction
      digitalWrite(Y_DIR, digitalRead(Y_DIR ^ 1));
      
      //===accelerate to target speed
          while(y_sps > current_y_sps){
                y_step_period = y_step_period - 1;
                Timer3.setPeriod(y_step_period);            //set step period in microseconds
                current_y_sps=(1000000/y_step_period); //calculate current steps per second from step period
                delayMicroseconds(current_y_sps/5);//this is accel rate
       }
      
    }
}
//==============================================================
void enable_steppers(){
  
  Serial.println("Motors Enabled");
   digitalWrite(ENABLE, LOW);
  return; 
  
}
//==============================================================
void home_axes(){
  Serial.println("Homing Y axis");
  digitalWrite(S_DIR, LOW);//set y axis direction towards limit switch
  y_move=true;
 // while(!Y_FREEZE){
   // Serial.println("HOMING");
 // }
  
  //  Serial.println("TRIGGERED");
  //y_move=false;
  
}
//==============================================================
void spindle_step(){
    digitalWrite(S_STEP, digitalRead(S_STEP) ^ 1);
}

void Y_axis_step(){
  digitalWrite(S_STEP, digitalRead(S_STEP) ^ 1);
  //track y position in stepper motor steps away from home position
  if(current_y_dir){
    y_step_count+=1;}
  else{
    y_step_count+=-1;
  }
}
//==============================================================
void y_axis_limit_triggered(){
    if((long)(micros() - last_micros) >= debouncing_time * 1000) {
      y_move=false;
       Y_FREEZE=true;
        last_micros = micros();
         Y_LIMIT = true;
  //Serial.println("Limit triggered");
    }
}
//==============================================================
void move_distance(){
  
}
//==============================================================
void init_timers(){
    Timer3.initialize(1000000);
    Timer1.initialize(1000000);
    Timer1.attachInterrupt(spindle_step); //attach service routine
    Timer3.attachInterrupt(Y_axis_step);
return;
}

  
   
