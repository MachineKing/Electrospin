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

/*==============================================================
Definitions
==============================================================*/
//
#define EG_SPINDLE 1.0 
#define EG_Y 1.0
//OUTPUTS
#define S_STEP 2
#define S_DIR
#define S_EN

#define Y_STEP 
#define Y_DIR
#define Y_EN

#define EN_HEAT
#define S_HEAT

//INPUTS
#define y_limit
/*==============================================================
//SETUP
==============================================================*/

void setup() {
  // put your setup code here, to run once:

}

/*==============================================================
// MAIN
==============================================================*/

void loop() {
  // put your main code here, to run repeatedly:

}

/*==============================================================
FUNCTIONS
==============================================================*/
void receive_command(){
  
}
//==============================================================
void calc_rpm(float surf_speed, float diameter){ 
}
//==============================================================
void update_spindle(){
}
//==============================================================
void update_y_axis(){ 
}
//==============================================================
  
  
