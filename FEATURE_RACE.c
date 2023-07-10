#include "renesas_api.h"

#define ULTRAFAST 100.0
#define FAST 55.0
#define MEDIUM 40.0
#define SLOW 10.0
#define DEAD 0.0
#define BRAKE -40.0
#define TURN_BRAKE -40.0
#define ULTRABRAKE -100.0
#define cornerIn 40.0       //Max speed for safe corner driving (33) - NEW value 40 (using compass)
#define lineChange 40.0       //Max speed for safe line changing

#define STRICT 2000.0       //Need change
#define NORMAL 40.0
#define LOOSE 20.0
#define ULTRALOOSE 15.0

#define K_D 200.0   /*180*/
#define K_P 380.0   //350 
#define TARGET_LINE_VALUE 0.0


int pid(float line_value);
float mainNavigation();
float nearLineNavigation();
double compass();
void debug ();

enum states_t {                                 //main states
  FOLLOW,
  CORNER,
  LINE_CHANGE,
  RAMP_UP,
  RAMP_DOWN
} state = FOLLOW;

enum corner_Helper {                            //states to help CORNER handling
  stripDetected,
  breakingBeforeCorner,
  waitingForCorner,
  leftCorner,
  rightCorner
} cornerState = stripDetected;

enum lineChange_Helper {                        //states to help LINE CHANGING
  halfStripDetected,
  breakingBeforeLineChange,
  waitingForLineChange,
  lineChangeHappening,
  leftLineChange,
  rightLineChange
} lineChangeState = halfStripDetected, determinedChange;

double *speed;                                  //pointer to show the velosity of the 4 wheels  [back right, back left, front right, front left]
double *angles;                                 //pointer to show the direction of the vehicle  [roll, pitch, yaw]
unsigned short *sensors;                        //pointer to show the sensors values             [0 1 2 3 4 5 6 7]
double initDirection = 0;                       //the initial direction of the vehicle before line changing
int totalWhiteSensors;
int leftWhiteSensors;
int rightWhiteSensors;
float line;
float nearLine;
float prev_error = 0.0;
int output_pid;

double downspeed;
double BRAKER;

float lastTime = 0.0;

int main(int argc, char **argv) {
  wb_robot_init();                              // this call is required for WeBots initialisation
  init();                                       // initialises the renesas MCU controller
  while (wb_robot_step(TIME_STEP) != -1) {
    update();                                   //Each 5ms
    speed = encoders();                         //The "encoders()" return a matrix[0-3] with the velocity of each wheel
    angles = imu();	                            //imu()--> [roll, pitch, yaw] --> pitch is what we need
    sensors = line_sensor();
    output_pid = pid(line);
    totalWhiteSensors = 0;
    leftWhiteSensors = 0;
    rightWhiteSensors = 0;

    downspeed = 40 - 30 * get_weight_penalty();
    BRAKER = -45 - 10 * get_weight_penalty();

    for (int i = 0; i < 8; i++) {
      if (sensors[i] < 400){                     //0=LIGHT , +1000=DARK
        totalWhiteSensors++;
        if (i < 4)
          leftWhiteSensors++;
        else
          rightWhiteSensors++;
      }
    }
    line = mainNavigation();
    nearLine = nearLineNavigation();

      switch (state) {

        case FOLLOW:
          if ((speed[0] > 50) || (speed[1] > 50) || (speed[2] > 50) || (speed[3] > 50)){
            handle(-output_pid);                                                            //-60 to +60
            motor(BRAKE, BRAKE, BRAKE, BRAKE);                                                  //motor(back right, back left, front right, front left). (-100 to +100)
          }
          else{
            handle(-output_pid);
            motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST); 
          }
          if (fabs(line) > 0.01){                                                           //Turn detected
            printf("TURNING\n");
            if ((speed[0] > 50) || (speed[1] > 50) || (speed[2] > 50) || (speed[3] > 50)){
              //printf("BREAKING\n");
              motor(TURN_BRAKE, TURN_BRAKE, TURN_BRAKE, TURN_BRAKE);
            }
            else{
              //printf("ACCELERATING\n");
              motor(cornerIn, cornerIn, cornerIn, cornerIn)	;
            }
          }
          if (totalWhiteSensors == 8){                                                 //Fisrt full strip was detected
            state = CORNER;
            cornerState = stripDetected;
            initDirection = compass();
            lastTime = time();
            printf("FULL_STRIP_DETECTED --> Enter CORNER state\n");
          }
          else if ((leftWhiteSensors == 4) || (rightWhiteSensors == 4)){                    //Fisrt half strip was detected
            printf("FIRST_HALF_STRIP_DETECTED --> Enter LINE_CHANGE state\n");
            initDirection = compass();
            handle(NORMAL * line);
            motor(BRAKE, BRAKE, BRAKE, BRAKE);
            state = LINE_CHANGE;
            if (leftWhiteSensors == 4){
              printf("PREPARING LEFT_LINE_CHANGE\n");
              determinedChange = leftLineChange;
            }
            else if (rightWhiteSensors == 4){
              printf("PREPARING RIGHT_LINE_CHANGE\n");
              determinedChange = rightLineChange;
            }
          }
          else if (angles[1] > 0.1){        	                                                  //upward pitch rotation was detected
            state = RAMP_UP;
            printf("RAMP UP\n");
          }
          else if (angles[1] < -0.04){                                                      //downward pitch rotation was detected
            state = RAMP_DOWN;
            printf("RAMP DOWN\n");
          }
        break;
        
        case LINE_CHANGE:
          switch (lineChangeState){
            case halfStripDetected:
              if (totalWhiteSensors == 8){
                printf("FULL_STRIP_DETECTED\n");
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                handle(NORMAL * line);
                initDirection = compass();
                state = CORNER;
                cornerState = stripDetected;
                lastTime = time();
              }
              else{
                printf("HALF_STRIP_DETECTED ");
                if ((speed[0] > 50) || (speed[1] > 50) || (speed[2] > 50) || (speed[3] > 50)){
                  motor(BRAKE, BRAKE, BRAKE, BRAKE);
                  handle(LOOSE * line);
                }
                else{
                  motor(lineChange, lineChange, lineChange, lineChange);
                  handle(LOOSE * line);
                }
                if ((leftWhiteSensors != 4) && (rightWhiteSensors != 4)){
                  lineChangeState = breakingBeforeCorner;
                  printf("BREAKING_BEFORE_LINE_CHANGE\n");
                }
              }
            break;
            case breakingBeforeLineChange:
              if ((leftWhiteSensors == 4) || (rightWhiteSensors == 4)){                    //Fisrt half strip was detected
                printf("HALF_STRIP_DETECTED\n");
                initDirection = compass();
                handle(NORMAL * line);
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                state = LINE_CHANGE;
                lineChangeState = halfStripDetected;
                if (leftWhiteSensors == 4){
                  printf("PREPARING LEFT_LINE_CHANGE\n");
                  determinedChange = leftLineChange;
                }
                else if (rightWhiteSensors == 4){
                  printf("PREPARING RIGHT_LINE_CHANGE\n");
                  determinedChange = rightLineChange;
                }
              }
              else if (totalWhiteSensors == 0) {
                printf("LINE CHANGE HAPPENING \n");
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                handle(0);
                lineChangeState = lineChangeHappening;
              }
              else{
                printf("1) breakingBeforeLineChange ");
                if ((speed[0] > 50) || (speed[1] > 50) || (speed[2] > 50) || (speed[3] > 50)){
                  motor(BRAKE, BRAKE, BRAKE, BRAKE);
                  handle(ULTRALOOSE * nearLine);
                }
                else{
                  motor(BRAKE, BRAKE, BRAKE, BRAKE);
                  handle(ULTRALOOSE * nearLine);
                  lineChangeState = waitingForLineChange;
                }
              }
            break;
            case waitingForLineChange:
              if ((leftWhiteSensors == 4) || (rightWhiteSensors == 4)){                    //Fisrt half strip was detected
                printf("HALF_STRIP_DETECTED\n");
                initDirection = compass();
                handle(NORMAL * line);
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                state = LINE_CHANGE;
                lineChangeState = halfStripDetected;
                if (leftWhiteSensors == 4){
                  printf("PREPARING LEFT_LINE_CHANGE\n");
                  determinedChange = leftLineChange;
                }
                else if (rightWhiteSensors == 4){
                  printf("PREPARING RIGHT_LINE_CHANGE\n");
                  determinedChange = rightLineChange;
                }
              }
              else if (totalWhiteSensors == 0) {
                printf("LINE CHANGE HAPPENING \n");
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                handle(0);
                lineChangeState = lineChangeHappening;
              }
              else {
                printf("waitingForLineChange ");
                if ((speed[0] > 50) || (speed[1] > 50) || (speed[2] > 50) || (speed[3] > 50)){
                  motor(BRAKE, BRAKE, BRAKE, BRAKE);
                  handle(ULTRALOOSE * nearLine);
                }
                else{
                  motor(lineChange, lineChange, lineChange, lineChange);
                  handle(ULTRALOOSE * nearLine);
                }
              }
            break;
            case lineChangeHappening:
              switch (determinedChange){
                case leftLineChange:
                  if ((leftWhiteSensors == 4) || (rightWhiteSensors == 4)){                    //Fisrt half strip was detected
                    printf("HALF_STRIP_DETECTED\n");
                    initDirection = compass();
                    handle(NORMAL * line);
                    motor(BRAKE, BRAKE, BRAKE, BRAKE);
                    state = LINE_CHANGE;
                    lineChangeState = halfStripDetected;
                    if (leftWhiteSensors == 4){
                      printf("PREPARING LEFT_LINE_CHANGE\n");
                      determinedChange = leftLineChange;
                    }
                    else if (rightWhiteSensors == 4){
                      printf("PREPARING RIGHT_LINE_CHANGE\n");
                      determinedChange = rightLineChange;
                    }
                  }
                  else{
                    if ((totalWhiteSensors == 0) && (fabs(fabs(initDirection) - fabs(angles[2])) < 0.38)){
                      printf("1) turning ");
                      motor(lineChange, lineChange, lineChange, lineChange);
                      handle(30);
                    }
                    else if ((totalWhiteSensors == 0) && (fabs(fabs(initDirection) - fabs(angles[2])) >= 0.38)){
                      printf("2) straightening ");
                      motor(lineChange, lineChange, lineChange, lineChange);
                      handle(-30);
                    }
                    else if ((totalWhiteSensors >= 1) && (fabs(fabs(initDirection) - fabs(angles[2])) > 0.50)){
                      printf("3a) prepairing follow ");
                      //motor(lineChange, lineChange, lineChange, lineChange);
                      motor(SLOW, SLOW, SLOW, SLOW);
                      handle(-25);
                    }
                    else if ((totalWhiteSensors >= 1) && (fabs(fabs(initDirection) - fabs(angles[2])) > 0.30)){
                      printf("3b) prepairing follow ");
                      motor(SLOW, SLOW, SLOW, SLOW);
                      handle(-output_pid);
                      //motor(lineChange, lineChange, lineChange, lineChange);
                      //handle(LOOSE * line);
                    }
                    else {
                      printf("\nEXIT LINE_CHANGE state");
                      state = FOLLOW;
                      lineChangeState = halfStripDetected;    //Re-initiallize "lineChangeState" value
                    }
                  }
                break;
                case rightLineChange:
                  if ((leftWhiteSensors == 4) || (rightWhiteSensors == 4)){                    //Fisrt half strip was detected
                    printf("HALF_STRIP_DETECTED\n");
                    initDirection = compass();
                    handle(NORMAL * line);
                    motor(BRAKE, BRAKE, BRAKE, BRAKE);
                    state = LINE_CHANGE;
                    lineChangeState = halfStripDetected;
                    if (leftWhiteSensors == 4){
                      printf("PREPARING LEFT_LINE_CHANGE\n");
                      determinedChange = leftLineChange;
                    }
                    else if (rightWhiteSensors == 4){
                      printf("PREPARING RIGHT_LINE_CHANGE\n");
                      determinedChange = rightLineChange;
                    }
                  }
                  else{
                    if ((totalWhiteSensors == 0) && (fabs(fabs(initDirection) - fabs(angles[2])) < 0.38)){
                      printf("1) turning ");
                      motor(lineChange, lineChange, lineChange, lineChange);
                      handle(-30);
                    }
                    else if ((totalWhiteSensors == 0) && (fabs(fabs(initDirection) - fabs(angles[2])) >= 0.38)){
                      printf("2) straightening ");
                      motor(lineChange, lineChange, lineChange, lineChange);
                      handle(30);
                    }
                    else if ((totalWhiteSensors >= 1) && (fabs(fabs(initDirection) - fabs(angles[2])) > 0.50)){
                      printf("3a) prepairing follow ");
                      //motor(lineChange, lineChange, lineChange, lineChange);
                      motor(SLOW, SLOW, SLOW, SLOW);
                      handle(25);
                    }
                    else if ((totalWhiteSensors >= 1) && (fabs(fabs(initDirection) - fabs(angles[2])) > 0.30)){
                      printf("3b) prepairing follow ");
                      motor(SLOW, SLOW, SLOW, SLOW);
                      handle(-output_pid);
                      //motor(lineChange, lineChange, lineChange, lineChange);
                      //handle(LOOSE * line);
                    }
                    else {
                      printf("\nEXIT LINE_CHANGE state");
                      state = FOLLOW;
                      lineChangeState = halfStripDetected;    //Re-initiallize "lineChangeState" value
                    }
                  }
                break;
                case halfStripDetected:           //To avoid compilation warnings
                case breakingBeforeLineChange:    //To avoid compilation warnings
                case waitingForLineChange:        //To avoid compilation warnings
                case lineChangeHappening:         //To avoid compilation warnings
                break;
              };
            break;
            case leftLineChange:    //To avoid compilation warnings
            case rightLineChange:   //To avoid compilation warnings
            break;
          };
        break;

        case CORNER:
          switch (cornerState){
            case stripDetected:
              if (angles[1] > 0.1){
                printf("RAMP UP\n");
                motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST);
                handle(STRICT * line);
                state = RAMP_UP;
              }
              else{
                printf("FULL_STRIP_DETECTED ");
                initDirection = compass();
                if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40)){
                  motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                  handle(STRICT * line);
                }
                else{
                  motor(cornerIn, cornerIn, cornerIn, cornerIn);
                  handle(STRICT * line);
                }
                if ((totalWhiteSensors < 8) && (time() - lastTime > 0.1)){
                  cornerState = breakingBeforeCorner;
                  printf("BREAKING_BEFORE_CORNER\n");
                }
              }
            break;
            case breakingBeforeCorner:
              if (angles[1] > 0.1){
                printf("RAMP UP\n");
                motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST);
                handle(STRICT * line);
                state = RAMP_UP;
              }
              else if (totalWhiteSensors == 8){
                printf("FULL_STRIP_DETECTED\n");
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                handle(STRICT * line);
                lastTime = time();
                cornerState = stripDetected;
              }
              else if (rightWhiteSensors == 4) {
                printf("RIGHT CORNER \n");
                motor(ULTRABRAKE, cornerIn, ULTRABRAKE, cornerIn);
                handle(STRICT * line);
                cornerState = rightCorner;
              }
              else if (leftWhiteSensors == 4) {
                printf("LEFT CORNER \n");
                motor(cornerIn, ULTRABRAKE, cornerIn, ULTRABRAKE);
                handle(STRICT * line);
                cornerState = leftCorner;
              }
              else{
                printf("1) breakingBeforeCorner ");
                if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40)){
                  motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                  handle(STRICT * line);
                }
                else{
                  motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                  handle(STRICT * line);
                  cornerState = waitingForCorner;
                }
              }
            break;
            case waitingForCorner:
              if (angles[1] > 0.1){
                printf("RAMP UP\n");
                motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST);
                handle(STRICT * line);
                state = RAMP_UP;
              }
              else if (totalWhiteSensors == 8){
                printf("FULL_STRIP_DETECTED\n");
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                handle(STRICT * line);
                lastTime = time();
                cornerState = stripDetected;
              }
              else if (rightWhiteSensors == 4) {
                printf("RIGHT CORNER ");
                motor(ULTRABRAKE, cornerIn, ULTRABRAKE, cornerIn);
                handle(STRICT * line);
                cornerState = rightCorner;
              }
              else if (leftWhiteSensors == 4) {
                printf("LEFT CORNER ");
                motor(cornerIn, ULTRABRAKE, cornerIn, ULTRABRAKE);
                handle(STRICT * line);
                cornerState = leftCorner;
              }
              else {
                printf("2) waitingForCorner ");
                if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40)){
                  motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                  handle(STRICT * line);
                }
                else{
                  motor(cornerIn, cornerIn, cornerIn, cornerIn);
                  handle(STRICT * line);
                }
              }
            break;
            case rightCorner:
              if (totalWhiteSensors == 8){
                printf("FULL_STRIP_DETECTED\n");
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                handle(STRICT * line);
                lastTime = time();
                cornerState = stripDetected;
              }
              else{
                if (fabs(fabs(initDirection) - fabs(angles[2])) < 0.60){
                  if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40)){
                    printf("3a) rightCorner - Breaking ");
                    motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                    handle(-50);
                  }
                  else{
                    printf("3a) rightCorner ");
                    motor(DEAD, cornerIn, DEAD, cornerIn);
                    handle(-50);
                  }
                }
                else{
                  printf("\nEXIT CORNER state");
                  state = FOLLOW;
                  cornerState = stripDetected;    //Re-initiallize "cornerState" value
                }
              }
            break;
            case leftCorner:
              if (totalWhiteSensors == 8){
                printf("FULL_STRIP_DETECTED\n");
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                handle(STRICT * line);
                lastTime = time();
                cornerState = stripDetected;
              }
              else{
                if (fabs(fabs(initDirection) - fabs(angles[2])) < 0.60){
                  if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40)){
                    printf("3b) leftCorner - Breaking ");
                    motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                    handle(50);
                  }
                  else{
                    printf("3b) leftCorner ");
                    motor(cornerIn, DEAD, cornerIn, DEAD);
                    handle(50);
                  }
                }
                else{
                  printf("\nEXIT CORNER state");
                  state = FOLLOW;
                  cornerState = stripDetected;    //Re-initiallize "cornerState" value
                }
              }
            break;
          }
        break;

        case RAMP_UP:
          printf("RAMP_UP\n");
          if ((speed[0] > 60) || (speed[1] > 60) || (speed[2] > 60) || (speed[3] > 60))     //The same speed limit like FOLLOW state
            motor(DEAD, DEAD, DEAD, DEAD);
          else
            motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST);
          handle(STRICT * line);
          if (totalWhiteSensors == 0){                                                        //at the end of the RAMP_UP slow down to avoid large jump
            if ((speed[0] > 30) || (speed[1] > 30) || (speed[2] > 30) || (speed[3] > 30))     //The same speed limit like FOLLOW state
              motor(DEAD, DEAD, DEAD, DEAD);
            else
              motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST);
          }
          if (angles[1] < 0.1){
            state = FOLLOW;
            printf("FOLLOW\n");
          }
        break;

        case RAMP_DOWN:
          printf("RAMP_DOWN\n");
          if ((speed[0] > downspeed) || (speed[1] > downspeed) || (speed[2] > downspeed) || (speed[3] > downspeed))     //The same speed limit like FOLLOW state
            motor(BRAKER, BRAKER, BRAKER, BRAKER);
          else
            motor(FAST, FAST, FAST, FAST);
          handle(-output_pid);
          //handle(STRICT * line);
          if (totalWhiteSensors == 8){                                                       //at the end of the RAMP_DOWN crashing on the ground
            if ((speed[0] > 20) || (speed[1] > 20) || (speed[2] > 20) || (speed[3] > 20))     //The same speed limit like FOLLOW state
              motor(BRAKER, BRAKER, BRAKER, BRAKER);
            else
              motor(SLOW, SLOW, SLOW, SLOW);
          }
          if (angles[1] > -0.01){
            state = FOLLOW;
            printf("FOLLOW\n");
          }
        break;
      }

    debug();
  };
  wb_robot_cleanup(); // this call is required for WeBots cleanup
  return 0;
}

float mainNavigation(){
  float sum = 0, weighted_sum = 0;
  for (int i = 0; i < 8; i++) {
    weighted_sum += sensors[i] * i;
    sum += sensors[i];
  }
  return weighted_sum / sum - 3.5;
}

float nearLineNavigation(){
  float sum = 0, weighted_sum = 0;
  weighted_sum += sensors[0] * 0;
  weighted_sum += sensors[4] * 1;
  weighted_sum += sensors[5] * 2;
  weighted_sum += sensors[6] * 3;
  weighted_sum += sensors[1] * 4;
  weighted_sum += sensors[2] * 5;
  weighted_sum += sensors[3] * 6;
  weighted_sum += sensors[7] * 7;
  for (int i = 0; i < 8; i++) {
    sum += sensors[i];
  }
  return weighted_sum / sum - 3.5;
}

double compass(){
  if ((angles[2] < -0.79) && (angles[2] > -2.36))           //EAST
    return -1.57;
  else if ((angles[2] < -2.36) || (angles[2] > 2.36))       //SOUTH
    return -3.14;
  else if ((angles[2] < 2.36) && (angles[2] > 0.79))        //WEST
    return 1.57;
  else if ((angles[2] < 0.79) && (angles[2] > -0.79))       //NORTH
    return 0.0;
  return 0.0;   //Just to avoid compilation warnings (one of the above cases should happen)
}

int pid(float line_value) {
  float error,pid_proportional,pid_derivative,pid_output;
  error = TARGET_LINE_VALUE - line_value;
  pid_proportional = K_P * error;           // proportional term
  pid_derivative = K_D * (error - prev_error);    //derivative term
  /*integral term pending*/
  pid_output = pid_proportional + pid_derivative;
  prev_error = error;          //update error
  return pid_output;        //multiply by 100 so you get non-zero values inside handle (doesn't turn otherwise)
}

void debug (){
  //printf("line = %f \n", line);
  //printf("prev_error = %f \n", line);
  //printf("output_pid = %d \n", output_pid);
  //printf("handle = %f \n", line*STRICT);
  //printf("totalWhiteSensors = %d \n", totalWhiteSensors);
  //printf("leftWhiteSensors = %d \n", leftWhiteSensors);
  //printf("rightWhiteSensors = %d \n", rightWhiteSensors);
  //printf("velocity = [%.2f %.2f %.2f %.2f]\n", speed[0], speed[1], speed[2], speed[3]);                       //speed[0]=back right, speed[1]=back left, speed[2]=front right, speed[3]=front left 
  //printf("time = %.2f , meanSpeed = %.2f\n", time() , (speed[0] + speed[1] + speed[2] + speed[3])/4 );
  printf("angles[0]=%.2f angles[1]=%.2f angles[2]=%.2f\n", angles[0], angles[1], angles[2]);
  //printf("sensors = [%d %d %d %d %d %d %d %d]\n", sensors[0], sensors[1], sensors[2], sensors[3], sensors[4], sensors[5], sensors[6], sensors[7]);
}