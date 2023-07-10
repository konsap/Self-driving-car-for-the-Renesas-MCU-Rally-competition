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
#define lineChange 60       //Max speed for safe line changing
#define slowerLineChange 50

#define STRICT 2000.0       //Need change
#define NORMAL 30.0
#define LOOSE 20.0
#define ULTRALOOSE 6.0
#define ULTRAULTRALOOSE 5.0

#define K_D 200.0   /*180*/
#define K_P 380.0   //350 
#define TARGET_LINE_VALUE 0.0

int pid(float line_value);
float mainNavigation();
float nearLineNavigation();
double compass();
void debug ();

enum states_t {                                 //main states
  start,
  turn1,
  corner1,
  straight0,
  corner2,
  straight1,
  rampdown1,
  turn234,
  linechange1,
  turn5,
  straight2,
  rampup,
  rampstraight,
  rampdown2,
  chaos,
  corner3,
  turn6,
  linechange2,
  straight3,
  linechange3,
  turns,
  corner4,
  final,
} state = start;

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
int counter = 0;

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
    BRAKER = -40 - 10 * get_weight_penalty();

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

        case start:
          printf("start\n");
          handle(NORMAL * line);
          if ((speed[0] > 100.0) || (speed[1] > 100.0) || (speed[2] > 100.0) || (speed[3] > 100.0))
              motor(DEAD, DEAD, DEAD, DEAD);
          else
              motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST); 
          if (fabs(line) > 0.05)
              state = turn1;
        break;

        case turn1:
          printf("turn1\n");
          handle(-output_pid);
          if ((speed[0] > 95.0) || (speed[1] > 95.0) || (speed[2] > 95.0) || (speed[3] > 95.0))
            motor(DEAD, DEAD, DEAD, DEAD);
          else
              motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST);
          if (fabs(line) > 0.05){
            printf("TURNING\n");
            if ((speed[0] > 70.0) || (speed[1] > 70.0) || (speed[2] > 70.0) || (speed[3] > 70.0))
              motor(TURN_BRAKE, TURN_BRAKE, TURN_BRAKE, TURN_BRAKE);
            else
              motor(cornerIn, cornerIn, cornerIn, cornerIn);
          }
          if (totalWhiteSensors == 8){                                                 //Fisrt full strip was detected
            state = corner1;
            cornerState = stripDetected;
            initDirection = compass();
            lastTime = time();
          }
        break;

        case corner1:
          printf("corner1\n");
          switch (cornerState){
            case stripDetected:
              printf("FULL_STRIP_DETECTED ");
              initDirection = compass();
              handle(STRICT * line);
              if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40))
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
              else
                motor(cornerIn, cornerIn, cornerIn, cornerIn);
              if ((totalWhiteSensors < 8) && (time() - lastTime > 0.1)){
                cornerState = waitingForCorner;
                printf("BREAKING_BEFORE_CORNER\n");
              }
            break;
            case waitingForCorner:
              if (totalWhiteSensors == 8){
                printf("FULL_STRIP_DETECTED\n");
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                handle(STRICT * line);
                lastTime = time();
                cornerState = stripDetected;
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
                  state = straight0;
                  cornerState = stripDetected;    //Re-initiallize "cornerState" value
                }
              }
            break;
            case breakingBeforeCorner:     //To avoid compilation warnings
            case rightCorner:              //To avoid compilation warnings
            break;
          }
        break;

        case straight0:
          printf("straight0\n");
          handle(NORMAL * line);
          if ((speed[0] > 70.0) || (speed[1] > 70.0) || (speed[2] > 70.0) || (speed[3] > 70.0))
              motor(DEAD, DEAD, DEAD, DEAD);
          else
              motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST); 
          if (totalWhiteSensors == 8){                                                 //Fisrt full strip was detected
              state = corner2;
              cornerState = stripDetected;
              initDirection = compass();
              lastTime = time();
          }
        break;

        case corner2:
          printf("corner2\n");
          switch (cornerState){
            case stripDetected:
              printf("FULL_STRIP_DETECTED ");
              initDirection = compass();
              handle(STRICT * line);
              if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40))
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
              else
                motor(cornerIn, cornerIn, cornerIn, cornerIn);
              if ((totalWhiteSensors < 8) && (time() - lastTime > 0.1)){
                cornerState = waitingForCorner;
                printf("BREAKING_BEFORE_CORNER\n");
              }
            break;
            case waitingForCorner:
              if (totalWhiteSensors == 8){
                printf("FULL_STRIP_DETECTED\n");
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                handle(STRICT * line);
                lastTime = time();
                cornerState = stripDetected;
              }
              else if (leftWhiteSensors == 4) {
                printf("LEFT CORNER ");
                motor(cornerIn, ULTRABRAKE, cornerIn, ULTRABRAKE);
                handle(STRICT * line);
                cornerState = leftCorner;
              }
              else {
                printf("2) waitingForCorner ");
                handle(STRICT * line);
                if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40))
                  motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                else
                  motor(cornerIn, cornerIn, cornerIn, cornerIn);
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
                  state = straight1;
                  cornerState = stripDetected;    //Re-initiallize "cornerState" value
                }
              }
            break;
            case breakingBeforeCorner:     //To avoid compilation warnings
            case rightCorner:              //To avoid compilation warnings
            break;
          }
        break;

        case straight1:
          printf("straight1\n");
          handle(NORMAL * line);
          motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST); 
          if (angles[1] < -0.1)
            state = rampdown1;
        break;

        case rampdown1:
          printf("rampdown1\n");
          handle(NORMAL * line);
          motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST); 
          if (angles[1] > -0.01)
            state = turn234;
        break;

        case turn234:
          printf("turn234\n");
          handle(-output_pid);
          if ((speed[0] > 80.0) || (speed[1] > 80.0) || (speed[2] > 80.0) || (speed[3] > 80.0))
            motor(DEAD, DEAD, DEAD, DEAD);
          else
            motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST);
          if (fabs(line) > 0.04){
            printf("TURNING\n");
            if ((speed[0] > 60.0) || (speed[1] > 60.0) || (speed[2] > 60.0) || (speed[3] > 60.0))
              motor(TURN_BRAKE, TURN_BRAKE, TURN_BRAKE, TURN_BRAKE);
            else
              motor(cornerIn, cornerIn, cornerIn, cornerIn);
          }
          if (rightWhiteSensors == 4){                    //Fisrt half strip was detected
            printf("FIRST_HALF_STRIP_DETECTED --> PREPARING RIGHT_LINE_CHANGE\n");
            initDirection = compass();
            handle(NORMAL * line);
            state = linechange1;
            determinedChange = rightLineChange;
          }
        break;

        case linechange1:
          printf("linechange1\n");
          switch (lineChangeState){
            case halfStripDetected:
              printf("HALF_STRIP_DETECTED ");
              handle(NORMAL * line);
              if ((speed[0] > 75) || (speed[1] > 75) || (speed[2] > 75) || (speed[3] > 75))
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
              else
                motor(lineChange, lineChange, lineChange, lineChange);
              if ((leftWhiteSensors != 4) && (rightWhiteSensors != 4))
                lineChangeState = waitingForLineChange;
            break;
            case waitingForLineChange:
              if (rightWhiteSensors == 4){                    //Fisrt half strip was detected
                printf("FIRST_HALF_STRIP_DETECTED --> PREPARING RIGHT_LINE_CHANGE\n");
                initDirection = compass();
                handle(NORMAL * line);
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                state = linechange1;
                lineChangeState = halfStripDetected;
                determinedChange = rightLineChange;
              }
              else if (totalWhiteSensors == 0) {
                printf("LINE CHANGE HAPPENING \n");
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                handle(0);
                lineChangeState = lineChangeHappening;
              }
              else {
                printf("2) waitingForLineChange ");
                handle(ULTRALOOSE * nearLine);
                if ((speed[0] > 75) || (speed[1] > 75) || (speed[2] > 75) || (speed[3] > 75))
                  motor(BRAKE, BRAKE, BRAKE, BRAKE);
                else
                  motor(lineChange, lineChange, lineChange, lineChange);
              }
            break;
            case lineChangeHappening:
              switch (determinedChange){
                case rightLineChange:
                  if ((totalWhiteSensors == 0) && (fabs(fabs(initDirection) - fabs(angles[2])) < 0.25)){
                    printf("1) turning ");
                    motor(lineChange, lineChange, lineChange, lineChange);
                    handle(-30);
                  }
                  else if ((totalWhiteSensors == 0) && (fabs(fabs(initDirection) - fabs(angles[2])) >= 0.25)){
                    printf("2) straightening ");
                    motor(lineChange, lineChange, lineChange, lineChange);
                    handle(30);
                  }
                  else if ((totalWhiteSensors >= 1) && (fabs(fabs(initDirection) - fabs(angles[2])) > 0.42)){
                    printf("3a) prepairing follow ");
                    motor(lineChange, lineChange, lineChange, lineChange);
                    //motor(SLOW, SLOW, SLOW, SLOW);
                    handle(30);
                  }
                  else if ((totalWhiteSensors >= 1) && (fabs(fabs(initDirection) - fabs(angles[2])) > 0.10)){
                    printf("3b) prepairing follow ");
                    //motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
                    handle(-output_pid);
                    motor(lineChange, lineChange, lineChange, lineChange);
                    //handle(LOOSE * line);
                  }
                  else {
                    printf("\nEXIT LINE_CHANGE state");
                    initDirection = compass();
                    state = turn5;
                    lineChangeState = halfStripDetected;    //Re-initiallize "lineChangeState" value
                  }
                break;
                case leftLineChange:
                case halfStripDetected:           //To avoid compilation warnings
                case breakingBeforeLineChange:    //To avoid compilation warnings
                case waitingForLineChange:        //To avoid compilation warnings
                case lineChangeHappening:         //To avoid compilation warnings
                break;
              };
            break;
            case breakingBeforeLineChange:    //To avoid compilation warnings
            case leftLineChange:              //To avoid compilation warnings
            case rightLineChange:             //To avoid compilation warnings
            break;
          };
        break;

        case turn5:
          printf("turn5\n");
          handle(-output_pid);
          if ((speed[0] > 65.0) || (speed[1] > 65.0) || (speed[2] > 65.0) || (speed[3] > 65.0))
            motor(DEAD, DEAD, DEAD, DEAD);
          else
            motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST);
          if (fabs(line) > 0.04){
            printf("TURNING\n");
            if ((speed[0] > 60.0) || (speed[1] > 60.0) || (speed[2] > 60.0) || (speed[3] > 60.0))
              motor(TURN_BRAKE, TURN_BRAKE, TURN_BRAKE, TURN_BRAKE);
            else
              motor(cornerIn, cornerIn, cornerIn, cornerIn);
          }
          if (fabs(fabs(initDirection) - fabs(angles[2])) > 1.50)
            state = straight2;
        break;

        case straight2:
          printf("straight2\n");
          handle(NORMAL * line);
          motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST); 
          if (angles[1] > 0.06)
            state = rampup;
        break;

        case rampup:
          printf("rampup\n");
          handle(NORMAL * line);
          motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST); 
          if (angles[1] < 0.01)
            state = rampstraight;
        break;

        case rampstraight:
          printf("rampstraight\n");
          handle(NORMAL * line);
          if ((speed[0] > 70.0) || (speed[1] > 70.0) || (speed[2] > 70.0) || (speed[3] > 70.0))
            motor(DEAD, DEAD, DEAD, DEAD);
          else
            motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST); 
          if (angles[1] < -0.15)
            state = rampdown2;
        break;

        case rampdown2:
          printf("rampdown2\n");
          handle(-output_pid);
          //handle(STRICT * line);
          if ((speed[0] > 30) || (speed[1] > 30) || (speed[2] > 30) || (speed[3] > 30))
            motor(-50, -50, -50, -50);
          else
            motor(SLOW, SLOW, SLOW, SLOW);
          if (totalWhiteSensors == 8){                                                       //at the end of the RAMP_DOWN crashing on the ground
            handle(-30);
            if ((speed[0] > 25) || (speed[1] > 25) || (speed[2] > 25) || (speed[3] > 25))
              motor(-50, -50, -50, -50);
            else
              motor(SLOW, SLOW, SLOW, SLOW);
          }
          if (angles[1] > -0.01)
            state = chaos;
        break;

        case chaos:
          printf("chaos\n");
          handle(-output_pid);
          //handle(STRICT * line);
          if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40))
            motor(0, 0, 0, 0);
          else
            motor(30, 30, 30, 30);
          if (totalWhiteSensors == 8){                                                 //Fisrt full strip was detected
            state = corner3;
            cornerState = stripDetected;
            initDirection = compass();
            lastTime = time();
          }
        break;

        case corner3:
          printf("corner3\n");
          switch (cornerState){
            case stripDetected:
              printf("FULL_STRIP_DETECTED ");
              initDirection = compass();
              handle(STRICT * line);
              if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40))
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
              else
                motor(cornerIn, cornerIn, cornerIn, cornerIn);
              if ((totalWhiteSensors < 8) && (time() - lastTime > 0.1)){
                cornerState = waitingForCorner;
                printf("BREAKING_BEFORE_CORNER\n");
              }
            break;
            case waitingForCorner:
              if (totalWhiteSensors == 8){
                printf("FULL_STRIP_DETECTED\n");
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                handle(STRICT * line);
                lastTime = time();
                cornerState = stripDetected;
              }
              else if (leftWhiteSensors == 4) {
                printf("LEFT CORNER ");
                motor(cornerIn, ULTRABRAKE, cornerIn, ULTRABRAKE);
                handle(STRICT * line);
                cornerState = leftCorner;
              }
              else {
                printf("2) waitingForCorner ");
                handle(STRICT * line);
                if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40))
                  motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                else
                  motor(cornerIn, cornerIn, cornerIn, cornerIn);
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
                  state = turn6;
                  cornerState = stripDetected;    //Re-initiallize "cornerState" value
                }
              }
            break;
            case breakingBeforeCorner:     //To avoid compilation warnings
            case rightCorner:              //To avoid compilation warnings
            break;
          }
        break;

        case turn6:
          printf("turn6\n");
          handle(-output_pid);
          if ((speed[0] > 85.0) || (speed[1] > 85.0) || (speed[2] > 85.0) || (speed[3] > 85.0))
            motor(DEAD, DEAD, DEAD, DEAD);
          else
            motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST);
          if (fabs(line) > 0.04){
            printf("TURNING\n");
            if ((speed[0] > 60.0) || (speed[1] > 60.0) || (speed[2] > 60.0) || (speed[3] > 60.0))
              motor(TURN_BRAKE, TURN_BRAKE, TURN_BRAKE, TURN_BRAKE);
            else
              motor(cornerIn, cornerIn, cornerIn, cornerIn);
          }
          if (rightWhiteSensors == 4){                    //Fisrt half strip was detected
            printf("FIRST_HALF_STRIP_DETECTED --> PREPARING RIGHT_LINE_CHANGE\n");
            initDirection = compass();
            handle(NORMAL * line);
            state = linechange2;
            determinedChange = rightLineChange;
          }
        break;

        case linechange2:
          printf("linechange2\n");
          switch (lineChangeState){
            case halfStripDetected:
              printf("HALF_STRIP_DETECTED ");
              handle(NORMAL * line);
              if ((speed[0] > 75) || (speed[1] > 75) || (speed[2] > 75) || (speed[3] > 75))
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
              else
                motor(lineChange, lineChange, lineChange, lineChange);
              if ((leftWhiteSensors != 4) && (rightWhiteSensors != 4))
                lineChangeState = waitingForLineChange;
            break;
            case waitingForLineChange:
              if (rightWhiteSensors == 4){                    //Fisrt half strip was detected
                printf("FIRST_HALF_STRIP_DETECTED --> PREPARING RIGHT_LINE_CHANGE\n");
                initDirection = compass();
                handle(NORMAL * line);
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                state = linechange2;
                lineChangeState = halfStripDetected;
                determinedChange = rightLineChange;
              }
              else if (totalWhiteSensors == 0) {
                printf("LINE CHANGE HAPPENING \n");
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                handle(0);
                lineChangeState = lineChangeHappening;
              }
              else {
                printf("2) waitingForLineChange ");
                handle(ULTRALOOSE * nearLine);
                if ((speed[0] > 75) || (speed[1] > 75) || (speed[2] > 75) || (speed[3] > 75))
                  motor(BRAKE, BRAKE, BRAKE, BRAKE);
                else
                  motor(lineChange, lineChange, lineChange, lineChange);
              }
            break;
            case lineChangeHappening:
              switch (determinedChange){
                case rightLineChange:
                  if ((totalWhiteSensors == 0) && (fabs(fabs(initDirection) - fabs(angles[2])) < 0.25)){
                    printf("1) turning ");
                    motor(lineChange, lineChange, lineChange, lineChange);
                    handle(-30);
                  }
                  else if ((totalWhiteSensors == 0) && (fabs(fabs(initDirection) - fabs(angles[2])) >= 0.25)){
                    printf("2) straightening ");
                    motor(lineChange, lineChange, lineChange, lineChange);
                    handle(30);
                  }
                  else if ((totalWhiteSensors >= 1) && (fabs(fabs(initDirection) - fabs(angles[2])) > 0.42)){
                    printf("3a) prepairing follow ");
                    motor(lineChange, lineChange, lineChange, lineChange);
                    //motor(SLOW, SLOW, SLOW, SLOW);
                    handle(30);
                  }
                  else if ((totalWhiteSensors >= 1) && (fabs(fabs(initDirection) - fabs(angles[2])) > 0.10)){
                    printf("3b) prepairing follow ");
                    //motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
                    handle(-output_pid);
                    motor(lineChange, lineChange, lineChange, lineChange);
                    //handle(LOOSE * line);
                  }
                  else {
                    printf("\nEXIT LINE_CHANGE state");
                    state = straight3;
                    lineChangeState = halfStripDetected;    //Re-initiallize "lineChangeState" value
                  }
                break;
                case leftLineChange:
                case halfStripDetected:           //To avoid compilation warnings
                case breakingBeforeLineChange:    //To avoid compilation warnings
                case waitingForLineChange:        //To avoid compilation warnings
                case lineChangeHappening:         //To avoid compilation warnings
                break;
              };
            break;
            case breakingBeforeLineChange:    //To avoid compilation warnings
            case leftLineChange:              //To avoid compilation warnings
            case rightLineChange:             //To avoid compilation warnings
            break;
          };
        break;

        case straight3:
          printf("straight3\n");
          handle(NORMAL * line);
          if ((speed[0] > 100.0) || (speed[1] > 100.0) || (speed[2] > 100.0) || (speed[3] > 100.0))
            motor(DEAD, DEAD, DEAD, DEAD);
          else
            motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST); 
          if (leftWhiteSensors == 4){                    //Fisrt half strip was detected
            printf("FIRST_HALF_STRIP_DETECTED --> PREPARING RIGHT_LINE_CHANGE\n");
            initDirection = compass();
            handle(LOOSE * line);
            state = linechange3;
            determinedChange = leftLineChange;
          }
        break;

        case linechange3:
          printf("linechange3\n");
          switch (lineChangeState){
            case halfStripDetected:
              printf("HALF_STRIP_DETECTED ");
              handle(ULTRAULTRALOOSE * line);
              if ((speed[0] > 65) || (speed[1] > 65) || (speed[2] > 65) || (speed[3] > 65))
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
              else
                motor(slowerLineChange, slowerLineChange, slowerLineChange, slowerLineChange);
              if ((leftWhiteSensors != 4) && (rightWhiteSensors != 4))
                lineChangeState = waitingForLineChange;
            break;
            case waitingForLineChange:
              if (leftWhiteSensors == 4){                    //Fisrt half strip was detected
                printf("FIRST_HALF_STRIP_DETECTED --> PREPARING LEFT_LINE_CHANGE\n");
                initDirection = compass();
                handle(ULTRAULTRALOOSE * line);
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                state = linechange3;
                lineChangeState = halfStripDetected;
                determinedChange = leftLineChange;
              }
              else if (totalWhiteSensors == 0) {
                printf("LINE CHANGE HAPPENING \n");
                motor(BRAKE, BRAKE, BRAKE, BRAKE);
                handle(30);
                lineChangeState = lineChangeHappening;
              }
              else {
                printf("2) waitingForLineChange ");
                handle(ULTRAULTRALOOSE * nearLine);
                if ((speed[0] > 65) || (speed[1] > 65) || (speed[2] > 65) || (speed[3] > 65))
                  motor(BRAKE, BRAKE, BRAKE, BRAKE);
                else
                  motor(slowerLineChange, slowerLineChange, slowerLineChange, slowerLineChange);
              }
            break;
            case lineChangeHappening:
              switch (determinedChange){
                case leftLineChange:
                  if ((totalWhiteSensors == 0) && (fabs(fabs(initDirection) - fabs(angles[2])) < 0.31)){
                    printf("1) turning ");
                    motor(slowerLineChange, slowerLineChange, slowerLineChange, slowerLineChange);
                    handle(30);
                  }
                  else if ((totalWhiteSensors == 0) && (fabs(fabs(initDirection) - fabs(angles[2])) >= 0.31)){
                    printf("2) straightening ");
                    motor(slowerLineChange, slowerLineChange, slowerLineChange, slowerLineChange);
                    handle(-30);
                  }
                  else if ((totalWhiteSensors >= 1) && (fabs(fabs(initDirection) - fabs(angles[2])) > 0.42)){
                    printf("3a) prepairing follow ");
                    motor(slowerLineChange, slowerLineChange, slowerLineChange, slowerLineChange);
                    //motor(SLOW, SLOW, SLOW, SLOW);
                    handle(-30);
                  }
                  else if ((totalWhiteSensors >= 1) && (fabs(fabs(initDirection) - fabs(angles[2])) > 0.10)){
                    printf("3b) prepairing follow ");
                    //motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
                    handle(-output_pid);
                    motor(slowerLineChange, slowerLineChange, slowerLineChange, slowerLineChange);
                    //handle(LOOSE * line);
                  }
                  else {
                    printf("\nEXIT LINE_CHANGE state");
                    state = turns;
                    lineChangeState = halfStripDetected;    //Re-initiallize "lineChangeState" value
                  }
                break;
                case rightLineChange:
                case halfStripDetected:           //To avoid compilation warnings
                case breakingBeforeLineChange:    //To avoid compilation warnings
                case waitingForLineChange:        //To avoid compilation warnings
                case lineChangeHappening:         //To avoid compilation warnings
                break;
              };
            break;
            case breakingBeforeLineChange:    //To avoid compilation warnings
            case leftLineChange:              //To avoid compilation warnings
            case rightLineChange:             //To avoid compilation warnings
            break;
          };
        break;

        case turns:
          printf("turns\n");
          handle(-output_pid);
          if ((speed[0] > 70.0) || (speed[1] > 75.0) || (speed[2] > 75.0) || (speed[3] > 75.0))
            motor(DEAD, DEAD, DEAD, DEAD);
          else
            motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST);
          if (fabs(line) > 0.04){
            printf("TURNING\n");
            if ((speed[0] > 50.0) || (speed[1] > 50.0) || (speed[2] > 50.0) || (speed[3] > 50.0))
              motor(TURN_BRAKE, TURN_BRAKE, TURN_BRAKE, TURN_BRAKE);
            else
              motor(cornerIn, cornerIn, cornerIn, cornerIn);
          }
          if (totalWhiteSensors == 8){                                                 //Fisrt full strip was detected
            state = corner4;
            cornerState = stripDetected;
            initDirection = compass();
            lastTime = time();
          }
        break;

        case corner4:
          printf("corner4\n");
          switch (cornerState){
            case stripDetected:
              printf("FULL_STRIP_DETECTED ");
              initDirection = compass();
              handle(STRICT * line);
              if ((speed[0] > 40) || (speed[1] > 40) || (speed[2] > 40) || (speed[3] > 40))
                motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
              else
                motor(cornerIn, cornerIn, cornerIn, cornerIn);
              if ((totalWhiteSensors < 8) && (time() - lastTime > 0.1)){
                cornerState = waitingForCorner;
                printf("BREAKING_BEFORE_CORNER\n");
              }
            break;
            case waitingForCorner:
              if (totalWhiteSensors == 8){
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
                    printf("3b) rightCorner - Breaking ");
                    motor(ULTRABRAKE, ULTRABRAKE, ULTRABRAKE, ULTRABRAKE);
                    handle(-50);
                  }
                  else{
                    printf("3b) rightCorner ");
                    motor(DEAD, cornerIn, DEAD, cornerIn);
                    handle(-50);
                  }
                }
                else{
                  printf("\nEXIT CORNER state");
                  state = final;
                  cornerState = stripDetected;    //Re-initiallize "cornerState" value
                }
              }
            break;
            case breakingBeforeCorner:     //To avoid compilation warnings
            case leftCorner:              //To avoid compilation warnings
            break;
          }
        break;

        case final:
          printf("final\n");
          handle(NORMAL * line);
          motor(ULTRAFAST, ULTRAFAST, ULTRAFAST, ULTRAFAST); 
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
