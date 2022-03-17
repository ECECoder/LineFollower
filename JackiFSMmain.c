#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/TExaS.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/Bump.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"

// Linked data structure
struct State {
  uint16_t L_Duty;             // 0-14998
  uint16_t R_Duty;             // 0-14998
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 3-bit input is 0-7
};
typedef const struct State State_t;

#define Center &fsm[0]
#define Left   &fsm[1]
#define Right  &fsm[2]
State_t fsm[3]={
  {500, 500, { Right, Left,   Right,  Center }},  // Center
  {500, 200, { Left,  Center, Right,  Center }},  // Left of line (turn right)
  {200, 500, { Right, Left,   Center, Center }}   // Right of line (turn left)
};
State_t *Spt;  // pointer to the current state
uint16_t L_Motor;
uint16_t R_Motor;
int32_t Output;

void main(void){
    // write this as a robot challenge
    Clock_Init48MHz();
    Motor_Init();
    LaunchPad_Init();
    Reflectance_Init();
    Spt = Center;

    while(1){
        L_Motor = Spt->L_Duty;            // set output from FSM
        R_Motor = Spt->R_Duty;            // set output from FSM
        Motor_Forward(L_Motor,R_Motor));
        Clock_Delay1ms(Spt->delay);   // wait
        Input = LaunchPad_Input();    // read sensors
        Spt = Spt->next[Input];       // next depends on input and state

  }

}
