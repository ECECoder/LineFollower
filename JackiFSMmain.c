#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
//#include "../inc/PWM.h"
#include "LaunchPad.h"
//#include "../inc/TExaS.h"
//#include "../inc/AP.h"
//#include "../inc/UART0.h"
#include "BumpInt.h"
#include "Reflectance.h"
#include "Motor.h"

uint16_t L_Motor;
uint16_t R_Motor;
int32_t Output;
uint8_t Data;
int32_t Dist;
uint8_t NS;

// Linked data structure
struct State {
  uint16_t L_Duty;             // 0-14998
  uint16_t R_Duty;             // 0-14998
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[5]; // Next if 3-bit input is 0-7
};
typedef const struct State State_t;

#define Center &fsm[0]
#define Left   &fsm[1]
#define Right  &fsm[2]
#define Stop   &fsm[3]
State_t fsm[4]={
  {5000, 5000, 100, { Center, Left, Right, Stop, Stop}},   // Center
  {3000, 1000, 100, { Center, Left, Right, Stop, Stop }},  // Left of line (turn right)
  {1000, 3000, 100, { Center, Left, Right, Stop, Stop }},  // Right of line (turn left)
  {  0,   0, 500, { Stop,   Stop, Stop,  Stop, Stop }}   // Right of line (turn left)
};

State_t *Spt;  // pointer to the current state

void PORT4_IRQHandler(void){
    // write this as part of Lab 14
    P4->IFG &= ~0xED; // clear flags
    Spt = Stop;
//    if(Spt == Stop){
//        Spt = Center;
//    }
//    else{
//        Spt = Stop;
//    }
}

uint8_t nextStateIDX(int32_t D){
    if(D<=-14300){
        return 1;
    }
    if(D>=14300){
        return 2;
    }
    return 0;
}


void main(void){
    // write this as a robot challenge
    Clock_Init48MHz();
    Motor_Init();
    LaunchPad_Init();
    BumpInt_Init();
    Reflectance_Init();
    Spt = Center;

    while(1){
        L_Motor = Spt->L_Duty;            // set output from FSM
        R_Motor = Spt->R_Duty;            // set output from FSM
        Motor_Forward(L_Motor,R_Motor);
        Clock_Delay1ms(Spt->delay);   // wait
        Data = Reflectance_Read(1000);
        Dist = Reflectance_Position(Data);    // read sensors
        NS = nextStateIDX(Dist);

        Spt = Spt->next[NS];       // next depends on input and state

  }

}
