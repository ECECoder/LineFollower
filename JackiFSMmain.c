#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
//#include "../inc/PWM.h"
//#include "LaunchPad.h"
//#include "../inc/TExaS.h"
//#include "../inc/AP.h"
//#include "../inc/UART0.h"
#include "BumpInt.h"
#include "Reflectance.h"
#include "Motor.h"

uint16_t L_Motor;
uint16_t R_Motor;
//int32_t Output;
uint8_t Data;
int32_t Dist;
uint8_t NS;

// Linked data structure
struct State {
  uint16_t L_Duty;             // 0-14998
  uint16_t R_Duty;             // 0-14998
  uint32_t delay;              // time to delay in 1ms
  uint8_t function;
  const struct State *next[6]; // Next if 3-bit input is 0-7
};
typedef const struct State State_t;

#define L_Center &fsm[0]
#define R_Center &fsm[1]
#define Left   &fsm[2]
#define H_left &fsm[3]
#define Right  &fsm[4]
#define H_right &fsm[5]
#define L_Lost &fsm[6]
#define R_Lost &fsm[7]
#define Stop   &fsm[8]
State_t fsm[9]={
  {6000, 6000,  20, 0, { L_Center, Left, H_left, Right, H_right, L_Lost }},   // Left Center
  {6000, 6000,  20, 0, { R_Center, Left, H_left, Right, H_right, R_Lost }},   // Right Center
  {6000, 5000,  60, 0, { L_Center, Left, H_left, Right, H_right, L_Center }},  // Left of line (turn right)
  {3500, 1500,  60, 1, { L_Center, Left, H_left, Right, H_right, L_Lost }},  // H_left of line (turn hard right)
  {5000, 5000,  60, 0, { R_Center, Left, H_left, Right, H_right, R_Center }},  // Right of line (turn left)
  {1500, 3500,  60, 2, { R_Center, Left, H_left, Right, H_right, R_Lost }},  // H_right of line (turn hard left)
  {4000, 3500, 100, 1, { L_Center, Left, H_left, Right, H_right, L_Lost }},  // Left lost
  {3500, 4000, 100, 2, { R_Center, Left, H_left, Right, H_right, R_Lost }},  // Right lost
  {   0,    0, 500, 0, { Stop,   Stop, Stop,  Stop, Stop, Stop }}   // Stop
};

State_t *Spt;  // pointer to the current state

void PORT4_IRQHandler(void){
    // write this as part of Lab 14
    P4->IFG &= ~0xED; // clear flags
    Spt = Stop;
}

uint8_t nextStateIDX(int32_t D, uint8_t bits){
    // Left
    if(D<=-14300 && D>=-23800){
        return 1;
    }
    // Hard Left
    if(D<-23800){
        return 2;
    }
    // Right
    if(D>=14300 && D<=23800){
        return 3;
    }
    // Hard Right
    if(D>23800){
        return 4;
    }
    // Stop
    if(bits == 0x00000000){
        return 5;
    }
    return 0;
}


void main(void){
    // write this as a robot challenge
    Clock_Init48MHz();
    Motor_Init();
    //LaunchPad_Init();
    BumpInt_Init();
    Reflectance_Init();
    Spt = L_Center;

    while(1){
        L_Motor = Spt->L_Duty;            // set output from FSM
        R_Motor = Spt->R_Duty;            // set output from FSM
        if(Spt->function == 1){
            Motor_Right(L_Motor,R_Motor);
        }
        else if(Spt->function == 2){
            Motor_Left(L_Motor,R_Motor);
        }
        else{
            Motor_Forward(L_Motor,R_Motor);
        }
        Clock_Delay1ms(Spt->delay);   // wait
        Data = Reflectance_Read(1000);
        Dist = Reflectance_Position(Data);    // read sensors
        NS = nextStateIDX(Dist, Data);

        Spt = Spt->next[NS];       // next depends on input and state

  }

}
