/*
 * Deal_Turn.h
 *
 *  Created on: 2026Äê2ÔÂ23ÈÕ
 *      Author: kedaya
 */

#ifndef CODE_DEAL_TURN_H_
#define CODE_DEAL_TURN_H_
enum T_Flag
{
  NO,
  YES,
};
enum T_Turn
{
    T2_Left,
    T2_Right,
};
enum T_Flag2
{
    C_Left,
    C_Right,
    C_Top,
};
extern uint8_t road ;
extern uint8_t Turn1_Flag ;
extern uint8_t Turn1_Flag2;
extern uint8_t Turn2_Flag ;
extern uint8_t Turn3_Flag ;
extern uint8_t Flag_Count3 ;
extern enum T_Flag T_Turn1;
extern enum T_Turn T_Turn2 ;
extern enum T_Flag2 T_Turn3 ;
extern uint8_t DIST_read;
void Flag_Monitor(void);
void Turn_Control(void);
void Select_Flag(void);


#endif /* CODE_DEAL_TURN_H_ */
