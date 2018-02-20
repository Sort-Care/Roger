/*************************************************************************/
/* File: project2.c                                                      */
/* Description: Kinematic function for Arms                              */
/*                 fwd_arm_kinematics() - maps joint angles in the arm   */
/*                    into endpoint coordinates in the base frame        */
/*                 inv_arm_kinematics() - maps endpoint position for the */
/*                    specified arm in base frame into two possible joint*/
/*                    angle configurations and selects one of the two    */
/*                    writes arm cspace setpoints                        */
/* Date:        1-2015                                                 */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

/*************************************************************************/
/*** PROJECT #2 - FORWARD KINEMATICS: MAP (THETA1, THETA2) TO (X,Y)_B  ***/
/***              INVERSE KINEMATICS: MAP (X,Y)_B TO (THETA1, THETA2)  ***/
/*************************************************************************/
void fwd_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{ }

int inv_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{
    double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

    double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
    double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
    double theta1_plus, theta1_minus;

        // input (x,y) is in world frame coordinates - map it into the base frame
    construct_wTb(roger->base_position, wTb);
        //HT_invert(in, out)
    HT_invert(wTb,bTw);

  
    ref_w[0] = x;
    ref_w[1] = y;
    ref_w[2] = 0.0;
    ref_w[3] = 1.0;

    matrix_mult(4, 4, bTw, 1, ref_w, ref_b);
  
    if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
    else ref_b[Y] += ARM_OFFSET;

        //printf("%f, %f",ref_b[X], ref_b[Y]);
        //GIVEN the endpoint position goal in ref_b, determine whether it is in the reach.
    r2 = ref_b[X] * ref_b[X] + ref_b[Y] * ref_b[Y];
    c2 = (r2 - L_ARM1 * L_ARM1 - L_ARM2 * L_ARM2) / (2 * L_ARM1 * L_ARM2);
    if(c2 >= -1 && c2 <= 1){
        s2_plus = sqrt(1 - c2 * c2);
        s2_minus = - sqrt(1 - c2 * c2);
        
        theta2_plus = atan2(s2_plus, c2);
        theta2_minus = atan2(s2_minus,c2);

        k1 = L_ARM1 + L_ARM2 * c2;
        k2_plus = L_ARM2 * s2_plus;
        k2_minus = L_ARM2 * s2_minus;

        alpha_plus = atan2(k2_plus, k1);
        alpha_minus = atan2(k2_minus, k1);
        
        theta1_plus = atan2(ref_b[Y], ref_b[X]) - alpha_plus;
        theta1_minus = atan2(ref_b[Y], ref_b[X]) - alpha_minus;


        printf("1p: %f, 1m: %f, 2p: %f, 2m: %f\n", theta1_plus, theta1_minus, theta2_plus, theta2_minus);
        
        
        if(limb == LEFT){
            /*     if(c2 > 0){ */
            roger->arm_setpoint[limb][0] = theta1_minus;
            roger->arm_setpoint[limb][1] = theta2_minus;
            /*     }else{ */
            /*         roger->arm_setpoint[limb][0] = theta1_plus; */
            /*         roger->arm_setpoint[limb][1] = theta2_; */
        }else{
            roger->arm_setpoint[limb][0] = theta1_plus;
            roger->arm_setpoint[limb][1] = theta2_plus;
        }
        
            /*     roger->arm_setpoint[limb][0] = theta2_minus; */
            /*     roger->arm_setpoint[limb][1] = theta1_plus; */
            /* } */
        return TRUE;
      
    }else{//out of reach
        printf("x = %6.4lf  y = %6.4lf OUT OF REACH\n", ref_b[X], ref_b[Y]);
        return FALSE;
    }
}

/*************************************************************************/
/* project development interface - called by GUI                         */
/*************************************************************************/
/* executed automatically when                                           */
/* control mode = PROJECT2; input mode = BALL INPUTS                     */
void project2_control(roger, time)
Robot * roger;
double time;
{ }


void project2_reset(roger)
Robot* roger;
{ }

void project2_enter_params() 
{
    printf("Project 6 enter_params called. \n");
}

void project2_visualize(roger)
Robot* roger;
{ }


