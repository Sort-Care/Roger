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
void construct_transfer(theta, x, y, T)
double theta; // (x,y,theta)
double x;
double y;
double T[4][4];
{
    double s0, c0;
    s0 = sin(theta);
    c0 = cos(theta);

    T[0][0] = c0;  T[0][1] = -s0; T[0][2] = 0.0; T[0][3] = x;
    T[1][0] = s0;  T[1][1] = c0;  T[1][2] = 0.0; T[1][3] = y;
    T[2][0] = 0.0; T[2][1] = 0.0; T[2][2] = 1.0; T[2][3] = 0.0;
    T[3][0] = 0.0; T[3][1] = 0.0; T[3][2] = 0.0; T[3][3] = 1.0;
}

double* fwd_arm_kinematics(roger, limb, res)
Robot * roger;
int limb;
double *res;
{
    double wTb, bTone[4][4], oneTtwo[4][4], twoTthree[4][4];
    construct_transfer(roger->arm_theta[limb][0], 0.0, 0.0, bTone);
    construct_transfer(roger->arm_theta[limb][1], L_ARM1, 0.0, oneTtwo);
    construct_transfer(0.0, L_ARM2, 0.0, twoTthree);

        //coordinates of endpoint in endpoint frame
    double ref_3[4];
    ref_3[0] = 0.0;
    ref_3[1] = 0.0;
    ref_3[2] = 0.0;
    ref_3[3] = 1.0;

    double ref_2[4], ref_1[4];
    matrix_mult(4, 4, twoTthree, 1, ref_3, ref_2);

    matrix_mult(4, 4, oneTtwo, 1, ref_2, ref_1);

    matrix_mult(4, 4, bTone, 1, ref_1, res);
    
    return res;
    
}

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


//        printf("1p: %f, 1m: %f, 2p: %f, 2m: %f\n", theta1_plus, theta1_minus, theta2_plus, theta2_minus);
        
        
        if(limb == LEFT){ //left arm
                /*     if(c2 > 0){ */
            if(theta1_minus >= -M_PI/4 && theta1_minus <= 5 * M_PI/4){
                roger->arm_setpoint[limb][0] = theta1_minus;
                roger->arm_setpoint[limb][1] = theta2_minus;
                
                return TRUE;
            }else{
                printf("x = %6.4lf  y = %6.4lf OUT OF REACH\n", ref_b[X], ref_b[Y]);
                return FALSE;
            }
                /*     }else{ */
                /*         roger->arm_setpoint[limb][0] = theta1_plus; */
                /*         roger->arm_setpoint[limb][1] = theta2_; */
        }else{//right arm
            if(theta1_plus >= - 5 * M_PI/4 && theta1_plus <= M_PI/4){
                roger->arm_setpoint[limb][0] = theta1_plus;
                roger->arm_setpoint[limb][1] = theta2_plus;
            }else{
                printf("x = %6.4lf  y = %6.4lf OUT OF REACH\n", ref_b[X], ref_b[Y]);
                return FALSE;
            }
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
{
    /* FILE *af = fopen("./ag.dat", "a");  */
    /* double res[4]; */
    /* fwd_arm_kinematics(roger,LEFT, res); */
    /* fprintf(af,"%f\t%f\t%f\n", time, res[0], res[1]+ARM_OFFSET); */
    /* fclose(af); */
    
}


void project2_reset(roger)
Robot* roger;
{   
}

void project2_enter_params() 
{
    printf("Project 6 enter_params called. \n");
}

void project2_visualize(roger)
Robot* roger;
{ }


