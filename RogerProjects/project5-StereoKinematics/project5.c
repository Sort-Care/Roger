/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #5 - empty project directory for project    */
/*              developement                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "models.h"
#include "matrix_math.h"

#define N_ST_ACTIONS  2  // 0:SEARCH and 1:TRACK
#define SEA 0
#define TRCK 1
#define N_ST_DOF      3  // 0:BASE ROTATION, 1:LEFT EYE, 2:RIGHT EYE
#define LEFT_EYE 1
#define RIGHT_EYE 2

int SEARCH(), TRACK(), compute_average_red_pixel();
void draw_observation();


void printObs(Observation *obs){
    printf("X:%f\nY:%f\nCov:%f\t%f\n%f\t%f\n",
           obs->pos[0], obs->pos[1],
           obs->cov[0][0], obs->cov[0][1],
           obs->cov[1][0], obs->cov[1][1]);
    
}

void printMatrix(char *s, double m[2][2]){
    printf("%s\n%f\t%f\n\%f\t%f\n", s,
           m[0][0],m[0][1],
           m[1][0],m[1][1]);
}

double recommended_setpoints[N_ST_ACTIONS][N_ST_DOF];

void stereo_observation(Robot * roger, double time, Observation * obs){
    
/*
 * Function:  stereo_observation
 * --------------------
 *  Estimate position of signal source in the (x,y) plane
 *
 *
 *  roger: Robot
 *  time: Double
 *
 *  returns: (x,y) coordinates
 */
    double ur, ul;
    double gm_l, gm_r, x_b, y_b, x_w, y_w;
    
    if (compute_average_red_pixel(roger, &ur, &ul) == TRUE) {
//        gm_l = roger->eye_theta[LEFT];
//        gm_r = roger->eye_theta[RIGHT];
        gm_l = atan2((ul-63.5), 64.0) + roger->eye_theta[LEFT];
        gm_r = atan2((ur-63.5), 64.0) + roger->eye_theta[RIGHT];

//        gm_l = fabs(gm_l);
//        gm_r = fabs(gm_r);
        
        

        x_b = 2 * BASELINE * (cos(gm_r)* cos(gm_l))/(sin(gm_r - gm_l));
        y_b = BASELINE +  2 * BASELINE * (cos(gm_r) * sin(gm_l))/(sin(gm_r - gm_l));


        double wTb[4][4], ref_w[4];
        double ref_b[4] = {x_b, y_b, 0.0, 1.0};
        construct_wTb(roger->base_position, wTb);

            //homogeneous transform to world frame
        matrix_mult(4,4, wTb, 1, ref_b, ref_w);
        obs->pos[0] = ref_w[0];
        obs->pos[1] = ref_w[1];
        double cov_b[2][2], cov_w[2][2], wRb[2][2];
    
        double com_fac = 2 * BASELINE / (sin(gm_r - gm_l)*sin(gm_r - gm_l));
        cov_b[0][0] = com_fac * pow(cos(gm_r),2);
        cov_b[0][1] = com_fac * -pow(cos(gm_l), 2);
        cov_b[1][0] = com_fac * sin(gm_r) * cos(gm_r);
        cov_b[1][1] = com_fac * -sin(gm_l) * cos(gm_l);

        double base_error = roger->base_position[THETA];
        wRb[0][0] = cos(base_error);
        wRb[0][1] = -sin(base_error);
        wRb[1][0] = sin(base_error);
        wRb[1][1] = cos(base_error);

            //rotate Jacobian
        matrix_mult(2,2, wRb, 2, cov_b, cov_w);
        
        double tran_cov[2][2];
        matrix_transpose(2,2,cov_w, tran_cov);

        
        double rcov[2][2];
        matrix_mult(2,2,cov_w,2,tran_cov,rcov);
        memcpy(obs->cov, rcov, sizeof(obs->cov));
        
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j<2; ++j) {
                obs->cov[i][j] *= 0.01562 * 0.01562;
            }
        }

        
        obs->time = time;
    }
}






int SEARCHTRACKLOCALIZE(roger, time)
Robot *roger;
double time;
{
    static int return_state = NO_REFERENCE;
    static int internal_state[2] = { NO_REFERENCE, NO_REFERENCE };
        //                               [0]->SEARCH    [1]->TRACK
    int i, state;
    Observation obs;
  
        //  printf("   in searchtrack: state=%d\n", return_state);
  
        /**********************************************************************/
        /* PROJECT4 PART III - the FSA for SEARCHTRACK                        */
        /*    internal_state=[ 0:SEARCH 1:TRACK ]                             */
        /**********************************************************************/
    internal_state[1] = SEARCH(roger,time);
    internal_state[0] = TRACK(roger, time);
    /* printf("sea %d\n", internal_state[1]); /\*  *\/ */
    /* printf("trc %d\n", internal_state[0]); */
  
    state = internal_state[1]*3 + internal_state[0];
    switch (state) {
            // Based on the state feedback call SEARCH(), TRACK() or both to
            // update the recommended_setpoints. 
            // Write the appropriate recommended_setpoints[i] to the setpoints
            // for motor units
            // based on the status feedback
            //    roger->base_setpoint[THETA] = ...
            //    roger->eyes_setpoint[LEFT] = ...
            //    roger->eyes_setpoint[RIGHT] = ...

            //   SEARCH   -  TRACK
            // ----------------------
        case 0:                              //   NO_REF   -  NO_REF
                //no target, sample and search
            printf("Case 0\n");    
            SEARCH(roger,time);
            roger->base_setpoint[THETA] = recommended_setpoints[SEA][BASE];
            roger->eyes_setpoint[LEFT] = recommended_setpoints[SEA][LEFT_EYE];
            roger->eyes_setpoint[RIGHT] = recommended_setpoints[SEA][RIGHT_EYE];
            return_state = NO_REFERENCE;
            break;
        case 1:                              //   NO_REF   - TRANSIENT
                //have target
            printf("Case 1\n");    
            roger->base_setpoint[THETA] = recommended_setpoints[TRCK][BASE];
            roger->eyes_setpoint[LEFT] = recommended_setpoints[TRCK][LEFT_EYE];
            roger->eyes_setpoint[RIGHT] = recommended_setpoints[TRCK][RIGHT_EYE];
            stereo_observation(roger,time, &obs);
//            printObs(&obs);
            return_state = TRANSIENT;
            break;
        case 2:                              //   NO_REF   - CONVERGED
                //end
//            draw_observation(stereo_observation(roger,time));
            printf("Case 2\n");    
            return_state = CONVERGED;
            break;
        case 3:                              //  TRANSIENT -  NO_REF
                //moving to sample point
            printf("Case 3\n");    
            roger->base_setpoint[THETA] = recommended_setpoints[SEA][BASE];
            roger->eyes_setpoint[LEFT] = recommended_setpoints[SEA][LEFT_EYE];
            roger->eyes_setpoint[RIGHT] = recommended_setpoints[SEA][RIGHT_EYE];
            stereo_observation(roger,time, &obs);
            return_state = NO_REFERENCE;
            break;
        case 4:                              //  TRANSIENT - TRANSIENT
                //find ball during moving to sample point
            printf("Case 4\n");    
            roger->base_setpoint[THETA] = recommended_setpoints[TRCK][BASE];
            roger->eyes_setpoint[LEFT] = recommended_setpoints[TRCK][LEFT_EYE];
            roger->eyes_setpoint[RIGHT] = recommended_setpoints[TRCK][RIGHT_EYE];
            stereo_observation(roger,time, &obs);
            return_state = TRANSIENT;
            break;
        case 5:                              //  TRANSIENT - CONVERGED
                //stop search and end
//            draw_observation(stereo_observation(roger,time));
            printf("Case 5\n");    
            stereo_observation(roger,time, &obs);
            return_state = CONVERGED;
            break;
        case 6:                              //  CONVERGED -  NO_REF
                //new sample
            printf("Case 6\n");    
            SEARCH(roger,time);
            return_state = NO_REFERENCE;
            break;
        case 7:                              //  CONVERGED - TRANSIENT
                //find ball, start track
            printf("Case 7\n");    
            roger->base_setpoint[THETA] = recommended_setpoints[TRCK][BASE];
            roger->eyes_setpoint[LEFT] = recommended_setpoints[TRCK][LEFT_EYE];
            roger->eyes_setpoint[RIGHT] = recommended_setpoints[TRCK][RIGHT_EYE];
//            draw_observation(stereo_observation(roger,time));
            stereo_observation(roger,time, &obs);
            return_state = TRANSIENT;
            break;
        case 8:                              //  CONVERGED - CONVERGED
                //sample point has a ball there
//            draw_observation(stereo_observation(roger,time));
            printf("Case 8\n");    
            stereo_observation(roger,time, &obs);
//            printObs(&obs);
            return_state = CONVERGED;
            break;
         
        default:
            internal_state[0] = SEARCH(roger,time);
            internal_state[1] = TRACK(roger, time);
            return_state = NO_REFERENCE;
            break;
    }
        /* decide on the return logic for your implementation of SEACHTRACK( ) */
        /*   return value in { NO_REFERENCE, TRANSIENT, CONVERGED } to the     */
        /*   calling procedure "project4_control( )"                           */
    return(return_state);

        /***********************************************************************/
        /* PROJECT4 PART III - END                                             */
        /***********************************************************************/
}






void project5_control(roger, time)
Robot* roger;
double time;
{
    printf("SEARCHTRACK state=%d\n", SEARCHTRACKLOCALIZE(roger, time));
    
}

/************************************************************************/
void project5_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project5_enter_params() 
{
    printf("Project 5 enter_params called. \n");
}


//function called when the 'visualize' button on the gui is pressed
void project5_visualize(roger)
Robot* roger;
{
    Observation obs;
    double time = 0.0;
    
    stereo_observation(roger,time, &obs);
    printObs(&obs);
    draw_observation(obs);

/* /\*     X:0.785347 *\/ */
/* /\* Y:-0.837563 *\/ */
/* /\* Cov:0.017818	-0.019014 *\/ */
/* /\* -0.019014	0.020639 *\/ */
/*     Observation obs2; */
/*     obs2.pos[0] = 0.785347; */
/*     obs2.pos[1] = -0.837563; */
/*     obs2.cov[0][0] = 0.017818; */
/*     obs2.cov[0][1] = -0.019014; */
/*     obs2.cov[1][0] = -0.019014; */
/*     obs2.cov[1][1] = 0.020639; */
/*     draw_observation(obs2); */

/* /\*     X:-0.018877 *\/ */
/* /\* Y:-1.275725 *\/ */
/* /\* Cov:0.000214	0.000844 *\/ */
/* /\* 0.000844	0.051806 *\/ */
/*     Observation obs3; */
/*     obs3.pos[0] = -0.018877; */
/*     obs3.pos[1] = -1.275725; */
/*     obs3.cov[0][0] =0.000214; */
/*     obs3.cov[0][1] = 0.000844; */
/*     obs3.cov[1][0] = 0.000844; */
/*     obs3.cov[1][1] = 0.051806; */
/*     draw_observation(obs3); */
    
    
}
