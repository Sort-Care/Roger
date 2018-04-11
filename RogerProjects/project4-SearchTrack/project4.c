/*************************************************************************/
/* File:        project4.c                                               */
/* Description: User project #4                                          */
/*       01-2015 - empty project directory for project development       */
/*       04-2018 - added additional starter code                         */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int sample_gaze_direction(), compute_average_red_pixel();

#define N_ST_ACTIONS  2  // 0:SEARCH and 1:TRACK
#define SEA 0
#define TRCK 1
#define N_ST_DOF      3  // 0:BASE ROTATION, 1:LEFT EYE, 2:RIGHT EYE
#define LEFT_EYE 1
#define RIGHT_EYE 2


double recommended_setpoints[N_ST_ACTIONS][N_ST_DOF];

/* NEW RETURN STATUS FOR ALL ACTIONS:   defined in "include/control.h"   */
// enum {
//   NO_REFERENCE = 0,
//   TRANSIENT,
//   CONVERGED
// };
//  0-"NO_REFERENCE", 1-"TRANSIENT", 2-"CONVERGED"

int compute_average_red_pixel(roger, rc, lc)
Robot * roger;
double *rc;
double *lc;
{
    double res[NEYES];
    for(int i = 0; i < NEYES; i++){// for both eyes
            //loop through the image plane,
        int ul = 0, ur = 0;
        for(int j = 0; j < NPIXELS; j++){
            if(roger->image[i][ul][0] == 255){//left is found, need to find right
                if(roger->image[i][j][0] == 255){
                    ur = j;
                }else{
                    break;
                }
            }else{
                if(roger->image[i][j][0] == 255){
                    ul = j;
                    ur = ul;
                    continue;
                }else{
                    ul += 1;
                    ur += 1;
                }
            }
        }
        res[i] = (double)(ul + ur)/2;
    }
    
    *rc = res[RIGHT];
    *lc = res[LEFT];
    
    if (*rc < NPIXELS && *lc < NPIXELS) { // there are red pixels in both eyes
        return TRUE;
    }else{ // else
        return FALSE;     
    }
    
}

/*************************************************************************/
/* SEARCH controller -                                                   */
/* explore samples from the SEARCH distribution for the target red ball  */
/*   IMPLEMENTATION NOTES:                                               */
/*   (1) we will not include a means for SEARCH( )to retun "NO_REFERENCE */
/*************************************************************************/
int SEARCH(roger, time)
Robot * roger;
double time;
{
  double heading_error_base;
  static double search_heading;
  static int return_state = NO_REFERENCE;   
  /* static variables are preserved in memory between SEARCH( ) calls    */

  // SAMPLE A NEW REFERENCE HEADING
  if (return_state != TRANSIENT) {
    // sample a new gaze heading from the SEARCH distribution
    sample_gaze_direction(&search_heading);
    return_state = TRANSIENT;
  }

  else { // in transient
    /***********************************************************************/
    /* PROJECT4 PART I - complete the code to explore sampled              */
    /*    search_heading using base and eyes                               */
    /***********************************************************************/
    heading_error_base = search_heading - roger->base_position[THETA]; 
    while (heading_error_base > M_PI) heading_error_base -= 2.0 * M_PI;
    while (heading_error_base < -M_PI) heading_error_base += 2.0 * M_PI;

    
    // define new recommended setpoints for the base and the eyes
    recommended_setpoints[SEA][BASE]= search_heading;
    if (heading_error_base > M_PI/2 || heading_error_base < - M_PI/2) {
            //the search heading is out of sight
        recommended_setpoints[SEA][LEFT_EYE] = (heading_error_base > 0) ? M_PI/2 : -M_PI/2;
        recommended_setpoints[SEA][RIGHT_EYE] = (heading_error_base > 0) ? M_PI/2 : -M_PI/2;
    }else{
            //the search heading is in sight
        recommended_setpoints[SEA][LEFT_EYE] =  heading_error_base;
        recommended_setpoints[SEA][RIGHT_EYE] =  heading_error_base;
    }

    //fail fast
    if (fabs(roger->eye_theta[LEFT]-heading_error_base) < 0.1 &&
        fabs(roger->eye_theta[RIGHT]-heading_error_base) < 0.1){
        return_state = NO_REFERENCE;
    }
        //CONVERGE
    if ( fabs(heading_error_base) < 0.01) return_state = CONVERGED;
    /***********************************************************************/
    /* PROJECT4 PART I - END                                               */
    /***********************************************************************/
  }
  return(return_state);
}

/*************************************************************************/
// primitive TRACK controller - TRACK the red ball with base and eyes
/*************************************************************************/
int TRACK(roger, time)
Robot* roger;
double time;
{
  double ul, ur, error_eye[2], error_base;
  
  static int return_state = NO_REFERENCE;

  // control eyes independently and triangulate is ball is visible in both eyes
  if (compute_average_red_pixel(roger, &ur, &ul) == TRUE) {
    /***********************************************************************/
    /* PROJECT4 PART II - complete the code to TRACK the red ball          */
    /*    using base and eyes                                              */
    /***********************************************************************/
    error_eye[LEFT] = atan2((ul-63.5), 64.0);
    error_eye[RIGHT] = atan2((ur-63.5), 64.0);

    double l_tan = tan(roger->eye_theta[LEFT]);
    double r_tan = tan(roger->eye_theta[RIGHT]);
    
    error_base = atan2((l_tan + r_tan), 1.0);

    // define new recommended setpoints for the base and the eyes
    recommended_setpoints[TRCK][BASE]= roger->base_position[THETA] + error_base;
    recommended_setpoints[TRCK][LEFT_EYE] = roger->eye_theta[LEFT] + error_eye[LEFT];
    recommended_setpoints[TRCK][RIGHT_EYE] = roger->eye_theta[RIGHT] + error_eye[RIGHT];

    // check for CONVERGE
    if ((fabs(error_eye[LEFT]) < 0.1) && (fabs(error_eye[RIGHT]) < 0.1) &&
	(fabs(error_base) < 0.1)) {
      return_state = CONVERGED;
    }
    else { return_state = TRANSIENT; }
    
    /***********************************************************************/
    /* PROJECT4 PART II - END                                              */
    /***********************************************************************/
  }
  else {
    // No ball in view -> no reference
    return_state = NO_REFERENCE;
  }
  return(return_state);
}

/*************************************************************************/
// SEARCH/TRACK the red ball using primitive SEARCH and TRACK controllers
/*************************************************************************/
int SEARCHTRACK(roger, time)
Robot *roger;
double time;
{
  static int return_state = NO_REFERENCE;
  static int internal_state[2] = { NO_REFERENCE, NO_REFERENCE };
  //                               [0]->SEARCH    [1]->TRACK
  int i, state;
  
  //  printf("   in searchtrack: state=%d\n", return_state);
  
  /**********************************************************************/
  /* PROJECT4 PART III - the FSA for SEARCHTRACK                        */
  /*    internal_state=[ 0:SEARCH 1:TRACK ]                             */
  /**********************************************************************/
  internal_state[1] = SEARCH(roger,time);
  internal_state[0] = TRACK(roger, time);
  printf("sea %d\n", internal_state[1]);
  printf("trc %d\n", internal_state[0]);
  
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
         SEARCH(roger,time);
         roger->base_setpoint[THETA] = recommended_setpoints[SEA][BASE];
         roger->eyes_setpoint[LEFT] = recommended_setpoints[SEA][LEFT_EYE];
         roger->eyes_setpoint[RIGHT] = recommended_setpoints[SEA][RIGHT_EYE];
         return_state = NO_REFERENCE;
         break;
     case 1:                              //   NO_REF   - TRANSIENT
             //have target
         roger->base_setpoint[THETA] = recommended_setpoints[TRCK][BASE];
         roger->eyes_setpoint[LEFT] = recommended_setpoints[TRCK][LEFT_EYE];
         roger->eyes_setpoint[RIGHT] = recommended_setpoints[TRCK][RIGHT_EYE];
         return_state = CONVERGED;
         break;
     case 2:                              //   NO_REF   - CONVERGED
             //end
         return_state = CONVERGED;
         break;
     case 3:                              //  TRANSIENT -  NO_REF
             //moving to sample point
         roger->base_setpoint[THETA] = recommended_setpoints[SEA][BASE];
         roger->eyes_setpoint[LEFT] = recommended_setpoints[SEA][LEFT_EYE];
         roger->eyes_setpoint[RIGHT] = recommended_setpoints[SEA][RIGHT_EYE];
         return_state = NO_REFERENCE;
         break;
     case 4:                              //  TRANSIENT - TRANSIENT
             //find ball during moving to sample point
         roger->base_setpoint[THETA] = recommended_setpoints[TRCK][BASE];
         roger->eyes_setpoint[LEFT] = recommended_setpoints[TRCK][LEFT_EYE];
         roger->eyes_setpoint[RIGHT] = recommended_setpoints[TRCK][RIGHT_EYE];
         return_state = TRANSIENT;
         break;
     case 5:                              //  TRANSIENT - CONVERGED
             //stop search and end
         return_state = CONVERGED;
         break;
     case 6:                              //  CONVERGED -  NO_REF
             //new sample
         SEARCH(roger,time);
         return_state = NO_REFERENCE;
         break;
     case 7:                              //  CONVERGED - TRANSIENT
             //find ball, start track
         roger->base_setpoint[THETA] = recommended_setpoints[TRCK][BASE];
         roger->eyes_setpoint[LEFT] = recommended_setpoints[TRCK][LEFT_EYE];
         roger->eyes_setpoint[RIGHT] = recommended_setpoints[TRCK][RIGHT_EYE];
         return_state = TRANSIENT;
         break;
     case 8:                              //  CONVERGED - CONVERGED
             //sample point has a ball there
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

/*************************************************************************/
/* project development interface - called by GUI                         */
/*************************************************************************/
void project4_control(roger,time)
Robot * roger;
double time;
{

  printf("SEARCHTRACK state=%d\n", SEARCHTRACK(roger, time));

  /* FILE *fp = fopen("./berror.bat", "a"); */
  /* double base_error = roger->base_setpoint[THETA] - roger->base_position[THETA]; */
  /* fprintf(fp, "%f\t%f\n", time, base_error); */
  /* fclose(fp); */
}


/************************************************************************/
void project4_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project4_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project4_visualize(roger)
Robot* roger;
{ }
