/*************************************************************************/
/* File:        project3.c                                               */
/* Description: User project #3 - empty project directory for project    */
/*              developement                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

void compute_average_location_of_red_pixels(roger, res)
Robot* roger;
double res[NEYES];
{

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
    
}


void project3_control(roger, time)
Robot* roger;
double time;
{
    double result[NEYES];
    compute_average_location_of_red_pixels(roger,result);
        //printf("%f, %f\n", result[0], result[1]);

    for(int i = 0; i < NEYES; i++){
        if(result[i] < NPIXELS){//out of sight
            double theta = atan2(result[i]-63.5, 64);
            roger->eyes_setpoint[i] = theta + roger->eye_theta[i];
        }else{
            roger->eyes_setpoint[i] = 0.0;
            continue;
        }
    }
}

/************************************************************************/
void project3_reset(roger)
Robot* roger;
{}

// prompt for and read user customized input values
void project3_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project3_visualize(roger)
Robot* roger;
{ }


