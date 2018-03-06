/*************************************************************************/
/* File:        project1.c                                               */
/* Description: PD control analogs of the biological motor units for     */
/*              every degree of freedom: eyes, arms, base rotate and     */
/*              base translate --- motor units execute every simulated   */
/*              millisecond and are never disengaged. Higher-level       */
/*              control applications submit sequences of setpoints to    */
/*              combinations of motorunits.                              */
/* Date:        1-2015                                                   */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

void update_setpoints();

/*************************************************************************/
/* PROJECT #1 - COMPLETE THE FOLLOWING CONTROL PROCEDURES                */
// gains for the PD controllers EYES
double Kp_eye = 20.5;
double Kd_eye = 0.338;  //0.04
double passive_Kd_eye = 0.001;

// ARMS 
double Kp_arm = 10;  //
double Kd_arm = 2.5;  //0.8
double passive_Kd_arm = 1.0;

// BASE TRANSLATION
double Kp_base_trans = 25.0; // to make it move fast enough
double Kd_base_trans = 14.5; 
double passive_Kd_base_trans = 2.0;

// BASE ROTATION
double Kp_base_rot =  8.0;
double Kd_base_rot = 4.8;
double passive_Kd_base_rot = 1.0;

//Uniform Gains
double Kp = 5.0;
double Kd = 0.5;
/*************************************************************************/

/* PROJECT #1.1 - PD CONTROLLER FOR THE EYES                             */
/* setpoints are joint angle values in radians for the eyes              */
void PDController_eyes(roger, time)
Robot * roger;
double time;
{
    int i;
    double theta_error, theta_dot_error;

    for (i = 0; i < NEYES; i++) {
        theta_error = roger->eyes_setpoint[i] - roger->eye_theta[i];
        theta_dot_error = 0.0 - roger->eye_theta_dot[i];
        if (ACTUATE_EYES) {
                // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE EYE
                // USING YOUR GAINS
                //torque = -k*x - B*x_dot
                //theta_error * Kp_eye + passive_Kd_eye * theta_dot_error
            roger->eye_torque[i] = theta_error * Kp_eye + Kd_eye * theta_dot_error;

                /*
                  output under- over- critically damped responses
                */

                /* FILE *feye; */
                /* feye = fopen("./eye_theo.dat" , "a"); */
                /* fprintf(feye,"%f\t%f\n", time, roger->eyes_setpoint[0] - roger->eye_theta[0]); */
            
            
        } 
        else roger->eye_torque[i] = passive_Kd_eye*theta_dot_error;
    }
}


double PDLeftEye(roger, time)
Robot * roger;
double time;
{
    double theta_error, theta_dot_error, res;
    theta_error = roger->eyes_setpoint[LEFT] - roger->eye_theta[LEFT];
    theta_dot_error = 0.0 - roger->eye_theta_dot[LEFT];
    if (ACTUATE_EYES){
        res = theta_error * Kp_eye + Kd_eye * theta_dot_error;
    }
    else res = passive_Kd_eye * theta_dot_error;
    return res;
}

double PDRightEye(roger, time)
Robot * roger;
double time;
{
    double theta_error, theta_dot_error;
    theta_error = roger->eyes_setpoint[RIGHT] - roger->eye_theta[RIGHT];
    theta_dot_error = 0.0 - roger->eye_theta_dot[RIGHT];
    if (ACTUATE_EYES){
        roger->eye_torque[RIGHT] = theta_error * Kp_eye + Kd_eye * theta_dot_error;
    }
    else roger->eye_torque[RIGHT] = passive_Kd_eye * theta_dot_error;
    return roger->eye_torque[RIGHT];
}

/* PROJECT #1.2 - PD CONTROLLER FOR THE ARMS                             */
/* setpoints - joint angles in radians for the shoulders and elbows      */
void PDController_arms(roger, time)
Robot * roger;
double time;
{
    int i, j;
    double theta_error, theta_dot_error;

    for (i=LEFT; i<=RIGHT; ++i) {
        for (j=0; j<NARM_JOINTS; ++j) {

                // roger->arm_setpoint[LEFT][j] = M_PI/2;
            
            theta_error = roger->arm_setpoint[i][j] - roger->arm_theta[i][j];
            theta_dot_error = 0.0 - roger->arm_theta_dot[i][j];

            while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
            while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

                // tune kp_arm and kd_arm by changing their value using enter_params()
            if (ACTUATE_ARMS) {
                    // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE ARM
                    // USING YOUR GAINS
                roger->arm_torque[i][j] = theta_error * Kp_arm + Kd_arm * theta_dot_error;

                    /* FILE *farm; */
                    /* farm = fopen("./arm.dat" , "a"); */
                    /* double q1_error = roger->arm_setpoint[LEFT][0] - roger->arm_theta[LEFT][0]; */
                    /* double q2_error = roger->arm_setpoint[LEFT][1] - roger->arm_theta[LEFT][1]; */
                    /* fprintf(farm,"%f\t%f\t%f\n",time, q1_error, q2_error); */
                    /* fclose(farm); */
                
                
            }
            else {
                roger->arm_torque[i][j] = passive_Kd_arm * theta_dot_error;
            }
        }
    }
}

double PDLeftShoulder(roger, time)
Robot * roger;
double time;
{
    double theta_error, theta_dot_error;
    theta_error = roger->arm_setpoint[LEFT][0] - roger->arm_theta[LEFT][0];
    theta_dot_error = 0.0 - roger->arm_theta_dot[LEFT][0];
    while(theta_error > M_PI) theta_error -= 2.0 * M_PI;
    while(theta_error < - M_PI) theta_error += 2.0 * M_PI;
    if(ACTUATE_ARMS){
        roger->arm_torque[LEFT][0] = theta_error * Kp_arm + Kd_arm * theta_dot_error;
    }else{
        roger->arm_torque[LEFT][0] = theta_error * Kp_arm + Kd_arm * theta_dot_error;
    }
    return roger->arm_torque[LEFT][0];
}

double PDLeftElbow(roger, time)
Robot * roger;
double time;
{
    double theta_error, theta_dot_error;
    theta_error = roger->arm_setpoint[LEFT][1] - roger->arm_theta[LEFT][1];
    theta_dot_error = 0.0 - roger->arm_theta_dot[LEFT][1];
    while(theta_error > M_PI) theta_error -= 2.0 * M_PI;
    while(theta_error < - M_PI) theta_error += 2.0 * M_PI;
    if(ACTUATE_ARMS){
        roger->arm_torque[LEFT][1] = theta_error * Kp_arm + Kd_arm * theta_dot_error;
    }else{
        roger->arm_torque[LEFT][1] = theta_error * Kp_arm + Kd_arm * theta_dot_error;
    }
    return roger->arm_torque[LEFT][1];
}


double PDRightShoulder(roger, time)
Robot * roger;
double time;
{
    double theta_error, theta_dot_error;
    theta_error = roger->arm_setpoint[RIGHT][0] - roger->arm_theta[RIGHT][0];
    theta_dot_error = 0.0 - roger->arm_theta_dot[RIGHT][0];
    while(theta_error > M_PI) theta_error -= 2.0 * M_PI;
    while(theta_error < - M_PI) theta_error += 2.0 * M_PI;
    if(ACTUATE_ARMS){
        roger->arm_torque[RIGHT][0] = theta_error * Kp_arm + Kd_arm * theta_dot_error;
    }else{
        roger->arm_torque[RIGHT][0] = theta_error * Kp_arm + Kd_arm * theta_dot_error;
    }
    return roger->arm_torque[RIGHT][0];
}

double PDRightElbow(roger, time)
Robot * roger;
double time;
{
    double theta_error, theta_dot_error;
    theta_error = roger->arm_setpoint[RIGHT][1] - roger->arm_theta[RIGHT][1];
    theta_dot_error = 0.0 - roger->arm_theta_dot[RIGHT][1];
    while(theta_error > M_PI) theta_error -= 2.0 * M_PI;
    while(theta_error < - M_PI) theta_error += 2.0 * M_PI;
    if(ACTUATE_ARMS){
        roger->arm_torque[RIGHT][1] = theta_error * Kp_arm + Kd_arm * theta_dot_error;
    }else{
        roger->arm_torque[RIGHT][1] = theta_error * Kp_arm + Kd_arm * theta_dot_error;
    }
    return roger->arm_torque[RIGHT][1];
}


/* Base PD controller, Cartesian reference */
double PDBase_translate(roger, time) 
Robot * roger;
double time;
{ 
    double Fx, error[2], trans_error, trans_vel;
    
        // roger->base_setpoint[X] = 0.75;
        //roger->base_setpoint[Y] = 0.0;
    
    
    error[X] = roger->base_setpoint[X] - roger->base_position[X];
    error[Y] = roger->base_setpoint[Y] - roger->base_position[Y];

    trans_error = error[X]*cos(roger->base_position[THETA]) +
        error[Y]*sin(roger->base_position[THETA]);

    trans_vel = roger->base_velocity[X]*cos(roger->base_position[THETA]) +
        roger->base_velocity[Y]*sin(roger->base_position[THETA]);

    if (ACTUATE_BASE) {
            // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE BASE
            // USING YOUR GAINS
            //Fx = - passive_Kd_base_trans * trans_vel;
        Fx = Kp_base_trans * trans_error - Kd_base_trans * trans_vel;
    }
    else {
        Fx = - passive_Kd_base_trans * trans_vel;
    }
    return(Fx);
}

/* Base PD controller, Cartesian reference */
double PDBase_rotate(roger, time) 
Robot * roger;
double time;
{
    double Mz, theta_error, theta_dot_error;
        //roger->base_setpoint[THETA] = 0;
    double epsilon = 0.01;
    

    theta_error = roger->base_setpoint[THETA] - roger->base_position[THETA];
    theta_dot_error = 0.0 - roger->base_velocity[THETA];
    while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
    while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

    
        //compute real time theta_error
    double error[2], rad = 0.0, cur_theta_error = 0.0;
    error[X] = roger->base_setpoint[X] - roger->base_position[X];
    error[Y] = roger->base_setpoint[Y] - roger->base_position[Y];
//    if(fabs(error[X]) > epsilon || fabs(error[Y]) > epsilon){
    if(error[X] != 0 || error[Y] != 0){
            //radian error
        rad = atan2(error[Y], error[X]);
            // printf("First if: rad is %f\n", rad);
        
    }else{
        rad = 0.0;
            //printf("Else: rad is %f\n", rad);
    }
        //printf("%f  %f\n", error[X], error[Y]);
        //printf("%.15f %.15f\n", roger->base_position[X], roger->base_position[Y]);
    
    cur_theta_error = rad - roger->base_position[THETA];
    

    
        /* if( rad != 0.0){ */
        /*     if(error[X] >= 0){ */
        /*         cur_theta_error = rad - roger->base_position[THETA]; */
        /*     }else if(error[Y] >= 0){ */
        /*         cur_theta_error = M_PI + rad - roger->base_position[THETA]; */
        /*     }else{ */
        /*         cur_theta_error = -M_PI + rad - roger->base_position[THETA]; */
        /*     } */
        /*         //printf("error first if: cur is %f\n", cur_theta_error); */
        /* } *///else if(rad != 0.0 && error[X] < 0){
        // cur_theta_error = M_PI + rad - roger->base_position[THETA];
        // }
    
    while (cur_theta_error > M_PI) cur_theta_error -= 2.0 * M_PI;
    while (cur_theta_error < -M_PI) cur_theta_error += 2.0 * M_PI;
        //printf("cur is %f\n", cur_theta_error);

    if (ACTUATE_BASE) {
            // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE BASE
            // USING YOUR GAINS
        Mz = Kp_base_rot * theta_error + Kd_base_rot * theta_dot_error;
    }
    else {
        Mz = passive_Kd_base_rot * theta_dot_error;
    }
  
    return(Mz);
}

/* PROJECT #1.3 - PD CONTROLLER FOR THE BASE                             */
/* setpoints - (xy) location for translation heading in radians          */

/*   the base differential drive Jacobian:                               */
/*    |tau_l|     |Fx|      |  x_dot  |     |theta_dot_left |            */
/*    |     |= JT |  |      |         | = J |               |            */
/*    |tau_r|     |Mz|      |theta_dot|     |theta_dot_right|            */

double baseJT[2][2] = 
{{(1.0/2.0), -(1.0/(2.0*R_AXLE))}, {(1.0/2.0), (1.0/(2.0*R_AXLE))}};

void PDController_base(roger, time)
Robot * roger;
double time;
{ 
    double Fx, Mz, PDBase_translate(), PDBase_rotate();

        /* Fx = 0.0;                          // rotate in current footprint */
        /* Mz = PDBase_rotate(roger,time); */

    
    
        /* Fx = PDBase_translate(roger,time); // translate along current heading */
        /* Mz = 0.0; */
        /* roger->wheel_torque[LEFT] = baseJT[0][0]*Fx + baseJT[0][1]*Mz; */
        /* roger->wheel_torque[RIGHT] = baseJT[1][0]*Fx + baseJT[1][1]*Mz; */
    
    Fx = PDBase_translate(roger,time);     // translate and rotate
    Mz = PDBase_rotate(roger,time);

        //plot x, y and theta errors in the base position as a function of time
        //x error

        /* FILE *fp; */
        /* fp = fopen("./left.dat", "a"); */
    
        /* double error_x = roger->base_setpoint[X] -  roger->base_position[X]; */
        /*     //y error */
        /* double error_y = roger->base_setpoint[Y] - roger->base_position[Y]; */
        /*     //theta error */
        /* double error_theta = roger->base_setpoint[THETA] - roger->base_position[THETA]; */
    
        /* while (error_theta > M_PI) error_theta -= 2.0 * M_PI; */
        /* while (error_theta < -M_PI) error_theta += 2.0 * M_PI; */
        /* fprintf(fp,"%f\t%f\t%f\t%f\n", time, error_x, -error_y, error_theta); */
        /* fclose(fp); */
    
        // integrated wheel torque control
    roger->wheel_torque[LEFT] = baseJT[0][0]*Fx + baseJT[0][1]*Mz;
    roger->wheel_torque[RIGHT] = baseJT[1][0]*Fx + baseJT[1][1]*Mz;

    
    
}


void initialize_q_double_dot(roger, time, qdd)
Robot * roger;
double time;
double qdd[8];
{
    qdd[0] = PDBase_translate(roger,time)/(M_BASE * R_BASE);
    qdd[1] = PDBase_rotate(roger,time)/(M_BASE * R_BASE);
    qdd[2] = PDLeftEye(roger, time)/(M_EYE * L_EYE);
//    qdd[2] = PDLeftEye(roger, time);
    qdd[3] = PDRightEye(roger, time)/(M_EYE * L_EYE);
    qdd[4] = PDLeftShoulder(roger, time)/(M_ARM1 * L_ARM1);
    qdd[5] = PDLeftElbow(roger, time)/(M_ARM2 * L_ARM2);
    qdd[6] = PDRightShoulder(roger,time)/(M_ARM1 * L_ARM1);
    qdd[7] = PDRightElbow(roger,time)/(M_ARM2 * L_ARM2);
}

void decouple(roger, time, torques)
Robot * roger;
double time;
double torques[8];
{
        //Fx
    double Fx = torques[0];

        //Mz
    double Mz = torques[1];

    roger->wheel_torque[LEFT] = baseJT[0][0]*Fx + baseJT[0][1]*Mz;
    roger->wheel_torque[RIGHT] = baseJT[1][0]*Fx + baseJT[1][1]*Mz;


        //left Eye
    double le = torques[2];
    roger->eye_torque[LEFT] = le;
    double re = torques[3];
    roger->eye_torque[RIGHT] = re;


        //left arm
    double lshoulder = torques[4];
    roger->arm_torque[LEFT][0] += lshoulder;
    double lelbow = torques[5];
    roger->arm_torque[LEFT][1] += lelbow;

        //right arm
    double rshoulder = torques[6];
    roger->arm_torque[RIGHT][0] += rshoulder;
    double relbow = torques[7];
    roger->arm_torque[RIGHT][1] += relbow;
}

void set_input(roger, time)
Robot * roger;
double time;
{
    roger->eyes_setpoint[LEFT] = 1;
    roger->arm_setpoint[LEFT][0] = 1;
    roger->arm_setpoint[LEFT][1] = 1;
    roger->arm_setpoint[RIGHT][0] = -1;
    roger->arm_setpoint[RIGHT][1] = -1;
    roger->base_setpoint[X] = 0;
    roger->base_setpoint[Y] = -1;
    roger->base_setpoint[THETA] = 1;
    
}

void record_error(roger, time)
Robot * roger;
double time;
{
        //record error of left eye, left arms, base theta and base position
    FILE *record;
    record = fopen("./simu.dat", "a");
    double eye_error = roger->eyes_setpoint[LEFT] - roger->eye_theta[LEFT];
    double shoulder_error = roger->arm_setpoint[LEFT][0] - roger->arm_theta[LEFT][0];
    double elbow_error = roger->arm_setpoint[LEFT][1] - roger->arm_theta[LEFT][1];

    double error[2];
    error[X] = roger->base_setpoint[X] - roger->base_position[X];
    error[Y] = roger->base_setpoint[Y] - roger->base_position[Y];
    double trans_error = error[X]*cos(roger->base_position[THETA]) +
        error[Y]*sin(roger->base_position[THETA]);
    
    double base_error_theta = roger->base_setpoint[THETA] - roger->base_position[THETA];

    fprintf(record, "%f\t%f\t%f\t%f\t%f\t%f\n",
            time,
            eye_error,
            shoulder_error,
            elbow_error,
            trans_error,
            base_error_theta
            );

    fclose(record);
    
}


/*************************************************************************/
/*       THE SIMULATOR EXECUTES control_roger() EVERY CONTROL CYCLE      */
/*                        *** DO NOT ALTER ***                           */
/*************************************************************************/
void control_roger(roger, time)
Robot * roger;
double time;
{
    update_setpoints(roger); // check_GUI_inputs(roger)
//    set_input(roger,time);
    

        // turn setpoint references into torques
//    PDController_eyes(roger, time);
/*     PDLeftEye(roger, time); */
/*     PDRightEye(roger, time); */
/* //    PDController_arms(roger, time); */
/*     PDRightShoulder(roger,time); */
/*     PDRightElbow(roger,time); */
/*     PDLeftShoulder(roger, time); */
/*     PDLeftElbow(roger, time); */
/*     PDController_base(roger,time); */

        //M: generalized inertia matrix
        //V: column vector of Coriolis/centrifugal loads acting on each DOF
        //G: column vector of gravity loads
        //F: a column vector of external forces applied to the robot
    double M[8][8], V[8], G[8], F[8];
    double torques[8];
        //get dynamics of Roger
    get_dynamics(M, V, G, F);
    
    
    
        //Initialize accelerationg column
    double q_double_dot[8];
    initialize_q_double_dot(roger,time,q_double_dot);
        //M * theta_double_dot
    matrix_mult(8, 8, M, 1, q_double_dot, torques);
        //+ V
    matrix_add(8, 1, torques, V, torques);
        //+ G
    matrix_add(8, 1, torques, G, torques);
        //+ F
    matrix_add(8, 1, torques, F, torques);
    decouple(roger, time, torques);
//    record_error(roger, time);

    
}





/*************************************************************************/
void project1_reset(roger)
Robot* roger;
{ }

/*************************************************************************/
/* prompt for and read user customized input values                      */
/*************************************************************************/
void project1_enter_params()
{
        // put anything in here that you would like to change at run-time
        // without re-compiling user project codes
    printf("ARM: K=%6.4lf  B=%6.4lf\n", Kp_arm, Kd_arm);
    printf("ARM: enter 'K B'\n"); fflush(stdout);
    scanf("%lf %lf", &Kp_arm, &Kd_arm);
}

/*************************************************************************/
// function called when the 'visualize' button on the gui is pressed            
void project1_visualize(roger)
Robot* roger;
{ }

