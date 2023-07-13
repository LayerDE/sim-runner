#include "defines.h"
#include "pushed_follower.hpp"
#include <cmath>
#include "math_functions.h"


#include <stdio.h>


typedef struct {float beta_start; float beta_end; float steering_angle;} lookup_steering;

static inline float pow2(float x){
  return x*x;
}

static void trail_point_out(void* context, point x0, float direction){

}

static void car_point_out(void* context, point x0, float direction){

}

static bool isNear_points(point x0, point x1, float range){
    point x2 = {.x = x0.x -x1.x, .y = x0.y - x1.y};
    float _straight = sqrt(pow2(x2.x) + pow2(x2.y));
    if(_straight<range)
        return true;
    else
        return false;
}

pushed_follower::pushed_follower(int c_wheelbase, int rc_axle2hitch, int hitch2trail_axle, float beta_protect, unsigned int lookup_alpha_size, float sim_distance,
            get_float steering_ptr, get_float hitch_angle_ptr, get_int speed_ptr, double ki, double kp, double kd)
            : simulation(this, 0, 0, (float)c_wheelbase/100.0,(float)rc_axle2hitch/100.0,0,steering_ptr(),(float)hitch2trail_axle/100.0, hitch_angle_ptr(),0.01){
    hitch2axle = hitch2trail_axle;
    car2hitch = rc_axle2hitch;
    car_wheelbase = c_wheelbase;
    alpha_lookup_size = lookup_alpha_size;

    // allocating alpha sim lookup
    alpha_sim_lookup = new float*[alpha_lookup_size/2];
    for(int i = 0; i < alpha_lookup_size/2; i++)
        alpha_sim_lookup[i] = new float[alpha_lookup_size];

    beta_max = beta_protect;
    simulator_distance = sim_distance;
    create_alpha_lookup();
    create_alpha_sim_lookup(simulator_distance);
    simulation.set_output(car_point_out,trail_point_out, false);
    alpha_calc = new PID(&isPoint, &output, &setPoint, ki, kp, kd, 0);
}

pushed_follower::~pushed_follower(){
    delete alpha_calc;
    for(int i = 0; i < alpha_lookup_size/2; i++)
        delete [] alpha_sim_lookup[i];
    delete [] alpha_sim_lookup;
}

double pushed_follower::calculate(int des_speed, float des_steering){ // simple pid
    isPoint = get_hitch_angle();
    setPoint = des_steering;
    output = get_steering();
    alpha_calc->Compute();
    return output;
}

float pushed_follower::calc_alpha_const(float beta){ // todo
    //lookup table reader
    if(fabs(beta)> beta_max){
        return 1/0 * SIGN(beta);
    }
    return beta/c_alpha_beta_factor;
}

float pushed_follower::calc_beta_const(float alpha_steer){ // todo
    return alpha_steer * c_alpha_beta_factor;;
    //float delta_2 = sin(hitch2axle/sqrt(pow2(V_bw)+pow2(car2hitch)));
    //return delta_1;
}

float pushed_follower::calc_alpha(float beta_old, float beta_new){
    unsigned int indexA = round(fabs(beta_old) / (beta_max/(float)(alpha_lookup_size/2)));
    unsigned int indexB = round(beta_new / (beta_max/(float)(alpha_lookup_size/2))) + alpha_lookup_size/2;
    return alpha_sim_lookup[indexA][indexB];
}

float pushed_follower::calc_beta(float alpha, float beta_old, float distance){
    // so simple but so complex
    simulation.set_values(0,0,0,alpha,beta_old);
    simulation.simulate(distance);
    return simulation.output();
}

void pushed_follower::create_alpha_sim_lookup(float distance){
    // todo
}


//linear
void pushed_follower::create_alpha_lookup(){
    float alpha_steer = deg2rad(10);
    float V_bw = car_wheelbase/tan(alpha_steer);
    float V_fbw = sqrt(pow2(V_bw)+pow2(car2hitch));
    float delta_2 = asin(car2hitch/V_fbw);
    float delta_1 = asin(hitch2axle/V_fbw);
    float beta = delta_1+delta_2;
    c_alpha_beta_factor = beta/alpha_steer;
    alpha_max = calc_alpha_const(beta_max);
    printf("const_factor: %f\n", c_alpha_beta_factor);
    for(float i = 0; i<20; i+=1.75){
        float i_star = deg2rad(i);
        printf("i:%f a:%f b:%f\n",i,rad2deg(calc_alpha_const(i_star)), rad2deg(calc_beta_const(i_star)));
    }
}