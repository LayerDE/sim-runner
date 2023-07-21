#define _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_WARNINGS
#include <string> 
#include <iostream> 
#include <sstream>   



#define _USE_MATH_DEFINES
#include "simulator/position.hpp"
#include "simulator/car.hpp"
#include "simulator/simulator.hpp"

#include "bobbycar-files/pushed_follower.hpp"

#include "bobbycar-files/config.h"

#include <math.h>

using namespace std;

FILE *fp;

pushed_follower* trail;

simulator* sim;

float rad2deg(float in){
  return in * 45.0 / M_PI_4;
}
float deg2rad(float in){
  return in * M_PI_4 / 45.0;
}


void _point_out(FILE *fp, const char* name, float x, float y, float direction){
    stringstream buffer;
    buffer << name <<": " << x <<", " << y << ": " << rad2deg(direction) << endl;
    fprintf(fp, buffer.str().c_str());
    cout << buffer.str();
}


void trail_point_out(void* context, point x0, float direction){
    _point_out((FILE*)context, "Trailer", x0.x, x0.y, direction);
}

void car_point_out(void* context, point x0, float direction){
    _point_out((FILE*)context, "Car", x0.x, x0.y, direction);
}

float alpha = 0;
float beta = 0;

float get_steering(){
    return alpha;
}

float get_hitch(){
    return beta;
}

int get_speed(){
    return 125;
}


 const float des_beta = deg2rad(-10);
void init(){

    const float bbwb = 0.35, bbr2h = 0.05, followerlen = 0.60;
    const float bbx = 0.0, bby = 0.0, bbangle = 0.0, bbalpha = deg2rad(20);
    const float followerbeta = deg2rad(10);
    beta = followerbeta;
    alpha = bbalpha;
    const char *filename = "out.txt";

    // open the file for writing
    if ((fp = fopen(filename, "w")) == NULL)
        {
            printf("Error opening the file %s", filename);
        }

//    const float stepsize = 0.001;
    cout << "Simulator init..." << endl;
    sim = new simulator(fp, bbx, bby, bbwb, bbr2h, bbangle, bbalpha, followerlen, followerbeta, 0.0010);
    cout << "Simulate..." << endl;
    

    trail = new pushed_follower(L_WHEELBASE, L_REAR_TO_HITCH, L_HITCH_TO_FOLLOWER_AXLE, deg2rad(20), 20, 50.0,
            get_steering, get_hitch, get_speed,
            0, 0, 0);
}

void cleanup(){
    fclose(fp);      
    cout << "Simulation finished" << endl;
    delete sim;
    delete trail;
}

// simulator(float bbx, float bby, float bbwb, float bbr2h, float bbangle, float bbalpha, float followerlen, float followerbeta, float step_size)

int main(int argc, char *argv[]){
    init();
    while(1){
        cout << "wait" << endl;
        cin.ignore();
        {
            sim->set_output(car_point_out, trail_point_out,false);
            cout << "sim[" << sim->get_distance() << "]" << endl;
            alpha = trail->create_alpha_sim(beta,0,deg2rad(1),0.5);
            sim->set_alpha(deg2rad(3));
            sim->simulate(-0.05);
            beta = sim->output();
            cout << "alpha:" << rad2deg(alpha)<< "\tbeta:" << rad2deg(beta) << endl;
        }
    }
    cleanup();
    return 0;
}