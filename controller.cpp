#include "controller.h"
Ctrl::Ctrl() {
    
}


void Ctrl::init(){
    lp_w = 0;
    error_last = 0;
    volt = 0;
    w0 = 0;
    w1 = 0;
    w2 = 0;
    theta_last = 0;
    dt = 0.02;
}

float Ctrl::PID() {
    error = ref - lp_w;
    volt += error*0.018 - error_last*0.01689; 
    error_last = error;
    return volt;
}

float Ctrl::get_omega(){
    w2 = (theta - theta_last)/dt;
    theta_last = theta;
    return w2;
}

float Ctrl::low_pass_filter(){
    lp_w = (w0 + 2*w1 + w2)/4;
    w1 = w2;
    w0 = w1;
    return lp_w;
}

void Ctrl::omega_processor(){
    Ctrl::get_omega();    
    Ctrl::low_pass_filter();
}

/*

int main(void) {
    
    Ctrl t;
    t.ref = 11; 
    t.lp_w=0;

    while(1) {
        t.ref = 11; 
        gain = t.PID();
        
        if .... 
        endif

        t.theta_last = t.theta;
        t.theta = counter /4;

        t.w0 = t.w1;
        t.w1 = t.w2;
        t.w2 = t.get_omega();
        t.lp_w = t.low_pass_filter();

    }

    return 0;
}

*/