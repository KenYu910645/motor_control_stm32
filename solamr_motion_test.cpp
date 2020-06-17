#include "mbed.h"
#include "ros.h"
#include <std_msgs/String.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "Encoder.h"
#include "controller.h"

ros::NodeHandle nh;
// Pin Out
DigitalOut  my_led(LED1);

PwmOut      motor_left_blue(D10); 
//PwmOut      motor_left_green(PB_7);
DigitalOut  motor_left_green(D15);


PwmOut      motor_right_blue(D8); 
//PwmOut      motor_right_green(D7); 
DigitalOut  motor_right_green(D14);

// Define Publisher
std_msgs::Float64 deg_left;
std_msgs::Float64 deg_right;
ros::Publisher encoder_left("/Encoder_left", &deg_left);
ros::Publisher encoder_right("/Encoder_right", &deg_right);
geometry_msgs::Twist odom_data;
ros::Publisher Odom_topic("/STM32_twist", &odom_data);


//STM mbed bug: these macros are MISSING from stm32f3xx_hal_tim.h
#ifdef TARGET_STM32F4
#define __HAL_TIM_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNT)
#define __HAL_TIM_IS_TIM_COUNTING_DOWN(__HANDLE__)            (((__HANDLE__)->Instance->CR1 &(TIM_CR1_DIR)) == (TIM_CR1_DIR))
#endif
TIM_Encoder_InitTypeDef encoder2;
TIM_HandleTypeDef  timer2;
TIM_Encoder_InitTypeDef encoder1;
TIM_HandleTypeDef  timer1;

// Define Subscriber
double ref_left, ref_right;
double vx = 0, wz = 0;
void get_cmd_vel( const geometry_msgs::Twist &cmd_vel ){
    vx = cmd_vel.linear.x;
    wz = cmd_vel.angular.z;
    ref_left  = (vx - wz * 0.45 / 2) / 0.075 *180 / 3.1415926;
    ref_right = (vx + wz * 0.45 / 2) / 0.075 *180 / 3.1415926;
}
ros::Subscriber<geometry_msgs::Twist> cmd("/cmd_vel", get_cmd_vel );

double odom_vx, odom_wz;
void get_odom_twist(double v_left, double v_right){
    odom_vx = (v_left + v_right)/2 * 0.075 /180 * 3.1415926;
    odom_wz = (v_right - v_left)/0.45 * 0.75 /180 * 3.1415926;
    odom_data.linear.x = odom_vx;
    odom_data.angular.z = odom_wz;
    Odom_topic.publish( &odom_data );

}
// Define Motor Driver Functions
float saturation(float cmd){
    float sig = cmd;
    if(sig >= 1){
        sig = 1;
        }
    else if(sig <= -1){
        sig = -1;
        }
    return sig;
}

void motor_drive_left( float pwm_cmd ){
    if (pwm_cmd >= 0){
    motor_left_blue.write(pwm_cmd);
    motor_left_green = 0;
    }
    else{
    motor_left_blue.write(1 + pwm_cmd);
    motor_left_green = 1;
    }
    
}
void motor_drive_right( float pwm_cmd ){
    if (pwm_cmd >= 0){
    motor_right_blue.write(pwm_cmd);
    motor_right_green = 0;
    }
    else{
    motor_right_blue.write(1 + pwm_cmd);
    motor_right_green = 1;
    }
}


// Define initial setup
void init_setup(){
    // ROS initial setup
    nh.initNode();//
    nh.advertise(encoder_left);
    nh.advertise(encoder_right);
    nh.advertise(Odom_topic);
    // Encoder Init
    EncoderInit(&encoder2, &timer2, TIM2, 0xffffffff, TIM_ENCODERMODE_TI12);
    EncoderInit(&encoder1, &timer1, TIM3, 0xffffffff, TIM_ENCODERMODE_TI12);
    // Set pwm frequency
    motor_left_blue.period_ms(10);      // 100 Hz
//    motor_left_green.period_ms(10);     // 100 Hz
    motor_right_blue.period_ms(10);     // 100 Hz
//    motor_right_green.period_ms(10);    // 100 Hz
    
    nh.subscribe(cmd);
}

int32_t count_left  = 0, count_left_last  = 0;
int32_t count_right = 0, count_right_last = 0;
int32_t cycle_left  = 0, cycle_right      = 0;
float enc_left = 0, enc_right = 0;

void get_cycle(){
    if (count_left - count_left_last > 55000){        //BW Cycle
        cycle_left -= 1;
    }
    else if (count_left - count_left_last < -55000){  //FW Cycle
        cycle_left += 1;
    }     
    if (count_right - count_right_last > 55000){        //BW Cycle
        cycle_right -= 1;
    }
    else if (count_right - count_right_last < -55000){  //FW Cycle
        cycle_right += 1;
    }     
    count_left_last  = count_left;
    count_right_last = count_right; 
}
    
    
    
    
    
int main() {
    // Initialization
    init_setup();

    // Define Controller Class
    Ctrl Processor_left;
    Ctrl Processor_right;
    
    // Initial Conditions of processor 
    Processor_left.init();
    Processor_right.init();
    
    
    while (1) {
        
        // Read Encoder Count
        // Count to Deg: deg = 360/960*count;     
        count_left  = __HAL_TIM_GET_COUNTER(&timer2);
        count_right = __HAL_TIM_GET_COUNTER(&timer1);
        get_cycle();
        enc_left  = float(count_left  + 65536*cycle_left ) *360/960;
        enc_right = float(count_right + 65536*cycle_right) *360/960;        
        deg_left.data  = enc_left;
        deg_right.data = enc_right;
        
//        encoder_left.publish( &deg_left );
//        encoder_right.publish( &deg_right ); 
     
        // Get reference input
        if (ref_left != 0){
            Processor_left.ref  = ref_left;
            float gain_left  = Processor_left.PID()/24*1;
            float sig_left = saturation(gain_left); 
            motor_drive_left(sig_left);
            }
        else{
            motor_drive_left(0);
            Processor_left.volt        = 0;
            Processor_left.error       = 0;
            Processor_left.error_last  = 0;
            }
        if (ref_right != 0){
            Processor_right.ref = ref_right;
            float gain_right = Processor_right.PID()/24*1;
            float sig_right = saturation(gain_right); 
            motor_drive_right(sig_right);
        }
        else{
            motor_drive_right(0);
            Processor_right.volt        = 0;
            Processor_right.error       = 0;
            Processor_right.error_last  = 0;
            }
        
        // Measurement and Filter
        Processor_left.theta = enc_left;
        Processor_left.omega_processor();
        
        Processor_right.theta = enc_right;
        Processor_right.omega_processor();
        
        get_odom_twist(Processor_left.lp_w, Processor_right.lp_w);
        
        
        nh.spinOnce();
        wait_ms(20);
    }
}
