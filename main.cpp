#include "mbed.h"
#include "ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include "Encoder.h"
#include "controller.h"

ros::NodeHandle nh;
// Pin Out
DigitalOut  my_led(LED1);

PwmOut      motor_left_blue(D10); 
PwmOut      motor_left_green(PB_7);

PwmOut      motor_right_blue(D8); 
PwmOut      motor_right_green(D7); 

// Define Publisher
std_msgs::Float64 deg_left;
std_msgs::Float64 deg_right;
ros::Publisher encoder_left("/Encoder_left", &deg_left);
ros::Publisher encoder_right("/Encoder_right", &deg_right);

// Test Publisher
std_msgs::Float64 test;
ros::Publisher Test("/Test", &test);

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
void get_cmd_vel( const geometry_msgs::Twist &cmd_vel ){
    double vx = cmd_vel.linear.x;
    double wz = cmd_vel.angular.z;
    ref_left = (vx - wz * 0.45 / 2) / 0.075 *180 / 3.1415926;
    ref_right = (vx + wz * 0.45 / 2) / 0.075 *180 / 3.1415926;
}
ros::Subscriber<geometry_msgs::Twist> cmd("/cmd_vel", get_cmd_vel );

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
    motor_left_green.write(0);
    }
  else{
    motor_left_blue.write(0);
    motor_left_green.write(-pwm_cmd);
    }
    
}
void motor_drive_right( float pwm_cmd ){
  if (pwm_cmd >= 0){
    motor_right_blue.write(pwm_cmd);
    motor_right_green.write(0);
    }
  else{
    motor_right_blue.write(0);
    motor_right_green.write(-pwm_cmd);
    }
}


// Define initial setup
void init_setup(){
    // ROS initial setup
    nh.initNode();//
    nh.advertise(encoder_left);
    nh.advertise(encoder_right);
    nh.advertise(Test);
    // Encoder Init
    EncoderInit(&encoder2, &timer2, TIM2, 0xffffffff, TIM_ENCODERMODE_TI12);
    EncoderInit(&encoder1, &timer1, TIM3, 0xffffffff, TIM_ENCODERMODE_TI12);
    // Set pwm frequency
    motor_left_blue.period_ms(10);      // 100 Hz
    motor_left_green.period_ms(10);     // 100 Hz
    motor_right_blue.period_ms(10);     // 100 Hz
    motor_right_green.period_ms(10);    // 100 Hz
    
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

    // Define Class
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
        
        encoder_left.publish( &deg_left );
        encoder_right.publish( &deg_right );
/*        
        motor_right_blue.write(0.1);
        motor_right_green.write(0.0);
        
        motor_left_blue.write(0.1);
        motor_left_green.write(0.0);
*/        
        // Get reference input
        Processor_left.ref  = ref_left;
        Processor_right.ref = ref_right;
        
        // Calculate volt command: duty = volt / 24 * 1;
        float gain_left  = Processor_left.PID()/24*1;
        float sig_left = saturation(gain_left); 
        motor_drive_left(sig_left);
            
        float gain_right = Processor_right.PID()/24*1;
        float sig_right = saturation(gain_right); 
        motor_drive_right(sig_right);
        
//        test.data = ttt;
//        Test.publish(&test);
        
        // Measurement and Filter
        Processor_left.theta = enc_left;
        Processor_left.omega_processor();
        
        Processor_right.theta = enc_right;
        Processor_right.omega_processor();

        nh.spinOnce();
        wait_ms(20);
    }
}
