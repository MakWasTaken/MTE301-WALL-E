/*This code was developed by Joel Valley, Nathan Mak, and Juan Forero*/
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

class Motor
{
    private:
    unsigned int IN1;
    unsigned int IN2;

    public:
    Motor() {}  //Default Constructor
    Motor(unsigned int in1, unsigned int in2) : IN1(in1), IN2(in2) {}   //Parametric Assignment Constructor

    const int getIN1() const    //IN1 Getter
    {
        return(IN1);
    }

    const int getIN2() const    //IN2 Getter
    {
        return(IN2);
    }

    void setIN1(const unsigned int &in1) //IN1 Setter
    {
        IN1 = in1;
    }

    void setIN2(const unsigned int &in2) //IN2 Setter
    {
        IN2 = in2;
    }

    void setup()    //Setup the motor pins to be output pins
    {
        gpio_init(IN1);
        gpio_set_dir(IN1, GPIO_OUT);
        gpio_set_function(IN1, GPIO_FUNC_PWM);  //Set the first pin to be a PWM pin type (Pulse Width Modulation)
        unsigned int slice_num = pwm_gpio_to_slice_num(IN1);    //Get the PWM slice num for the first pin
        pwm_set_wrap(slice_num, 255);   //Make sure motor speed adjusts smoothly
        pwm_set_enabled(slice_num, true);   //Turn on PWM signal for first pin

        gpio_init(IN2);
        gpio_set_dir(IN2, GPIO_OUT);
    }
    

    void forward(unsigned int speed)  //Turn motor forwards
    {
        int speedPercent;

        if(speed > 100)
        {
            speed = 100;
        }

        speedPercent = (speed*255)/100;

        pwm_set_gpio_level(IN1, speedPercent);
        gpio_put(IN2, 0);
    }

    void reverse(unsigned int speed)  //Turn motor backwards
    {
        int speedPercent;

        if(speed > 100)
        {
            speed = 100;
        }

        speedPercent = (speed*255)/100;

        pwm_set_gpio_level(IN1, speedPercent);
        gpio_put(IN2, 1);
    }

    void stop() //Stop the motor
    {
        pwm_set_gpio_level(IN1, 0);
        gpio_put(IN2, 0);
    }
};

class Robot
{
    private:
    Motor motor1;   //Front Left
    Motor motor2;   //Back Left
    Motor motor3;   //Front Right
    Motor motor4;   //Back Right

    public:
    Robot() : motor1(18, 19), motor2(21, 20), motor3(7, 6), motor4(9, 8) {} //Default Assignment Constructor

    void setup()    //Robot Setup
    {
        motor1.setup();
        motor2.setup();
        motor3.setup();
        motor4.setup();
    }

    void moveForward(unsigned int speed)  //Robot Move Forward
    {
        motor1.forward(speed);   
        motor2.forward(speed);   
        motor3.forward(speed);   
        motor4.forward(speed);   
    }

    void moveBackward(unsigned int speed)  //Robot Move Backward
    {
        motor1.reverse(speed);   
        motor2.reverse(speed);   
        motor3.reverse(speed);   
        motor4.reverse(speed);   
    }

    void stop() //Robot Stop
    {
        motor1.stop();
        motor2.stop();
        motor3.stop();
        motor4.stop();
    }

    void turnLeft()
    {
        motor1.reverse(20);
        motor2.reverse(20);
        motor3.forward(20);
        motor4.forward(20);
    }
};


//CHATGPT CODE IS AFTER THIS COMMENT, LOOK AT IT FOR REFERENCE OF HOW TO USE CERTAIN FUNCTIONS//

/*
// Motor GPIO pins
#define MOTOR_1_IN1 18
#define MOTOR_1_IN2 19
#define MOTOR_2_IN1 20
#define MOTOR_2_IN2 21

// Ultrasonic sensor GPIO pins
#define TRIG_PIN 4
#define ECHO_PIN 5

// Motor control functions
void setup_motors() {
    gpio_init(MOTOR_1_IN1);
    gpio_set_dir(MOTOR_1_IN1, GPIO_OUT);
    gpio_init(MOTOR_1_IN2);
    gpio_set_dir(MOTOR_1_IN2, GPIO_OUT);

    gpio_init(MOTOR_2_IN1);
    gpio_set_dir(MOTOR_2_IN1, GPIO_OUT);
    gpio_init(MOTOR_2_IN2);
    gpio_set_dir(MOTOR_2_IN2, GPIO_OUT);
}

void move_forward() {
    gpio_put(MOTOR_1_IN1, 1);
    gpio_put(MOTOR_1_IN2, 0);
    gpio_put(MOTOR_2_IN1, 1);
    gpio_put(MOTOR_2_IN2, 0);
}

void stop_motors() {
    gpio_put(MOTOR_1_IN1, 0);
    gpio_put(MOTOR_1_IN2, 0);
    gpio_put(MOTOR_2_IN1, 0);
    gpio_put(MOTOR_2_IN2, 0);
}

// Ultrasonic sensor functions
void setup_ultrasonic() {
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

float get_distance() {
    // Send a 10us pulse on TRIG_PIN to start measurement
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    // Wait for the pulse on the ECHO_PIN
    while (gpio_get(ECHO_PIN) == 0);
    absolute_time_t start_time = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 1);
    absolute_time_t end_time = get_absolute_time();

    // Calculate the distance based on time and speed of sound
    int64_t pulse_duration = absolute_time_diff_us(start_time, end_time);
    float distance = (pulse_duration * 0.0343f) / 2.0f; // Speed of sound in cm/us

    return distance;
}
*/

int main()
{
    Robot robot;
    robot.setup();
    
    while(true) 
    {
        robot.turnLeft();
        sleep_ms(2000);

        robot.stop();
        sleep_ms(1000);
    }
}
