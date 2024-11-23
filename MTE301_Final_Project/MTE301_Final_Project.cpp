/*This code was developed by Joel Valley, Nathan Mak, and Juan Forero*/
#include <iostream>
#include <cstdint>
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
        gpio_put(IN1, 0);

        gpio_set_function(IN1, GPIO_FUNC_PWM);  //Set the first pin to be a PWM pin type (Pulse Width Modulation)
        int slice_num1 = pwm_gpio_to_slice_num(IN1);    //Get the PWM slice num for the first pin
        pwm_set_wrap(slice_num1, 255);   //Make sure motor speed adjusts smoothly
        pwm_set_enabled(slice_num1, true);   //Turn on PWM signal for first pin

        gpio_init(IN2);
        gpio_set_dir(IN2, GPIO_OUT);
        gpio_put(IN2, 0);

        gpio_set_function(IN1, GPIO_FUNC_PWM);  //Set the second pin to be a PWM pin type (Pulse Width Modulation)
        int slice_num2 = pwm_gpio_to_slice_num(IN2);    //Get the PWM slice num for the second pin
        pwm_set_wrap(slice_num2, 255);   //Make sure motor speed adjusts smoothly
        pwm_set_enabled(slice_num2, true);   //Turn on PWM signal for second pin
    }
    

    void forward(unsigned int speed)  //Turn motor forwards
    {
        int speedPercent;

        if(speed > 100)
        {
            speed = 100;
        }

        speedPercent = (speed*255)/100;

        gpio_put(IN1, 1);
        gpio_put(IN2, 0);
        pwm_set_gpio_level(IN1, speedPercent);
        pwm_set_gpio_level(IN2, 0);
    }

    void reverse(unsigned int speed)  //Turn motor backwards
    {
        int speedPercent;

        if(speed > 100)
        {
            speed = 100;
        }

        speedPercent = (speed*255)/100;

        gpio_put(IN1, 0);
        gpio_put(IN2, 1);
        pwm_set_gpio_level(IN1, 0);
        pwm_set_gpio_level(IN2, speedPercent);
    }

    void stop() //Stop the motor
    {
        gpio_put(IN1, 0);
        gpio_put(IN2, 0);
        pwm_set_gpio_level(IN1, 0);
        pwm_set_gpio_level(IN2, 0);
    }
};

class Sensor
{
    private:
    unsigned int trigPin;
    unsigned int echoPin;

    public:
    Sensor() {} //Default Constructor
    Sensor(unsigned int tp, unsigned int ep) : trigPin(tp), echoPin(ep) {} //Parametric Assignment Constructor

    const int getTrig() const    //Trig Pin Getter
    {
        return(trigPin);
    }

    const int getEcho() const    //Echo Pin Getter
    {
        return(echoPin);
    }

    void setTrig(const unsigned int &pin) //Trig Pin Setter
    {
        trigPin = pin;
    }

    void setEcho(const unsigned int &pin) //Echo Pin Setter
    {
        echoPin = pin;
    }

    void setup()    //Set up Trigger Pin to be Input and Echo Pin to be Output
    {
        gpio_init(trigPin);
        gpio_set_dir(trigPin, GPIO_OUT);
        gpio_put(trigPin, 0);

        gpio_init(echoPin);
        gpio_set_dir(echoPin, GPIO_IN);
    }

    double findDis()
    {
        gpio_put(trigPin, 0);   //Start With Trigger Pin Off
        sleep_us(2);

        gpio_put(trigPin, 1);   //Trigger Pin Pulse 10 Microseconds
        sleep_us(10);

        gpio_put(trigPin, 0);

        while (!gpio_get(echoPin)) {}   //Wait for Echo Pin to Trigger
        absolute_time_t start_time = get_absolute_time();    //Find the Absolute Start Time the Sensor Starts to Trigger

        while (gpio_get(echoPin)) {}   //Wait for Echo Pin to Stop
        absolute_time_t end_time = get_absolute_time();  //Find the Absolute End Time the Sensor Stops

        //Unsigned Integer With 64 bits for Increased Precision
        uint64_t duration = absolute_time_diff_us(start_time, end_time);    //Calculate the Difference Between Start and End Times

        double distance_cm = (0.0343*duration)/2.0;  //Distance Function

        return distance_cm;
    }
};

class onBoardLED
{
    private:
    unsigned int PIN;

    public:
    onBoardLED() {} //Default Constructor
    onBoardLED(unsigned int pin) : PIN(pin) {} //Parametric Assignment Constructor

    void setup()
    {
        gpio_init(PIN);
        gpio_set_dir(PIN, GPIO_OUT);
        gpio_put(PIN, 0);
    }

    void ledON()
    {
        gpio_put(PIN, 1);
    }

    void ledOFF()
    {
        gpio_put(PIN, 0);
    }
};

class Servo
{
    private:


};

class Robot
{
    private:
    Motor motor1;   //Front Left
    Motor motor2;   //Back Left
    Motor motor3;   //Front Right
    Motor motor4;   //Back Right

    Sensor sensor;  //Ultrasonic Sensor
    onBoardLED led; //Pico On Board LED

    public:
    Robot() : motor1(18, 19), motor2(21, 20), motor3(7, 6), motor4(9, 8), sensor(4, 5), led(25) {} //Default Assignment Constructor

    void setup()    //Robot Setup
    {
        motor1.setup();
        motor2.setup();
        motor3.setup();
        motor4.setup();

        sensor.setup();
        led.setup();
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

    void turnRight()
    {
        motor1.forward(20);
        motor2.forward(20);
        motor3.reverse(20);
        motor4.reverse(20);
    }

    void tooClose()
    {
        double distance = sensor.findDis();

        if(distance < 30.0)
        {
            led.ledON();
        }
        else
        {
            led.ledOFF();
        }
    }
};

int main()
{
    Robot robot;
    robot.setup();
    
    while(true)
    {
        robot.tooClose();
        sleep_ms(100);
    }

    return 0;
}
