/*This code was developed by Joel Valley, Nathan Mak, and Juan Forero*/
#include <iostream>
#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

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
        pwm_set_wrap(slice_num1, 255);   //Configure the slice
        pwm_set_enabled(slice_num1, true);   //Turn on PWM signal for first pin

        gpio_init(IN2);
        gpio_set_dir(IN2, GPIO_OUT);
        gpio_put(IN2, 0);

        gpio_set_function(IN1, GPIO_FUNC_PWM);  //Set the second pin to be a PWM pin type (Pulse Width Modulation)
        int slice_num2 = pwm_gpio_to_slice_num(IN2);    //Get the PWM slice num for the second pin
        pwm_set_wrap(slice_num2, 255);   //Configure the slice
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

class Servo
{
    private:
    unsigned int PIN;

    public:
    Servo() {} //Default constructor 
    Servo(unsigned int p) : PIN(p) {} //Parametric Assignment Constructor 

    void setMillis(float millis)
    {
        pwm_set_gpio_level(PIN, (millis/20000.f)*39062.f);
    }

    void setup(float startMillis)
    {
        float clockDiv = 64;   
        float wrap = 39062;

        gpio_set_function(PIN, GPIO_FUNC_PWM);  //Initialize Pin as PWM Pin Type
        uint slice_num = pwm_gpio_to_slice_num(PIN);    //Find Slice Num for Pin

        pwm_config config = pwm_get_default_config();   //Config
        
        uint64_t clockspeed = clock_get_hz(clk_sys);    //Find the System's Clockspeed
        clockDiv = 64;
        wrap = 39062;

        while (clockspeed/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64; 
        wrap = clockspeed/clockDiv/50;  //Calculate Wrap Value

        pwm_config_set_clkdiv(&config, clockDiv);   //Set ClockDiv
        pwm_config_set_wrap(&config, wrap); //Set Wrap

        pwm_init(slice_num, &config, true); //Initialize the PWM Pin

        setMillis(startMillis); //SetMillis
    }

    void setAngle(double angle)
    {
        if (angle > 180.0)
        {
            angle = 180.0;
        }
        if (angle < 0.0)
        {
            angle = 0.0;
        }

        double duty = 500.0 + (angle * 2000.0) / 180.0;

        setMillis(duty);
    }

    void stop()
    {
        pwm_set_gpio_level(PIN, 0);
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

        while (!gpio_get(echoPin)) {}   //When the Echo Pin Does Not Read Anything
        absolute_time_t start_time = get_absolute_time();    //Find the Absolute Start Time when the Signal is Sent

        while (gpio_get(echoPin)) {}   //When the Echo Pin Reads the Signal
        absolute_time_t end_time = get_absolute_time();  //Find the Absolute End Time when the Signal is Read

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

class Robot
{
    public:
    Motor motor1;   //Front Left
    Motor motor2;   //Back Left
    Motor motor3;   //Front Right
    Motor motor4;   //Back Right

    Sensor sensor;  //Ultrasonic Sensor
    onBoardLED led; //Pico On Board LED
    Servo servo;

    public:
    Robot() : motor1(18, 19), motor2(21, 20), motor3(7, 6), motor4(9, 8), sensor(4, 5), led(25), servo(13) {} //Default Assignment Constructor

    void setup()    //Robot Setup
    {
        motor1.setup();
        motor2.setup();
        motor3.setup();
        motor4.setup();

        sensor.setup();
        led.setup();
        servo.setup(1500);
    }

    void moveForward(unsigned int speed)  //Robot Move Forward
    {
        motor1.forward(speed);   
        motor2.forward(speed);   
        motor3.forward(speed);   
        motor4.forward(speed);   
        
        sleep_ms(10);
    }

    void moveBackward(unsigned int speed)  //Robot Move Backward
    {
        motor1.reverse(speed);   
        motor2.reverse(speed);   
        motor3.reverse(speed);   
        motor4.reverse(speed);

        sleep_ms(10);   
    }

    void stop() //Robot Stop
    {
        motor1.stop();
        motor2.stop();
        motor3.stop();
        motor4.stop();

        sleep_ms(1000);
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

    bool tooClose()
    {
        double distance = sensor.findDis();

        if(distance < 50.0)
        {
            led.ledON();
            return true;
        }
        else
        {
            led.ledOFF();
            return false;
        }
    }

    void moveServo(double angle)
    {
        servo.setAngle(angle);
    }
};

int main()
{
    Robot robot;
    robot.setup();

    double angle = 90;
    bool direction = true;
    bool right_hit = false;
    bool left_hit = false;

    while(true)
    {
        //Sweep Algorithm

        angle += direction ? 1 : -1;

        if(angle >= 100)
        {
            direction = false;
        }
        if(angle <= 80)
        {
            direction = true;
        }

        robot.moveServo(angle);
        
        sleep_ms(5);

        //Movement Algorithm

        if(robot.tooClose())
        {
            robot.stop();
            sleep_ms(800);

            bool temp_right_hit = false;
            bool temp_left_hit = false;

            //Change Servo Angle

            robot.moveServo(140);
            sleep_ms(100);

            //Scan Left

            left_hit = false;   //Ensure Left Hit is False Before Scan
            
            for(int i = 140 ; i < 150 ; i+=1)
            {
                robot.moveServo(i);

                sleep_ms(50);

                if(robot.tooClose())
                {
                    temp_left_hit = true;
                }
            }

            //Reset Servo Angle

            robot.moveServo(90);
            sleep_ms(100);

            //Change Servo Angle

            robot.moveServo(40);
            sleep_ms(100);

            //Scan Right

            right_hit = false;   //Ensure Right Hit is False Before Scan

            for(int i = 40 ; i > 30 ; i-=1)
            {
                robot.moveServo(i);

                sleep_ms(50);

                if(robot.tooClose())
                {
                    temp_right_hit = true;
                }
            }

            //Reset Servo Angle

            robot.moveServo(90);
            sleep_ms(100);

            left_hit = temp_left_hit;
            right_hit = temp_right_hit;

            if(right_hit == true && left_hit == true)   //If Wall, Turn 180 Around
            {
                robot.turnLeft();
                sleep_ms(1100);
            }
            else if(left_hit == true)  //If Left Obstacle, Turn Right
            {
                robot.turnRight();
                sleep_ms(300);
            }
            else if(right_hit == true) //If Right Obstacle, Turn Left
            {
                robot.turnLeft();
                sleep_ms(300);
            }

            //Reset Hit Bools

            right_hit = false;
            left_hit = false;
        }
        else
        {
            robot.moveForward(30);
        }
    }
    
    return 0;
}