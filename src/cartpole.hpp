#pragma once
#include <Arduino.h>

namespace cartpole{



// State variables
float x = 0;                       // x position of the cart in centimeters
float x_dot = 0;                   // x velocity of the cart in centimeters per second
float theta = 0;                   // theta angle of the pendulum in degrees (where zero is balanced)
float theta_dot = 0;               // theta angular velocity of the pendulum in degrees per second

// Hardware pins
const int x_sensor_pin = A2;
const int theta_sensor_pin = A5;
const int u_motor_forward_speed_pin = 11;
const int u_motor_reverse_speed_pin = 3;
const int u_motor_enable_pin = 8;

// Control variables
float u = 0;

// General-purpose config variables
const float derivative_softness = 0.9;




void initialize(){
  pinMode(x_sensor_pin, INPUT);
  pinMode(theta_sensor_pin, INPUT);
  pinMode(u_motor_enable_pin, OUTPUT);
  pinMode(u_motor_forward_speed_pin, OUTPUT);
  pinMode(u_motor_reverse_speed_pin, OUTPUT);
} // void initialize()


void read_theta() {
    const int raw_min = 322;      // theta = -90
    const int raw_balanced = 587; // theta = 0
    const int raw_max = 853;      // theta = 90
    int raw_value = analogRead(theta_sensor_pin);
    if(raw_value<raw_balanced)  theta =  map(raw_value, raw_min,      raw_balanced, -90, 0 ); 
    else                        theta =  map(raw_value, raw_balanced, raw_max,       0,  90);

    // Calculate theta_dot
    static long previous_measurement_time = micros();
    static float theta_prev = theta;
    theta_dot = theta_dot*derivative_softness + (theta - theta_prev)/(micros()-previous_measurement_time)*1e6*(1-derivative_softness);
    theta_prev = theta;
    previous_measurement_time = micros();
} // void read_theta()

void read_x(){ // Reads the x position of the cart in centimeters using a linear potentiometer
    const float ticks_per_centimeter = 4.5669; // ticks per centimeter
    static const int raw_initial = analogRead(x_sensor_pin);
    int raw_value = analogRead(x_sensor_pin);
    x = (raw_value-raw_initial)/ticks_per_centimeter;

    // Calculate x_dot
    static long previous_measurement_time = micros();
    static float x_prev = x;
    x_dot = x_dot*derivative_softness + (x - x_prev)/(micros()-previous_measurement_time)*1e6*(1-derivative_softness);
    x_prev = x;
    previous_measurement_time = micros();
} // void read_x()



void set_power(){
    if(u==0){
        digitalWrite(u_motor_enable_pin, LOW);
        analogWrite(u_motor_forward_speed_pin, 0);
        analogWrite(u_motor_reverse_speed_pin, 0);
        return; 
    }
    else if(u>0){
        digitalWrite(u_motor_enable_pin, HIGH);
        analogWrite(u_motor_forward_speed_pin, u*256);
        analogWrite(u_motor_reverse_speed_pin, 0);
    }
    else if(u<0){
        digitalWrite(u_motor_enable_pin, HIGH);
        analogWrite(u_motor_forward_speed_pin, 0);
        analogWrite(u_motor_reverse_speed_pin, -u*256);
    }
} // void set_power()

void update(){
    read_x();
    read_theta();
    set_power();
} // void update()


} // namespace cartpole