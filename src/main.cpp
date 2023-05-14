#include <Arduino.h>

// If memory is an issue, define CPL_DISABLE_RECURSIVE
// to undefine recursively defined templates that are
// an inefficient use of memory
#include <ControlPolicyLibrary.h>
#include <dynamicalsystem_dt_lti.h>
#include <matrix.h>
#include <squarematrix.h>
#include <controllers/PID.h>
#include <controllers/full_state_feedback.h>


#include "cartpole.hpp"






cpl::Controller_PID<1> pid;
// cpl::Controller_FullStateFeedback<2,2> lqr;

// cpl::DynamicalSystem_DiscreteTime_LinearTimeInvariant<5,4,3> mysystem;




void setup() {
  // put your setup code here, to run once:
  // auto controller = cpl::LinearQuadraticRegulator<5,4,3>(mysystem, cpl::Identity<5>(), cpl::Identity<3>());

  cartpole::initialize();
  Serial.begin(9600);
  cartpole::set_power();

  // cpl::Scalar A_raw[3][3] = {
  // {1, 2, 3},
  // {4, 5, 6},
  // {9, 8, 9}
  // };
  // cpl::SquareMatrix<3> A;
  // // A = A_raw;
  // auto A_inv = A.inverse();

  // cpl::Scalar B;
  // cpl::SquareMatrix<4> C;
  // auto D = C * A;

  // mysystem.update();
}

void loop() {
  // put your main code here, to run repeatedly: 

  // set control
  cartpole::u = sin(micros()/1e6*2*PI) / 3;

  // Print sensor values
  cartpole::update();
  Serial.print("x: ");
  Serial.print(cartpole::x);
  Serial.print(" x_dot: ");
  Serial.print(cartpole::x_dot);
  Serial.print(" theta: ");
  Serial.print(cartpole::theta);
  Serial.print(" theta_dot: ");
  Serial.print(cartpole::theta_dot);
  Serial.println();


}