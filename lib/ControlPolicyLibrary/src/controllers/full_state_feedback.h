#pragma once
#include "../controller.h"
#include "../dynamicalsystem_dt_lti.h"
#include "../squarematrix.h"


namespace cpl{


    // An implementation of a full-state feedback controller.
    // Here "inputs" is the full state "x" of a dynamical system,
    // and "outputs" is the control vector "u" that are used to 
    // drive a dynamical system.
    template<Size n, Size m>
    class Controller_FullStateFeedback : public Controller<n,m>{
        public:
        Matrix<n,m> K;
        // Use the inputs to compute the outputs
        virtual void update(){
            Controller<n,m>::outputs = K*Controller<n,m>::inputs;
        }
    };


    // Discrete-time Linear Quadratic Regulator implementation that 
    // the system "sys" to zero with the lowest cost governed by 
    // configurable weights Q and R.
    // Here "inputs" is the n-dimensional full state "x" of a dynamical system,
    // and "outputs" is the p-dimensional control vector "u" that are used to 
    // drive a dynamical system.

    // This method assumes the purpose of this controller is to stabilize
    // AROUND THE ORIGIN, NOT ANYWHERE ELSE                   (TODO easy fix?)
    template<Size n, Size m, Size p>
    Controller_FullStateFeedback<n,p> LinearQuadraticRegulator(
        const DynamicalSystem_DiscreteTime_LinearTimeInvariant<n,m,p> &sys, 
        const SquareMatrix<n> &Q, 
        const SquareMatrix<p> &R
    ) {
        Controller_FullStateFeedback<n,p> ctl;

        // Step 1: solve Algebraic Ricatti Equation for S
        // See https://en.wikipedia.org/wiki/Algebraic_Riccati_equation
        SquareMatrix<n> A = sys.A;
        Matrix<n,p> B = sys.B;
        SquareMatrix<n> Ait = A.inverse().transpose();
        SquareMatrix<p> Ri = R.inverse();
        auto Z11 = A + B*Ri*(B.transpose())*Ait*Q;
        auto Z12 = (A + B*Ri*(B.transpose())*Ait).negate();
        auto Z21 = (Ait*Q).negate();
        auto Z22 = Ait;
        auto Z1 = Z11.augmentAfter(Z12);
        auto Z2 = Z21.augmentAfter(Z22);
        auto Z = Z1.augmentBelow(Z2);
        //TODO


        // Step 2: Compute K = R^-1 B^T S
        
        return ctl;
    }

} // end namespace cpl