#pragma once
#include "../controller.h"


namespace cpl{


    // An implementation of an n-dimensional PID controller.
    // Each input and output is controlled separately with different P, I, and D gains
    template<Size n>
    class Controller_PID : public Controller<n,n>{
        public:
        Vector<n> P, I, D;
        Vector<n> inputs_previous, inputs_derivative, inputs_integral;
        // Use the inputs to compute the outputs
        virtual void update(){
            inputs_derivative        = Controller<n,n>::inputs    - inputs_previous;
            inputs_integral          = inputs_integral            + Controller<n,n>::inputs;
            inputs_previous          = Controller<n,n>::inputs;
            Controller<n,n>::outputs = P*Controller<n,n>::inputs  + I*inputs_integral + D*inputs_derivative;
        }
    };

} // end namespace cpl