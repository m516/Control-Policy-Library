#pragma once
#include "vector.h"


namespace cpl{


    // An abstract class for systems that attempt to bring 
    // "inputs" closer to "inputs_target" by adjusting "outputs"
    template<Size num_inputs, Size num_outputs>
    class Controller{
        public:
        Vector<num_inputs> inputs;
        Vector<num_inputs> inputs_target;
        Vector<num_outputs> outputs;
        // Use the inputs to compute the outputs
        virtual void update() = 0;
    };

} // end namespace cpl