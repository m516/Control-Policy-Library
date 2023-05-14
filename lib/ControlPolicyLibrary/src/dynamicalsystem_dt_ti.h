// Discrete Time (DT) Time-Invariant (TI) Dynamical System

#pragma once
#include "vector.h"

namespace cpl{

    
    /// @brief an abstract N-input, M-output system type
    /// @tparam N the number of states
    /// @tparam M the number of outputs
    /// @tparam P the number of control inputs
    template<cpl::Size N, cpl::Size M, cpl::Size P>
    class DynamicalSystem_DiscreteTime_TimeInvariant{
        public:
        Vector<N> x;
        Vector<P> u;
        Vector<M> y;
        virtual Vector<N> f(Vector<N> x, Vector<P> u) = 0;
        virtual Vector<M> h(Vector<N> x, Vector<P> u) = 0;
        virtual void update(){
            x = x + f(x, u);
            y =     h(x, u);
        }
    };

}