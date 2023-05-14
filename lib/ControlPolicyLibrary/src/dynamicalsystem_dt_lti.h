// Discrete Time (DT) Linear Time-Invariant (LTI) Dynamical System

#pragma once
#include "dynamicalsystem_dt_ti.h"
#include "squarematrix.h"

namespace cpl{

    
    /// @brief an abstract N-input, M-output system type
    /// @tparam N the number of states
    /// @tparam M the number of outputs
    /// @tparam P the number of control inputs
    template<cpl::Size N, cpl::Size M, cpl::Size P>
    class DynamicalSystem_DiscreteTime_LinearTimeInvariant : public DynamicalSystem_DiscreteTime_TimeInvariant<N,M,P> {
        public:
            SquareMatrix<N> A;
            Matrix<N,P> B;
            Matrix<M,N> C;
            Matrix<M,P> D;
            virtual Vector<N> f(Vector<N> x, Vector<P> u){ return Vector<N>(A*x + B*u); }
            virtual Vector<M> h(Vector<N> x, Vector<P> u){ return Vector<M>(C*x + D*u); }
            virtual void      update(){DynamicalSystem_DiscreteTime_TimeInvariant<N,M,P>::update();}
    }; // End class DynamicalSystem_DiscreteTime_LinearTimeInvariant

}