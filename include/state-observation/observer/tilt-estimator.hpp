/**
 * \file      tilt-estimator.hpp
 * \author    Rafael Cisneros, Mehdi Benallegue
 * \date       2018
 * \brief      Defines the class for the tilt estimator.
 *
 * \details
 *
 *
 */



#ifndef TILTESTIMATORHPP
#define TILTESTIMATORHPP

#include <state-observation/observer/zero-delay-observer.hpp>


namespace stateObservation
{

/**
  * \class  TiltEstimator
  * \brief
  *        Description is pending
  *
  *
  *
  */
  class TiltEstimator: public ZeroDelayObserver
  {
  public:

    TiltEstimator(unsigned alpha, unsigned beta);

  protected:

    /// The parameters of the estimator chosen such that
    /// beta_ * gravityConstant < alpha_^2
    unsigned alpha_, beta_;

    /// The tilt estimator loop
    virtual StateVector oneStepEstimation_();

    struct optimizationContainer
    {
      Vector y1;
      Vector x1;
      Vector dx1_hat;
      Vector dx2_hat;
    } oc_;
  };
  
  
}

#endif //TILTESTIMATORHPP
