#include <state-observation/observer/tilt-estimator.hpp>

namespace stateObservation
{

  TiltEstimator::TiltEstimator(unsigned alpha, unsigned beta)
    : ZeroDelayObserver(6, 6), alpha_(alpha), beta_(beta)
  {
  }

  ObserverBase::StateVector TiltEstimator::oneStepEstimation_()
  {

    BOOST_ASSERT(this->y_.size() > 0 && this->y_.checkIndex(k+1) && "ERROR: The measurement vector is not set");

    oc_.y1 = R_S_C * yg - omega_S_C;
    oc_.x1 = p_S_C.cross(oc_.y1) - dp_S_C;

    oc_.dx1_hat = oc_.x1_hat.cross(oc_.y1) + gravityConstant * oc_.x2_hat - R_S_C * ya + alpha_ * (oc_.x1 - oc_.x1_hat);
    oc_.dx2_hat = oc_.x2_hat.cross(oc_.y1 - beta_ * oc_.x2_hat.cross(oc_.x1 - oc_.x1_hat));
    
    return oc_.x2_hat;
  }

  
}
