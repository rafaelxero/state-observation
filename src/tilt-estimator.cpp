#include <state-observation/observer/tilt-estimator.hpp>

namespace stateObservation
{

  TiltEstimator::TiltEstimator(double alpha, double beta)
    : ZeroDelayObserver(6, 6), alpha_(alpha), beta_(beta),
      p_S_C(Vector3::Zero()), R_S_C(Matrix3::Zero()), v_S_C(Vector3::Zero()), w_S_C(Vector3::Zero())
  {
  }
  
  void TiltEstimator::setMeasurement(const Vector ya_k, const Vector yg_k, TimeIndex k)
  {
    ObserverBase::MeasureVector y_k;
    y_k << ya_k, yg_k;

    ZeroDelayObserver::setMeasurement(y_k, k);
  }

  ObserverBase::StateVector TiltEstimator::oneStepEstimation_()
  {
    TimeIndex k = this->x_.getTime();
    
    BOOST_ASSERT(this->y_.size() > 0 && this->y_.checkIndex(k+1) && "ERROR: The measurement vector is not set");

    Vector3 ya = getMeasurement(k+1).head(3);
    Vector3 yg = getMeasurement(k+1).tail(3);
    
    Vector3 y1 = R_S_C * yg - w_S_C;
    Vector3 x1 = p_S_C.cross(y1) - v_S_C;

    ObserverBase::StateVector x_hat = getEstimatedState(k);
    Vector3 x1_hat = x_hat.head(3);
    Vector3 x2_hat = x_hat.tail(3);
    
    Vector6 dx_hat;
    dx_hat.head(3) = x1_hat.cross(y1) + cst::gravityConstant * x2_hat - R_S_C * ya + alpha_ * (x1 - x1_hat);
    dx_hat.tail(3) = x2_hat.cross(y1 - beta_ * x2_hat.cross(x1 - x1_hat));
    
    x_hat += dx_hat * dt_;
    
    setState(x_hat, k+1);
    
    return x_hat;
  }

  
}
