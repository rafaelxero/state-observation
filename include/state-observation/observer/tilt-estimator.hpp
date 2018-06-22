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

    /// The constructor
    ///  \li alpha : parameter related to the convergence of the opposite of the
    ///              linear velocity of the IMU expressed in the control frame
    ///  \li beta  : parameter related to the convergence of the tilt
    TiltEstimator(unsigned alpha, unsigned beta);

    void setAlpha(const unsigned alpha) { alpha_ = alpha; }
    unsigned getAlpha() const { return alpha_; }
    
    void setBeta(const unsigned beta) { beta_ = beta; }
    unsigned getBeta() const { return beta_; }

    void setSamplingTime(const unsigned dt) { dt_ = dt; }
    unsigned getSamplingTime() const { return dt_; }

    void setSensorPositionInC(const Vector3& p) { p_S_C = p; }
    Vector3 getSensorPositionInC() { return p_S_C; }
    
    void setSensorOrientationInC(const Matrix3& R) { R_S_C = R; }
    Matrix3 getSensorOrientationInC() { return R_S_C; }
    
    void setSensorLinearVelocityInC(const Vector3& v) { v_S_C = v; }
    Vector3 getSensorLinearVelocityInC() { return v_S_C; }
    
    void setSensorAngularVelocityInC(const Vector3& w) { w_S_C = w; }
    Vector3 getSensorAngularVelocityInC() { return w_S_C; }
    
    void setMeasurement(const Vector ya_k, const Vector yg_k, TimeIndex k) override;
    
  protected:

    /// The parameters of the estimator chosen such that
    /// beta_ * gravityConstant < alpha_^2
    unsigned alpha_, beta_;

    /// Sampling time
    unsigned dt_;
    
    /// Position of the IMU in the control frame
    Vector3 p_S_C;

    /// Orientation of the IMU in the control frame
    Matrix3 R_S_C;

    /// Linear velocity of the IMU in the control frame
    Vector3 v_S_C;

    /// Angular velocity of the IMU in the control frame
    Vector3 w_S_C;
    
    /// The tilt estimator loop
    StateVector oneStepEstimation_();
  };
  
}

#endif //TILTESTIMATORHPP
