/**
 * \file      extended-kalman-filter-base.hpp
 * \author    Mehdi Benallegue
 * \date       2012
 * \brief      Defined the class to intanciate to use an extended Kalman filter.
 *
 *             x_{k+1}=f(x_k,u_k)+v_k
 *             y_k=h(x_k,u_k)+w_k
 *
 * \details
 *
 *
 */


#ifndef STATEOBSERVER_EXTENDEDKALMANFILTERHPP
#define STATEOBSERVER_EXTENDEDKALMANFILTERHPP

#include <state-observation/observer/kalman-filter-base.hpp>
#include <state-observation/dynamical-system/dynamical-system-functor-base.hpp>

namespace stateObservation
{
/**
     * \class  ExtendedKalmanFilter
     * \brief
     *
     *        The class to intanciate to use an extended Kalman filter.
     *        To use this class, one needs to provide a pointer on a functor
     *        that describes the state dynamics and the measurement dynamics.
     *        The functor type needs to be a derived class from the class
     *        DynamicsFunctorBase.
     *
     *        x_{k+1}=f(x_k,u_k)+v_k
     *
     *        y_k=h(x_k,u_k)+w_k
     *
     *
     *
     */

    class ExtendedKalmanFilter: public KalmanFilterBase
    {

    public:

        /// The constructor.
        ///  \li n : size of the state vector
        ///  \li m : size of the measurements vector
        ///  \li p : size of the input vector
        ///  \li The parameter directInputOutputFeedthrough defines whether (true) or not (false) the measurement y_k requires the input u_k
        ///  \li The parameter directInputStateProcessFeedthrough defines whether (true) or not (false) the state x_{k+1} requires the input u_k

        ExtendedKalmanFilter(unsigned n,unsigned m,unsigned p=0,
                bool directInputOutputFeedthrough=true,
                bool directInputStateProcessFeedthrough=true)
            :KalmanFilterBase(n,m,p),
            directInputOutputFeedthrough_(directInputOutputFeedthrough),
            directInputStateProcessFeedthrough_(directInputStateProcessFeedthrough), f_(0x0)

        {
#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
            std::cout<<std::endl<<"ExtendedKalmanFilter Constructor"<<std::endl;
#endif //STATEOBSERVATION_VERBOUS_CONSTRUCTOR

            if (p==0)
                directInputOutputFeedthrough_=false;
        }


        /// Set a pointer to the functor that defines the dynamics of the states
        ///and the measurement the user is responsible for the validity of the
        ///pointer during the execution of the kalman filter
        void setFunctor(DynamicalSystemFunctorBase* f);

        ///Gets a pointer to the functor
        DynamicalSystemFunctorBase* functor() const;

        /// Clear the value of the functor
        ///Does not destroy the pointed object
        void clearFunctor();

        /// Precise whether (true) or not (false) the measurement y_k requires
        ///the input u_k
        void setDirectInputOutputFeedthrough(bool b=true);

        /// Precise whether (true) or not (false) the estimation of the state x_{k+1} requires
        ///the input u_k
        void setDirectInputStateFeedthrough(bool b=true);



        ///Give an estimation of A matrix using
        ///finite difference method (the forward difference method)
        ///the parameter dx is the step vector for derivation
        virtual Amatrix getAMatrixFD(const StateVector &dx);

        ///Give an estimation of C matrix using
        ///finite difference method (the forward difference method)
        ///the parameter dx is the step vector for derivation
        virtual Cmatrix getCMatrixFD(const StateVector &dx);

        /// Reset the extended kalman filter (call also the reset function of the dynamics functor)
        virtual void reset();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        /// simulate the dynamics of the state using the functor
        virtual StateVector prediction_(unsigned k);

        /// simulate the dynamic of the measurement using the functor
        virtual MeasureVector simulateSensor_(const StateVector& x, unsigned k);

        /// simulate the dynamic of the measurement using the functor
        virtual MeasureVector predictSensor_(const StateVector& x, unsigned k);

        /// container for the prediction
        IndexedMatrix xbar_;

        /// container for the prediction of the sensor
        IndexedMatrix ybar_;

        /// boolean that provides if theris a need of not for input for the masurement
        bool directInputOutputFeedthrough_;

        /// boolean that provides if theris a need of not for input for the state dynamics
        bool directInputStateProcessFeedthrough_;

        /// pointer on the dynamics functor
        DynamicalSystemFunctorBase* f_;

        //optimization
        struct Optimization
        {
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          ObserverBase::InputVector u_;
          KalmanFilterBase::Amatrix a_;
          KalmanFilterBase::Cmatrix c_;
          ObserverBase::StateVector x_;
          ObserverBase::StateVector xbar_;
          ObserverBase::StateVector xp_;
          ObserverBase::MeasureVector y_;
          ObserverBase::MeasureVector yp_;

        } opt;
    };

}

#endif //EXTENDEDKALMANFILTERHPP
