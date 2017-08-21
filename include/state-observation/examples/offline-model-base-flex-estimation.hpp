/**
 * \file      offline-ekf-flexibility-estimation.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief      Gives an implementation of flexibility estimation for IMU reconstruction
 *             with or without given input using a Kalman filter. The source is in a file
 *             imu-attitude-trajectory-reconstruction.hxx
 *
 * \details
 *
 *
 */

#ifndef FLEXIBILITYESTIMATION_OFFLINEMODELBASEFLEXESTIMATOR_H
#define FLEXIBILITYESTIMATION_OFFLINEMODELBASEFLEXESTIMATOR_H

#include <state-observation/flexibility-estimation/model-base-ekf-flex-estimator-imu.hpp>
#include <vector>

namespace stateObservation
{
    namespace examples
    {

        /*! \fn IndexedMatrixArray offlineEKFFlexibilityEstimation(
         *   const stateObservation::IndexedMatrixArray & y,
         *   const stateObservation::IndexedMatrixArray & u,
         *   const Matrix & xh0,
         *   unsigned numberOfContacts,
         *   const std::vector<Vector3> & contactsPositions,
         *   double dt);
         *
         *  \brief Provides the estimation of the flexibility of a robot
         *         given the measurements of an IMU and the input
         *         (which provides the reference trajectory of the IMU). This method uses
         *         extended Kalman filtering, we need to provide it with an
         *         initial guess, the number of contacts and their positions, and
         *         the time sampling period.
         *
         *
         *  \param y IMU measurements
         *  \param u the inputs of the dynamical system
         *  \param xh0 an initial guess of the state
         *  \param numberOfContacts the number of contacts
         *  \param contactsPositions a vector of positions of the vector
         *  \param dt the time discretization period
         */
        stateObservation::IndexedMatrixArray offlineModelBaseFlexEstimation(
            const stateObservation::IndexedMatrixArray & y,
            const stateObservation::IndexedMatrixArray & u,
            const Matrix & xh0,
            const stateObservation::IndexedMatrixArray numberOfContacts,
            double dt,
            double mass,
            const stateObservation::IndexedMatrixArray & Q = stateObservation::IndexedMatrixArray(),
            const stateObservation::IndexedMatrixArray & R = stateObservation::IndexedMatrixArray(),
            const Matrix3 & kfe = Matrix3::Zero(),
            const Matrix3 & kfv = Matrix3::Zero(),
            const Matrix3 & kte = Matrix3::Zero(),
            const Matrix3 & ktv = Matrix3::Zero(),
            IndexedMatrixArray * inovation=0x0,
            IndexedMatrixArray * predictedMeasurements = 0x0,
            IndexedMatrixArray * simulatedMeasurements = 0x0,
            int verbose=0x0);


# include <state-observation/examples/offline-model-base-flex-estimation.hxx>

    }

}

#endif // FLEXIBILITYESTIMATION_OFFLINEEKFFLEXIBILITYESTIMATION_H
