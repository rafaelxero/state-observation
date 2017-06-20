const double acc_cov_const=1e-4;
const double gyr_cov_const=1e-10;
const double state_fc_const=1e-0;
const double state_cov_const=1e-12;
typedef flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state state;

stateObservation::IndexedMatrixArray offlineModelBaseFlexEstimation(
  const stateObservation::IndexedMatrixArray & y,
  const stateObservation::IndexedMatrixArray & u,
  const Matrix & xh0,
  const stateObservation::IndexedMatrixArray numberOfContacts,
  double dt,
  double mass,
  IndexedMatrixArray * ino,
  IndexedMatrixArray * premea,
  int verbose)
{


  flexibilityEstimation::ModelBaseEKFFlexEstimatorIMU estimator;

  estimator.setContactModel(flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::contactModel::elasticContact);
  estimator.setSamplingPeriod(dt);
  estimator.setRobotMass(mass);

  Matrix R,Q,P;

  int measurementSize=6;
  int stateSize=flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state::size;

  double acceleroCovariance=acc_cov_const;
  double gyroCovariance=gyr_cov_const;
  double stateCov=state_cov_const;
  double stateForceCov=state_fc_const;

  R.noalias()=Matrix::Identity(measurementSize,measurementSize)*acceleroCovariance;
  R(3,3)=R(4,4)=R(5,5)=gyroCovariance;
  Q.noalias()=Matrix::Identity(stateSize,stateSize)*stateCov;

  Q.diagonal().segment<12>(state::fc).setConstant(stateForceCov);

  estimator.setProcessNoiseCovariance(Q);
  estimator.setMeasurementNoiseCovariance(R);
  estimator.setForceVariance(1e1);

  ///initialize flexibility
  estimator.setFlexibilityCovariance(Q);
  estimator.setFlexibilityGuess(xh0);


  bool withForce=true;
  estimator.setWithForcesMeasurements(withForce);

  ///the array of the state estimations over time
  stateObservation::IndexedMatrixArray xh;
  xh.setValue(xh0,y.getFirstIndex()-1);

  ///the reconstruction of the state
  for (unsigned i=y.getFirstIndex(); i<=y.getLastIndex(); ++i)
  {

    estimator.setContactsNumber(numberOfContacts[i](0));

    if (verbose>0)
    {
      if (i%100==0 || verbose >1)
      {
        std::cout << "iteration: " << i  << std::endl ;

        if (verbose >2)
        {
          std::cout << "number of contacts: "<< numberOfContacts[i]<< std::endl;
          std::cout << "size of the measurement: "<< y[i].size()
          << ", supposed to be " << estimator.getMeasurementSize() << std::endl;
          std::cout << "size of the input: "<< u[i].size()
          << ", supposed to be " << estimator.getInputSize() << std::endl;

          if (verbose > 3)
          {

            std::cout << "numberOfContacts: " << numberOfContacts[i].transpose() << std::endl;
            std::cout << "Measurement: " << y[i].transpose() << std::endl;
            std::cout << "Input: " << u[i].transpose() << std::endl;

          }
        }
      }


    }

    ///introduction of the measurement
    estimator.setMeasurement(Vector(y[i]).head(estimator.getMeasurementSize()));


    estimator.setMeasurementInput(u[i]);

    ///get the estimation and give it to the array
    Vector xhk=estimator.getFlexibilityVector();



    xh.pushBack(xhk);

    if (ino != 0)
    {
      ino->setValue(estimator.getInovation(),i);
    }

    if (premea != 0)
    {
      premea->setValue(estimator.getPredictedMeasurement(),i);
    }

    if (verbose >1)
    {
      std::cout << "Success";
      if (verbose >2)
      {
        std::cout << ", rebuilt state "<<std::endl;
        std::cout << xhk.transpose();
      }

      std::cout<<std::endl;

    }

  }

  return xh;
}



