IndexedVectorArray imuMultiplicativeAttitudeReconstruction
(
  const IndexedVectorArray & y,
  const IndexedVectorArray & u,
  const Vector & xh0,
  const Matrix & p,
  const Matrix & q,
  const Matrix & r,
  double dt)
{
  ///Sizes of the states for the state, the measurement, and the input vector
  const unsigned stateSize=19;
  const unsigned measurementSize=6;
  const unsigned inputSize=6;

  typedef kine::indexes<kine::quaternion> indexes;
  typedef kine::indexes<kine::rotationVector> indexesTangent;

  ///initialization of the extended Kalman filter
  ExtendedKalmanFilter filter(stateSize, indexesTangent::size, measurementSize, inputSize, false);


  ///initalization of the functor
  IMUMltpctiveDynamicalSystem imuFunctor;
  imuFunctor.setSamplingPeriod(dt);
  filter.setFunctor(& imuFunctor);
  filter.setSumFunction(imuFunctor.stateSum);
  filter.setDifferenceFunction(imuFunctor.stateDifference);

  ///the initalization of the estimation of the initial state
  filter.setState(xh0,y.getFirstIndex()-1);

  ///computation and initialization of the covariance matrix of the initial state
  filter.setStateCovariance(p);

  ///set initial input
  filter.setInput(u[y.getFirstIndex()-1],y.getFirstIndex()-1);

  ///The covariance matrix of the process noise and the measurement noise
  /// for the extended Kalman filter
  filter.setR(r);
  filter.setQ(q);

  ///set the derivation step for the finite difference method
  Vector dx=filter.stateVectorConstant(1)*1e-2;

  ///the array of the state estimations over time
  IndexedVectorArray xh;
  xh.setValue(xh0,y.getFirstIndex()-1);

  ///the reconstruction of the state
  for (TimeIndex i=y.getFirstIndex(); i<y.getNextIndex(); ++i)
  {
    ///introduction of the measurement
    filter.setMeasurement(y[i],i);

    ///introduction of the input
    if (i<y.getLastIndex())
      filter.setInput(u[i],i);

    ///get the jacobians by finite differences and provide
    ///them to the Kalman filter
    ///Matrix a=filter.getAMatrixFD(dx);
    Matrix c= filter.getCMatrixFD(dx);

    Matrix a = filter.getAmatrixIdentity();

    a.block<12,12>(0,indexesTangent::linVel).diagonal().setConstant(dt);
    a.block<6,6>(0,indexesTangent::linAcc).diagonal().setConstant(dt*dt*0.5);

    //std::cout<<"a" << std::endl << a <<std::endl;
    //std::cout<<"c" << std::endl << c <<std::endl;


    filter.setA(a);
    filter.setC(c);

    ///get the estimation and give it to the array
    Vector xhk=filter.getEstimatedState(i);


    //std::cout<<"xh"<< xhk.transpose() <<std::endl;

    ///regulate the part of orientation vector in the state vector
    xhk.segment(indexes::ori,3)=kine::regulateOrientationVector
                                (xhk.segment(indexes::ori,3));

    ///give the new value of the state to the kalman filter.
    ///This step is usually unnecessary, unless we modify the
    ///value of the state esimation which is the case here.
    filter.setState(xhk,i);

    xh.setValue(xhk,i);
  }

  return xh;
}

IndexedVectorArray imuMultiplicativeAttitudeReconstruction(
  const IndexedVectorArray & y,
  const Vector & xh0,
  const Matrix & p,
  const Matrix & q,
  const Matrix & r,
  double dt)
{
  const unsigned inputSize=6;

  ///initialization of a zero input
  IndexedVectorArray u;
  for (TimeIndex k=y.getFirstIndex()-1; k<y.getNextIndex(); ++k)
  {
    u.setValue(Vector::Zero(inputSize,1),k);
  }

  return imuMultiplicativeAttitudeReconstruction (y, u, xh0, p, q, r, dt);
}




