#include <iostream>
#include <fstream>
#include <stdexcept>      // std::invalid_argument

#include <stdlib.h>

//#include <state-observation/noise/gaussian-white-noise.hpp>
//#include <state-observation/examples/offline-ekf-flexibility-estimation.hpp>
//#include <state-observation/dynamical-system/dynamical-system-simulator.hpp>
//#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <state-observation/examples/offline-model-base-flex-estimation.hpp>

/// ///////////////////////////////////////////////////////////////

int main (int argc, char **argv)

{

  (void) argc;
  (void) argv;

  using namespace stateObservation;
  const unsigned long long size=50000000;

  const double dt=0.005;

  IndexedVectorArray linAcc;
  IndexedVectorArray angAcc;

  for (unsigned long long i = 0 ; i<size ; ++i )
  {
    Vector3 linAcci = Vector3::Random();
    Vector3 angAcci = Vector3::Random();

    linAcc.pushBack(linAcci);
    angAcc.pushBack(angAcci);



  }

  Vector3 initPos = Vector3::Random();
  Vector3 initOriV = Vector3::Random();
  Matrix3 initOriR = kine::rotationVectorToRotationMatrix(initOriV);
  Quaternion initOriQ(initOriR);

  Vector3 initlinvel = Vector3::Random();
  Vector3 initangvel = Vector3::Random();




  Vector3 pos = initPos;
  Matrix3 oriR = initOriR;

  Vector3 linvel = initangvel;
  Vector3 angvel = initlinvel;

  std::cout << std::endl << std::endl << "++++++++++++++ TEST 1 ++++++++++++++++"<<std::endl;
  {
    auto_cpu_timer t;
    for (unsigned i = 0 ; i<size ; ++i )
    {
      kine::integrateKinematics(pos,linvel,linAcc[i],oriR,angvel,angAcc[i],dt);

    }
  }

  AngleAxis oriaa(oriR);

  std::cout << "pos    "<<pos.transpose()<<std::endl;
  std::cout << "linvel "<<linvel.transpose()<<std::endl;
  std::cout << "ori    "<<oriaa.axis().transpose()*oriaa.angle() <<std::endl;
  std::cout << "angvel "<<angvel.transpose()<<std::endl;

  pos = initPos;
  Quaternion oriQ = initOriQ;

  linvel = initangvel;
  angvel = initlinvel;

  std::cout << std::endl << std::endl << "++++++++++++++ TEST 2 ++++++++++++++++"<<std::endl;
  {
    auto_cpu_timer t;
    for (unsigned i = 0 ; i<size ; ++i )
    {
      kine::integrateKinematics(pos,linvel,linAcc[i],oriQ,angvel,angAcc[i],dt);

    }
  }

  oriaa=AngleAxis(oriQ);

  std::cout << "pos    "<<pos.transpose()<<std::endl;
  std::cout << "linvel "<<linvel.transpose()<<std::endl;
  std::cout << "ori    "<<oriaa.axis().transpose()*oriaa.angle() <<std::endl;
  std::cout << "angvel "<<angvel.transpose()<<std::endl;






  std::cout << std::endl << std::endl << "++++++++++++++ TEST 3 ++++++++++++++++"<<std::endl;
  pos = initPos;
  oriR = initOriR;

  linvel = initangvel;
  angvel = initlinvel;

  {
    auto_cpu_timer t;
    for (unsigned i = 0 ; i<size ; ++i )
    {
      kine::integrateKinematics(pos,linvel,linAcc[i],dt);
      kine::integrateKinematics(oriR,angvel,angAcc[i],dt);

    }
  }

  oriaa=AngleAxis(oriR);

  std::cout << "pos    "<<pos.transpose()<<std::endl;
  std::cout << "linvel "<<linvel.transpose()<<std::endl;
  std::cout << "ori    "<<oriaa.axis().transpose()*oriaa.angle() <<std::endl;
  std::cout << "angvel "<<angvel.transpose()<<std::endl;

  pos = initPos;
  oriQ = initOriQ;

  linvel = initangvel;
  angvel = initlinvel;

  std::cout << std::endl << std::endl << "++++++++++++++ TEST 4 ++++++++++++++++"<<std::endl;
  {
    auto_cpu_timer t;
    for (unsigned i = 0 ; i<size ; ++i )
    {
      kine::integrateKinematics(pos,linvel,linAcc[i],dt);
      kine::integrateKinematics(oriQ,angvel,angAcc[i],dt);

    }
  }

  oriaa=AngleAxis(oriQ);

  std::cout << "pos    "<<pos.transpose()<<std::endl;
  std::cout << "linvel "<<linvel.transpose()<<std::endl;
  std::cout << "ori    "<<oriaa.axis().transpose()*oriaa.angle() <<std::endl;
  std::cout << "angvel "<<angvel.transpose()<<std::endl;




  return 0;
}
