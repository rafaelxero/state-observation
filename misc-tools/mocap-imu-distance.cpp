#include <iostream>
#include <fstream>

#include <stdlib.h>
#include <algorithm>

//#include <state-observation/noise/gaussian-white-noise.hpp>
//#include <state-observation/examples/offline-ekf-flexibility-estimation.hpp>
//#include <state-observation/dynamical-system/dynamical-system-simulator.hpp>
//#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <state-observation/examples/offline-model-base-flex-estimation.hpp>

typedef stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::input input;

#include <time.h>


using namespace stateObservation;

main (int argc, char *argv[])
{
  if (argc!=5)
  {
    std::cout << "Parameters : IMU-abs IMU-reconstruct IMU-abs-vel IMU-reconst-vel"<<std::endl;
    std::cout << "Leaving"<<std::endl;

  }
  else
  {
    IndexedMatrixArray abs,recons, absVel, reconsVel, errorPos, errorVelocity, unbiasedIMU,absPos;

    std::cout << "Read absolute position: "<<argv[1]<< std::endl;
    abs.readFromFile(argv[1],3,4,false);
    std::cout << "Absolute pos read: size "<< abs.size() << std::endl;
    std::cout << "Read reconstructed position: "<<argv[2]<< std::endl;
    recons.readFromFile(argv[2],3,4);
    std::cout << "Reconstructed position read, size:"<< recons.size() << std::endl;

    std::cout << "Read absolute Velocity: "<<argv[3]<< std::endl;
    absVel.readVectorsFromFile(argv[3],false);
    std::cout << "Absolute Velocity read: size "<< absVel.size() << std::endl;
    std::cout << "Read reconstructed Velocity: "<<argv[4]<< std::endl;
    reconsVel.readVectorsFromFile(argv[4]);
    std::cout << "Reconstructed Velocity read, size:"<< reconsVel.size() << std::endl;

    abs.setFirstIndex(recons.getFirstIndex());
    absVel.setFirstIndex(reconsVel.getFirstIndex());

    double rmsePos=0, rmseVel=0;

    std::cout << "Computing Bias" <<std::endl;
    Vector3 bias;
    bias.setZero();
    for (int i=abs.getFirstIndex()+1000; i< abs.getFirstIndex()+1800; ++i)
    {
      bias += recons[i].col(3) - abs[i].col(3);
    }
    bias = bias/800;

    std::cout << "Bias Computed " << bias.transpose() << std::endl;

    int firstIndex =abs.getFirstIndex();

    int lastIndex = std::min(abs.getLastIndex(),
                             std::min(recons.getLastIndex(),
                                      std::min(recons.getLastIndex(),reconsVel.getLastIndex())));
    int numberOfsamples = lastIndex-firstIndex;

    for (int i=abs.getFirstIndex(); i< lastIndex; ++i)
    {
      Matrix3 absRoti, reconsRoti;
      Vector3 absPosi, reconsPosi;
      Vector absVeli, reconsVeli;
      Vector3 errorPosi, errorVelocityi;

      absRoti = abs[i].block(3,3,0,0);
      reconsRoti = recons [i].block(3,3,0,0);

      absPosi = abs[i].col(3);
      reconsPosi = recons[i].col(3);

      unbiasedIMU.pushBack(reconsPosi-bias);
      absPos.pushBack(absPosi);

      absVeli = absVel [i];
      reconsVeli = reconsVel [i];

      errorPosi = reconsPosi-absPosi-bias;
      errorVelocityi = (reconsVeli-absVeli).head<3>();

      errorPos.pushBack(errorPosi);
      errorVelocity.pushBack(errorVelocityi);

      rmsePos+=errorPosi.squaredNorm();
      rmseVel+=errorVelocityi.squaredNorm();

    }

    rmsePos = sqrt(rmsePos/numberOfsamples);
    rmseVel = sqrt(rmseVel/numberOfsamples);

    errorPos.writeInFile("position-error.log");
    errorVelocity.writeInFile("velocity-error.log");
    unbiasedIMU.writeInFile("IMUPos-unbiased.log");
    absPos.writeInFile("IMUPos-absolute.log");

    std::cout << "RMSE Position " <<rmsePos <<std::endl;
    std::cout << "RMSE Velocity " <<rmseVel <<std::endl;

  }
  return 0;

}





