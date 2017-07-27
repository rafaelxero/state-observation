#include <iostream>
#include <fstream>

#include <stdlib.h>

//#include <state-observation/noise/gaussian-white-noise.hpp>
//#include <state-observation/examples/offline-ekf-flexibility-estimation.hpp>
//#include <state-observation/dynamical-system/dynamical-system-simulator.hpp>
//#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <state-observation/examples/offline-model-base-flex-estimation.hpp>

typedef stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::input input;
typedef stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state state;


#include <time.h>


using namespace stateObservation;

main (int argc, char *argv[])
{
  if (argc!=6)
  {
    std::cout << "Parameters : measurements inputs numberOfContacts dt m"<<std::endl;
    std::cout << "Leaving"<<std::endl;

  }
  else
  {
    IndexedMatrixArray y,u,numberOfContacts;

    std::cout << "Read Sensors filename: "<<argv[1]<< std::endl;
    y.readVectorsFromFile(argv[1],false);
    std::cout << "Sensors loaded, size:"<< y.size() << std::endl;
    y.setFirstIndex(0);

    std::cout << "Read Inputs, filename: "<<argv[2]<< std::endl;
    u.readVectorsFromFile(argv[2],false);
    std::cout << "Inputs loaded, size:"<< u.size() << std::endl;
    u.setFirstIndex(0);


    //conversion from Alexis to my format
    for (int i=u.getFirstIndex(); i< u.getLastIndex(); ++i)
    {
      Vector ui, ut;

      ui=u[i];

      ut.resize(u[i].size()+6);
      ut.head<input::additionalForces>()= ui.head<input::additionalForces>();

      ut.segment<6>(input::additionalForces).setZero();

      ut.tail(ui.size()-input::additionalForces) = ui.tail(ui.size()-input::additionalForces);

      u[i]=ut;

      Vector3 gyroBias;
      gyroBias << -0.04,-0.045,-0.064;

      Vector yi=y[i];
      yi.segment<3>(3)-=gyroBias;

      y[i]=yi;
    }

    std::cout << "Read contact number, filename: "<<argv[3]<< std::endl;
    numberOfContacts.readVectorsFromFile(argv[3],false);
    std::cout << "Contact numbers loaded, size:"<< numberOfContacts.size() << std::endl;

    Matrix xh0 = Vector::Zero(flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state::size,1);

    double dt=atof(argv[4]);
    double mass=atof(argv[5]);

    IndexedMatrixArray predictedMea, innovation, simumea;

    std::cout << "Rebuiding state" << std::endl;

    //setting stiffness
    Matrix3 kfe= Matrix3::Zero();
    kfe << 4034, 0, 0,
        0, 23770, 0,
        0,     0,  239018;

    Matrix3 kfv = Matrix3::Zero();

    Matrix3 kte = Matrix3::Zero();
    kte << 707, 0, 0,
        0, 502, 0,
        0,     0,  936;
    Matrix3 ktv = Matrix3::Zero();

    //kfe=40000*Matrix3::Identity();
    //kte=600*Matrix3::Identity();
    //kfv=600*Matrix3::Identity();
    //ktv=60*Matrix3::Identity();

    IndexedMatrixArray xhat=
      examples::offlineModelBaseFlexEstimation( y, u, xh0, numberOfContacts, dt, mass,
          kfe, kfv, kte, ktv,
          &innovation, &predictedMea, &simumea, 1);
    std::cout << "State rebuilt, size " << xhat.size()  <<std::endl;

    std::cout << "Writting Files" << std::endl;

    numberOfContacts.writeInFile("contNum.log");
    y.writeInFile("y.log");
    u.writeInFile("u.log");
    xhat.writeInFile("xhat.log");
    innovation.writeInFile("innovation.log");
    predictedMea.writeInFile("predictedMeasurement.log");
    simumea.writeInFile("simulatedMeasurement.log");

    std::cout << "Files written" << std::endl;




    //reconstruction of the position of the IMU
    IndexedMatrixArray IMUpos, IMUvel;
    Matrix pos;
    pos.resize(3,4);

    Vector6 vel;

    for (int i = u.getFirstIndex(); i<u.getLastIndex(); ++i)
    {
      Vector ui, xhati;

      ui = u[i];
      xhati = xhat[i];

      Matrix3 Rflex = kine::rotationVectorToRotationMatrix(xhati.segment<3>(state::ori));



      Vector3 IMUPos = Rflex *ui.segment<3>(input::posIMU) + xhati.segment<3>(state::pos);

      Matrix3 IMURot = Rflex *kine::rotationVectorToRotationMatrix(ui.segment<3>(input::oriIMU));

      pos.block<3,3>(0,0)= IMURot;
      pos.col(3)= IMUPos;

      vel.segment<3>(0) = xhati.segment<3>(state::linVel)
                          + Rflex * ui.segment<3>(input::linVelIMU)
                          + xhati.segment<3>(state::angVel).cross(IMUPos);

      vel.segment<3>(3) = xhati.segment<3>(state::angVel)
                          + Rflex * ui.segment<3>(input::angVelIMU) ;

      IMUpos.pushBack(pos);
      IMUvel.pushBack(vel);


    }

    std::cout << "Writting IMU Files" << std::endl;
    IMUpos.writeInFile("IMUPos.log");
    IMUvel.writeInFile("IMUvel.log");
    std::cout << "Files written" << std::endl;

    std::cout << "Last State value" << std::endl << xhat[xhat.getLastIndex()].transpose() <<std::endl;



  }
  return 0;

}





