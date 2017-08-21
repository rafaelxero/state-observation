#include <iostream>
#include <fstream>

#include <stdlib.h>

//#include <state-observation/noise/gaussian-white-noise.hpp>
//#include <state-observation/examples/offline-ekf-flexibility-estimation.hpp>
//#include <state-observation/dynamical-system/dynamical-system-simulator.hpp>
//#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <state-observation/examples/offline-model-base-flex-estimation.hpp>

#include <time.h>

using namespace stateObservation::flexibilityEstimation;
typedef IMUElasticLocalFrameDynamicalSystem::state state;
typedef IMUElasticLocalFrameDynamicalSystem::input Input;

///StateEstimatorTest.header
//struct indexes
//{
//  ///indexes of the different components of a vector of the input state
//  static const unsigned posCom = 0;
//  static const unsigned accCom = 3;
//  static const unsigned velCom = 6;
//
//  static const unsigned AngAccIMU = 9;
//  static const unsigned LinAccIMU = 12;
//  static const unsigned oriIMU = 15;
//  static const unsigned posIMU = 19;
//  static const unsigned angVelIMU = 22;
//  static const unsigned linVelIMU = 25;
//
//  static const unsigned leftFootForce = 28;
//  static const unsigned leftFootMoment = 31;
//  static const unsigned leftFootOri = 34;
//  static const unsigned leftFootPos = 38;
//
//  static const unsigned leftHandForce = 41;
//  static const unsigned leftHandMoment = 44;
//  static const unsigned leftHandOri = 47;
//  static const unsigned leftHandPos = 51;
//
//  static const unsigned linMomentum = 54;
//  static const unsigned angMomentum = 57;
//
//  static const unsigned linMomentumDot = 60;
//  static const unsigned angMomentumDot = 63;
//
//  static const unsigned rightFootForce = 66;
//  static const unsigned rightFootMoment = 69;
//  static const unsigned rightFootOri = 72;
//  static const unsigned rightFootPos = 76;
//
//  static const unsigned rightHandForce = 79;
//  static const unsigned rightHandMoment = 82;
//  static const unsigned rightHandOri = 85;
//  static const unsigned rightHandPos = 89;
//
//  static const unsigned inertia = 92;
//  static const unsigned rootOri = 101;
//  static const unsigned rootPos = 105;
//
//  static const unsigned rootAngVel = 108;
//  static const unsigned rootLinVel = 111;
//
//  static const unsigned accelerometer=114;
//
//  static const unsigned mass = 130;
//
//  static const unsigned gyrometer = 218;
//
//  static const unsigned time = 225;
//
//};

///StateEstimatorTestFullCorrect-original.header
struct indexes
{
  ///indexes of the different components of a vector of the input state
  static const unsigned posCom = 0;
  static const unsigned accCom = 3;
  static const unsigned velCom = 6;

  static const unsigned AngAccIMU = 9;
  static const unsigned LinAccIMU = 12;
  static const unsigned oriIMU = 15;
  static const unsigned posIMU = 19;
  static const unsigned angVelIMU = 22;
  static const unsigned linVelIMU = 25;

  static const unsigned leftFootForce = 28;
  static const unsigned leftFootMoment = 31;
  static const unsigned leftFootOri = 34;
  static const unsigned leftFootPos = 38;

  static const unsigned leftHandForce = 41;
  static const unsigned leftHandMoment = 44;
  static const unsigned leftHandOri = 47;
  static const unsigned leftHandPos = 51;

  static const unsigned linMomentum = 54;
  static const unsigned angMomentum = 57;

  static const unsigned linMomentumDot = 60;
  static const unsigned angMomentumDot = 63;

  static const unsigned rightFootForce = 66;
  static const unsigned rightFootMoment = 69;
  static const unsigned rightFootOri = 72;
  static const unsigned rightFootPos = 76;

  static const unsigned rightHandForce = 79;
  static const unsigned rightHandMoment = 82;
  static const unsigned rightHandOri = 85;
  static const unsigned rightHandPos = 89;

  static const unsigned inertia = 92;
  static const unsigned rootOri = 101;
  static const unsigned rootPos = 105;

  static const unsigned rootAngVel = 108;
  static const unsigned rootLinVel = 111;

  static const unsigned accelerometer=114;

  static const unsigned mass = 131;

  static const unsigned gyrometer = 219;

  static const unsigned time = 226;

};


using namespace stateObservation;

void loadInputsAndMeasurements(const IndexedMatrixArray array,
                               IndexedMatrixArray & u, IndexedMatrixArray & y,
                               IndexedMatrixArray & nbrOfContacts,
                               double & mass, double & dt)
{

  Vector uk, yk, nbrCont;
  bool withForce=true;
  Vector ark;

  if (withForce)
    yk.resize(18);
  else
    yk.resize(6);

  yk.setZero();

  nbrCont.resize(1);

  Quaternion q;

  Matrix3 Rl,Rr;

  Vector6 additionalForces, prevInertia;

  Vector3 virtualCoMBias;

  virtualCoMBias << 0.01,0.,0.;



  for (int k=array.getFirstIndex(); k<array.getNextIndex(); ++k)
  {
    ark=array[k];

    //std::cout <<"k " << k << "Line size " << ark.size() <<std::endl;

    mass = ark(indexes::mass);
    if (k>array.getFirstIndex())
    {
      dt = ark(indexes::time)- array[k-1](indexes::time);
    }


    yk.head<6>() <<ark.segment<3>(indexes::accelerometer),ark.segment<3>(indexes::gyrometer);

    unsigned measurementIndex=6;

    unsigned cnbr=2;

    bool rightFootIn;

    if (withForce)
    {
      cnbr=0;
      if (ark(indexes::rightFootForce+2)>mass*9.8*0.01)
      {
        ++cnbr;
        yk.segment<6>(measurementIndex) << ark.segment<3>(indexes::rightFootForce),
                   ark.segment<3>(indexes::rightFootMoment);
        rightFootIn=true;
        measurementIndex+=6;
      }
      else
        rightFootIn=false;

      if (ark(indexes::leftFootForce+2)>mass*9.8*0.01)
      {
        ++cnbr;
        yk.segment<6>(measurementIndex) << ark.segment<3>(indexes::leftFootForce),
                   ark.segment<3>(indexes::leftFootMoment);
        measurementIndex+=6;
      }
    }

    nbrCont << cnbr;

    uk.resize(Input::sizeBase+12*cnbr);
    uk.setZero();

/// -------------COM---------------------///
    uk.segment<3> (Input::posCom)= ark.segment<3>(indexes::posCom) + virtualCoMBias ;
    uk.segment<3> (Input::velCom)= ark.segment<3>(indexes::velCom) ;
    uk.segment<3> (Input::accCom)= ark.segment<3>(indexes::velCom) ;

/// -------------Inertia Matrix---------------------///
    Matrix3 inrtMtrxInOrign;
    inrtMtrxInOrign<<ark.segment<3>(indexes::inertia),
                     ark.segment<3>(indexes::inertia+3),
                     ark.segment<3>(indexes::inertia+6);
    //std::cout <<"inrtMtrxInRoot" <<std::endl << inrtMtrxInOrign << std::endl;
    inrtMtrxInOrign +=mass * kine::skewSymmetric2(ark.segment<3>(indexes::posCom)-ark.segment<3>(indexes::rootPos));
    //std::cout <<"inrtMtrxInCoM" <<std::endl << inrtMtrxInOrign << std::endl;
    inrtMtrxInOrign -=mass * kine::skewSymmetric2(ark.segment<3>(indexes::posCom));
    uk.segment<6> (Input::inertia)<<  inrtMtrxInOrign(0,0),
                                      inrtMtrxInOrign(1,1),
                                      inrtMtrxInOrign(2,2),
                                      inrtMtrxInOrign(0,1),
                                      inrtMtrxInOrign(0,2),
                                      inrtMtrxInOrign(1,2);
    if (k>array.getFirstIndex())
    {
      uk.segment<6> (Input::dotInertia)<<(uk.segment<6> (Input::inertia)-prevInertia)/dt;
    }

    prevInertia = uk.segment<6> (Input::inertia);

/// -------------Amgular momentum-------------------///
    uk.segment<3> (Input::angMoment)=ark.segment<3>(indexes::angMomentum);
    uk.segment<3> (Input::dotAngMoment)=ark.segment<3>(indexes::angMomentumDot);

/// -------------IMU position-------------------///
    uk.segment<3> (Input::posIMU)=ark.segment<3> (indexes::posIMU);

    q = Quaternion(ark(indexes::oriIMU),ark(indexes::oriIMU+1),ark(indexes::oriIMU+2),ark(indexes::oriIMU+3));





    uk.segment<3> (Input::oriIMU) = kine::quaternionToRotaionVector(q);
    uk.segment<3> (Input::linVelIMU)= ark.segment<3>(indexes::linVelIMU);
    uk.segment<3> (Input::angVelIMU)= ark.segment<3>(indexes::angVelIMU);

    ///accelerations of the IMU are expressed in the frame of the IMU
    uk.segment<3> (Input::linAccIMU) =  q.toRotationMatrix()* ark.segment<3>(indexes::LinAccIMU);




/// -------------Additional Forces-------------------///
    q = Quaternion(ark(indexes::rightHandOri),ark(indexes::rightHandOri+1),ark(indexes::rightHandOri+2),ark(indexes::rightHandOri+3));
    Rr = q.toRotationMatrix();
    q = Quaternion(ark(indexes::leftHandOri),ark(indexes::leftHandOri+1),ark(indexes::leftHandOri+2),ark(indexes::leftHandOri+3));
    Rl = q.toRotationMatrix();

    additionalForces.head<3>() =
      Rr*ark.segment<3>(indexes::rightHandForce)
     +Rl*ark.segment<3>(indexes::leftHandForce);
    additionalForces.tail<3>() =
      Rr* ark.segment<3>(indexes::rightHandMoment)
     +Rl* ark.segment<3>(indexes::leftHandMoment)
     +ark.segment<3>(indexes::rightHandPos).cross
      (Rr* ark.segment<3>(indexes::rightHandForce))
      + ark.segment<3>(indexes::leftHandPos).cross
      (Rl* ark.segment<3>(indexes::leftHandForce));

    uk.segment<6> (Input::additionalForces)<<additionalForces;

/// --------- Modeled Contact Positions -------------///
    int remainCont = cnbr;
    if (rightFootIn)
    {
      q = Quaternion(ark(indexes::rightFootOri),ark(indexes::rightFootOri+1),ark(indexes::rightFootOri+2),ark(indexes::rightFootOri+3));
      Rr = q.toRotationMatrix();
      uk.segment<12>(Input::contacts)<<
        ark.segment<3>(indexes::rightFootPos),
        kine::rotationMatrixToRotationVector(Rr),
        Vector3::Zero(),
        Vector3::Zero();
        remainCont --;
    }
    if (remainCont>0)
    {
      q = Quaternion(ark(indexes::leftFootOri),ark(indexes::leftFootOri+1),ark(indexes::leftFootOri+2),ark(indexes::leftFootOri+3));
      Rl = q.toRotationMatrix();
      uk.segment<12>(Input::contacts+12)<<
      ark.segment<3>(indexes::leftFootPos),
      kine::rotationMatrixToRotationVector(Rl),
      Vector3::Zero(),
      Vector3::Zero();

    }
    u.setValue(uk,k);
    y.setValue(yk,k);
    nbrOfContacts.setValue(nbrCont,k);
  }
}


/// ///////////////////////////////////////////////////////////////

int main (int argc, char *argv[])
{

  char * filename;

  if (argc!=2)
  {
    std::cout << "Parameters : filename"<<std::endl;
    std::cout << "Leaving"<<std::endl;

    filename="/home/benallegue/devel/logs/2017/humanoids/sample/StateEstimatorTest.log";
    //return 1;

  }
  else
  {
    filename=argv[1];
  }

  IndexedMatrixArray initialFile;
  IndexedMatrixArray y,u,numberOfContacts;

  std::cout << "Readind initial file "<<filename<< std::endl;
  initialFile.readVectorsFromFile(filename,false);
  std::cout << "File loaded, size:"<< initialFile.size() << std::endl;

  double dt;
  double mass;

  loadInputsAndMeasurements(initialFile,u,y,numberOfContacts,mass,dt);

  Matrix xh0 = Vector::Zero(flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state::size,1);

  std::cout << "Rebuiding state" << std::endl;


  IndexedMatrixArray predictedMea, innovation, simumea;

  IndexedMatrixArray xhat=
  examples::offlineModelBaseFlexEstimation( y, u, xh0, numberOfContacts,
                                            dt, mass, IndexedMatrixArray(), IndexedMatrixArray(),
                                            Matrix3::Zero(),Matrix3::Zero(),
                                           Matrix3::Zero(),Matrix3::Zero(),
                                             &innovation, &predictedMea,
                                            &simumea, 1);
  std::cout << "State rebuilt" << std::endl;

  std::cout << "Last value" << std::endl << xhat[xhat.getLastIndex()].transpose() <<std::endl;


  numberOfContacts.writeInFile("contNum.log");
  y.writeInFile("y.log");
  u.writeInFile("u.log");
  xhat.writeInFile("xhat.log");

  innovation.writeInFile("innovation.log");
  predictedMea.writeInFile("predictedMeasurement.log");
  simumea.writeInFile("simulatedMeasurement.log");

  std::cout << "mass " << mass<<std::endl;
  std::cout << "dt " << dt<<std::endl;

  return 0;

}





