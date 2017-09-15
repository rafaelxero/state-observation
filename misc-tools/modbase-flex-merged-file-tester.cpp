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

void loadInputsAndMeasurements(const IndexedMatrixArray & array,
                               IndexedMatrixArray & u, IndexedMatrixArray & y,
                               IndexedMatrixArray & nbrOfContacts,
                               double & mass, double & dt, bool simu,
                               bool useHandSensor, bool withForce)
{

  Vector uk, yk, nbrCont;
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

      Vector3 footweight;
      if (simu)
        footweight=stateObservation::cst::gravity*1.325;//simulation
      else
        //footweight=-stateObservation::cst::gravity*1.057253969892212764e+00;//real robot
        footweight<<-5.5,0,-stateObservation::cst::gravityConstant*1.6;//real robot
      cnbr=0;
      if (ark(indexes::rightFootForce+2)>mass*9.8*0.01)
      {
        ++cnbr;

        q = Quaternion(ark(indexes::rightFootOri),ark(indexes::rightFootOri+1),ark(indexes::rightFootOri+2),ark(indexes::rightFootOri+3));
        Rr = q.toRotationMatrix().transpose();

        yk.segment<6>(measurementIndex) << ark.segment<3>(indexes::rightFootForce)+Rr.transpose()*footweight,
                   ark.segment<3>(indexes::rightFootMoment);
        rightFootIn=true;
        measurementIndex+=6;
      }
      else
        rightFootIn=false;

      if (ark(indexes::leftFootForce+2)>mass*9.8*0.01)
      {
        ++cnbr;

        q = Quaternion(ark(indexes::leftFootOri),ark(indexes::leftFootOri+1),ark(indexes::leftFootOri+2),ark(indexes::leftFootOri+3));
        Rl = q.toRotationMatrix().transpose();

        yk.segment<6>(measurementIndex) << ark.segment<3>(indexes::leftFootForce)+Rl.transpose()*footweight,
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
    uk.segment<3> (Input::accCom)= ark.segment<3>(indexes::accCom) ;

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





    uk.segment<3> (Input::oriIMU) = -kine::quaternionToRotationVector(q);
    uk.segment<3> (Input::linVelIMU)= ark.segment<3>(indexes::linVelIMU);
    uk.segment<3> (Input::angVelIMU)= ark.segment<3>(indexes::angVelIMU);

    ///accelerations of the IMU are expressed in the frame of the IMU
    uk.segment<3> (Input::linAccIMU) =  q.toRotationMatrix()* ark.segment<3>(indexes::LinAccIMU);




/// -------------Additional Forces-------------------///
    if (useHandSensor)
    {
      q = Quaternion(ark(indexes::rightHandOri),ark(indexes::rightHandOri+1),ark(indexes::rightHandOri+2),ark(indexes::rightHandOri+3));
      Rr = q.toRotationMatrix().transpose();
      q = Quaternion(ark(indexes::leftHandOri),ark(indexes::leftHandOri+1),ark(indexes::leftHandOri+2),ark(indexes::leftHandOri+3));
      Rl = q.toRotationMatrix().transpose();

      Vector3 handWeight;

      if (simu)
        handWeight=stateObservation::cst::gravity *
                   1.09413163; ///Simulation
      else
        handWeight=stateObservation::cst::gravity *
                   1.274430130714297782e+00; //real robot


      Vector3 initialHandForceOffset;
      if (simu)
      {
        initialHandForceOffset.setZero();
      }
      else
      {
        initialHandForceOffset<<-6.554883830982205506e-01,
                               -3.661343805957282260e+00,
                               1.197941548311729321e+01;///in the case of the real robot
      }


      Matrix3 initialRotOffset(Matrix3::Identity());
      if (simu)
      {
        initialRotOffset <<
                         0,  -1,  0,
                         1, 0,  0,
                         0,  0,  1;
      }


      Vector3 rightHandF= Rr*(initialRotOffset*(ark.segment<3>(indexes::rightHandForce)-initialHandForceOffset));
      Vector3 leftHandF = Rl*(initialRotOffset*(ark.segment<3>(indexes::leftHandForce) -initialHandForceOffset));

      // std::cout << k << " raw R " << ark.segment<3>(indexes::rightHandForce).transpose() << std::endl;
      // std::cout << k << " raw L " << ark.segment<3>(indexes::leftHandForce).transpose() << std::endl;

      // std::cout << k << " rot raw R " << (Rr*initialRotOffset*ark.segment<3>(indexes::rightHandForce)).transpose() << std::endl;
      // std::cout << k << " rot raw L " << (Rl*initialRotOffset*ark.segment<3>(indexes::leftHandForce)).transpose() << std::endl;

      // std::cout << k << " unbiased R " << rightHandF.transpose() << std::endl;
      // std::cout << k << " unbiased L " << leftHandF.transpose() << std::endl;

      Vector3 rightHandM= Rr*(initialRotOffset*(ark.segment<3>(indexes::rightHandMoment)));
      Vector3 leftHandM = Rl*(initialRotOffset*(ark.segment<3>(indexes::leftHandMoment )));

      // std::cout << k << " Rr " << std::endl <<Rr << std::endl ;
      // std::cout << k << " Rl" << std::endl <<Rl << std::endl ;

      rightHandF+=handWeight;
      leftHandF +=handWeight;

      // std::cout << k << " treated right  " << rightHandF.transpose() << std::endl;
      // std::cout << k << " treated left "   << leftHandF.transpose() << std::endl;

      leftHandF.setZero();///useless in this experiment
      leftHandM.setZero();///useless in this experiment

      additionalForces.head<3>() =rightHandF+leftHandF;
      additionalForces.tail<3>() =rightHandM
                                  +leftHandM
                                  +ark.segment<3>(indexes::rightHandPos).cross
                                  (rightHandF)
                                  +ark.segment<3>(indexes::leftHandPos).cross
                                  (leftHandF);



      uk.segment<6> (Input::additionalForces)<<additionalForces;
    }

/// --------- Modeled Contact Positions -------------///
    int remainCont = cnbr;
    int contactIndex =0;
    if (rightFootIn)
    {
      q = Quaternion(ark(indexes::rightFootOri),ark(indexes::rightFootOri+1),ark(indexes::rightFootOri+2),ark(indexes::rightFootOri+3));
      Rr = q.toRotationMatrix().transpose();
      uk.segment<12>(Input::contacts)<<
                                     ark.segment<3>(indexes::rightFootPos),
                                                 kine::rotationMatrixToRotationVector(Rr),
                                                 Vector3::Zero(),
                                                 Vector3::Zero();
      remainCont --;
      contactIndex+=12;
    }
    if (remainCont>0)
    {
      q = Quaternion(ark(indexes::leftFootOri),ark(indexes::leftFootOri+1),ark(indexes::leftFootOri+2),ark(indexes::leftFootOri+3));
      Rl = q.toRotationMatrix().transpose();
      uk.segment<12>(Input::contacts+contactIndex)<<
          ark.segment<3>(indexes::leftFootPos),
                      kine::rotationMatrixToRotationVector(Rl),
                      Vector3::Zero(),
                      Vector3::Zero();
      remainCont --;
      contactIndex+=12;
    }
    u.setValue(uk,k);
    y.setValue(yk,k);
    nbrOfContacts.setValue(nbrCont,k);
  }
}

void kanekoExtForceEstMethod(const IndexedMatrixArray & xhat,
                             const IndexedMatrixArray & y,
                             const IndexedMatrixArray & u,
                             double mass, double dt,
                             IndexedMatrixArray & estForce)
{
  Vector3 filtForce;
  filtForce.setZero();

  for (int k=xhat.getFirstIndex(); k<xhat.getNextIndex(); ++k)
  {
    Vector3 acc,force1,force2;

    Vector xhatk=xhat[k];
    Vector yk=y[k];
    Vector uk=u[k];

    acc=kine::rotationVectorToRotationMatrix(xhatk.segment<3>(state::ori))*
        kine::rotationVectorToRotationMatrix(uk.segment<3>(Input::oriIMU))*
        yk.head<3>();

    force1=kine::rotationVectorToRotationMatrix(xhatk.segment<3>(state::ori))*
           kine::rotationVectorToRotationMatrix(uk.segment<3>(Input::contacts+3))*
           yk.segment<3>(6);

    force2=kine::rotationVectorToRotationMatrix(xhatk.segment<3>(state::ori))*
           kine::rotationVectorToRotationMatrix(uk.segment<3>(Input::contacts+12))*
           yk.segment<3>(12);

    Vector3 force;

    force =mass*acc -force1 -force2;

    double cutoffFreq = 100;

    filtForce+= cutoffFreq*dt *( force -filtForce );

    estForce.pushBack(filtForce);


  }
}



/// ///////////////////////////////////////////////////////////////

int main (int argc, char *argv[])
{

  std::string filename;
  bool simulation=false;
  bool useHandSensor=true;
  bool withForceSensor = true;

  if (argc<2 || argc>5)
  {
    std::cout << "Parameters : filename"<<std::endl;
    std::cout << "Leaving"<<std::endl;

    std::exit(1);

  }

  filename=argv[1];

  std::cout << "Use of hand force sensor: ";

  if (std::string(argv[2])=="-u")
  {
    useHandSensor = true;
    std::cout << "True "<< std::endl;
  }
  else
  {
    useHandSensor= false;
    std::cout << "False "<< std::endl;
  }

  std::cout << "Simulation: ";

  if (argc>3)
    if (std::string(argv[3])=="-s")
      simulation = true;
    else
      simulation = false;

  if (simulation)
    std::cout << "true"<<std::endl;
  else
    std::cout << "false"<<std::endl;

  std::cout << "Use Force sensor ";

  if (argc>4)
    if (std::string(argv[4])=="-f")
      withForceSensor = true;
    else
      withForceSensor = false;

  if (withForceSensor)
    std::cout << "true"<<std::endl;
  else
    std::cout << "false"<<std::endl;

  IndexedMatrixArray initialFile;
  IndexedMatrixArray y,u,numberOfContacts;

  std::cout << "Readind initial file "<<filename<< std::endl;
  initialFile.readVectorsFromFile(filename.c_str(),false);
  std::cout << "File loaded, size:"<< initialFile.size() << std::endl;

  double dt;
  double mass;



  loadInputsAndMeasurements(initialFile,u,y,numberOfContacts,mass,dt,simulation,useHandSensor,withForceSensor);

  Matrix xh0 = Vector::Zero(state::size,1);

  std::cout << "Rebuiding state" << std::endl;


  IndexedMatrixArray prediction, predictedMea, innovation, simumea;

  IndexedMatrixArray Q, R;

  Matrix Qi = Matrix::Zero(state::size, state::size);

  Vector Qidiag(state::size);
  Qidiag.setZero();

  Qidiag.segment<12>(state::pos).setConstant(0); /// kinematics
  Qidiag.segment<12>(state::fc).setConstant(1e-15); /// contact forces model
  Qidiag.segment<2>(state::fc)=Qidiag.segment<2>(state::fc+6).setConstant(1e0) ;///tangential forces model
  Qidiag(state::fc+5)=Qidiag(state::fc+11) = 1e-15; ///yaw moment model
  Qidiag.segment<6> (state::unmodeledForces).setConstant(1e-5); /// unmodeled forces and torques


  if (!withForceSensor)
  {
    Qidiag.segment<6> (state::unmodeledForces).setConstant(1e-8); /// unmodeled forces and torques  (force sensor NOT USed)
    Qidiag.segment<2>(state::fc)=Qidiag.segment<2>(state::fc+6).setConstant(1e-15) ;///tangential forces model (force sensor NOT USed)
  }



  //Qidiag.segment<3> (state::unmodeledForces).setConstant(0); /// unmodeled forces
  //Qidiag(state::unmodeledForces+5)=0;///yaw umodeled forces  (force sensor NOT USed)

  Qi.diagonal()<<Qidiag;
  Q.pushBack(Qi);

  Matrix Ri;

  if (withForceSensor)
  {
    Ri = Matrix::Zero(18,18);
  }
  else
  {
    Ri = Matrix::Zero(6,6);
  }
  Ri.diagonal().segment<3>(0).setConstant(1e-8); ///accelerometer
  Ri.diagonal().segment<3>(0).setConstant(1e-8); ///accelerometer (when force sensor is NOT used)
  if (withForceSensor)
  {
    Ri.diagonal().segment<3>(3).setConstant(1e-8); ///gyrometer
  }
  else
  {
    Ri.diagonal().segment<3>(3).setConstant(1e-8); ///gyrometer
  }
  if (withForceSensor)
  {
    Ri.diagonal().segment<12>(6).setConstant(1e-15); ///force sensor
  }


  R.pushBack(Ri);

  //setting stiffness
  Matrix3 kfe= Matrix3::Zero();
  Matrix3 kfv = Matrix3::Zero();
  Matrix3 kte = Matrix3::Zero();
  Matrix3 ktv = Matrix3::Zero();

  kfe.diagonal()<<40000,
               40000,
               100000;
  kte.diagonal()<<400,
               400,
               6000;
  kfv=6000*Matrix3::Identity();
  ktv=60*Matrix3::Identity();


  IndexedMatrixArray xhat=
    examples::offlineModelBaseFlexEstimation( y, u, xh0, numberOfContacts,
        dt, mass, withForceSensor, Q, R,
        kfe,kfv,
        kte,ktv,
        &prediction, &innovation, &predictedMea,
        &simumea, 1);
  std::cout << "State rebuilt" << std::endl;

  std::cout << "Last value" << std::endl << xhat[xhat.getLastIndex()].transpose() <<std::endl;

  std::cout << "Kaneko Estimation start" <<std::endl;
  IndexedMatrixArray kanekoEstimation;

  kanekoExtForceEstMethod(xhat,y,u,mass,dt,kanekoEstimation);
  std::cout << "Done" << std::endl;



  numberOfContacts.writeInFile("contNum.log");
  y.writeInFile("y.log");
  u.writeInFile("u.log");
  xhat.writeInFile("xhat.log");

  prediction.writeInFile("prediction.log");
  innovation.writeInFile("innovation.log");
  predictedMea.writeInFile("predictedMeasurement.log");
  simumea.writeInFile("simulatedMeasurement.log");

  kanekoEstimation.writeInFile("kanekoEstimation");

  std::cout << "mass " << mass<<std::endl;
  std::cout << "dt " << dt<<std::endl;

  return 0;

}





