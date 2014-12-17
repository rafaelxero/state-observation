 /*
 * dynamical-system.cpp
 *
 *  Created on: 19 mai 2014
 *      Author: alexis
 */



#include <state-observation/flexibility-estimation/imu-elastic-local-frame-dynamical-system.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>
#include <stdexcept>

//#include <iostream>

namespace stateObservation
{
namespace flexibilityEstimation
{
    using namespace stateObservation;



   IMUElasticLocalFrameDynamicalSystem::
    	IMUElasticLocalFrameDynamicalSystem(double dt):
        processNoise_(0x0), dt_(dt),
        measurementSize_(measurementSizeBase_),
        robotMassInv_(1/hrp2::m),
        robotMass_(hrp2::m)
    {
#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
       // std::cout<<std::endl<<"IMUElasticLocalFrameDynamicalSystem Constructor"<<std::endl;

#endif //STATEOBSERVATION_VERBOUS_CONSTRUCTOR
      Kfe_=40000*Matrix3::Identity();
      Kte_=600*Matrix3::Identity();
      Kfv_=600*Matrix3::Identity();
      Ktv_=60*Matrix3::Identity();

      sensor_.setMatrixMode(true);
      contactModel_=0;


      kcurrent_=-1;

    }

    IMUElasticLocalFrameDynamicalSystem::
                    ~IMUElasticLocalFrameDynamicalSystem()
    {
        //dtor
    }

    void IMUElasticLocalFrameDynamicalSystem::setRobotMass(double m)
    {
      robotMass_ = m;
      robotMassInv_=1/m;
    }

   void IMUElasticLocalFrameDynamicalSystem::setContactModelNumber(unsigned nb)
   {
       contactModel_=nb;
   }

    void IMUElasticLocalFrameDynamicalSystem::computeElastPendulumForcesAndMoments
                              (const IndexedMatrixArray& PeArray,
                               const IndexedMatrixArray& PrArray,
                               const Vector3& position, const Vector3& linVelocity,
                               const Vector3& oriVector, const Matrix3& orientation,
                               const Vector3& angVel,
                               Vector3& forces, Vector3& moments)
    {
        unsigned nbContacts(getContactsNumber());
        fc_.resize(nbContacts*3);
        tc_.resize(nbContacts*3);
        forces.setZero();
        moments.setZero();


        Vector3 forcei;
        Vector3 momenti;
        Vector3 globalContactPos;

        Vector3 contactOriUnitVector;

        double stringLength;
        double modifiedStringLength;

        Matrix3 u;

        for (int i = 0; i<nbContacts ; ++i)
        {
            globalContactPos = position ;
            globalContactPos.noalias() += orientation*PrArray[i] ;

            stringLength=(PrArray[i]-PeArray[i]).norm();
            modifiedStringLength=(globalContactPos-PeArray[i]).norm();
            contactOriUnitVector= (PeArray[i]-globalContactPos)/modifiedStringLength;

            forcei = (modifiedStringLength-stringLength)*Kfe_*contactOriUnitVector;
            fc_.segment<3>(3*i)= forcei;
            forces += forcei;

            momenti.noalias() = kine::skewSymmetric(globalContactPos)*forcei;
            tc_.segment<3>(3*i)= momenti;
            moments += momenti;
        }

        moments.noalias() += - Kte_*oriVector;
        moments.noalias() += - Ktv_*angVel;
        forces.noalias() += -Kfv_*linVelocity;
    }


    Vector IMUElasticLocalFrameDynamicalSystem::getForcesAndMoments()
    {
        Vector x;
        x.resize(fc_.size()+tc_.size());

        x.head(fc_.size()) = fc_;
        x.tail(tc_.size()) = tc_;

        return x;
    }

    void IMUElasticLocalFrameDynamicalSystem::computeElastFeetContactForcesAndMoments
                              (const IndexedMatrixArray& contactPosArray,
                               const IndexedMatrixArray& contactOriArray,
                               const Vector3& position, const Vector3& linVelocity,
                               const Vector3& oriVector, const Matrix3& orientation,
                               const Vector3& angVel,
                               Vector3& forces, Vector3& moments)
    {
        unsigned nbContacts(getContactsNumber());
        fc_.resize(nbContacts*3);
        tc_.resize(nbContacts*3);
        forces.setZero();
        moments.setZero();


      for (int i = 0; i<nbContacts ; ++i)
      {
        op_.contactPos = contactPosArray[i];

        op_.Rci = contactOriArray[i];
        op_.Rcit.noalias()= op_.Rci.transpose();

        op_.RciContactPos.noalias()= orientation*op_.contactPos;

        op_.globalContactPos = position;

        op_.globalContactPos += op_.RciContactPos ;

        op_.forcei.noalias() = - op_.Rci*Kfe_*op_.Rcit*(op_.globalContactPos-op_.contactPos);
        op_.forcei.noalias() += - op_.Rci*Kfv_*op_.Rcit*(kine::skewSymmetric(angVel)*op_.RciContactPos
                                  +linVelocity);

        fc_.segment<3>(3*i)= op_.forcei;

        forces += op_.forcei;

        op_.momenti.noalias() = -op_.Rci*Kte_*op_.Rcit*oriVector;
        op_.momenti.noalias() += -op_.Rci*Ktv_*op_.Rcit*angVel;
        op_.momenti.noalias() += kine::skewSymmetric(op_.globalContactPos)*op_.forcei;

        tc_.segment<3>(3*i)= op_.momenti;

        moments += op_.momenti;

        }
        
    }


    void IMUElasticLocalFrameDynamicalSystem::computeForcesAndMoments
                          (const IndexedMatrixArray& position1,
                           const IndexedMatrixArray& position2,
                           const Vector3& position, const Vector3& linVelocity,
                           const Vector3& oriVector, const Matrix3& orientation,
                           const Vector3& angVel,
                           Vector3& forces, Vector3& moments)
    {
        switch(contactModel_){
        case 1 : computeElastFeetContactForcesAndMoments(position1, position2, position, linVelocity, oriVector, orientation, angVel, forces, moments);
                 break;

        case 2 : computeElastPendulumForcesAndMoments(position1, position2, position, linVelocity, oriVector, orientation, angVel, forces, moments);
                 break;

        default: throw std::invalid_argument("IMUElasticLocalFrameDynamicalSystem");
        }
    }




    Vector3 IMUElasticLocalFrameDynamicalSystem::computeAccelerations
       (const Vector3& positionCom, const Vector3& velocityCom,
        const Vector3& accelerationCom, const Vector3& AngMomentum,
        const Vector3& dotAngMomentum,
        const Matrix3& Inertia, const Matrix3& dotInertia,
        const IndexedMatrixArray& contactPosV,
        const IndexedMatrixArray& contactOriV,
        const Vector3& position, const Vector3& linVelocity, Vector3& linearAcceleration,
        const Vector3 &oriVector ,const Matrix3& orientation,
        const Vector3& angularVel, Vector3& angularAcceleration)
    {
        op_.skewV=kine::skewSymmetric(angularVel);
        op_.skewV2=kine::skewSymmetric2(angularVel);
        op_.skewVR=op_.skewV * orientation;
        op_.skewV2R=op_.skewV2 * orientation;


        op_.rFlexT=orientation.transpose();

        computeForcesAndMoments (contactPosV, contactOriV,
                          position, linVelocity, oriVector, orientation,
                             angularVel, op_.fc, op_.tc);

        op_.wx2Rc=op_.skewV2R*positionCom;
        op_._2wxRv=2*op_.skewVR*velocityCom;
        op_.Ra = orientation*accelerationCom;
        op_.Rc = orientation * positionCom;
        op_.Rcp = op_.Rc+position;
        op_.RIRT = orientation*Inertia*op_.rFlexT;

        op_.vf =robotMassInv_*op_.fc;
        op_.vf.noalias() -= op_.Ra;
        op_.vf.noalias() -= op_._2wxRv;
        op_.vf.noalias() -= op_.wx2Rc;
        op_.vf.noalias() -= cst::gravity;

        op_.vt =op_.tc;
        op_.vt.noalias() -= op_.skewV * (op_.RIRT * angularVel);
        op_.vt.noalias() -= orientation * (dotInertia * (op_.rFlexT * angularVel)+dotAngMomentum) ;
        op_.vt.noalias() -= op_.skewVR * AngMomentum;
        op_.vt.noalias() -= robotMass_* (kine::skewSymmetric(position) *
                            (op_.wx2Rc+ op_._2wxRv + op_.Ra ));
        op_.vt.noalias() -= robotMass_* kine::skewSymmetric(op_.Rcp) * cst::gravity;

        angularAcceleration = ( op_.RIRT + robotMass_*kine::skewSymmetric2(op_.Rc)).llt().solve(
                                (op_.vt - robotMass_*kine::skewSymmetric(op_.Rcp)*op_.vf));

        linearAcceleration = op_.vf;
        linearAcceleration += kine::skewSymmetric(op_.Rc)*angularAcceleration;

    }

    void IMUElasticLocalFrameDynamicalSystem::iterateDynamicsEuler
             (const Vector3& positionCom, const Vector3& velocityCom,
              const Vector3& accelerationCom, const Vector3& AngMomentum,
              const Vector3& dotAngMomentum,
              const Matrix3& inertia, const Matrix3& dotInertia,
              const IndexedMatrixArray& contactPos,
              const IndexedMatrixArray& contactOri,
              Vector3& position, Vector3& linVelocity, Vector3& linearAcceleration,
              Vector3 &oriVector, Vector3& angularVel, Vector3& angularAcceleration,
              double dt)
    {

        op_.rFlex = computeRotation_(oriVector,0);
        //compute new acceleration with the current flex position and velocity and input


        //integrate kinematics with the last acceleration
        integrateKinematics(position, linVelocity, linearAcceleration,
            op_.rFlex,angularVel, angularAcceleration, dt);


        computeAccelerations (positionCom, velocityCom,
        accelerationCom, AngMomentum, dotAngMomentum,
        inertia, dotInertia,  contactPos, contactOri, position, linVelocity, linearAcceleration,
                       oriVector, op_.rFlex, angularVel, angularAcceleration);


        op_.orientationAA=op_.rFlex;
        oriVector=op_.orientationAA.angle()*op_.orientationAA.axis();
    }

    void IMUElasticLocalFrameDynamicalSystem::iterateDynamicsRK4
             (const Vector3& positionCom, const Vector3& velocityCom,
              const Vector3& accelerationCom, const Vector3& AngMomentum,
              const Vector3& dotAngMomentum,
              const Matrix3& inertia, const Matrix3& dotInertia,
              const IndexedMatrixArray& contactPos,
              const IndexedMatrixArray& contactOri,
              Vector3& position, Vector3& linVelocity, Vector3& linearAcceleration,
              Vector3 &oriVector, Vector3& angularVel, Vector3& angularAcceleration,
              double dt)
    {

        Matrix3 orientationFlex(computeRotation_(oriVector,0));

        Vector3 linAcc1;
        Vector3 angAcc1;


        Vector3 pos2;
        Vector3 oriv2;
        AngleAxis oriaa2;
        Matrix3 ori2;

        Vector3 linVelocity2;
        Vector3 angVelocity2;

        Vector3 linAcc2;
        Vector3 angAcc2;


        Vector3 pos3;
        Vector3 oriv3;
        AngleAxis oriaa3;
        Matrix3 ori3;

        Vector3 linVelocity3;
        Vector3 angVelocity3;

        Vector3 linAcc3;
        Vector3 angAcc3;


        Vector3 pos4;
        Vector3 oriv4;
        AngleAxis oriaa4;
        Matrix3 ori4;

        Vector3 linVelocity4;
        Vector3 angVelocity4;

        Vector3 linAcc4;
        Vector3 angAcc4;

        //////////1st/////////////
        computeAccelerations (positionCom, velocityCom,
                        accelerationCom, AngMomentum, dotAngMomentum,
                        inertia, dotInertia,  contactPos, contactOri,
                        position, linVelocity, linAcc1, oriVector,
                        orientationFlex, angularVel, angAcc1);


        //////////2nd//////////////
        pos2 = position;
        ori2 = orientationFlex;

        integrateConfiguration(pos2, linVelocity, ori2, angularVel, dt/2);

        oriaa2.fromRotationMatrix(ori2);
        oriv2 = oriaa2.angle()*oriaa2.axis();

        linVelocity2 = linVelocity + (dt/2)*linAcc1;
        angVelocity2 = angularVel + (dt/2)*angAcc1;


        computeAccelerations (positionCom, velocityCom,
                        accelerationCom, AngMomentum, dotAngMomentum,
                        inertia, dotInertia,  contactPos, contactOri,
                        pos2, linVelocity2, linAcc2, oriv2,
                        ori2, angVelocity2, angAcc2);

        ////////////3rd/////////////

        pos3 = position;
        ori3 = orientationFlex;

        integrateConfiguration( pos3, linVelocity2, ori3, angVelocity2, dt/2);

        oriaa3.fromRotationMatrix(ori3);
        oriv3 = oriaa3.angle()*oriaa3.axis();

        linVelocity3 = linVelocity + (dt/2)*linAcc2;
        angVelocity3 = angularVel + (dt/2)*angAcc2;

        computeAccelerations (positionCom, velocityCom,
                        accelerationCom, AngMomentum, dotAngMomentum,
                        inertia, dotInertia,  contactPos, contactOri,
                        pos3, linVelocity3, linAcc3, oriv3,
                        ori3, angVelocity3, angAcc3);


        ////////////4th/////////////
        pos4 = position;
        ori4 = orientationFlex;

        integrateConfiguration( pos4, linVelocity3, ori4, angVelocity3, dt);

        oriaa4.fromRotationMatrix(ori4);
        oriv4 = oriaa4.angle()*oriaa4.axis();

        linVelocity4 = linVelocity + (dt)*linAcc3;
        angVelocity4 = angularVel + (dt)*angAcc3;

        computeAccelerations (positionCom, velocityCom,
                        accelerationCom, AngMomentum, dotAngMomentum,
                        inertia, dotInertia,  contactPos, contactOri,
                        pos4, linVelocity4, linAcc4, oriv4,
                        ori4, angVelocity4, angAcc4);

        /////////////////////////////

        integrateConfiguration( position, linVelocity, orientationFlex, angularVel, dt/6);
        integrateConfiguration( position, linVelocity2, orientationFlex, angVelocity2, dt/3);
        integrateConfiguration( position, linVelocity3, orientationFlex, angVelocity3, dt/3);
        integrateConfiguration( position, linVelocity4, orientationFlex, angVelocity4, dt/6);

        linVelocity+= (dt/6)*(linAcc1+2*linAcc2+2*linAcc3+linAcc4);
        angularVel+= (dt/6)*(angAcc1+2*angAcc2+2*angAcc3+angAcc4);

        linearAcceleration = linAcc4;
        angularAcceleration = angAcc4;


        AngleAxis orientationAA(orientationFlex);
        oriVector=orientationAA.angle()*orientationAA.axis();
    }

    Vector IMUElasticLocalFrameDynamicalSystem::stateDynamics
        (const Vector& x, const Vector& u, unsigned k)
    {
        assertStateVector_(x);

        op_.positionFlex=x.segment(kine::pos,3);
        op_.velocityFlex=x.segment(kine::linVel,3);
        op_.accelerationFlex=x.segment(kine::linAcc,3);
        op_.orientationFlexV=x.segment(kine::ori,3);
        op_.angularVelocityFlex=x.segment(kine::angVel,3);
        op_.angularAccelerationFlex=x.segment(kine::angAcc,3);


        kine::computeInertiaTensor(u.segment<6>(input::inertia),op_.inertia);
        kine::computeInertiaTensor(u.segment<6>(input::dotInertia),op_.dotInertia);

        unsigned nbContacts(getContactsNumber());

        for (int i = 0; i<nbContacts ; ++i)
        {
          op_.contactPosV.setValue(u.segment<3>(input::contacts + 6*i),i);
          op_.contactOriV.setValue(u.segment<3>(input::contacts +6*i+3),i);
        }

        op_.positionCom=u.segment<3>(input::posCom);
        op_.velocityCom=u.segment<3>(input::velCom);
        op_.accelerationCom=u.segment<3>(input::accCom);
        op_.AngMomentum=u.segment<3>(input::angMoment);
        op_.dotAngMomentum=u.segment<3>(input::dotAngMoment);


        int subsample=1;
        for (int i=0; i<subsample; ++i)
        {
          iterateDynamicsEuler (
                          op_.positionCom, op_.velocityCom,
                          op_.accelerationCom, op_.AngMomentum, op_.dotAngMomentum,
                          op_.inertia, op_.dotInertia,  op_.contactPosV, op_.contactOriV,
                          op_.positionFlex, op_.velocityFlex, op_.accelerationFlex,
                          op_.orientationFlexV, op_.angularVelocityFlex,
                          op_.angularAccelerationFlex, dt_/subsample);
        }

        //x_{k+1}

        op_.xk1=x;

        op_.xk1.segment<3>(kine::pos) = op_.positionFlex;
        op_.xk1.segment<3>(kine::linVel) = op_.velocityFlex;
        op_.xk1.segment<3>(kine::linAcc) = op_.accelerationFlex;



        op_.xk1.segment<3>(kine::ori) =  op_.orientationFlexV;
        op_.xk1.segment<3>(kine::angVel) = op_.angularVelocityFlex;
        op_.xk1.segment<3>(kine::angAcc) = op_.angularAccelerationFlex;

        if (processNoise_!=0x0)
            return processNoise_->addNoise(op_.xk1);
         else
            return op_.xk1;
    }


    inline Matrix3& IMUElasticLocalFrameDynamicalSystem::computeRotation_
                (const Vector3 & x, int i)
    {
        Vector3 & oriV = op_.orientationVector(i);
        Matrix3 & oriR = op_.curRotation(i);
        if (oriV!=x)
        {
            oriV = x;
            oriR = kine::rotationVectorToAngleAxis(x).toRotationMatrix();
        }

        return oriR;
    }

    Vector IMUElasticLocalFrameDynamicalSystem::measureDynamics
                (const Vector& x, const Vector& u, unsigned k)
    {
        assertStateVector_(x);

        op_.positionFlex=x.segment(kine::pos,3);
        op_.velocityFlex=x.segment(kine::linVel,3);
        op_.accelerationFlex=x.segment(kine::linAcc,3);
        op_.orientationFlexV=x.segment(kine::ori,3);
        op_.angularVelocityFlex=x.segment(kine::angVel,3);
        op_.angularAccelerationFlex=x.segment(kine::angAcc,3);

        op_.rFlex =computeRotation_(op_.orientationFlexV,0);

        assertInputVector_(u);


        op_.positionControl=u.segment(input::posIMU,3);
        op_.velocityControl=u.segment(input::linVelIMU,3);
        op_.accelerationControl=u.segment(input::linAccIMU,3);
        op_.orientationControlV=u.segment(input::oriIMU,3);
        op_.angularVelocityControl=u.segment(input::angVelIMU,3);

        op_.rControl=computeRotation_(op_.orientationControlV,1);

        op_.rimu = op_.rFlex * op_.rControl;

        // Translation sensor dynamic
        Vector3 acceleration;

        acceleration << 0,0,0;

        acceleration += 2*kine::skewSymmetric(op_.angularVelocityFlex) * op_.rFlex * op_.velocityControl;
        acceleration += op_.accelerationFlex + op_.rFlex * op_.accelerationControl;
        acceleration += (
                    kine::skewSymmetric(op_.angularAccelerationFlex)
                    + tools::square(kine::skewSymmetric(op_.angularVelocityFlex))
                )*op_.rFlex * op_.positionControl;

        // Rotation sensor dynamic
        Vector3 angularVelocity( op_.angularVelocityFlex + op_.rFlex * op_.angularVelocityControl);

        // Set sensor state before measurement
        Vector v(Vector::Zero(15,1));
        v.head<9>() = Eigen::Map<Eigen::Matrix<double, 9, 1> >(&op_.rimu(0,0));
        v.segment<3>(9)=acceleration;
        v.tail<3>()=angularVelocity;

        sensor_.setState(v,k);

        // Measurement
        Vector y (Matrix::Zero(measurementSize_,1));
        y.head(sensor_.getMeasurementSize()) = sensor_.getMeasurements();

        return y;
    }

    void IMUElasticLocalFrameDynamicalSystem::setProcessNoise(NoiseBase * n)
    {
        processNoise_=n;
    }

    void IMUElasticLocalFrameDynamicalSystem::resetProcessNoise()
    {
        processNoise_=0x0;
    }

    void IMUElasticLocalFrameDynamicalSystem::setMeasurementNoise
                ( NoiseBase * n)
    {
        sensor_.setNoise(n);
    }

    void IMUElasticLocalFrameDynamicalSystem::resetMeasurementNoise()
    {
        sensor_.resetNoise();
    }

    void IMUElasticLocalFrameDynamicalSystem::setSamplingPeriod(double dt)
    {
        dt_=dt;
    }

    unsigned IMUElasticLocalFrameDynamicalSystem::getStateSize()
    {
        return stateSize_;
    }

    unsigned IMUElasticLocalFrameDynamicalSystem::getInputSize()
    {
        return inputSize_;
    }

    void IMUElasticLocalFrameDynamicalSystem::setInputSize(unsigned i)
    {
        inputSize_=i;
    }

    unsigned IMUElasticLocalFrameDynamicalSystem::getMeasurementSize()
    {
        return measurementSize_;
    }

    NoiseBase * IMUElasticLocalFrameDynamicalSystem::getProcessNoise() const
    {
        return processNoise_;
    }

    NoiseBase * IMUElasticLocalFrameDynamicalSystem::
                                                    getMeasurementNoise() const
    {
        return sensor_.getNoise();
    }

    void IMUElasticLocalFrameDynamicalSystem::setContactsNumber(unsigned i)
    {
        nbContacts_=i;
    }

    void IMUElasticLocalFrameDynamicalSystem::setContactPosition
                                        (unsigned i, const Vector3 & position)
    {
        BOOST_ASSERT( i< contactPositions_.size() &&
                    "ERROR: The index of contact is out of range.");

        contactPositions_[i] = position;
    }

    Vector3 IMUElasticLocalFrameDynamicalSystem::getContactPosition(unsigned i)
    {
        return contactPositions_[i];
    }


    inline unsigned IMUElasticLocalFrameDynamicalSystem::getContactsNumber(void) const
    {
        return  nbContacts_;
    }

    void IMUElasticLocalFrameDynamicalSystem::setKfe(const Matrix3 & m)
    {
        Kfe_=m;
    }

    void IMUElasticLocalFrameDynamicalSystem::setKfv(const Matrix3 & m)
    {
        Kfv_=m;
    }

    void IMUElasticLocalFrameDynamicalSystem::setKte(const Matrix3 & m)
    {
        Kte_=m;
    }

    void IMUElasticLocalFrameDynamicalSystem::setKtv(const Matrix3 & m)
    {
        Ktv_=m;
    }
}
}


