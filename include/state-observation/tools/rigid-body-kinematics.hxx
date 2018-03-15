namespace stateObservation
{
  namespace algorithm

  {


    inline void RigidBodyKinematics::integrateKinematics(Vector3 & position, const Vector3 & velocity, double dt)
    {
      optRBKyn_.vector3Vel.noalias() = velocity*dt;
      position.noalias()+=optRBKyn_.vector3Vel;
    }

    inline void RigidBodyKinematics::integrateKinematics(Vector3 & position, Vector3 & velocity,
                                    const Vector3 & acceleration, double dt)
    {
      optRBKyn_.vector3Acc.noalias() = acceleration*dt;
      optRBKyn_.vector3Vel.noalias() = velocity*dt;
      optRBKyn_.vector3Vel.noalias() += optRBKyn_.vector3Acc*(dt*0.5);

      velocity.noalias()+=optRBKyn_.vector3Acc;
      position.noalias()+=optRBKyn_.vector3Vel;
    }

    inline void RigidBodyKinematics::integrateKinematics( Matrix3 & orientation, const Vector3 & rotationVelocity,
                                     double dt)
    {
      optRBKyn_.angAxis=kine::rotationVectorToAngleAxis(rotationVelocity);
      optRBKyn_.angAxis.angle() = optRBKyn_.angAxis.angle()*dt;

      optRBKyn_.rotMat.noalias() = optRBKyn_.angAxis.toRotationMatrix() * orientation ;

      orientation = optRBKyn_.rotMat;
    }


    inline void integrateKinematics( Matrix3 & orientation, Vector3 & rotationVelocity,
                                     const Vector3 & rotationAcceleration, double dt);

    inline void integrateKinematics( Quaternion & orientation, Vector3 & rotationVelocity,
                                     double dt);

    inline void integrateKinematics( Quaternion & orientation, Vector3 & rotationVelocity,
                                     const Vector3 & rotationAcceleration, double dt);

    inline void RigidBodyKinematics::integrateKinematics
    (Vector3 & position, Vector3 & velocity, const Vector3 & acceleration,
     Quaternion & orientation, Vector3 & rotationVelocityVector,
     const Vector3 & rotationVelocityVectorRate, double dt)
    {
      position +=  dt * velocity + 0.5 * dt * dt * acceleration;
      velocity +=  dt * acceleration;

      orientation = Quaternion( kine::rotationVectorToAngleAxis
                                (rotationVelocityVector*dt) )
                    * orientation;

      rotationVelocityVector += dt * rotationVelocityVectorRate;

    }

    inline void RigidBodyKinematics::integrateKinematics
    (Vector3 & position, Vector3 & velocity, const Vector3 & acceleration,
     Matrix3 & orientation, Vector3 & rotationVelocityVector,
     const Vector3 & rotationVelocityVectorRate, double dt)
    {
      position.noalias() +=  dt * velocity;
      position.noalias() +=  + 0.5 * dt * dt * acceleration;
      velocity +=  dt * acceleration;

      orientation = kine::rotationVectorToAngleAxis (rotationVelocityVector*dt + 0.5 * dt * dt*rotationVelocityVectorRate).toRotationMatrix()
                    * orientation;

      rotationVelocityVector += dt * rotationVelocityVectorRate;

    }

    inline void RigidBodyKinematics::integrateKinematics(Vector3 & position,
        const Vector3 & velocity, Matrix3 & orientation,
        const Vector3 & rotationVelocity, double dt)
    {
      position.noalias() +=  dt * velocity;

      orientation = kine::rotationVectorToAngleAxis (rotationVelocity*dt).toRotationMatrix()
                    * orientation;
    }
  }
}
