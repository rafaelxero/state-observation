#include <fstream>
#include <sstream>
#include <stdexcept>
#include <state-observation/tools/definitions.hpp>


namespace stateObservation
{

  namespace detail
  {
    const bool defaultTrue = true;
    const char* defaultErrorMSG = "The Object is not initialized. \
         If this happened during initialization then run command chckitm_set() \
         to switch it to set. And if the initialization is incomplete, run \
         chckitm_reset() afterwards.";
    const std::runtime_error defaultException(defaultErrorMSG);
    const std::exception* defaultExcepionAddr=&defaultException;


    void defaultSum(const  Vector& stateVector, const Vector& tangentVector, Vector& result)
    {
      result.noalias() = stateVector + tangentVector;
    }

    void defaultDifference(const  Vector& stateVector1, const Vector& stateVector2, Vector& difference)
    {
      difference.noalias() = stateVector1 - stateVector2;
    }

  }

}
