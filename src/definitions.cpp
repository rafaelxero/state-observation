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
    }

}
