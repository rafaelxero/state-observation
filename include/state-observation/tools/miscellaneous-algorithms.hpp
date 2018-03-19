/**
 * \file      miscellaneous-algorithms.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief      Gathers many kinds of algorithms
 *
 *
 *
 */


#ifndef STATEOBSERVATIONTOOLSMISCELANEOUSALGORITHMS
#define STATEOBSERVATIONTOOLSMISCELANEOUSALGORITHMS


#include <boost/utility.hpp>

#include <state-observation/tools/definitions.hpp>


namespace stateObservation
{
    namespace tools
    {

        ///computes the square of a value of any type
        template <class T>
        inline T square (const T & x)
        {
            return T(x*x);
        }

        ///derivates any type with finite differences
        template <class T>
        inline T derivate(const T & o1 , const T & o2 , double dt)
        {
            T o(o2-o1);
            return o*(1/dt);
        }

        ///gives the sign of a variable (1, 0 or -1)
        template <typename T> inline
        int signum(T x)
        {
          return (T(0) < x) - (x < T(0));
        }

        template<typename T>
        std::string toString(T val)
        {
          std::stringstream ss("");
          ss << val;
          return ss.str();
        }
    }


}



#endif //STATEOBSERVATIONTOOLSMISCELANEOUSALGORITHMS
