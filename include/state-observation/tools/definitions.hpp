/**
 * \file     definitions.hpp
 * \author   Mehdi Benallegue
 * \date     2013
 * \brief    Definitions of types and some structures.
 *
 * \details
 *
 *
 */

#ifndef STATEOBSERVATIONDEFINITIONSHPP
#define STATEOBSERVATIONDEFINITIONSHPP

//#define STATEOBSERVATION_VERBOUS_CONSTRUCTORS

#include <vector>
#include <deque>

#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
#   include <iostream>
#endif

#include <boost/assert.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>



// basic file operations
#include <fstream>


namespace stateObservation
{
    ///Dynamic sized scalar vector
    typedef Eigen::VectorXd Vector;

    ///3D vector
    typedef Eigen::Vector3d Vector3;

    ///3D vector unaligned
    typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector3Unaligned;

    ///4D vector
    typedef Eigen::Vector4d Vector4;

    /// 6D vector
    typedef Eigen::Matrix<double,6,1> Vector6;

    ///Dynamic sized Matrix
    typedef Eigen::MatrixXd Matrix;

    ///3x3 Scalar Matrix
    typedef Eigen::Matrix3d Matrix3;

    ///6x6 Scalar Matrix
    typedef Eigen::Matrix<double, 6, 6> Matrix6;

    ///3x3 Scalar Matrix Unaligned
    typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign> Matrix3Unaligned;

    ///3x3 Scalar Matrix
    typedef Eigen::Matrix3d Matrix3;

    ///4x4 Scalar Matrix
    typedef Eigen::Matrix4d Matrix4;

    ///Quaternion
    typedef Eigen::Quaterniond Quaternion;

    ///Quaternion Unaligned
    typedef Eigen::Quaternion<double, Eigen::DontAlign> QuaternionUnaligned;

    ///Euler Axis/Angle representation of orientation
    typedef Eigen::AngleAxis<double> AngleAxis;

#undef SO_DEBUG_ONLY(expr)
#ifndef NDEBUG
  static const bool isDebug=true;
#else
  static const bool isDebug=false;
#endif // NDEBUG

  template <typename T, const T& defaultValue=T(), bool debug=true>
  class DebugItem
  {
  public:
    DebugItem():b_(defaultValue) {}
    explicit DebugItem(const T& v):b_(v) {}
    inline T operator=(T v)
    {
      return b_=v;
    }
    inline operator T()const
    {
      return b_;
    }
    inline T set(const T& v)
    {
      return b_=v;
    }
    T get()const
    {
      return b_;
    }
  private:
    T b_;

  };

  template <typename T, const T& defaultValue>
  class DebugItem<T,defaultValue,false>
  {
    DebugItem() {}
    explicit DebugItem(T v) {}
    inline T operator=(T v)
    {
      return defaultValue;
    }
    inline operator T() const
    {
      return defaultValue;
    }
    inline T set(T v)
    {
      return defaultValue;
    }
    T get()const
    {
      return defaultValue;
    }
  private:
    ///no boolean
  };

 
    /**
     * \class    IndexedMatrix
     * \brief    This class describes a structure composed by a matrix
     *           of a given size and a time-index parameter. It can tell also if
     *           it initialized or not.
     *
     *
     */
    class IndexedMatrix
    {
    public:
        ///Default constructor
        IndexedMatrix();

        ///A constructor with a given matrix value and a time index
        IndexedMatrix(const Matrix& v, unsigned k);

        ///Set the value of the matrix and the time sample
        inline void set(const Matrix& v,unsigned k);

        ///set the index of the matrix
        inline void setIndex(int index);

        ///Get the matrix value
        inline Matrix operator()() const;

        ///Get the time index
        inline unsigned getTime() const;

        ///Says whether the matrix is initialized or not
        inline bool isSet() const;

        ///Switch off the initalization flag, the value is no longer accessible
        inline void reset();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        ///Checks whether the matrix is set or not (assert)
        ///does nothing in release mode
        inline void check_() const;

        unsigned k_;
        Matrix v_;
    };

    /**
     * \class    IndexedMatrix
     * \brief    This class describes a structure that enables to store array of matrices
     *           with time indexation.
     *
     */

    class IndexedMatrixArray
    {
        public:
        ///Default constructor
        IndexedMatrixArray();

        ///Sets the vector v at the time index k
        ///It checks the time index, the array must have contiguous indexes
        ///It can be used to push a value into the back of the array
        inline void setValue(const Matrix& v,unsigned k);

        ///Pushes back the matrix to the array, the new value will take the next time
        inline void pushBack(const Matrix& v);

        ///removes the first (oldest) element of the array
        inline void popFront();

        ///gets the value with the given time index
        inline Matrix operator[](unsigned index) const;

        ///gets the value with the given time index, non const version
        inline Matrix  & operator[](unsigned index);

        ///gets the first value
        inline const Matrix & front() const;

        ///gets the first value
        inline Matrix& front();

        ///gets the last value
        inline const Matrix & back() const;

        ///gets the last value
        inline Matrix & back();

        ///removes all the elements with larger or equal indexes than timeIndex
        void truncate(unsigned timeIndex);

        ///resizes the array
        inline void resize(unsigned i, const Matrix & m= Matrix::Zero(0,0));

        ///Get the time index
        inline int getLastIndex() const;

        ///Get the time index of the next value that will be pushed back
        /// Can be used in for loops
        inline unsigned getNextIndex() const;

        ///Set the time index of the last element
        inline unsigned setLastIndex(int index);

        ///Get the time index
        inline unsigned getFirstIndex() const;

        ///set the time index of the first element
        inline unsigned setFirstIndex(int index);

        inline unsigned size() const;

        ///Resets the array to initial state
        ///the value is no longer accessible
        inline void reset();

        ///Clears the vector but keeps the last index
        inline void clear();

        ///converts the array into a standard vector
        std::vector<Matrix> getArray() const;

        ///checks whether the index is present in the array
        inline bool checkIndex(unsigned k) const;

        ///gets the array from a file
        ///the line starts with the time index and then the matrix is read
        ///row by row
        ///WARNING: this resets the array
        void readFromFile(const char * filename, size_t rows, size_t cols=1, bool withTimeStamp = true);

        ///gets the array from a file
        ///the line starts with the time index and then every line of the file
        /// is converted into a vector
        ///WARNING: this resets the array
        void readVectorsFromFile(const char * filename, bool withTimeStamp = true );

        ///write the array in a a file
        ///the line starts with the time index and then the matrix is described
        ///row by row
        /// When clear is set, the array is cleared but the time index is conserved
        /// When append is set to true, the output is appended to file
        void writeInFile(const char * filename, bool clear=false, bool append =false);

        ///write the array in a a file
        ///the line starts with the time index and then the matrix is described
        ///row by row
        /// When clear is set, the array is cleared but the time index is conserved
        /// When append is set to true, the output is appended to file
        void writeInFile(const std::string & filename, bool clear=false, bool append =false);

    protected:
        ///Asserts that the index is present in the array
        ///does nothing in release mode
        inline void check_(unsigned time) const;

        ///Asserts that the array is not empty
        ///does nothing in release mode
        inline void check_() const;

        inline void checkNext_(unsigned time) const;

        unsigned k_;

        std::deque<Matrix> v_;

    };

    namespace kine
    {
        ///indexes of the different components of a vector of the kinematic state
        const unsigned pos = 0;
        const unsigned ori = 3;
        const unsigned linVel = 6;
        const unsigned angVel = 9;
        const unsigned linAcc = 12;
        const unsigned angAcc = 15;
    }

    namespace cst
    {
        const double gravityConstant = 9.8;

        ///Gravity Vector along Z
        const Vector gravity= gravityConstant * Vector3::UnitZ();

        ///angles considered Zero
        const double epsilonAngle=1e-16;

    }


    #include <state-observation/tools/definitions.hxx>
}

#endif //STATEOBSERVATIONDEFINITIONSHPP
