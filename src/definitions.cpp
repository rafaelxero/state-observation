#include <fstream>
#include <sstream>
#include <stdexcept>
#include <state-observation/tools/definitions.hpp>


namespace stateObservation
{

    namespace checkedItemDetail
    {
      const bool defaultTrue = true;
      const char* defaultErrorMSG = "The Object is not initialized. \
         If this happened during initialization then run command chckitm_set() \
         to switch it to set. And if the initialization is incomplete, run \
         chckitm_reset() afterwards.";
      const std::runtime_error defaultException(defaultErrorMSG);
      const std::exception* defaultExcepionAddr=&defaultException;
    }


    ///Default constructor
    IndexedMatrixArray::IndexedMatrixArray():
            k_(0)
    {
    }

    std::vector<Matrix> IndexedMatrixArray::getArray() const
    {
        std::vector<Matrix> v;

        for (unsigned i=0;i<v_.size();++i)
        {
            v.push_back(v_[i]);
        }

        return v;
    }

    void IndexedMatrixArray::truncate(unsigned time)
    {
        if (v_.size()>0)
        {
            if (time > getFirstIndex())
            {
                for (unsigned i=getLastIndex(); i>=time ;--i)
                {
                    v_.pop_back();
                }
            }
            else
            {
                v_.clear();
            }
        }
    }

    void IndexedMatrixArray::readFromFile(const char * filename , size_t rows, size_t cols, bool withTimeStamp)
    {
      reset();

      std::ifstream f;

      f.open(filename);

      if (f.is_open())
      {

        Matrix m(Matrix::Zero(rows,cols));

        bool continuation=true;
        int k=0;

        while (continuation)
        {


          if (withTimeStamp)
          {
            f >> k;
          }


          if (f.fail())
            continuation=false;
          else
          {
            for (size_t i = 0 ; i<rows; ++i)
            {
              for (size_t j = 0 ; j<cols; ++j)
              {
                f >> m(i,j);
              }
            }

            setValue(m,k);
            ++k;
          }
        }
      }
    }

    void IndexedMatrixArray::readVectorsFromFile(const char * filename , bool withTimeStamp )
    {
      reset();

      std::ifstream f;

      f.open(filename);

      if (f.is_open())
      {
        std::string s;
        Vector v;
        int k=0;

        bool continuation=true;

        while (continuation)
        {
          std::getline(f,s);

          std::stringstream ss(s);

          if (withTimeStamp)
          {
            ss >> k;
          }



          if (f.fail())
            continuation=false;
          else
          {
            int size=0;
            std::vector<double> doublecontainer;
            double component;
            bool readingVector = true;
            while (readingVector)
            {
              ss>> component;

              if (ss.fail())
              {
                readingVector=false;
              }
              else
              {
                doublecontainer.push_back(component);
              }
            }
            v.resize(doublecontainer.size());
            for (unsigned i=0 ; i<doublecontainer.size() ; ++i)
            {
              v(i)=doublecontainer[i];
            }
            setValue(v,k);
            ++k;

          }
        }
      }
    }

    void IndexedMatrixArray::writeInFile(const std::string & filename,  bool clearLog, bool append)
    {
      writeInFile(filename.c_str(),  clearLog, append);
    }


    void IndexedMatrixArray::writeInFile(const char * filename, bool clearLog, bool append)
    {
    	std::ofstream f;
    	if (!append)
      {
        f.open(filename);
      }
      else
      {
        f.open(filename,std::ofstream::app);
      }

      if (f.is_open())
      {
        if (size()>0)
        {

          for (size_t k=getFirstIndex();k<getNextIndex();++k)
          {

            f << k;

            Matrix & m = operator[](k);

            for (int i = 0 ; i< m.rows(); ++i)
            {
              for (int j = 0 ; j< m.cols(); ++j)
              {
            		f << " "<< m(i,j);
              }
            }
            f << std::endl;
          }
        }

        if (clearLog)
        {
          clear();
        }
      }
      else
      {
        std::stringstream ss;
        ss<< "Logger: File " <<filename<<" could not be created/opened.";
        std::runtime_error e(ss.str().c_str());
        throw e;
      }

    }

}
