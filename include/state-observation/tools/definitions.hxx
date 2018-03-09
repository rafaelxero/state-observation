template <typename T, bool lazy, bool alwaysCheck , bool assertion, bool eigenAlignedNew>
CheckedItem<T,lazy,alwaysCheck,assertion , eigenAlignedNew>::CheckedItem():
  IsSet(false)
{
}

template <typename T, bool lazy, bool alwaysCheck , bool assertion, bool eigenAlignedNew>
CheckedItem<T,lazy,alwaysCheck,assertion, eigenAlignedNew>::CheckedItem(const T&):
  IsSet(true)
{
}

template <typename T, bool lazy, bool alwaysCheck , bool assertion, bool eigenAlignedNew>
inline T CheckedItem<T,lazy,alwaysCheck , assertion,eigenAlignedNew>::operator=(const T& v)
{
  IsSet::set(true);
  return v_=v;
}

template <typename T, bool lazy, bool alwaysCheck , bool assertion, bool eigenAlignedNew>
inline  CheckedItem<T,lazy,alwaysCheck , assertion,eigenAlignedNew>::operator T() const
{
  chckitm_check_();
  return v_;
}

template <typename T, bool lazy, bool alwaysCheck , bool assertion, bool eigenAlignedNew>
inline bool CheckedItem<T,lazy,alwaysCheck , assertion,eigenAlignedNew>::chckitm_check_() const throw(std::exception)
{
  if (assertion)
  {
    BOOST_ASSERT_MSG(chckitm_isSet(),AssertMsg::get());
  }

  if (alwaysCheck || isDebug)
  {
    if (!chckitm_isSet())
    {
      throw (*(ExceptionPtr::get()));
    }
  }
  return (chckitm_isSet());
}

template <typename T, bool lazy, bool alwaysCheck , bool assertion, bool eigenAlignedNew>
inline bool CheckedItem<T,lazy,alwaysCheck , assertion,eigenAlignedNew>::chckitm_isSet() const
{
  return IsSet::get();
}

template <typename T, bool lazy, bool alwaysCheck , bool assertion, bool eigenAlignedNew>
inline void CheckedItem<T,lazy,alwaysCheck , assertion,eigenAlignedNew>::chckitm_reset()
{
  IsSet::set(false);
}

template <typename T, bool lazy, bool alwaysCheck , bool assertion, bool eigenAlignedNew>
inline void CheckedItem<T,lazy,alwaysCheck , assertion,eigenAlignedNew>::chckitm_set(bool value)
{
  IsSet::set(value);
}


template <typename T, bool lazy, bool alwaysCheck , bool assertion, bool eigenAlignedNew>
inline void CheckedItem<T,lazy,alwaysCheck , assertion,eigenAlignedNew>::chckitm_setAssertMessage(std::string s)
{
  AssertMsg::set(s);
}

template <typename T, bool lazy, bool alwaysCheck , bool assertion, bool eigenAlignedNew>
inline void CheckedItem<T,lazy,alwaysCheck , assertion,eigenAlignedNew>::chckitm_setExceptionPtr(std::exception* e)
{
  ExceptionPtr::set(e);
}





template <typename MatrixType, bool lazy>
inline IndexedMatrixT<MatrixType,lazy>::IndexedMatrixT(const MatrixType& v,unsigned k):
  IsSet(true),
  k_(k),
  v_(v)
{
}

template <typename MatrixType, bool lazy>
inline IndexedMatrixT<MatrixType,lazy>::IndexedMatrixT():
  IsSet(false),
  k_(0),
  v_(Matrix::Zero(0,0))
{
}

///Says whether the matrix is initialized or not
template <typename MatrixType, bool lazy>
inline bool IndexedMatrixT<MatrixType,lazy>::isSet()const
{
    return (IsSet::get());
}

///Set the value of the matrix and the time sample
template <typename MatrixType, bool lazy>
inline void IndexedMatrixT<MatrixType,lazy>::set(const MatrixType& v,unsigned k)
{
  IsSet::set(true);
  k_=k;
  v_=v;
}

///Set the value of the matrix and the time sample
template <typename MatrixType, bool lazy>
inline void IndexedMatrixT<MatrixType,lazy>::reset()
{
  IsSet::set(false);
}



///Checks whether the matrix is set or not (assert)
///does nothing in release mode
template <typename MatrixType, bool lazy>
inline bool IndexedMatrixT<MatrixType,lazy>::check_() const
{
    BOOST_ASSERT(isSet() && "Error: Matrix not initialized, if you are initializing it, \
                            use set() function.");
    return isSet();
}




template <typename MatrixType, bool lazy>
inline void IndexedMatrixT<MatrixType,lazy>::setIndex(int k)
{
  check_();
  k_=k;
}

///Get the matrix value
template <typename MatrixType, bool lazy>
inline  const MatrixType & IndexedMatrixT<MatrixType,lazy>::operator()()const
{
  check_();
  return v_;
}

///Get the matrix value
template <typename MatrixType, bool lazy>
inline MatrixType & IndexedMatrixT<MatrixType,lazy>::operator()()
{
  check_();
  return v_;
}

///Get the time index
template <typename MatrixType, bool lazy>
unsigned IndexedMatrixT<MatrixType,lazy>::getTime()const
{
  check_();
  return k_;
}





///Set the value of the matrix and the time sample
inline void IndexedMatrixArray::setValue(const Matrix& v,unsigned k)
{
    if (checkIndex(k))
    {
        (*this)[k]=v;
    }
    else
    {
        checkNext_(k);
        if (v_.size()==0)
            k_=k;

        v_.push_back(v);
    }
}

inline void IndexedMatrixArray::pushBack(const Matrix& v)
{
    v_.push_back(v);
}

void IndexedMatrixArray::popFront()
{
    check_();
    v_.pop_front();
    ++k_;
}

///Get the matrix value
Matrix IndexedMatrixArray::operator[](unsigned time)const
{
    check_(time);
    return v_[time - k_];
}

///Get the matrix value
Matrix & IndexedMatrixArray::operator[](unsigned time)
{
    check_(time);
    return v_[time - k_];
}


///gets the first value
const Matrix & IndexedMatrixArray::front() const
{
    return v_.front();
}

///gets the first value
Matrix& IndexedMatrixArray::front()
{
    return v_.front();
}

///gets the last value
const Matrix & IndexedMatrixArray::back() const
{
    return v_.back();
}

///gets the last value
Matrix & IndexedMatrixArray::back()
{
    return v_.back();
}

///Get the time index
int IndexedMatrixArray::getLastIndex()const
{
  return k_+v_.size()-1;
}

///Get the time index
unsigned IndexedMatrixArray::getNextIndex()const
{
  return k_+v_.size();
}


///Get the time index
unsigned IndexedMatrixArray::getFirstIndex()const
{
  return k_;
}

unsigned IndexedMatrixArray::setLastIndex(int index)
{
  k_=index-(v_.size()+1);
}

unsigned IndexedMatrixArray::setFirstIndex(int index)
{
  k_=index;
}

unsigned IndexedMatrixArray::size() const
{
    return v_.size();
}

///Switch off the initialization flag, the value is no longer accessible
void IndexedMatrixArray::reset()
{
    k_=0;
    v_.clear();
}

void IndexedMatrixArray::clear()
{
  k_=k_+v_.size();
  v_.clear();
}

bool IndexedMatrixArray::checkIndex(unsigned time) const
{
    return (v_.size()>0 && k_<=time && k_+v_.size() > time);
}

///Checks whether the matrix is set or not (assert)
///does nothing in release mode
void IndexedMatrixArray::check_(unsigned time)const
{
    (void)time;//avoid warning
    BOOST_ASSERT(checkIndex(time) && "Error: Time out of range");
}

///Checks whether the matrix is set or not (assert)
///does nothing in release mode
void IndexedMatrixArray::check_()const
{
    BOOST_ASSERT(v_.size() && "Error: Matrix array is empty");
}

void IndexedMatrixArray::checkNext_(unsigned time)const
{
    (void)time;//avoid warning
    BOOST_ASSERT( (v_.size()==0 || k_+v_.size() == time )&&
                  "Error: New time instants must be consecutive to existing ones");
}

///resizes the array
void IndexedMatrixArray::resize(unsigned i, const Matrix & m )
{
    v_.resize(i,m);
}


