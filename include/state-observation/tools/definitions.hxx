///Set the value of the matrix and the time sample
inline void IndexedMatrix::set(const Matrix& v,unsigned k)
{
    k_=k;
    v_=v;
}

void IndexedMatrix::setIndex(int k)
{
  check_();
  k_=k;
}

///Get the matrix value
Matrix IndexedMatrix::operator()()const
{
    check_();
    return v_;
}

///Get the time index
unsigned IndexedMatrix::getTime()const
{
    check_();
    return k_;
}

///Says whether the matrix is initialized or not
bool IndexedMatrix::isSet()const
{
    return ( v_.rows()>0 && v_.cols() > 0 );
}

///Switch off the initalization flag, the value is no longer accessible
void IndexedMatrix::reset()
{
  k_=0;
  v_.resize(0,0);
}

///Checks whether the matrix is set or not (assert)
///does nothing in release mode
void IndexedMatrix::check_()const
{
    BOOST_ASSERT(isSet() && "Error: Matrix not initialized");
}

///Set the value of the matrix and the time sample
void IndexedMatrixArray::setValue(const Matrix& v,unsigned k)
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

void IndexedMatrixArray::pushBack(const Matrix& v)
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
unsigned IndexedMatrixArray::getLastIndex()const
{
  return k_+v_.size()-1;
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


