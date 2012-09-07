#include "fcl/ccd/interpolation/interpolation.h"

namespace fcl 
{

Interpolation::Interpolation() :
  value_0_(0.0),
  value_1_(1.0) 
{}

Interpolation::Interpolation(FCL_REAL start_value, FCL_REAL end_value) :
  value_0_(start_value),
  value_1_(end_value)
{}

void Interpolation::setStartValue(FCL_REAL start_value)
{
  value_0_ = start_value;
}

void Interpolation::setEndValue(FCL_REAL end_value)
{
  value_1_ = end_value;
}

bool Interpolation::operator == (const Interpolation& interpolation) const
{
  return 
    (this->getType() == interpolation.getType()) &&
    (this->value_0_ == interpolation.value_0_) &&
    (this->value_1_ == interpolation.value_1_);
}

bool Interpolation::operator != (const Interpolation& interpolation) const
{
  return !(*this == interpolation);
}

}
