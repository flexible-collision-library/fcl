#include "fcl/ccd/interpolation/interpolation_linear.h"
#include "fcl/ccd/interpolation/interpolation_factory.h"

namespace fcl 
{

InterpolationType interpolation_linear_type = LINEAR;

InterpolationLinear::InterpolationLinear() : Interpolation(0.0, 1.0)
{}

InterpolationLinear::InterpolationLinear(FCL_REAL start_value, FCL_REAL end_value) : Interpolation(start_value, end_value)
{}

FCL_REAL InterpolationLinear::getValue(FCL_REAL time) const
{
  return value_0_ + (value_1_ - value_0_) * time;
}

FCL_REAL InterpolationLinear::getValueLowerBound() const
{
  return value_0_;
}

FCL_REAL InterpolationLinear::getValueUpperBound() const
{
  return value_1_;
}

InterpolationType InterpolationLinear::getType() const
{
  return interpolation_linear_type;
}

boost::shared_ptr<Interpolation> InterpolationLinear::create(FCL_REAL start_value, FCL_REAL end_value)
{
  return boost::shared_ptr<Interpolation>(new InterpolationLinear(start_value, end_value) );
}

void InterpolationLinear::registerToFactory()
{
  InterpolationFactory::instance().registerClass(interpolation_linear_type, create);
}

FCL_REAL InterpolationLinear::getMovementLengthBound(FCL_REAL time) const
{
  return getValueUpperBound() - getValue(time);
}

FCL_REAL InterpolationLinear::getVelocityBound(FCL_REAL time) const
{
  return (value_1_ - value_0_);
}

}
