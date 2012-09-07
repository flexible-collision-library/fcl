#ifndef FCL_CCD_INTERPOLATION_INTERPOLATION_LINEAR_H
#define FCL_CCD_INTERPOLATION_INTERPOLATION_LINEAR_H

#include "fcl/data_types.h"
#include "fcl/ccd/interpolation/interpolation.h"

#include <boost/shared_ptr.hpp>

namespace fcl 
{

class InterpolationFactory;

class InterpolationLinear : public Interpolation
{
public:
  InterpolationLinear();

  InterpolationLinear(FCL_REAL start_value, FCL_REAL end_value);

  virtual FCL_REAL getValue(FCL_REAL time) const;

  virtual FCL_REAL getValueLowerBound() const;
  virtual FCL_REAL getValueUpperBound() const;

  virtual InterpolationType getType() const;

  virtual FCL_REAL getMovementLengthBound(FCL_REAL time) const;

  virtual FCL_REAL getVelocityBound(FCL_REAL time) const;

public:
  static boost::shared_ptr<Interpolation> create(FCL_REAL start_value, FCL_REAL end_value);

  static void registerToFactory();
};

}

#endif
