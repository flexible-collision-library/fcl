#ifndef FCL_CCD_INTERPOLATION_INTERPOLATION_H
#define FCL_CCD_INTERPOLATION_INTERPOLATION_H

#include "fcl/data_types.h"

namespace fcl
{

enum InterpolationType
{
  LINEAR,
  STANDARD
};

class Interpolation
{
public:
  Interpolation();

  virtual ~Interpolation() {}

  Interpolation(FCL_REAL start_value, FCL_REAL end_value);

  void setStartValue(FCL_REAL start_value);
  void setEndValue(FCL_REAL end_value);

  virtual FCL_REAL getValue(FCL_REAL time) const = 0;

  /// @brief return the smallest value in time interval [0, 1]
  virtual FCL_REAL getValueLowerBound() const = 0;

  /// @brief return the biggest value in time interval [0, 1]
  virtual FCL_REAL getValueUpperBound() const = 0;

  virtual InterpolationType getType() const = 0;

  bool operator == (const Interpolation& interpolation) const;
  bool operator != (const Interpolation& interpolation) const;

  virtual FCL_REAL getMovementLengthBound(FCL_REAL time) const = 0;

  virtual FCL_REAL getVelocityBound(FCL_REAL time) const = 0;

protected:
  FCL_REAL value_0_; // value at time = 0.0
  FCL_REAL value_1_; // value at time = 1.0

};



}

#endif
