#ifndef FCL_CCD_INTERPOLATION_INTERPOLATION_FACTORY_H
#define FCL_CCD_INTERPOLATION_INTERPOLATION_FACTORY_H

#include "fcl/data_types.h"
#include "fcl/ccd/interpolation/interpolation.h"

#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace fcl 
{

class InterpolationFactory
{	
public:
  typedef boost::function<boost::shared_ptr<Interpolation>(FCL_REAL, FCL_REAL)> CreateFunction;

public:
  void registerClass(const InterpolationType type, const CreateFunction create_function);

  boost::shared_ptr<Interpolation> create(const InterpolationType type, FCL_REAL start_value, FCL_REAL end_value);

public:
  static InterpolationFactory& instance();

private:
  InterpolationFactory();

  InterpolationFactory(const InterpolationFactory&)
  {}

  InterpolationFactory& operator = (const InterpolationFactory&)
  {
    return *this;
  }

private:
  std::map<InterpolationType, CreateFunction> creation_map_;
};

}

#endif /* #ifndef FCL_INTERPOLATION_FACTORY_H */
