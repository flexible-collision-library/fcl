#include "fcl/ccd/interpolation/interpolation_factory.h"
#include "fcl/ccd/interpolation/interpolation_linear.h"

#include <boost/assert.hpp>

namespace fcl 
{

InterpolationFactory::InterpolationFactory()
{
  InterpolationLinear::registerToFactory();
}

InterpolationFactory& InterpolationFactory::instance()
{
  static InterpolationFactory instance;

  return instance;
}

void InterpolationFactory::registerClass(const InterpolationType type, const CreateFunction create_function)
{
  this->creation_map_[type] = create_function;
}

boost::shared_ptr<Interpolation> 
InterpolationFactory::create(const InterpolationType type, const FCL_REAL start_value, const FCL_REAL end_value)
{
  std::map<InterpolationType, CreateFunction>::const_iterator it = creation_map_.find(type);

  BOOST_ASSERT_MSG((it != creation_map_.end()), "CreateFunction wasn't found.");

  return (it->second)(start_value, end_value);  
}

}
