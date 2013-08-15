#ifndef FCL_EXCEPTION_H
#define FCL_EXCEPTION_H

#include <stdexcept>
#include <string>

namespace fcl
{

class Exception : public std::runtime_error
{
public:

  /** \brief This is just a wrapper on std::runtime_error */
  explicit
  Exception(const std::string& what) : std::runtime_error(what)
  {
  }

  /** \brief This is just a wrapper on std::runtime_error with a
      prefix added */
  Exception(const std::string &prefix, const std::string& what) : std::runtime_error(prefix + ": " + what)
  {
  }

  virtual ~Exception(void) throw()
  {
  }

};

}


#endif
