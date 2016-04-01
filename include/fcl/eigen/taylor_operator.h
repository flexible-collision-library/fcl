#ifndef FCL_CCD_TAYLOR_OPERATOR_H
#define FCL_CCD_TAYLOR_OPERATOR_H

#include <fcl/math/vec_3f.h>
#include <fcl/math/matrix_3f.h>

namespace fcl {

class TVector3;
class TMatrix3;

template<int Col> struct TaylorReturnType {};
template<> struct TaylorReturnType<1> { typedef TVector3 type; typedef Vec3f    eigen_type; };
template<> struct TaylorReturnType<3> { typedef TMatrix3 type; typedef Matrix3f eigen_type; };

template<typename Derived>
typename TaylorReturnType<Derived::ColsAtCompileTime>::type operator * (const FclType<Derived>& v, const TaylorModel& a)
{
  const typename TaylorReturnType<Derived::ColsAtCompileTime>::eigen_type b = v.fcl();
  return b * a;
}

}

#endif // FCL_CCD_TAYLOR_OPERATOR_H
