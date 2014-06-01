// Geometric Tools, LLC
// Copyright (c) 1998-2011
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// Modified by Florent Lamiraux 2014

#include <cmath>
#include <limits>
#include <fcl/math/transform.h>
#include <fcl/shape/geometric_shapes.h>
#include "distance_func_matrix.h"

// Note that partial specialization of template functions is not allowed.
// Therefore, two implementations with the default narrow phase solvers are
// provided. If another narrow phase solver were to be used, the default
// template implementation would be called, unless the function is also
// specialized for this new type.
//
// One solution would be to make narrow phase solvers derive from an abstract
// class and specialize the template for this abstract class.
namespace fcl {
  class GJKSolver_libccd;
  class GJKSolver_indep;

  template <>
  FCL_REAL ShapeShapeDistance <Capsule, Capsule, GJKSolver_libccd>
  (const CollisionGeometry* o1, const Transform3f& tf1,
   const CollisionGeometry* o2, const Transform3f& tf2,
   const GJKSolver_libccd*, const DistanceRequest& request,
   DistanceResult& result)
  {
    const Capsule* c1 = static_cast <const Capsule*> (o1);
    const Capsule* c2 = static_cast <const Capsule*> (o2);

    // We assume that capsules are centered at the origin.
    fcl::Vec3f center1 = tf1.getTranslation ();
    fcl::Vec3f center2 = tf2.getTranslation ();
    // We assume that capsules are oriented along z-axis.
    fcl::Vec3f direction1 = tf1.getRotation ().getColumn (2);
    fcl::Vec3f direction2 = tf2.getRotation ().getColumn (2);
    FCL_REAL halfLength1 = 0.5*c1->lz;
    FCL_REAL halfLength2 = 0.5*c2->lz;

    Vec3f diff = center1 - center2;
    FCL_REAL a01 = -direction1.dot (direction2);
    FCL_REAL b0 = diff.dot (direction1);
    FCL_REAL b1 = -diff.dot (direction2);
    FCL_REAL c = diff.sqrLength ();
    FCL_REAL det = fabs (1.0 - a01*a01);
    FCL_REAL s1, s2, sqrDist, extDet0, extDet1, tmpS0, tmpS1;
    FCL_REAL epsilon = std::numeric_limits<FCL_REAL>::epsilon () * 100;

    if (det >= epsilon) {
      // Segments are not parallel.
      s1 = a01*b1 - b0;
      s2 = a01*b0 - b1;
      extDet0 = halfLength1*det;
      extDet1 = halfLength2*det;

      if (s1 >= -extDet0) {
	if (s1 <= extDet0) {
	  if (s2 >= -extDet1) {
	    if (s2 <= extDet1) {  // region 0 (interior)
	      // Minimum at interior points of segments.
	      FCL_REAL invDet = (1.0)/det;
	      s1 *= invDet;
	      s2 *= invDet;
	      sqrDist = s1*(s1 + a01*s2 + 2.0*b0) +
		s2*(a01*s1 + s2 + 2.0*b1) + c;
	    }
	    else { // region 3 (side)
	      s2 = halfLength2;
	      tmpS0 = -(a01*s2 + b0);
	      if (tmpS0 < -halfLength1) {
		s1 = -halfLength1;
		sqrDist = s1*(s1 - 2.0*tmpS0) +
		  s2*(s2 + 2.0*b1) + c;
	      }
	      else if (tmpS0 <= halfLength1) {
		s1 = tmpS0;
		sqrDist = -s1*s1 + s2*(s2 + 2.0*b1) + c;
	      }
	      else {
		s1 = halfLength1;
		sqrDist = s1*(s1 - 2.0*tmpS0) +
		  s2*(s2 + 2.0*b1) + c;
	      }
	    }
	  }
	  else { // region 7 (side)
	    s2 = -halfLength2;
	    tmpS0 = -(a01*s2 + b0);
	    if (tmpS0 < -halfLength1) {
	      s1 = -halfLength1;
	      sqrDist = s1*(s1 - 2.0*tmpS0) +
		s2*(s2 + 2.0*b1) + c;
	    } else if (tmpS0 <= halfLength1) {
	      s1 = tmpS0;
	      sqrDist = -s1*s1 + s2*(s2 + 2.0*b1) + c;
	    } else {
	      s1 = halfLength1;
	      sqrDist = s1*(s1 - 2.0*tmpS0) +
		s2*(s2 + 2.0*b1) + c;
	    }
	  }
	}
	else {
	  if (s2 >= -extDet1) {
	    if (s2 <= extDet1) {  // region 1 (side)
	      s1 = halfLength1;
	      tmpS1 = -(a01*s1 + b1);
	      if (tmpS1 < -halfLength2) {
		s2 = -halfLength2;
		sqrDist = s2*(s2 - 2.0*tmpS1) +
		  s1*(s1 + 2.0*b0) + c;
	      }
	      else if (tmpS1 <= halfLength2) {
		s2 = tmpS1;
		sqrDist = -s2*s2 + s1*(s1 + 2.0*b0) + c;
	      }
	      else {
		s2 = halfLength2;
		sqrDist = s2*(s2 - 2.0*tmpS1) +
		  s1*(s1 + 2.0*b0) + c;
	      }
	    }
	    else { // region 2 (corner)
	      s2 = halfLength2;
	      tmpS0 = -(a01*s2 + b0);
	      if (tmpS0 < -halfLength1) {
		s1 = -halfLength1;
		sqrDist = s1*(s1 - 2.0*tmpS0) +
		  s2*(s2 + 2.0*b1) + c;
	      }
	      else if (tmpS0 <= halfLength1) {
		s1 = tmpS0;
		sqrDist = -s1*s1 + s2*(s2 + 2.0*b1) + c;
	      }
	      else {
		s1 = halfLength1;
		tmpS1 = -(a01*s1 + b1);
		if (tmpS1 < -halfLength2) {
		  s2 = -halfLength2;
		  sqrDist = s2*(s2 - 2.0*tmpS1) +
		    s1*(s1 + 2.0*b0) + c;
		}
		else if (tmpS1 <= halfLength2) {
		  s2 = tmpS1;
		  sqrDist = -s2*s2 + s1*(s1 + 2.0*b0) + c;
		}
		else {
		  s2 = halfLength2;
		  sqrDist = s2*(s2 - 2.0*tmpS1) +
		    s1*(s1 + 2.0*b0) + c;
		}
	      }
	    }
	  }
	  else {  // region 8 (corner)
	    s2 = -halfLength2;
	    tmpS0 = -(a01*s2 + b0);
	    if (tmpS0 < -halfLength1) {
	      s1 = -halfLength1;
	      sqrDist = s1*(s1 - 2.0*tmpS0) +
		s2*(s2 + 2.0*b1) + c;
	    }
	    else if (tmpS0 <= halfLength1) {
	      s1 = tmpS0;
	      sqrDist = -s1*s1 + s2*(s2 + 2.0*b1) + c;
	    }
	    else {
	      s1 = halfLength1;
	      tmpS1 = -(a01*s1 + b1);
	      if (tmpS1 > halfLength2) {
		s2 = halfLength2;
		sqrDist = s2*(s2 - 2.0*tmpS1) +
		  s1*(s1 + 2.0*b0) + c;
	      }
	      else if (tmpS1 >= -halfLength2) {
		s2 = tmpS1;
		sqrDist = -s2*s2 + s1*(s1 + 2.0*b0) + c;
	      }
	      else {
		s2 = -halfLength2;
		sqrDist = s2*(s2 - 2.0*tmpS1) +
		  s1*(s1 + 2.0*b0) + c;
	      }
	    }
	  }
	}
      }
      else {
	if (s2 >= -extDet1) {
	  if (s2 <= extDet1) { // region 5 (side)
	    s1 = -halfLength1;
	    tmpS1 = -(a01*s1 + b1);
	    if (tmpS1 < -halfLength2) {
	      s2 = -halfLength2;
	      sqrDist = s2*(s2 - 2.0*tmpS1) +
		s1*(s1 + 2.0*b0) + c;
	    }
	    else if (tmpS1 <= halfLength2) {
	      s2 = tmpS1;
	      sqrDist = -s2*s2 + s1*(s1 + 2.0*b0) + c;
	    }
	    else {
	      s2 = halfLength2;
	      sqrDist = s2*(s2 - 2.0*tmpS1) +
		s1*(s1 + 2.0*b0) + c;
	    }
	  }
	  else { // region 4 (corner)
	    s2 = halfLength2;
	    tmpS0 = -(a01*s2 + b0);
	    if (tmpS0 > halfLength1) {
	      s1 = halfLength1;
	      sqrDist = s1*(s1 - 2.0*tmpS0) +
		s2*(s2 + 2.0*b1) + c;
	    }
	    else if (tmpS0 >= -halfLength1) {
	      s1 = tmpS0;
	      sqrDist = -s1*s1 + s2*(s2 + 2.0*b1) + c;
	    }
	    else {
	      s1 = -halfLength1;
	      tmpS1 = -(a01*s1 + b1);
	      if (tmpS1 < -halfLength2) {
		s2 = -halfLength2;
		sqrDist = s2*(s2 - 2.0*tmpS1) +
		  s1*(s1 + 2.0*b0) + c;
	      }
	      else if (tmpS1 <= halfLength2) {
		s2 = tmpS1;
		sqrDist = -s2*s2 + s1*(s1 + 2.0*b0) + c;
	      }
	      else {
		s2 = halfLength2;
		sqrDist = s2*(s2 - 2.0*tmpS1) +
		  s1*(s1 + 2.0*b0) + c;
	      }
	    }
	  }
	}
	else {  // region 6 (corner)
	  s2 = -halfLength2;
	  tmpS0 = -(a01*s2 + b0);
	  if (tmpS0 > halfLength1) {
	    s1 = halfLength1;
	    sqrDist = s1*(s1 - 2.0*tmpS0) +
	      s2*(s2 + 2.0*b1) + c;
	  }
	  else if (tmpS0 >= -halfLength1) {
	    s1 = tmpS0;
	    sqrDist = -s1*s1 + s2*(s2 + 2.0*b1) + c;
	  }
	  else {
	    s1 = -halfLength1;
	    tmpS1 = -(a01*s1 + b1);
	    if (tmpS1 < -halfLength2) {
	      s2 = -halfLength2;
	      sqrDist = s2*(s2 - 2.0*tmpS1) +
		s1*(s1 + 2.0*b0) + c;
	    }
	    else if (tmpS1 <= halfLength2) {
	      s2 = tmpS1;
	      sqrDist = -s2*s2 + s1*(s1 + 2.0*b0) + c;
	    }
	    else {
	      s2 = halfLength2;
	      sqrDist = s2*(s2 - 2.0*tmpS1) +
		s1*(s1 + 2.0*b0) + c;
	    }
	  }
	}
      }
    }
    else {
      // The segments are parallel.  The average b0 term is designed to
      // ensure symmetry of the function.  That is, dist(seg0,seg1) and
      // dist(seg1,seg0) should produce the same number.
      FCL_REAL e0pe1 = halfLength1 + halfLength2;
      FCL_REAL sign = (a01 > 0.0 ? -1.0 : 1.0);
      FCL_REAL b0Avr = (0.5)*(b0 - sign*b1);
      FCL_REAL lambda = -b0Avr;
      if (lambda < -e0pe1) {
	lambda = -e0pe1;
      }
      else if (lambda > e0pe1) {
	lambda = e0pe1;
      }

      s2 = -sign*lambda*halfLength2/e0pe1;
      s1 = lambda + sign*s2;
      sqrDist = lambda*(lambda + 2.0*b0Avr) + c;
    }

    Vec3f closestOnSegment1 = center1 + s1*direction1;
    Vec3f closestOnSegment2 = center2 + s2*direction2;

    Vec3f unitVector = closestOnSegment2 - closestOnSegment1;
    FCL_REAL distance = sqrt (sqrDist);
    if (distance >= epsilon) {
      unitVector /= distance;
    } else {
      unitVector.setZero ();
    }
    // Update distance result.
    result.min_distance = distance - c1->radius - c2->radius;
    if (request.enable_nearest_points) {
      result.nearest_points [0] = closestOnSegment1 + c1->radius * unitVector;
      result.nearest_points [1] = closestOnSegment2 - c2->radius * unitVector;
    }
    result.o1 = o1;
    result.o2 = o2;
    result.b1 = -1;
    result.b2 = -1;
    return result.min_distance;
  }

  template <>
  FCL_REAL ShapeShapeDistance <Capsule, Capsule, GJKSolver_indep>
  (const CollisionGeometry* o1, const Transform3f& tf1,
   const CollisionGeometry* o2, const Transform3f& tf2,
   const GJKSolver_indep*, const DistanceRequest& request,
   DistanceResult& result)
  {
    GJKSolver_libccd* unused = 0x0;
    return ShapeShapeDistance <Capsule, Capsule, GJKSolver_libccd>
      (o1, tf1, o2, tf2, unused, request, result);
  }

  template <>
  std::size_t ShapeShapeCollide <Capsule, Capsule, GJKSolver_indep>
  (const CollisionGeometry* o1, const Transform3f& tf1,
   const CollisionGeometry* o2, const Transform3f& tf2,
   const GJKSolver_indep*, const CollisionRequest& request,
   CollisionResult& result)
  {
    GJKSolver_libccd* unused = 0x0;
    DistanceResult distanceResult;
    DistanceRequest distanceRequest (request.enable_contact);

    FCL_REAL distance = ShapeShapeDistance <Capsule, Capsule, GJKSolver_libccd>
      (o1, tf1, o2, tf2, unused, distanceRequest, distanceResult);

    if (distance <= 0) {
      if (request.enable_contact) {
	Contact contact (o1, o2, distanceResult.b1, distanceResult.b2);
	const Vec3f& p1 = distanceResult.nearest_points [0];
	const Vec3f& p2 = distanceResult.nearest_points [1];
	contact.pos = .5*(p1+p2);
	contact.normal = (p2-p1)/(p2-p1).length ();
	result.addContact (contact);
      }
      return 1;
    }
    return 0;
  }

  template<>
  std::size_t ShapeShapeCollide <Capsule, Capsule, GJKSolver_libccd>
  (const CollisionGeometry* o1, const Transform3f& tf1,
   const CollisionGeometry* o2, const Transform3f& tf2,
   const GJKSolver_libccd* nsolver, const CollisionRequest& request,
   CollisionResult& result)
  {
    GJKSolver_indep* unused = 0x0;
    return ShapeShapeCollide <Capsule, Capsule, GJKSolver_indep>
      (o1, tf1, o2, tf2, unused, request, result);
  }
} // namespace fcl
