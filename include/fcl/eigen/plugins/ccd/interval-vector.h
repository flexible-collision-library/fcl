template <typename Derived>
IVector3& operator=(const FclType<Derived>& other)
{
  const Vec3f& tmp = other.fcl();
  setValue (tmp);
  return *this;
}
