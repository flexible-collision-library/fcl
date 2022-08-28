#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "test_fcl_utility.h"
#include <fstream>
#include <string>

using namespace fcl;
using namespace detail;
using namespace test;

template<typename Shape1, typename Shape2>
void timeCollect(const Shape1 &s1, const aligned_vector<Transform3d> &transforms1,
        const Shape2 &s2, const aligned_vector<Transform3d> &transforms2,
        double &time1, double &time2, double &error);

int main()
{
    aligned_vector<Transform3d> transforms1, transforms2;
    double extents[] = {-30, -30, -30, 30, 30, 30};
    std::size_t n = 1000;
    generateRandomTransforms(extents, transforms1, n);
    generateRandomTransforms(extents, transforms2, n);

    double low = 1.0, high = 5.0;

    //==============================================================================
    //Box
    {
        Boxd    s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Boxd    s2(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Boxd, Boxd>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Box-Box " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Boxd    s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Ellipsoidd s2(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Boxd, Ellipsoidd>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Box-Ellipsoid " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Boxd    s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Sphered s2(rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Boxd, Sphered>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Box-Sphere " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Boxd    s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Cylinderd s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Boxd, Cylinderd>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Box-Cylinder " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Boxd    s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Capsuled s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Boxd, Capsuled>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Box-Capsule " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Boxd  s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Coned s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Boxd, Coned>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Box-Cone " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    std::cout << std::endl << std::endl;

    //==============================================================================
    //Ellipsoid
    {
        Ellipsoidd s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Ellipsoidd s2(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Ellipsoidd, Ellipsoidd>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Ellipsoid-Ellipsoidd " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Ellipsoidd s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Sphered s2(rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Ellipsoidd, Sphered>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Ellipsoid-Sphere " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Ellipsoidd s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Cylinderd s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Ellipsoidd, Cylinderd>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Ellipsoid-Cylinder " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Ellipsoidd s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Capsuled s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Ellipsoidd, Capsuled>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Ellipsoid-Capsule " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Ellipsoidd s1(rand_interval(low, high), rand_interval(low, high), rand_interval(low, high));
        Coned s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Ellipsoidd, Coned>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Ellipsoid-Cone " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    std::cout << std::endl << std::endl;

    //==============================================================================
    //Sphere
    {
        Sphered s1(rand_interval(low, high));
        Sphered s2(rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Sphered, Sphered>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Sphere-Sphere " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Sphered s1(rand_interval(low, high));
        Cylinderd s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Sphered, Cylinderd>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Sphere-Cylinder " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Sphered s1(rand_interval(low, high));
        Capsuled s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Sphered, Capsuled>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Sphere-Capsule " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Sphered s1(rand_interval(low, high));
        Coned s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Sphered, Coned>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Sphere-Cone " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }
    std::cout << std::endl << std::endl;

    //==============================================================================
    //Cylinder
    {
        Cylinderd s1(rand_interval(low, high), rand_interval(low, high));
        Cylinderd s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Cylinderd, Cylinderd>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Cylinder-Cylinder " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Cylinderd s1(rand_interval(low, high), rand_interval(low, high));
        Capsuled s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Cylinderd, Capsuled>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Cylinder-Capsule " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Cylinderd s1(rand_interval(low, high), rand_interval(low, high));
        Coned s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Cylinderd, Coned>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Cylinder-Cone " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }
    std::cout << std::endl << std::endl;

    //==============================================================================
    //Capsule
    {
        Capsuled s1(rand_interval(low, high), rand_interval(low, high));
        Capsuled s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Capsuled, Capsuled>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Capsule-Capsule " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    {
        Capsuled s1(rand_interval(low, high), rand_interval(low, high));
        Coned s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Capsuled, Coned>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Capsule-Cone " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }
    std::cout << std::endl << std::endl;

    //==============================================================================
    //Cone
    {
        Coned s1(rand_interval(low, high), rand_interval(low, high));
        Coned s2(rand_interval(low, high), rand_interval(low, high));
        double time1 = 0.0, time2 = 0.0, error = 0.0;
        timeCollect<Coned, Coned>(s1, transforms1, s2, transforms2, time1, time2, error);
        std::cout << "Cone-Cone " << "libccd_time " << time1 << " indep_time " << time2 << " error  " << error << std::endl;
    }

    return 0;
}

template<typename Shape1, typename Shape2>
void timeCollect(const Shape1 &s1, const aligned_vector<Transform3d> &transforms1,
        const Shape2 &s2, const aligned_vector<Transform3d> &transforms2,
        double &time1, double &time2, double &error)
{
    time1 = time2 = error = 0.0;
    test::Timer timer_dis;
    GJKSolver_libccdd gjk_solver_libccd;
    GJKSolver_indepd  gjk_solver_indep;
    for (std::size_t i = 0; i < transforms1.size(); i++)
    {
        int count = 0;
        for (std::size_t j = 0; j < transforms2.size(); j++)
        {
            double dist1, dist2;
            Vector3d p1, p2;

            timer_dis.start();
            bool res = gjk_solver_libccd.shapeDistance<Shape1, Shape2>(s1, transforms1[i], s2, transforms2[j], &dist1, &p1,  &p2);
            timer_dis.stop();
            if (res)
            {
                count++;
                time1 += timer_dis.getElapsedTimeInSec();

                timer_dis.start();
                gjk_solver_indep.shapeDistance<Shape1, Shape2>(s1, transforms1[i], s2, transforms2[j], &dist2, &p1,  &p2);
                timer_dis.stop();
                time2 += timer_dis.getElapsedTimeInSec();

                error += abs(dist1 - dist2);
            }
        }
    }
}
