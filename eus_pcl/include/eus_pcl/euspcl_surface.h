#ifndef __EUSPCL_SURFACE__
#define __EUSPCL_SURFACE__

#if __PCL_SELECT == 0
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#elif __PCL_SELECT == 17
#include <pcl17/surface/convex_hull.h>
#include <pcl17/surface/concave_hull.h>
#include <pcl17/surface/gp3.h>
#include <pcl17/surface/poisson.h>
#include <pcl17/surface/marching_cubes_hoppe.h>
#include <pcl17/surface/marching_cubes_rbf.h>
#endif

enum SURFACE_CONSTRUCT_TYPE {CONVEX_HULL, CONCAVE_HULL, GREEDY_PROJECTION, CONSTRUCT_LAST_TYPE};
enum SURFACE_RECONSTRUCT_TYPE {POISSON, MARCHING_CUBES, RECONSTRUCT_LAST_TYPE};

// eus functions
extern pointer PCL_SURFACE_CONSTRUCTION (register context *ctx, int n, pointer *argv);
extern pointer PCL_SURFACE_RECONSTRUCTION (register context *ctx, int n, pointer *argv);

#endif
