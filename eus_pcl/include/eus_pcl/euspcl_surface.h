#ifndef __EUSPCL_SURFACE__
#define __EUSPCL_SURFACE__

#if __PCL_SELECT == 0
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#elif __PCL_SELECT == 17
#include <pcl17/surface/convex_hull.h>
#include <pcl17/surface/concave_hull.h>
#endif

// eus functions
extern pointer PCL_CONVEX_HULL (register context *ctx, int n, pointer *argv);
extern pointer PCL_CONVEX_HULL_PLANE (register context *ctx, int n, pointer *argv);
extern pointer PCL_CONCAVE_HULL (register context *ctx, int n, pointer *argv);
extern pointer PCL_CONCAVE_HULL_PLANE (register context *ctx, int n, pointer *argv);

#endif
