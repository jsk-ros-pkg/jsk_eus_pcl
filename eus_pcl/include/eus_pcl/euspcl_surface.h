#ifndef __EUSPCL_SURFACE__
#define __EUSPCL_SURFACE__

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

// eus functions
extern pointer PCL_CONVEX_HULL (register context *ctx, int n, pointer *argv);
extern pointer PCL_CONVEX_HULL_PLANE (register context *ctx, int n, pointer *argv);
extern pointer PCL_CONCAVE_HULL (register context *ctx, int n, pointer *argv);
extern pointer PCL_CONCAVE_HULL_PLANE (register context *ctx, int n, pointer *argv);

#endif
