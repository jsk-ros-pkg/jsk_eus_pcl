#ifndef __EUSPCL_SURFACE__
#define __EUSPCL_SURFACE__
// write pcl include files to euspcl.h
#include <pcl/surface/convex_hull.h>
//#include <pcl/surface/concave_hull.h>

// eus functions
extern pointer PCL_CONVEX_HULL (register context *ctx, int n, pointer *argv);
//extern pointer PCL_CONCAVE_HULL (register context *ctx, int n, pointer *argv);

#endif
