#ifndef __EUSPCL_KDTREE__
#define __EUSPCL_KDTREE__
// write pcl include files to euspcl.h
#if __PCL_SELECT == 0
#include <pcl/kdtree/kdtree_flann.h>
#elif __PCL_SELECT == 17
#include <pcl17/kdtree/kdtree_flann.h>
#endif
// eus functions
extern pointer PCL_KDTREE_K_SEARCH (register context *ctx, int n, pointer *argv);
extern pointer PCL_KDTREE_R_SEARCH (register context *ctx, int n, pointer *argv);
extern pointer PCL_KDTREE_CREATE (register context *ctx, int n, pointer *argv);
extern pointer PCL_KDTREE_DELETE (register context *ctx, int n, pointer *argv);

#endif
