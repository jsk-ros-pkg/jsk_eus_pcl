#ifndef __EUSPCL_OCTREE__
#define __EUSPCL_OCTREE__

#if __PCL_SELECT == 0
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>
#elif __PCL_SELECT == 17
#include <pcl17/octree/octree.h>
#include <pcl17/octree/octree_impl.h>
#include <pcl17/filters/extract_indices.h>
#endif

// eus functions
extern pointer PCL_OCT_VOXEL (register context *ctx, int n, pointer *argv);
extern pointer PCL_COR_PTS (register context *ctx, int n, pointer *argv);
extern pointer PCL_OCT_POINT_VECTOR (register context *ctx, int n, pointer *argv);
extern pointer PCL_OCT_HEIGHTMAP_GRID (register context *ctx, int n, pointer *argv);
#endif
