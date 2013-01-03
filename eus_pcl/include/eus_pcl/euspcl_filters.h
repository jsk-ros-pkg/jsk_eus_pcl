#ifndef __EUSPCL_FILTERS__
#define __EUSPCL_FILTERS__

#if __PCL_SELECT == 0
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#elif __PCL_SELECT == 17
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/filters/extract_indices.h>
#endif

// eus functions
extern pointer PCL_VOXEL_GRID (register context *ctx, int n, pointer *argv);
extern pointer PCL_EXTRACT_INDICES (register context *ctx, int n, pointer *argv);

#endif
