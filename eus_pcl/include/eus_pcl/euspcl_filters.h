#ifndef __EUSPCL_FILTERS__
#define __EUSPCL_FILTERS__

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// eus functions
extern pointer PCL_VOXEL_GRID (register context *ctx, int n, pointer *argv);
extern pointer PCL_EXTRACT_INDICES (register context *ctx, int n, pointer *argv);

#endif
