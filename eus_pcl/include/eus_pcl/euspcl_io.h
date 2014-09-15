#ifndef __EUSPCL_IO__
#define __EUSPCL_IO__

#if __PCL_SELECT == 0
#include <pcl/io/pcd_io.h>
#elif __PCL_SELECT == 17
#include <pcl17/io/pcd_io.h>
#endif

#include <pcl_conversions/pcl_conversions.h>

// eus functions
extern pointer PCL_READ_PCD (register context *ctx, int n, pointer *argv);
extern pointer PCL_WRITE_PCD (register context *ctx, int n, pointer *argv);
extern pointer PCL_STEP_POINTCLOUD (register context *ctx, int n, pointer *argv);
#endif
