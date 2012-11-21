#ifndef __EUSPCL_IO__
#define __EUSPCL_IO__
// write pcl include files to euspcl.h
// #include <pcl/io/pcd_io.h>

extern pointer PCL_READ_PCD (register context *ctx, int n, pointer *argv);
extern pointer PCL_WRITE_PCD (register context *ctx, int n, pointer *argv);
#endif
