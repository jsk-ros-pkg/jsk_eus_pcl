#ifndef __EUSPCL_COMMON__
#define __EUSPCL_COMMON__
// write pcl include files to euspcl.h
#if __PCL_SELECT == 0
#include <pcl/common/pca.h>
#elif __PCL_SELECT == 17
#include <pcl17/common/pca.h>
#endif
// eus functions
extern pointer PCL_PCA (register context *ctx, int n, pointer *argv);

#endif
