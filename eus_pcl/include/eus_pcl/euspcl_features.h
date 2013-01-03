#ifndef __EUSPCL_FEATURES__
#define __EUSPCL_FEATURES__

#if __PCL_SELECT == 0
#include <pcl/features/normal_3d.h>
#elif __PCL_SELECT == 17
#include <pcl17/features/normal_3d.h>
#endif

// eus functions
extern pointer PCL_ADD_NORMAL (register context *ctx, int n, pointer *argv);

#endif
