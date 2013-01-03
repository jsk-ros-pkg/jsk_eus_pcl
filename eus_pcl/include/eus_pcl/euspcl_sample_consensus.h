#ifndef __EUSPCL_SAMPLE_CONSENSUS__
#define __EUSPCL_SAMPLE_CONSENSUS__

#if __PCL_SELECT == 0
#include <pcl/segmentation/sac_segmentation.h>
#elif __PCL_SELECT == 17
#include <pcl17/segmentation/sac_segmentation.h>
#endif


// eus functions
extern pointer PCL_SAC_SEGMENTATION (register context *ctx, int n, pointer *argv);

#endif
