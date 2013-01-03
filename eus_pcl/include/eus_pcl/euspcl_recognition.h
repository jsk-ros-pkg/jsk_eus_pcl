#ifndef __EUSPCL_RECOGNITION__
#define __EUSPCL_RECOGNITION__

#if __PCL_SELECT == 0
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/recognition/implicit_shape_model.h>
#elif __PCL_SELECT == 17
#include <pcl17/features/feature.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/features/fpfh.h>
#include <pcl17/recognition/implicit_shape_model.h>
#endif

// eus functions
extern pointer PCL_ISM_TRAINING (register context *ctx, int n, pointer *argv);
extern pointer PCL_ISM_DETECTION (register context *ctx, int n, pointer *argv);

#endif
