#ifndef __EUSPCL_RECOGNITION__
#define __EUSPCL_RECOGNITION__

#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/recognition/implicit_shape_model.h>

// eus functions
extern pointer PCL_ISM_TRAINING (register context *ctx, int n, pointer *argv);
extern pointer PCL_ISM_DETECTION (register context *ctx, int n, pointer *argv);

#endif
