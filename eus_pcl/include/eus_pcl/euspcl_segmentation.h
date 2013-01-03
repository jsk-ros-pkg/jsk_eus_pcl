#ifndef __EUSPCL_SEGMENTATION__
#define __EUSPCL_SEGMENTATION__

#if __PCL_SELECT == 0
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#elif __PCL_SELECT == 17
#include <pcl17/filters/extract_indices.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/segmentation/extract_clusters.h>
#endif

// eus functions
extern pointer PCL_EXTRACT_EUCLIDEAN_CLUSTERS (register context *ctx, int n, pointer *argv);
extern pointer PCL_EXTRACT_PLANES (register context *ctx, int n, pointer *argv);
#endif
