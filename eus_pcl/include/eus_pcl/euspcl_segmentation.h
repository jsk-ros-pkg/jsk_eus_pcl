#ifndef __EUSPCL_SEGMENTATION__
#define __EUSPCL_SEGMENTATION__

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_representation.h>

// eus functions
extern pointer PCL_EXTRACT_EUCLIDEAN_CLUSTERS (register context *ctx, int n, pointer *argv);
extern pointer PCL_EXTRACT_PLANES (register context *ctx, int n, pointer *argv);
#endif
