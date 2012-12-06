#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_sample_consensus.h"

#define SAC_SEGMENTATION_(PTYPE) \
  pcl::PointCloud< PTYPE >::Ptr pcl_cloud =                             \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, width, height); \
  pcl::PointCloud< PTYPE > ret_pcl_cloud;                               \
  pcl::SACSegmentation< PTYPE > seg;                                    \
  pcl::ModelCoefficients::Ptr out_coefficients (new pcl::ModelCoefficients); \
  pcl::PointIndices::Ptr out_inliers (new pcl::PointIndices);           \
  seg.setOptimizeCoefficients (true);                                   \
  seg.setModelType (pcl::SACMODEL_CYLINDER);                            \
  //seg.setNormalDistanceWeight (0.1);                                  \
  seg.setMethodType (pcl::SAC_RANSAC);                                  \
  seg.setMaxIterations (10000);                                         \
  seg.setDistanceThreshold (0.05);                                      \
  seg.setRadiusLimits (0, 0.1);                                         \
  seg.setInputCloud (pcl_cloud);                                        \
  seg.segment (*out_inliers, *out_coefficients);                        \
  std::cerr << ";; Cylinder coefficients: " << *out_coefficients << std::endl; \
  pcl::ExtractIndices< PTYPE > extract;                                 \
  extract.setInputCloud (pcl_cloud);                                    \
  extract.setIndices (out_inliers);                                     \
  extract.setNegative (false);                                          \
  extract.filter (ret_pcl_cloud);                                       \
  ret = make_pointcloud_from_pcl ( ctx, ret_pcl_cloud );                \
  vpush(ret); pc++;

#define SAC_SEGMENTATION_WITH_NORMAL_(PTYPE) \
  pcl::PointCloud< PTYPE >::Ptr pcl_cloud =                             \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, width, height); \
  pcl::PointCloud< PTYPE > ret_pcl_cloud;                               \
  pcl::SACSegmentationFromNormals< PTYPE, PTYPE > seg;                        \
  pcl::ModelCoefficients::Ptr out_coefficients (new pcl::ModelCoefficients); \
  pcl::PointIndices::Ptr out_inliers (new pcl::PointIndices);           \
  seg.setOptimizeCoefficients (true);                                   \
  seg.setModelType (pcl::SACMODEL_CYLINDER);                            \
  seg.setNormalDistanceWeight (0.1);                                    \
  seg.setMethodType (pcl::SAC_RANSAC);                                  \
  seg.setMaxIterations (10000);                                         \
  seg.setDistanceThreshold (0.05);                                      \
  seg.setRadiusLimits (0, 0.1);                                         \
  seg.setInputCloud (pcl_cloud);                                        \
  seg.segment (*out_inliers, *out_coefficients);                        \
  std::cerr << ";; Cylinder coefficients: " << *out_coefficients << std::endl; \
  pcl::ExtractIndices< PTYPE > extract;                                 \
  extract.setInputCloud (pcl_cloud);                                    \
  extract.setIndices (out_inliers);                                     \
  extract.setNegative (false);                                          \
  extract.filter (ret_pcl_cloud);                                       \
  ret = make_pointcloud_from_pcl ( ctx, ret_pcl_cloud );                \
  vpush(ret); pc++;

pointer SAC_SEGMENTATION (register context *ctx, int n, pointer *argv) {
  pointer in_cloud;
  pointer points, colors, normals;
  pointer ret = NIL;
  numunion nu;
  int pc = 0;
  // model_type, method_type, max_iter, distance_thre,
  // radius_limit, optimize, negative, return_indices
  // with_normal, normal_weight
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return ret;
  }
  in_cloud = argv[0];

  int width = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_POINTS);
  colors = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_COLORS);
  normals = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_NORMALS);

  if ( points != NIL && colors != NIL && normals != NIL ) {
    SAC_SEGMENTATION_(PointCN);
  } else if ( points != NIL && colors != NIL ) {
    SAC_SEGMENTATION_(PointC);
  } else if ( points != NIL && normals != NIL ) {
    SAC_SEGMENTATION_(PointN);
  } else if ( points != NIL ) {
    SAC_SEGMENTATION_(Point);
  } else {
    // warning there is no points.
  }

  while ( pc-- > 0) vpop();
  return ret;
}
