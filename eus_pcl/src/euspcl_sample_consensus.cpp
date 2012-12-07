#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_sample_consensus.h"

#define SAC_SEGMENTATION_(PTYPE) \
  pcl::PointCloud< PTYPE >::Ptr pcl_cloud =                             \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, curvatures, width, height); \
  pcl::PointCloud< PTYPE > ret_pcl_cloud;                               \
  pcl::SACSegmentation< PTYPE > seg;                                    \
  //pcl::SACSegmentationFromNormals< PTYPE, PTYPE > seg;                \
  pcl::ModelCoefficients::Ptr out_coefficients (new pcl::ModelCoefficients); \
  pcl::PointIndices::Ptr out_inliers (new pcl::PointIndices);           \
  seg.setOptimizeCoefficients (optimize_coeff);                         \
  seg.setModelType (sac_model_type);                                    \
  //seg.setNormalDistanceWeight (0.1);                                  \
  seg.setMethodType (sac_method_type);                                  \
  seg.setMaxIterations (sac_max_iter);                                  \
  seg.setDistanceThreshold (sac_distance_thre);                         \
  seg.setRadiusLimits (0, 0.1);                                         \
  seg.setInputCloud (pcl_cloud);                                        \
  seg.segment (*out_inliers, *out_coefficients);                        \
  std::cerr << ";; Cylinder coefficients: " << *out_coefficients << std::endl; \
  pcl::ExtractIndices< PTYPE > extract;                                 \
  extract.setInputCloud (pcl_cloud);                                    \
  extract.setIndices (out_inliers);                                     \
  extract.setNegative (extract_negative);                               \
  extract.filter (ret_pcl_cloud);                                       \
  ret = make_pointcloud_from_pcl ( ctx, ret_pcl_cloud );                \
  vpush(ret); pc++;

#define SAC_SEGMENTATION_WITH_NORMAL_(PTYPE) \


pointer PCL_SAC_SEGMENTATION (register context *ctx, int n, pointer *argv) {
  /* (pointcloud &optional (model_type) (method_type) */
  pointer in_cloud;
  pointer points, colors, normals, curvatures;
  pointer ret = NIL;
  numunion nu;
  int pc = 0;
  pcl::SacModel sac_model_type = pcl::SACMODEL_PLANE;
  int sac_method_type = pcl::SAC_RANSAC;
  eusinteger_t sac_max_iter = 10000;
  eusfloat_t sac_radius_limit;
  eusfloat_t sac_distance_thre = 0.05;
  bool optimize_coeff = true;
  bool extract_negative = false;
  bool with_normal, normal_weight;
  bool return_model_coefficient, return_indices;

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
  curvatures = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_CURVATURES);

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

  while (pc-- > 0) vpop();
  return ret;
}
