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
  seg.setMethodType (sac_method_type);                                  \
  seg.setMaxIterations (sac_max_iter);                                  \
  seg.setDistanceThreshold (sac_distance_thre);                         \
  seg.setRadiusLimits (sac_raidus_min, sac_radius_max);                 \
  //seg.setProbability (double probability)                             \
  //seg.setSamplesMaxDist (const double &radius, SearchPtr search)      \
  //seg.setAxis (const Eigen::Vector3f &ax)                             \
  //seg.setEpsAngle (double ea)                                         \
  seg.setInputCloud (pcl_cloud);                                        \
  //seg.setInputNormals (pcl_cloud);                                    \
  //seg.setNormalDistanceWeight (normal_distance_weight);               \
  //seg.setMinMaxOpeningAngle (const double &min_angle, const double &max_angle); \
  //seg.setDistanceFromOrigin (const double d);                         \
  seg.segment (*out_inliers, *out_coefficients);                        \
  //std::cerr << ";; solved coefficients: " << *out_coefficients << std::endl; \
  pcl::ExtractIndices< PTYPE > extract;                                 \
  extract.setInputCloud (pcl_cloud);                                    \
  extract.setIndices (out_inliers);                                     \
  extract.setNegative (extract_negative);                               \
  extract.filter (ret_pcl_cloud);                                       \
  if (return_indices) {                                                 \
    ret = makevector (C_INTVECTOR, out_inliers->indices.size());        \
    vpush (ret); pc++;                                                  \
    eusinteger_t *iv = ret->c.ivec.iv;                                  \
    for (size_t i = 0; i < out_inliers->indices.size(); i++) {          \
      iv[i] = out_inliers->indices[i];                                  \
    }                                                                   \
  } else {                                                              \
    ret = make_pointcloud_from_pcl (ctx, ret_pcl_cloud);                \
    vpush (ret); pc++;                                                  \
  }                                                                     \
  if (return_model_coefficients) {                                      \
    pointer coef = makefvector (out_coefficients->values.size());       \
    vpush (corf); pc++;                                                 \
    for (size_t i = 0; i < out_coefficients->values.size(); i++) {      \
      coef->c.fvec.fv[i] = out_coefficients->values[i];                 \
    }                                                                   \
    ret = rawcons (coef, ret);                                          \
    vpush (ret); pc++;                                                  \
  }

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
  eusfloat_t sac_radius_min = 0.0;
  eusfloat_t sac_radius_max = 0.1;
  eusfloat_t sac_distance_thre = 0.05;
  bool optimize_coeff = true;
  bool extract_negative = false;
  bool return_model_coefficients = false;
  bool return_indices = false;
  // for with_normal
  bool with_normal;
  eusfloat_t normal_distance_weight = 0.1;

  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return ret;
  }
  in_cloud = argv[0];

  if (n > 1) {
    sac_model_type = pcl::SacModel(intval(argv[1]));
  }
  if (n > 2) {
    sac_method_type = intval(argv[2]);
  }
  if (n > 3) {
    sac_max_iter = intval(argv[3]);
  }
  if (n > 4) {
    sac_radius_min = fltval(argv[4]);
  }
  if (n > 5) {
    sac_radius_max = fltval(argv[5]);
  }
  if (n > 6) {
    sac_distance_thre = fltval(argv[6]);
  }
  if (n > 7) {
    if (argv[7] == NIL) {
      optimize_coeff = false;
    }
  }
  if (n > 8) {
    if (argv[8] != NIL) {
      extract_negative = true;
    }
  }
  if (n > 9) {
    if (argv[9] != NIL) {
      return_model_coefficients = true;
    }
  }
  if (n > 10) {
    if (argv[10] != NIL) {
      return_indices = true;
    }
  }

  int width = intval(get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);
  colors = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_COLORS);
  normals = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_NORMALS);
  curvatures = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_CURVATURES);

  if (points != NIL && colors != NIL && normals != NIL) {
    SAC_SEGMENTATION_(PointCN);
  } else if (points != NIL && colors != NIL) {
    SAC_SEGMENTATION_(PointC);
  } else if (points != NIL && normals != NIL) {
    SAC_SEGMENTATION_(PointN);
  } else if (points != NIL) {
    SAC_SEGMENTATION_(Point);
  } else {
    // warning there is no points.
  }

  while (pc-- > 0) vpop();
  return ret;
}
