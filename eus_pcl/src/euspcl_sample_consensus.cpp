#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_sample_consensus.h"

#if __PCL_SELECT == 0
using namespace pcl;
#elif __PCL_SELECT == 17
using namespace pcl17;
#endif

#define SAC_SEGMENTATION_(PTYPE)                                        \
  PointCloud< PTYPE >::Ptr pcl_cloud =                                  \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, curvatures, width, height); \
  PointCloud< PTYPE > ret_pcl_cloud;                                    \
  SACSegmentation< PTYPE > seg;                                         \
  /*SACSegmentationFromNormals< PTYPE, PTYPE > seg;*/                   \
  ModelCoefficients::Ptr out_coefficients (new ModelCoefficients);      \
  PointIndices::Ptr out_inliers (new PointIndices);                     \
  seg.setOptimizeCoefficients (optimize_coeff);                         \
  seg.setModelType (sac_model_type);                                    \
  seg.setMethodType (sac_method_type);                                  \
  seg.setMaxIterations (sac_max_iter);                                  \
  seg.setDistanceThreshold (sac_distance_thre);                         \
  seg.setRadiusLimits (sac_radius_min, sac_radius_max);                 \
  /*seg.setProbability (double probability); */                         \
  /*seg.setSamplesMaxDist (const double &radius, SearchPtr search); */  \
  /*seg.setAxis (const Eigen::Vector3f &ax); */                         \
  /*seg.setEpsAngle (double ea); */                                     \
  seg.setInputCloud (pcl_cloud);                                        \
  /*seg.setInputNormals (pcl_cloud); */                                 \
  /*seg.setNormalDistanceWeight (normal_distance_weight);*/             \
  /*seg.setMinMaxOpeningAngle (const double &min_angle, const double &max_angle);*/ \
  /*seg.setDistanceFromOrigin (const double d);*/                       \
  seg.segment (*out_inliers, *out_coefficients);                        \
  std::cerr << ";; solved coefficients: " << *out_coefficients << std::endl; \
  ExtractIndices< PTYPE > extract;                                      \
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
    vpush (coef); pc++;                                                 \
    for (size_t i = 0; i < out_coefficients->values.size(); i++) {      \
      coef->c.fvec.fv[i] = out_coefficients->values[i];                 \
    }                                                                   \
    ret = rawcons (ctx, coef, ret);                                     \
    vpush (ret); pc++;                                                  \
  }
#define SAC_SEGMENTATION_NORMAL_(PTYPE)                                 \
  PointCloud< PTYPE >::Ptr pcl_cloud =                                  \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, curvatures, width, height); \
  PointCloud< PTYPE > ret_pcl_cloud;                                    \
  SACSegmentationFromNormals< PTYPE, PTYPE > seg;                       \
  ModelCoefficients::Ptr out_coefficients (new ModelCoefficients);      \
  PointIndices::Ptr out_inliers (new PointIndices);                     \
  seg.setOptimizeCoefficients (optimize_coeff);                         \
  seg.setModelType (sac_model_type);                                    \
  seg.setMethodType (sac_method_type);                                  \
  seg.setMaxIterations (sac_max_iter);                                  \
  seg.setDistanceThreshold (sac_distance_thre);                         \
  seg.setRadiusLimits (sac_radius_min, sac_radius_max);                 \
  /*seg.setProbability (double probability); */                         \
  /*seg.setSamplesMaxDist (const double &radius, SearchPtr search); */  \
  /*seg.setAxis (const Eigen::Vector3f &ax); */                         \
  /*seg.setEpsAngle (double ea); */                                     \
  seg.setInputCloud (pcl_cloud);                                        \
  seg.setInputNormals (pcl_cloud);                                      \
  /*seg.setNormalDistanceWeight (normal_distance_weight);*/             \
  /*seg.setMinMaxOpeningAngle (const double &min_angle, const double &max_angle);*/ \
  /*seg.setDistanceFromOrigin (const double d);*/                       \
  seg.segment (*out_inliers, *out_coefficients);                        \
  std::cerr << ";; solved coefficients: " << *out_coefficients << std::endl; \
  ExtractIndices< PTYPE > extract;                                      \
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
    vpush (coef); pc++;                                                 \
    for (size_t i = 0; i < out_coefficients->values.size(); i++) {      \
      coef->c.fvec.fv[i] = out_coefficients->values[i];                 \
    }                                                                   \
    ret = rawcons (ctx, coef, ret);                                     \
    vpush (ret); pc++;                                                  \
  }

  /* model
0 SACMODEL_PLANE,
1 SACMODEL_LINE,
2 SACMODEL_CIRCLE2D,
3 SACMODEL_CIRCLE3D,
4 SACMODEL_SPHERE,
5 SACMODEL_CYLINDER,
6 SACMODEL_CONE,
7 SACMODEL_TORUS,
8 SACMODEL_PARALLEL_LINE,
SACMODEL_PERPENDICULAR_PLANE,
SACMODEL_PARALLEL_LINES,
SACMODEL_NORMAL_PLANE,
SACMODEL_NORMAL_SPHERE,
SACMODEL_REGISTRATION,
SACMODEL_REGISTRATION_2D,
SACMODEL_PARALLEL_PLANE,
SACMODEL_NORMAL_PARALLEL_PLANE,
SACMODEL_STICK
  */
  /* method
SAC_RANSAC  = 0;
SAC_LMEDS   = 1;
SAC_MSAC    = 2;
SAC_RRANSAC = 3;
SAC_RMSAC   = 4;
SAC_MLESAC  = 5;
SAC_PROSAC  = 6;
  */
pointer PCL_SAC_SEGMENTATION (register context *ctx, int n, pointer *argv) {
  /* (pointcloud &optional (model_type) (method_type) */
  /* (max_iter 10000) (radius_min 0.0) (radius_max 0.1) (distance_thre 0.05) (optimize t) (extract_negative) */
  /* (retrun_model) (return_indices) */
  pointer in_cloud;
  pointer points, colors, normals, curvatures;
  pointer ret = NIL;
  numunion nu;
  int pc = 0;
  SacModel sac_model_type = SACMODEL_PLANE;
  int sac_method_type = SAC_RANSAC;
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
    sac_model_type = SacModel(intval(argv[1]));
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
    SAC_SEGMENTATION_NORMAL_(PointCN);
  } else if (points != NIL && colors != NIL) {
    SAC_SEGMENTATION_(PointC);
  } else if (points != NIL && normals != NIL) {
    SAC_SEGMENTATION_NORMAL_(PointN);
  } else if (points != NIL) {
    SAC_SEGMENTATION_(Point);
  } else {
    // warning there is no points.
  }

  while (pc-- > 0) vpop();
  return ret;
}
