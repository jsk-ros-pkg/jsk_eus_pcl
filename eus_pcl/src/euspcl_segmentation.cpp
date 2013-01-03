#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_segmentation.h"
#include <iostream>

#if __PCL_SELECT == 0
using namespace pcl;
#elif __PCL_SELECT == 17
using namespace pcl17;
#endif

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <boost/timer.hpp>
#endif

pointer PCL_EXTRACT_EUCLIDEAN_CLUSTERS (register context *ctx, int n, pointer *argv) {
  /* ( pointcloud &optional (minsize 500) (maxsize 20000) (tolerance 20.0) ) */
  pointer in_cloud;
  pointer points;
  pointer ret = NIL;
  eusfloat_t tolerance;
  eusinteger_t minsize, maxsize;
  numunion nu;
  int pc = 0;
  bool return_indices = false;

  tolerance = 0.02;
  minsize = 500;
  maxsize = 20000;

  ckarg2(1, 4);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return ret;
  }
  in_cloud = argv[0];

  if ( n > 1 ) {
    minsize = intval(argv[1]);
  }
  if ( n > 2 ) {
    maxsize = intval(argv[2]);
  }
  if ( n > 3 ) {
    tolerance = (fltval(argv[3])) / 1000.0;
  }

  int width = intval(get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);

  PointCloud< Point >::Ptr pcl_cloud =
    make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);

  search::KdTree< Point >::Ptr tree (new search::KdTree< Point > ());
  tree->setInputCloud (pcl_cloud); //??
  std::vector< PointIndices > cluster_indices;
  EuclideanClusterExtraction< Point > euc_ext;
  euc_ext.setInputCloud (pcl_cloud);
  euc_ext.setClusterTolerance (tolerance);
  euc_ext.setMinClusterSize (minsize);
  euc_ext.setMaxClusterSize (maxsize);
  euc_ext.setSearchMethod (tree);
  euc_ext.extract (cluster_indices);

  if (return_indices) {
    ret = makevector(C_INTVECTOR, width * height);
    vpush (ret); pc++;
    eusinteger_t *ret_vec = ret->c.ivec.iv;
    for (size_t i = 0; i < width * height; i++ ) ret_vec[i] = -1;

    for (size_t i = 0; i < cluster_indices.size(); i++) {
      for (size_t j = 0; j < cluster_indices[i].indices.size(); j++ ) {
        ret_vec[ cluster_indices[i].indices[j] ] = i;
      }
    }
  } else {
    for (size_t i = cluster_indices.size() - 1; i > 0; i--) {
      size_t vec_size = cluster_indices[i].indices.size();
      pointer tmp_vec = makevector (C_INTVECTOR, vec_size);
      vpush (tmp_vec); pc++;
      ret = rawcons (ctx, tmp_vec, ret);
      vpush (ret);
      for (size_t j = 0; j < vec_size; j++) {
        tmp_vec->c.ivec.iv[j] = cluster_indices[i].indices[j];
      }
    }
  }

  while (pc-- > 0) vpop();
  return ret;
}

pointer PCL_EXTRACT_PLANES (register context *ctx, int n, pointer *argv) {
  /* (pointcloud &optional (min_plane_points 500) (max_planes 20)
          (max_iterations 1000) (tolerance 40.0) (distance_threshold 15.0)) */
  pointer in_cloud;
  pointer points;
  pointer ret = NIL;
  pointer coeff_list = NIL;
  pointer indices_list = NIL;
  numunion nu;
  int pc = 0;

  // parameters
  eusfloat_t tolerance, distance_threshold;
  eusinteger_t min_plane_points, max_planes, max_iterations;
  bool plane_segmentation = true;

  min_plane_points = 500;
  max_planes = 20;
  max_iterations = 1000;
  tolerance = 0.04;
  distance_threshold = 0.015;

  ckarg2(1, 4);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return ret;
  }
  in_cloud = argv[0];

  if (n > 1) {
    min_plane_points = intval(argv[1]);
  }
  if (n > 2) {
    max_planes = intval(argv[2]);
  }
  if (n > 3) {
    max_iterations = intval(argv[3]);
  }
  if (n > 4) {
    tolerance = (fltval(argv[4])) / 1000.0;
  }
  if (n > 5) {
    distance_threshold = (fltval(argv[5])) / 1000.0;
  }

  int width = intval(get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);

  Points::Ptr pcl_cloud_ptr =
    make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);

  SACSegmentation< Point > plane_seg;

  plane_seg.setOptimizeCoefficients (true);
  plane_seg.setModelType (SACMODEL_PLANE);
  plane_seg.setMethodType (SAC_RANSAC);
  plane_seg.setDistanceThreshold (distance_threshold);
  plane_seg.setMaxIterations (max_iterations);
  //plane_seg.setProbability (_probability);

  ExtractIndices< Point > extractor;

  size_t point_size = pcl_cloud_ptr->points.size();
  int plane_num = 0;
#if DEBUG
  boost::timer tm;
  boost::timer total_tm;
#endif
  while (plane_num < max_planes) {
    // check valid points
#if DEBUG
    tm.restart();
#endif
    PointIndicesPtr pti (new PointIndices());
    {
#if DEBUG
      std::cerr << ";; point_size: " << point_size << std::endl;
#endif
      DefaultPointRepresentation<Point> pr;
      size_t cntr=0;
      for (Points::const_iterator it = pcl_cloud_ptr->begin();
           it != pcl_cloud_ptr->end(); it++) {
        if (pr.isValid (*it)) pti->indices.push_back (cntr);
        cntr++;
      }
#if DEBUG
      std::cerr << ";; valid points: " << pti->indices.size() << std::endl;
#endif
    }
#if DEBUG
    std::cerr << ";; 0 TIME = " << tm.elapsed() << std::endl;
    tm.restart();
#endif
    plane_seg.setInputCloud (pcl_cloud_ptr);
    plane_seg.setIndices (pti);

    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);

    plane_seg.segment (*inliers, *coefficients);

    if (inliers->indices.size() < min_plane_points) break;

    if (coefficients->values.size () >=3 ) {
      pointer fvec = makefvector(4); vpush (fvec); pc++;
      coeff_list = rawcons (ctx, fvec, coeff_list);
      vpush (coeff_list); pc++;

      fvec->c.fvec.fv[0] = coefficients->values[0];
      fvec->c.fvec.fv[1] = coefficients->values[1];
      fvec->c.fvec.fv[2] = coefficients->values[2];
      fvec->c.fvec.fv[3] = (coefficients->values[3]) * 1000.0;
    } else {
      std::cerr << ";; there is no plane." << std::endl;
      break;
    }
#if DEBUG
    std::cerr << ";; 1 TIME = " << tm.elapsed() << std::endl;
    tm.restart();
    std::cerr << ";; A inlier: " << inliers->indices.size() << std::endl;
#endif
    if (plane_segmentation) { // Euclidean Clustering for Points // search largest plane
      Points::Ptr tmpp (new Points);
      extractor.setInputCloud (pcl_cloud_ptr);
      extractor.setIndices (inliers);
      extractor.setNegative (false);
      extractor.setKeepOrganized (false);//??
      extractor.filter (*tmpp);

      // Creating the KdTree object for the search method of the extraction
      search::KdTree<Point>::Ptr tree (new search::KdTree<Point>);
      tree->setInputCloud (tmpp);

      std::vector< PointIndices > cluster_indices;
      EuclideanClusterExtraction< Point > ec_ext;
      ec_ext.setClusterTolerance (tolerance); // 2cm
      ec_ext.setMinClusterSize (min_plane_points);
      // ec_ext.setMaxClusterSize (25000); // no max limitation
      ec_ext.setSearchMethod (tree);
      ec_ext.setInputCloud (tmpp);
      ec_ext.extract (cluster_indices);

      if (cluster_indices.size()  > 0) {
        PointIndicesPtr tinlier (new PointIndices);
        for (std::vector<int>::iterator it = cluster_indices[0].indices.begin();
             it != cluster_indices[0].indices.end(); it++) {
          tinlier->indices.push_back (inliers->indices[*it]);
        }
        inliers = tinlier;
      } else {
        std::cerr << ";; there is no large cursters in " << inliers->indices.size();
        std::cerr << " points" << std::endl;
        break;
      }
    }
#if DEBUG
    std::cerr << ";; 2 TIME = " << tm.elapsed() << std::endl;
    tm.restart();
    std::cerr << ";; B inlier: " << inliers->indices.size() << std::endl;
#endif
    extractor.setInputCloud (pcl_cloud_ptr);
    extractor.setIndices (inliers);
    extractor.setNegative (true);
    extractor.setKeepOrganized (true); //??
    extractor.filterDirectly (pcl_cloud_ptr);
#if DEBUG
    std::cerr << ";; 3 TIME = " << tm.elapsed() << std::endl;
#endif
    {
      pointer ivec = makevector (C_INTVECTOR, inliers->indices.size());
      vpush (ivec); pc++;

      indices_list = rawcons (ctx, ivec, indices_list);
      vpush (indices_list); pc++;

      eusinteger_t *iv = ivec->c.ivec.iv;
      for (std::vector<int>::iterator it = inliers->indices.begin();
         it != inliers->indices.end(); it++) {
        *iv++ = (*it);
      }
    }
    plane_num++;

    point_size -= inliers->indices.size();
    if (point_size < min_plane_points) break;
#if DEBUG
    std::cerr << ";; TOTAL TIME = " << total_tm.elapsed() << std::endl << std::endl;
#endif
  }
#if DEBUG
    std::cerr << ";; TOTAL TIME = " << total_tm.elapsed() << std::endl;
#endif

  ret = rawcons (ctx, indices_list, NIL); vpush (ret); pc++;
  ret = rawcons (ctx, coeff_list, ret);

  while (pc-- > 0) vpop();
  return ret;
}
