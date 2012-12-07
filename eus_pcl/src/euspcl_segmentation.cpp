#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_segmentation.h"

pointer EXTRACT_EUCLIDEAN_CLUSTERS (register context *ctx, int n, pointer *argv) {
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

  pcl::PointCloud< Point >::Ptr pcl_cloud =
    make_pcl_pointcloud< Point > (ctx, points, NIL, NIL, width, height);

  pcl::search::KdTree< Point >::Ptr tree (new pcl::search::KdTree< Point > ());
  tree->setInputCloud (pcl_cloud); //??
  std::vector< pcl::PointIndices > cluster_indices;
  pcl::EuclideanClusterExtraction< Point > euc_ext;
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
