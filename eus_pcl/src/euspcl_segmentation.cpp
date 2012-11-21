#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_segmentation.h"

pointer EXTRACT_EUCLIDEAN_CLUSTERS (register context *ctx, int n, pointer *argv) {
  /* ( pointcloud &optional (minsize 500) (maxsize 20000) (tolerance 20.0) ) */
  pointer in_cloud;
  pointer points;
  pointer ret = NIL;
  eusinteger_t *ret_vec;
  eusfloat_t tolerance;
  eusinteger_t minsize, maxsize;
  numunion nu;
  int pc = 0;

  tolerance = 0.02;
  minsize = 500;
  maxsize = 20000;

  ckarg2(1, 4);
  in_cloud = argv[0];

  if ( n > 1 ) {
    minsize = intval(argv[1]);
  }
  if ( n > 2 ) {
    maxsize = intval(argv[2]);
  }
  if ( n > 3 ) {
    tolerance = ( fltval(argv[3]) ) / 1000.0;
  }

  int width = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_POINTS);

  pcl::PointCloud< Point >::Ptr pcl_cloud =
    make_pcl_pointcloud< Point > (ctx, points, NIL, NIL, width, height);

  ret = makevector(C_INTVECTOR, width * height);
  ret_vec = ret->c.ivec.iv;
  for ( size_t i = 0; i < width * height; i++ ) ret_vec[i] = -1;

  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
  tree->setInputCloud (pcl_cloud); //??
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setInputCloud (pcl_cloud);
  ec.setClusterTolerance (tolerance);
  ec.setMinClusterSize (minsize);
  ec.setMaxClusterSize (maxsize);
  ec.setSearchMethod (tree);
  ec.extract (cluster_indices);

  size_t cntr = 0;
  for (size_t i = 0; i < cluster_indices.size(); i++) {
    for ( size_t j = 0; j < cluster_indices[i].indices.size(); j++ ) {
      ret_vec[ cluster_indices[i].indices[j] ] = i;
    }
  }

  return ret;
}
