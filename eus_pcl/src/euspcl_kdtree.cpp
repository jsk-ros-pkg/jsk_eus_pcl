#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_kdtree.h"

#if __PCL_SELECT == 0
using namespace pcl;
#elif __PCL_SELECT == 17
using namespace pcl17;
#endif

pointer PCL_KDTREE_K_SEARCH (register context *ctx, int n, pointer *argv) {
  /* ptr point K */
  numunion nu;
  KdTreeFLANN < Point > *ptr;
  Point pt;
  eusinteger_t k;
  std::vector<int> indices;
  std::vector<float> distance;
  indices.resize(k);
  distance.resize(k);

  ptr = (KdTreeFLANN < Point > *)(intval(argv[0]));
  if (!isfltvector(argv[1]) || vecsize(argv[1]) != 3) {
    error(E_TYPEMISMATCH);
    return NIL;
  }
  pt.x = argv[1]->c.fvec.fv[0]*0.001;
  pt.y = argv[1]->c.fvec.fv[1]*0.001;
  pt.z = argv[1]->c.fvec.fv[2]*0.001;

  k = ckintval(argv[2]);

  ptr->nearestKSearch (pt, k, indices, distance);

  if(indices.size() == 0) {
    return NIL;
  }

  pointer eus_idx, eus_dist;
  eus_idx = makevector (C_INTVECTOR, indices.size());
  vpush (eus_idx);
  eus_dist = makefvector (distance.size());
  vpush(eus_dist);
  pointer ret = rawcons (ctx, eus_idx, eus_dist);
  vpop(); vpop(); vpush(ret);

  for(int i = 0; i < indices.size(); i++) {
    eus_idx->c.ivec.iv[i] = indices[i];
  }
  for(int i = 0; i < distance.size(); i++) {
    eus_dist->c.fvec.fv[i] = sqrt(distance[i]*1000000);
  }

  vpop();
  return ret;
}

pointer PCL_KDTREE_R_SEARCH (register context *ctx, int n, pointer *argv) {
  /* ptr point radius */
  numunion nu;
  KdTreeFLANN < Point > *ptr;
  Point pt;
  eusfloat_t radius;
  std::vector<int> indices;
  std::vector<float> distance;

  ckarg (3);

  ptr = (KdTreeFLANN < Point > *)(intval(argv[0]));
  if (!isfltvector(argv[1]) || vecsize(argv[1]) != 3) {
    error(E_TYPEMISMATCH);
    return NIL;
  }
  pt.x = argv[1]->c.fvec.fv[0]*0.001;
  pt.y = argv[1]->c.fvec.fv[1]*0.001;
  pt.z = argv[1]->c.fvec.fv[2]*0.001;

  radius = ckfltval(argv[2]) * 0.001;

  ptr->radiusSearch (pt, radius, indices, distance);

  if(indices.size() == 0) {
    return NIL;
  }

  pointer eus_idx, eus_dist;
  eus_idx = makevector (C_INTVECTOR, indices.size());
  vpush (eus_idx);
  eus_dist = makefvector (distance.size());
  vpush(eus_dist);
  pointer ret = rawcons (ctx, eus_idx, eus_dist);
  vpop(); vpop(); vpush(ret);

  for(int i = 0; i < indices.size(); i++) {
    eus_idx->c.ivec.iv[i] = indices[i];
  }
  for(int i = 0; i < distance.size(); i++) {
    eus_dist->c.fvec.fv[i] = sqrt(distance[i]*1000000);
  }

  vpop();
  return ret;
}

pointer PCL_KDTREE_CREATE (register context *ctx, int n, pointer *argv) {
  numunion nu;
  pointer in_cloud;
  pointer points;

  ckarg (1);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return NIL;
  }
  in_cloud = argv[0];

  int width  = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);

  PointCloud< Point >::Ptr pcl_cloud =
    make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);
  KdTreeFLANN < Point > *ptr = new KdTreeFLANN< Point >();
  ptr->setInputCloud(pcl_cloud);

  return makeint((eusinteger_t)ptr);
}

pointer PCL_KDTREE_DELETE (register context *ctx, int n, pointer *argv) {
  numunion nu;
  void *ptr;

  ckarg(1);
  ptr = (void *)(intval(argv[0]));

  delete ( (KdTreeFLANN< Point > *)ptr );

  return NIL;
}
