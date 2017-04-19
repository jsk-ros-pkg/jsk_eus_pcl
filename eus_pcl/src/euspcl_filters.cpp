#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_filters.h"

#if __PCL_SELECT == 0
using namespace pcl;
#elif __PCL_SELECT == 17
using namespace pcl17;
#endif

#define DOWNSAMPLE_(PTYPE)                                              \
  PointCloud< PTYPE >::Ptr pcl_cloud =                                  \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, curvatures, width, height); \
  PointCloud< PTYPE > pcl_cloud_filtered;                               \
  VoxelGrid< PTYPE > vg;                                                \
  vg.setInputCloud (pcl_cloud);                                         \
  vg.setLeafSize (leaf_x, leaf_y, leaf_z);                              \
  vg.filter (pcl_cloud_filtered);                                       \
  ret = make_pointcloud_from_pcl (ctx, pcl_cloud_filtered);             \
  vpush(ret); pc++;

pointer PCL_VOXEL_GRID (register context *ctx, int n, pointer *argv) {
  /* pointcloud &optional (leaf_x 0.02) (leaf_y 0.02) (leaf_z 0.02) */
  pointer in_cloud;
  pointer points,colors,normals, curvatures;
  pointer ret = NIL;
  eusfloat_t leaf_x, leaf_y, leaf_z;
  numunion nu;
  int pc = 0;

  leaf_x = 0.02;
  leaf_y = 0.02;
  leaf_z = 0.02;

  ckarg2 (1, 4);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return ret;
  }
  in_cloud = argv[0];

  if ( n > 1 ) {
    leaf_x = (ckfltval(argv[1])) / 1000.0;
  }
  if ( n > 2 ) {
    leaf_y = (ckfltval(argv[2])) / 1000.0;
  }
  if ( n > 3 ) {
    leaf_z = (ckfltval(argv[3])) / 1000.0;
  }

  int width = intval(get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);
  colors = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_COLORS);
  normals = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_NORMALS);
  curvatures = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_CURVATURES);

  if ( points != NIL && colors != NIL && normals != NIL ) {
    DOWNSAMPLE_(PointCN);
  } else if ( points != NIL && colors != NIL ) {
    DOWNSAMPLE_(PointC);
  } else if ( points != NIL && normals != NIL ) {
    DOWNSAMPLE_(PointN);
  } else if ( points != NIL ) {
    DOWNSAMPLE_(Point);
  } else {
    // warning there is no points.
  }

  while ( pc-- > 0) vpop();
  return ret;
}

#define EXTRACT_INDICES_(PTYPE)                                         \
  PointCloud< PTYPE >::Ptr pcl_cloud =                                  \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, curvatures, width, height); \
  PointCloud< PTYPE > pcl_cloud_filtered;                               \
  ExtractIndices< PTYPE > ext_ind;                                      \
  ext_ind.setInputCloud (pcl_cloud);                                    \
  ext_ind.setIndices (pcl_indices);                                     \
  ext_ind.setNegative (pcl_negative);                                   \
  ext_ind.filter (pcl_cloud_filtered);                                  \
  if (create_cloud) {                                                   \
    ret = make_pointcloud_from_pcl (ctx, pcl_cloud_filtered);           \
    vpush(ret); pc++;                                                   \
  } else {                                                              \
    ret = make_pointcloud_from_pcl (ctx, pcl_cloud_filtered, in_cloud); \
  }

pointer PCL_EXTRACT_INDICES (register context *ctx, int n, pointer *argv) {
  /* pointcloud indices &optional (negative nil) (create t) */
  pointer in_cloud;
  pointer points, colors, normals, curvatures;
  pointer ret = NIL;
  pointer eus_indices;
  bool pcl_negative = false;
  bool create_cloud = true;
  numunion nu;
  int pc = 0;

  ckarg2 (2, 4);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return ret;
  }
  in_cloud = argv[0];

  eus_indices = argv[1];

  if (n > 2) {
    if (argv[2] != NIL) {
      pcl_negative = true;
    }
  }
  if (n > 3) {
    if (argv[3] == NIL) {
      create_cloud = false;
    }
  }

  IndicesPtr pcl_indices (new Indices());
  if (isintvector(eus_indices)) {
    // intvec
    size_t vsize = vecsize(eus_indices);
    pcl_indices->resize(vsize);
    for(size_t i = 0; i < vsize; i++) {
      (*pcl_indices)[i] = eus_indices->c.ivec.iv[i];
    }
  } else if (iscons(eus_indices)) {
    // list
    pointer tmp = eus_indices;
    pcl_indices->resize(0);
    while (tmp != NIL) {
      pointer i = ccar(tmp);
      if (isint (i)) {
        pcl_indices->push_back (intval(i));
      }
      tmp = ccdr(tmp);
    }
  } else {
    error(E_TYPEMISMATCH);
  }

  int width = intval(get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);
  colors = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_COLORS);
  normals = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_NORMALS);
  curvatures = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_CURVATURES);

  if (points != NIL && colors != NIL && normals != NIL) {
    EXTRACT_INDICES_(PointCN);
  } else if (points != NIL && colors != NIL) {
    EXTRACT_INDICES_(PointC);
  } else if (points != NIL && normals != NIL) {
    EXTRACT_INDICES_(PointN);
  } else if (points != NIL) {
    EXTRACT_INDICES_(Point);
  } else {
    // warning there is no points.
  }

  while (pc-- > 0) vpop();
  return ret;
}
