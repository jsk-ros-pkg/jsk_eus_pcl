#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_filters.h"

#define DOWNSAMPLE_(PTYPE, leaf_x, leaf_y, leaf_z) \
  pcl::PointCloud< PTYPE >::Ptr pcl_cloud = \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, width, height); \
  pcl::PointCloud< PTYPE > pcl_cloud_filtered; \
  pcl::VoxelGrid< PTYPE > vg; \
  vg.setInputCloud ( pcl_cloud ); \
  vg.setLeafSize ( leaf_x, leaf_y, leaf_z ); \
  vg.filter ( pcl_cloud_filtered ); \
  ret = make_pointcloud_from_pcl ( ctx, pcl_cloud_filtered ); \
  vpush(ret); pc++;

pointer PCL_VOXEL_GRID (register context *ctx, int n, pointer *argv) {
  /* pointcloud &optional (leaf_x 0.02) (leaf_y 0.02) (leaf_z 0.02) */
  pointer in_cloud;
  pointer points,colors,normals;
  pointer ret = NIL;
  eusfloat_t leaf_x, leaf_y, leaf_z;
  numunion nu;
  int pc = 0;

  leaf_x = 0.02;
  leaf_y = 0.02;
  leaf_z = 0.02;
  ckarg2(1, 4);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return ret;
  }
  in_cloud = argv[0];

  if ( n > 1 ) {
    leaf_x = ( fltval(argv[1]) ) / 1000.0;
  }
  if ( n > 2 ) {
    leaf_y = ( fltval(argv[2]) ) / 1000.0;
  }
  if ( n > 3 ) {
    leaf_z = ( fltval(argv[3]) ) / 1000.0;
  }

  int width = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_POINTS);
  colors = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_COLORS);
  normals = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_NORMALS);

  if ( points != NIL && colors != NIL && normals != NIL ) {
    DOWNSAMPLE_(PointCN, leaf_x, leaf_y, leaf_z);
  } else if ( points != NIL && colors != NIL ) {
    DOWNSAMPLE_(PointC, leaf_x, leaf_y, leaf_z);
  } else if ( points != NIL && normals != NIL ) {
    DOWNSAMPLE_(PointN, leaf_x, leaf_y, leaf_z);
  } else if ( points != NIL ) {
    DOWNSAMPLE_(Point, leaf_x, leaf_y, leaf_z);
  } else {
    // warning there is no points.
  }

  while ( pc-- > 0) vpop();
  return ret;
}
