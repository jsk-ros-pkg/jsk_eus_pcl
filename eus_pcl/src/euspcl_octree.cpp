#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_octree.h"

#if __PCL_SELECT == 0
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>
#elif __PCL_SELECT == 17
#include <pcl17/octree/octree.h>
#include <pcl17/octree/octree_impl.h>
#include <pcl17/filters/extract_indices.h>
#endif

#if __PCL_SELECT == 0
using namespace pcl;
#elif __PCL_SELECT == 17
using namespace pcl17;
#endif

pointer PCL_OCT_VOXEL (register context *ctx, int n, pointer *argv) {
  /* ( pointcloud &optional (resolution 40.0) (listp) */
  pointer in_cloud;
  pointer ret = NIL;
  pointer points;
  eusfloat_t resolution = 40.0;
  numunion nu;
  int pc = 0;
  bool listp = false;

  ckarg2(1, 9);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return ret;
  }
  in_cloud = argv[0];

  if (n > 1) {
    resolution = ckfltval (argv[1]);
  }
  resolution /= 1000.0;

  if (n > 2) {
    if (argv[2] != NIL) {
      listp = true;
    }
  }

  int width = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);

  PointCloud< Point >::Ptr pcl_cloud =
    make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);

  //octree::OctreePointCloudVoxelCentroid < Point > oct(resolution);
  octree::OctreePointCloudOccupancy < Point > oct(resolution);

  // set bunding box
  if (n == 9) {
    eusfloat_t max_x_arg = ckfltval (argv[3]);
    eusfloat_t max_y_arg = ckfltval (argv[4]);
    eusfloat_t max_z_arg = ckfltval (argv[5]);
    eusfloat_t min_x_arg = ckfltval (argv[6]);
    eusfloat_t min_y_arg = ckfltval (argv[7]);
    eusfloat_t min_z_arg = ckfltval (argv[8]);
    oct.defineBoundingBox(min_x_arg, min_y_arg, min_z_arg,
                          max_x_arg, max_y_arg, max_z_arg);
  } else if (n == 6) {
    eusfloat_t max_x_arg = ckfltval (argv[3]);
    eusfloat_t max_y_arg = ckfltval (argv[4]);
    eusfloat_t max_z_arg = ckfltval (argv[5]);
    oct.defineBoundingBox(max_x_arg, max_y_arg, max_z_arg);
  }
  //
  oct.setInputCloud (pcl_cloud);
  oct.addPointsFromInputCloud ();

  //octree::OctreePointCloud < Point >::AlignedPointTVector cpts;
  Points::VectorType cpts;
  //size_t psize = oct.getVoxelCentroids (cpts);
  size_t psize = oct.getOccupiedVoxelCenters (cpts);
  //std::cerr << ";; ret = " << psize << std::endl;

  if (listp) {
    vpush (ret);
    for (size_t i = 0; i < psize; i++) {
      pointer tmp = makefvector(3);
      eusfloat_t *fp = tmp->c.fvec.fv;
      *fp++ = cpts[i].x * 1000.0;
      *fp++ = cpts[i].y * 1000.0;
      *fp++ = cpts[i].z * 1000.0;
      vpush (tmp);
      ret = rawcons (ctx, tmp, ret);
      vpop (); vpop();
      vpush (ret);
    }
    vpop ();
  } else {
    ret = makematrix(ctx, psize, 3);
    eusfloat_t *fp = ret->c.ary.entity->c.fvec.fv;
    for (size_t i = 0; i < psize; i++) {
      *fp++ = cpts[i].x * 1000.0;
      *fp++ = cpts[i].y * 1000.0;
      *fp++ = cpts[i].z * 1000.0;
    }
  }

  return ret;
}

pointer PCL_COR_PTS (register context *ctx, int n, pointer *argv) {
  /* ( pointclouda pointcloudb &optional (resolution 40.0) */
  pointer in_cloud_a, in_cloud_b ;
  pointer ret = NIL;
  eusfloat_t resolution = 40.0;
  numunion nu;
  int pc = 0;
  bool listp = false;

  ckarg2(2, 9);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return NIL;
  }
  in_cloud_a = argv[0];

  if (!isPointCloud (argv[1])) {
    error(E_TYPEMISMATCH);
    return NIL;
  }
  in_cloud_b = argv[1];

  if (n > 2) {
    resolution = ckfltval (argv[2]);
  }
  resolution /= 1000.0;

  PointCloud< Point >::Ptr pcl_cloud_a;
  {
    int width = intval (get_from_pointcloud (ctx, in_cloud_a, K_EUSPCL_WIDTH));
    int height = intval (get_from_pointcloud (ctx, in_cloud_a, K_EUSPCL_HEIGHT));
    pointer points = get_from_pointcloud (ctx, in_cloud_a, K_EUSPCL_POINTS);
    pcl_cloud_a =
      make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);
  }
  PointCloud< Point >::Ptr pcl_cloud_b;
  {
    int width = intval (get_from_pointcloud (ctx, in_cloud_b, K_EUSPCL_WIDTH));
    int height = intval (get_from_pointcloud (ctx, in_cloud_b, K_EUSPCL_HEIGHT));
    pointer points = get_from_pointcloud (ctx, in_cloud_b, K_EUSPCL_POINTS);
    pcl_cloud_b =
      make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);
  }

  pcl::octree::OctreePointCloudChangeDetector< Point > oct_a (resolution);
  pcl::octree::OctreePointCloudChangeDetector< Point > oct_b (resolution);
  // set bunding box
  if (n == 9) {
    eusfloat_t max_x_arg = ckfltval (argv[3]);
    eusfloat_t max_y_arg = ckfltval (argv[4]);
    eusfloat_t max_z_arg = ckfltval (argv[5]);
    eusfloat_t min_x_arg = ckfltval (argv[6]);
    eusfloat_t min_y_arg = ckfltval (argv[7]);
    eusfloat_t min_z_arg = ckfltval (argv[8]);
    oct_a.defineBoundingBox(min_x_arg, min_y_arg, min_z_arg,
                            max_x_arg, max_y_arg, max_z_arg);
    oct_b.defineBoundingBox(min_x_arg, min_y_arg, min_z_arg,
                            max_x_arg, max_y_arg, max_z_arg);
  } else if (n == 6) {
    eusfloat_t max_x_arg = ckfltval (argv[3]);
    eusfloat_t max_y_arg = ckfltval (argv[4]);
    eusfloat_t max_z_arg = ckfltval (argv[5]);
    oct_a.defineBoundingBox(max_x_arg, max_y_arg, max_z_arg);
    oct_b.defineBoundingBox(max_x_arg, max_y_arg, max_z_arg);
  }

  //
  oct_a.setInputCloud (pcl_cloud_a);
  oct_a.addPointsFromInputCloud ();

  oct_a.switchBuffers ();

  oct_a.setInputCloud (pcl_cloud_b);
  oct_a.addPointsFromInputCloud ();

  std::vector<int> new_idx_vec_b;

  oct_a.getPointIndicesFromNewVoxels (new_idx_vec_b);

  //
  oct_b.setInputCloud (pcl_cloud_b);
  oct_b.addPointsFromInputCloud ();

  oct_b.switchBuffers ();

  oct_b.setInputCloud (pcl_cloud_a);
  oct_b.addPointsFromInputCloud ();

  std::vector<int> new_idx_vec_a;

  oct_b.getPointIndicesFromNewVoxels (new_idx_vec_a);

  // remove new_idx_vec_a from pcl_cloud_a
  {
    ExtractIndices< Point > ext_ind;
    ext_ind.setInputCloud (pcl_cloud_a);
    IndicesPtr pcl_indices (&new_idx_vec_a);
    ext_ind.setIndices (pcl_indices);
    ext_ind.setNegative (true);
    ext_ind.filter (*pcl_cloud_a);
  }
  // remove new_idx_vec_b from pcl_cloud_b
  {
    ExtractIndices< Point > ext_ind;
    ext_ind.setInputCloud (pcl_cloud_b);
    IndicesPtr pcl_indices (&new_idx_vec_b);
    ext_ind.setIndices (pcl_indices);
    ext_ind.setNegative (true);
    ext_ind.filter (*pcl_cloud_b);
  }

  {
    pointer ret_a = NIL;
    pointer ret_b = NIL;
    ret_a = make_pointcloud_from_pcl (ctx, *pcl_cloud_a);
    vpush(ret_a);
    ret_b = make_pointcloud_from_pcl (ctx, *pcl_cloud_b);
    vpush(ret_b);
    ret = rawcons(ctx, ret_a, ret_b);
    vpop(); vpop();
  }

  return ret;
}
