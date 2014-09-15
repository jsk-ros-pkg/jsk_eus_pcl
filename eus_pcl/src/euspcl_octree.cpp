#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_octree.h"

#if __PCL_SELECT == 0
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#elif __PCL_SELECT == 17
#include <pcl17/octree/octree.h>
#include <pcl17/octree/octree_impl.h>
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

  ckarg2(1, 3);
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
