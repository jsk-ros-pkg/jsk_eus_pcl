#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_octree.h"

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
  /* ( pointclouda pointcloudb &optional (resolution 100.0) */
  pointer in_cloud_a, in_cloud_b ;
  pointer ret = NIL;
  eusfloat_t resolution = 100.0;
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

  PointCloud< Point > rap, rbp;
#if DEBUG
  fprintf(stderr, "first points / removing points %d / %d\n",
          new_idx_vec_a.size(),
          pcl_cloud_a->points.size());
#endif
  {
    ExtractIndices< Point > ext_ind;
    IndicesPtr pcl_indices (new Indices());
    *pcl_indices = new_idx_vec_a;
    ext_ind.setInputCloud (pcl_cloud_a);
    ext_ind.setIndices (pcl_indices);
    ext_ind.setNegative (true);
    ext_ind.filter (rap);
  }
#if DEBUG
  fprintf(stderr, "second points / removing points %d / %d\n",
          new_idx_vec_b.size(),
          pcl_cloud_b->points.size());
#endif
  {
    ExtractIndices< Point > ext_ind;
    IndicesPtr pcl_indices (new Indices());
    *pcl_indices = new_idx_vec_b;
    ext_ind.setInputCloud (pcl_cloud_b);
    ext_ind.setIndices (pcl_indices);
    ext_ind.setNegative (true);
    ext_ind.filter (rbp);
  }
#if DEBUG
  fprintf(stderr, "returning first points / second points %d / %d\n",
          rap.points.size(),
          rbp.points.size());
#endif
  {
    pointer ret_a = NIL;
    pointer ret_b = NIL;
    ret_a = make_pointcloud_from_pcl (ctx, rap);
    vpush(ret_a);
    ret_b = make_pointcloud_from_pcl (ctx, rbp);
    vpush(ret_b);
    ret = rawcons(ctx, ret_a, ret_b);
    vpop(); vpop();
  }

  return ret;
}

pointer PCL_OCT_HEIGHTMAP_GRID (register context *ctx, int n, pointer *argv) {
  /* ( pointcloud resolution max-x max-y min-x min-y */
  pointer in_cloud;
  pointer ret = NIL;
  eusfloat_t resolution;
  numunion nu;
  double max_x, min_x;
  double max_y, min_y;

  ckarg(6);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return NIL;
  }
  in_cloud = argv[0];

  resolution = ckfltval (argv[1]);
  max_x = ckfltval (argv[2]);
  max_y = ckfltval (argv[3]);
  min_x = ckfltval (argv[4]);
  min_y = ckfltval (argv[5]);
  // check min/max

  pointer points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);
  int width = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
  int psize = width * height;

  int xsize = ceil((max_x - min_x)/resolution);
  int ysize = ceil((max_y - min_y)/resolution);

  std::vector < std::vector<int> > indices_grid;
  indices_grid.resize(xsize*ysize);
  //std::cerr << "sz: " <<  psize  << ", " << width << " - " << height << std::endl;
  //std::cerr << "res: " <<  resolution  << ", " << xsize << " - " << ysize << std::endl;

  eusfloat_t *fvec = points->c.ary.entity->c.fvec.fv;
  for(int i = 0; i < psize; i++) {
    int p = i * 3;
    eusfloat_t x = fvec[p + 0];
    eusfloat_t y = fvec[p + 1];
    int xidx = floor((x - min_x)/resolution);
    int yidx = floor((y - min_y)/resolution);
    if (xidx >= 0 && xidx < xsize &&
        yidx >= 0 && yidx < ysize ) {
      indices_grid[yidx*xsize + xidx].push_back(i);
    }
  }

  vpush(ret);
  for(int n = 0; n < indices_grid.size(); n++) {
    int sz = indices_grid[n].size();
    //std::cerr << "n: " << n << ", " << sz << std::endl;
    if(sz > 0) {
      pointer eus_idx;
      std::vector< int > *idxlst = &(indices_grid[n]);
      eus_idx = makevector (C_INTVECTOR, sz);
      vpush (eus_idx);
      for(int i = 0; i < sz; i++) {
        eus_idx->c.ivec.iv[i] = idxlst->at(i);
      }
      ret = rawcons(ctx, eus_idx, ret);
      vpop(); vpop();
      vpush(ret);
    }
  }
  vpop();

  return ret;
}

pointer PCL_OCT_POINT_VECTOR (register context *ctx, int n, pointer *argv) {
  /* ( pointcloud &optional (resolution 100.0) (max-x) (max-y) (max-z) (min-x) (min-y) (min-z) */
  pointer in_cloud;
  pointer ret = NIL;
  eusfloat_t resolution = 100.0;
  numunion nu;

  ckarg2(1, 8);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
    return NIL;
  }
  in_cloud = argv[0];

  if (n > 1) {
    resolution = ckfltval (argv[1]);
  }
  resolution /= 1000.0;

  PointCloud< Point >::Ptr pcl_cloud;
  {
    int width = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
    int height = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
    pointer points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);
    pcl_cloud =
      make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);
  }

  octree::OctreePointCloudPointVector< Point > oct_pvec (resolution);

  // set bunding box
  if (n == 8) {
    eusfloat_t max_x_arg = ckfltval (argv[2])/1000.0;
    eusfloat_t max_y_arg = ckfltval (argv[3])/1000.0;
    eusfloat_t max_z_arg = ckfltval (argv[4])/1000.0;
    eusfloat_t min_x_arg = ckfltval (argv[5])/1000.0;
    eusfloat_t min_y_arg = ckfltval (argv[6])/1000.0;
    eusfloat_t min_z_arg = ckfltval (argv[7])/1000.0;
    //std::cerr << "min: " << min_x_arg << ", " << min_y_arg << ", " << min_z_arg << std::endl;
    //std::cerr << "max: " << max_x_arg << ", " << max_y_arg << ", " << max_z_arg << std::endl;
    oct_pvec.defineBoundingBox(min_x_arg, min_y_arg, min_z_arg,
                               max_x_arg, max_y_arg, max_z_arg);
  } else if (n == 5) {
    eusfloat_t max_x_arg = ckfltval (argv[2])/1000.0;
    eusfloat_t max_y_arg = ckfltval (argv[3])/1000.0;
    eusfloat_t max_z_arg = ckfltval (argv[4])/1000.0;
    oct_pvec.defineBoundingBox(max_x_arg, max_y_arg, max_z_arg);
  }

  //
  oct_pvec.setInputCloud (pcl_cloud);
  oct_pvec.addPointsFromInputCloud ();

  double min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
  oct_pvec.getBoundingBox(min_x_, min_y_, min_z_, max_x_, max_y_, max_z_);

  //std::cerr << "max depth: " << oct_pvec.getTreeDepth() << std::endl;
  //std::cerr << "leaf num: " << oct_pvec.getLeafCount() << std::endl;
  //std::cerr << "branch num: " << oct_pvec.getBranchCount() << std::endl;

  octree::OctreePointCloudPointVector< Point >::LeafNodeIterator it
    = oct_pvec.leaf_begin();
  octree::OctreePointCloudPointVector< Point >::LeafNodeIterator it_end
    = oct_pvec.leaf_end();

  vpush(ret);
  while (it != it_end) {
    octree::OctreeKey k = it.getCurrentOctreeKey();
    //center x,y,z
    double cent_x = (static_cast<double> (k.x) + 0.5f) * resolution + min_x_;
    double cent_y = (static_cast<double> (k.y) + 0.5f) * resolution + min_y_;
    double cent_z = (static_cast<double> (k.z) + 0.5f) * resolution + min_z_;
#if 0
    Eigen::Vector3f minp;
    Eigen::Vector3f maxp;
    oct_pvec.getVoxelBounds(it, minp, maxp);
    std::cerr << "min: " << minp.x() << ", " << minp.y() << ", " << minp.z() << std::endl;
    std::cerr << "max: " << maxp.x() << ", " << maxp.y() << ", " << maxp.z() << std::endl;
#endif
    //std::cerr << "depth: " << it.getCurrentOctreeDepth() << std::endl;
    octree::OctreePointCloudPointVector< Point >::LeafNodeIterator::LeafContainer lc = it.getLeafContainer();

    std::vector< int > idx = lc.getPointIndicesVector();

    pointer eus_idx;
    eus_idx = makevector (C_INTVECTOR, idx.size());
    vpush (eus_idx);
    for(int i = 0; i < idx.size(); i++) {
      eus_idx->c.ivec.iv[i] = idx[i];
    }
    ret = rawcons(ctx, eus_idx, ret);
    vpop(); vpop();
    vpush(ret);

    it++;
  }
  vpop();

  return ret;
}
