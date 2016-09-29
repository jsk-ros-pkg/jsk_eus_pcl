#include "eus_pcl/euspcl.h"

#include <octomap/octomap.h>

#if __PCL_SELECT == 0
using namespace pcl;
#elif __PCL_SELECT == 17
using namespace pcl17;
#endif

pointer OCTOMAP_CREATE (register context *ctx, int n, pointer *argv) {
  numunion nu;
  double resolution = 50;
  octomap::point3d pmin(-1.5,-1.5,0), pmax(1.5,1.5,3); // default

  ckarg2(0,3);
  if (n > 0) {
    resolution = ckfltval(argv[0]);
  }
  octomap::OcTree *tree_ptr;
  tree_ptr = new octomap::OcTree ( resolution / 1000.0 );

  if (n > 1 && isstring(argv[1])) {
    // use binary data
    std::stringstream datastream;
    long len = strlength(argv[1]);
    datastream.write((const char *)(argv[1]->c.str.chars), len);
    tree_ptr->readBinaryData(datastream);

    return makeint((eusinteger_t)tree_ptr);
  } else {
    if (n > 1) {
      pointer min_vec = argv[1];
      pmin.x() = min_vec->c.fvec.fv[0] / 1000;
      pmin.y() = min_vec->c.fvec.fv[1] / 1000;
      pmin.z() = min_vec->c.fvec.fv[2] / 1000;
    }
    if (n > 2) {
      pointer max_vec = argv[2];
      pmax.x() = max_vec->c.fvec.fv[0] / 1000;
      pmax.y() = max_vec->c.fvec.fv[1] / 1000;
      pmax.z() = max_vec->c.fvec.fv[2] / 1000;
    }
  }
  // printf("res = %f\n", resolution);
  tree_ptr->setBBXMax(pmax);
  tree_ptr->setBBXMin(pmin);
  tree_ptr->useBBXLimit(true); //???

  return makeint((eusinteger_t)tree_ptr);
}

pointer OCTOMAP_DELETE (register context *ctx, int n, pointer *argv) {
  numunion nu;
  void *octree;

  ckarg(1);
  octree = (void *)(intval(argv[0]));

  delete ( (octomap::OcTree *)octree );

  return NIL;
}

pointer OCTOMAP_NODE_NUM (register context *ctx, int n, pointer *argv) {
  numunion nu;
  octomap::OcTree *tree_ptr;

  ckarg(1);
  tree_ptr = (octomap::OcTree *)(intval(argv[0]));

  eusinteger_t ret = tree_ptr->calcNumNodes();
  return makeint(ret);
}

pointer OCTOMAP_READ_NODES (register context *ctx, int n, pointer *argv) {
  numunion nu;
  octomap::OcTree *tree_ptr;

  ckarg2(1, 2);
  tree_ptr = (octomap::OcTree *)(intval(argv[0]));

  int depth = 0;
  if (n > 1) {
    depth = intval(argv[1]);
  }

  PointCloud< Point > occ_points;
  PointCloud< Point > free_points;
  for (octomap::OcTree::iterator it = tree_ptr->begin(depth), end = tree_ptr->end();
       it != end; ++it) {
    if (tree_ptr->isNodeOccupied(*it)) { // occupied
      Point p(it.getX(),
              it.getY(),
              it.getZ());
      occ_points.push_back(p);
    } else { // free
      Point p(it.getX(),
              it.getY(),
              it.getZ());
      free_points.push_back(p);
    }
  }

  pointer ret = NIL;
  {
    pointer ret_free = make_pointcloud_from_pcl (ctx, free_points);
    vpush(ret_free);
    ret = rawcons (ctx, ret_free, ret);
    vpop(); vpush(ret);
    pointer ret_occ = make_pointcloud_from_pcl (ctx, occ_points);
    vpush(ret_occ);
    ret = rawcons (ctx, ret_occ, ret);
    vpop(); vpop();
  }

  return ret;
}

pointer OCTOMAP_READ_UNKNOWN (register context *ctx, int n, pointer *argv) {
  numunion nu;
  octomap::OcTree *tree_ptr;

  ckarg2(1,4);
  tree_ptr = (octomap::OcTree *)(intval(argv[0]));

  int depth = 0;
  if (n > 1) {
    depth = intval(argv[1]);
  }

  octomap::point3d pmin, pmax;
  pmax = tree_ptr->getBBXMax();
  pmin = tree_ptr->getBBXMin();

  if (n > 2) {
    pointer min_vec = argv[2];
    pmin.x() = min_vec->c.fvec.fv[0] / 1000;
    pmin.y() = min_vec->c.fvec.fv[1] / 1000;
    pmin.z() = min_vec->c.fvec.fv[2] / 1000;
  }
  if (n > 3) {
    pointer max_vec = argv[3];
    pmax.x() = max_vec->c.fvec.fv[0] / 1000;
    pmax.y() = max_vec->c.fvec.fv[1] / 1000;
    pmax.z() = max_vec->c.fvec.fv[2] / 1000;
  }

  //tree_ptr->getUnknownLeafCenters (node_centers, pmin, pmax);
  //// unknown leafs
  int tree_depth = tree_ptr->getTreeDepth();
  if (depth == 0)
    depth = tree_depth;
  double step_size =  tree_ptr->getResolution() * pow(2, tree_depth-depth);
  printf("%d %f\n", tree_depth, step_size);

  int start_idx[3];
  int end_idx[3];
  for (int i = 0; i < 3; i++) {
    end_idx[i] = floor((pmax(i)+step_size/2)/step_size);
    start_idx[i] = floor((pmin(i)+step_size/2)/step_size) + 1;
    //printf("%d / %d %d %f %f\n",i,start_idx[i], end_idx[i], pmax(i), pmin(i));
  }

  PointCloud< Point > pc;
  octomap::point3d pt;
  octomap::OcTree::NodeType* res;
  for (int x = start_idx[0]; x <= end_idx[0]; x++) {
    pt.x() = (x - 0.5) * step_size;
    for (int y = start_idx[1]; y <= end_idx[1]; y++) {
      pt.y() = (y - 0.5) * step_size;
      for (int z = start_idx[2]; z <= end_idx[2]; z++) {
        pt.z() = (z - 0.5) * step_size;
        res = tree_ptr->search(pt, depth);
          if (res == NULL) {
          Point p(pt.x(), pt.y(), pt.z());
          pc.push_back(p);
        }
      }
    }
  }
  //// end_of unknown leafs

  return make_pointcloud_from_pcl (ctx, pc);
}

pointer OCTOMAP_ADD_POINTS (register context *ctx, int n, pointer *argv) {
  /* octree_ptr points origin (coords) */
  numunion nu;
  pointer pcloud;
  octomap::OcTree *tree_ptr;

  ckarg(3);

  tree_ptr = (octomap::OcTree *)(intval(argv[0]));

  if (!isPointCloud (argv[1])) {
    error(E_TYPEMISMATCH);
  }
  pcloud = argv[1];
  int width = intval (get_from_pointcloud (ctx, pcloud, K_EUSPCL_WIDTH));
  int height = intval (get_from_pointcloud (ctx, pcloud, K_EUSPCL_HEIGHT));
  pointer points = get_from_pointcloud (ctx, pcloud, K_EUSPCL_POINTS);

  PointCloud< Point >::Ptr pcl_cloud =
    make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);

  pointer e_origin = argv[2];
  octomap::point3d origin(e_origin->c.fvec.fv[0] / 1000,
                          e_origin->c.fvec.fv[1] / 1000,
                          e_origin->c.fvec.fv[2] / 1000);

  octomap::Pointcloud pt;
  for (PointCloud< Point >::const_iterator it = pcl_cloud->begin(); it != pcl_cloud->end(); ++it) {
    pt.push_back(it->x, it->y, it->z);
  }

  tree_ptr->insertPointCloud (pt, origin);//
  //// finish insert

  eusinteger_t ret = tree_ptr->calcNumNodes();
  return makeint(ret);
}

pointer OCTOMAP_DUMP_BINARY (register context *ctx, int n, pointer *argv) {
  octomap::OcTree *tree_ptr;
  std::stringstream datastream;

  ckarg(1);

  tree_ptr = (octomap::OcTree *)(intval(argv[0]));

  if (!tree_ptr->writeBinaryData(datastream)) {
    return NIL;
  }

  return makestring((char *)datastream.str().c_str(), datastream.str().length());
}
