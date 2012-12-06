#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_registration.h"

pointer PCL_ICP_RAW (register context *ctx, int n, pointer *argv) {
  /* ( source_pointcloud target_pointcloud ) */
  pointer A_cloud, B_cloud;

  ckarg(2);//
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
  }
  A_cloud = argv[0];
  if (!isPointCloud (argv[1])) {
    error(E_TYPEMISMATCH);
  }
  B_cloud = argv[1];

  int a_width = intval (get_from_pointcloud (ctx, A_cloud, K_EUSPCL_WIDTH));
  int a_height = intval (get_from_pointcloud (ctx, A_cloud, K_EUSPCL_HEIGHT));
  pointer a_points = get_from_pointcloud (ctx, A_cloud, K_EUSPCL_POINTS);

  int b_width = intval (get_from_pointcloud (ctx, B_cloud, K_EUSPCL_WIDTH));
  int b_height = intval (get_from_pointcloud (ctx, B_cloud, K_EUSPCL_HEIGHT));
  pointer b_points = get_from_pointcloud (ctx, B_cloud, K_EUSPCL_POINTS);

  pointer ret = NIL;
  int pc = 0;

  Points::Ptr a_ptr =
    make_pcl_pointcloud< Point > (ctx, a_points, NULL, NULL, a_width, a_height);
  Points::Ptr b_ptr =
    make_pcl_pointcloud< Point > (ctx, b_points, NULL, NULL, b_width, b_height);

  pcl::Registration< Point, Point, float >::Ptr icp;

  icp.reset (new pcl::IterativeClosestPoint< Point, Point > ());
  //icp.reset (new pcl::IterativeClosestPointNonLinear< Point, Point >());
  //icp.reset (new pcl::GeneralizedIterativeClosestPoint< Point, Point >());

  icp->setInputCloud (a_ptr);
  icp->setInputTarget (b_ptr);

  Points Final;
  icp->align (Final);

  //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  //icp.getFitnessScore() << std::endl;

  Eigen::Matrix4f tma = icp->getFinalTransformation ();
  pointer pos, rot;
  pos = makefvector (3);
  vpush (pos); pc++;
  //std::cout << tma << std::endl;
  pos->c.fvec.fv[0] = tma (0, 3);
  pos->c.fvec.fv[1] = tma (1, 3);
  pos->c.fvec.fv[2] = tma (2, 3);

  rot = makematrix (ctx, 3, 3);
  vpush (rot); pc++;
  //Eigen::Quaternion<float> q (tma.block<3,3>(0,0));
  {
    eusfloat_t *fv = rot->c.ary.entity->c.fvec.fv;
    fv[0] = tma (0, 0); fv[1] = tma (0, 1); fv[2] = tma (0, 2);
    fv[3] = tma (1, 0); fv[4] = tma (1, 1); fv[5] = tma (1, 2);
    fv[6] = tma (2, 0); fv[7] = tma (2, 1); fv[8] = tma (2, 2);
  }
  ret = make_eus_coordinates (ctx, pos, rot);

  while (pc-- > 0) vpop();
  return ret;
}
