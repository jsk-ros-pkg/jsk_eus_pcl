#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_registration.h"

using namespace pcl;

pointer PCL_REGISTRATION_RAW (register context *ctx, int n, pointer *argv) {
  /* ( source_pointcloud target_pointcloud &optional (icp_type) (guess_coords)) */
  pointer A_cloud, B_cloud;
  EUS_REGIST_TYPE icp_type = REGIST_SVD;
  // TODO: parameter should be set
  Eigen::Matrix4f guess_mat;
  bool use_guess = false;
  ckarg2(2, 4);//
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
  }
  A_cloud = argv[0];
  if (!isPointCloud (argv[1])) {
    error(E_TYPEMISMATCH);
  }
  B_cloud = argv[1];

  if (n > 2) {
    icp_type = EUS_REGIST_TYPE (intval(argv[2]));
  }
  if (n > 3) {
    use_guess = true;
    guess_mat = convert_coordinates_to_eigenmatrix(ctx, argv[3]);
  }

  int a_width = intval (get_from_pointcloud (ctx, A_cloud, K_EUSPCL_WIDTH));
  int a_height = intval (get_from_pointcloud (ctx, A_cloud, K_EUSPCL_HEIGHT));
  pointer a_points = get_from_pointcloud (ctx, A_cloud, K_EUSPCL_POINTS);

  int b_width = intval (get_from_pointcloud (ctx, B_cloud, K_EUSPCL_WIDTH));
  int b_height = intval (get_from_pointcloud (ctx, B_cloud, K_EUSPCL_HEIGHT));
  pointer b_points = get_from_pointcloud (ctx, B_cloud, K_EUSPCL_POINTS);

  pointer ret = NIL;
  int pc = 0;

  Points::Ptr a_ptr =
    make_pcl_pointcloud< Point > (ctx, a_points, NULL, NULL, NULL, a_width, a_height);
  Points::Ptr b_ptr =
    make_pcl_pointcloud< Point > (ctx, b_points, NULL, NULL, NULL, b_width, b_height);

  Registration< Point, Point, float >::Ptr icp;

  switch (icp_type) {
  case REGIST_SVD:
    icp.reset (new IterativeClosestPoint< Point, Point > ());
    break;
  case REGIST_NL:
    icp.reset (new IterativeClosestPointNonLinear< Point, Point >());
    break;
  case REGIST_GICP:
    icp.reset (new GeneralizedIterativeClosestPoint< Point, Point >());
    break;
  case REGIST_NDT:
    icp.reset (new NormalDistributionsTransform< Point, Point >());
    break;
  default:
    // warning
    break;
  }

  icp->setInputCloud (a_ptr);
  icp->setInputTarget (b_ptr);

  Points Final;
  if (use_guess) {
    icp->align (Final, guess_mat);
  } else {
    icp->align (Final);
  }

  //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  //icp.getFitnessScore() << std::endl;
  Eigen::Matrix4f emat (icp->getFinalTransformation ());
  ret = convert_eigenmatrix_to_coordinates (ctx, emat);

  while (pc-- > 0) vpop();
  return ret;
}
