#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_registration.h"

#if __PCL_SELECT == 0
using namespace pcl;
#elif __PCL_SELECT == 17
using namespace pcl17;
#endif

pointer PCL_REGISTRATION_RAW (register context *ctx, int n, pointer *argv) {
  /* ( source_pointcloud target_pointcloud &optional (icp_type) (guess_coords)) */
  numunion nu;
  pointer A_cloud, B_cloud;
  EUS_REGIST_TYPE icp_type = REGIST_SVD;

  Eigen::Matrix4f guess_mat;
  bool use_guess = false;

  if (n < 2) error(E_MISMATCHARG);

  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
  }
  A_cloud = argv[0];
  if (!isPointCloud (argv[1])) {
    error(E_TYPEMISMATCH);
  }
  B_cloud = argv[1];

  if (n > 2) {
    if (argv[2] != NIL) icp_type = EUS_REGIST_TYPE (intval(argv[2]));
  }
  if (n > 3) {
    if (argv[3] != NIL) {
      use_guess = true;
      guess_mat = convert_coordinates_to_eigenmatrix(ctx, argv[3]);
    }
  }
  // extra arguments
  int    _RANSACIterations               = 0;
  double _RANSACOutlierRejectionThreshold= 0.05;
  int    _MaximumIterations              = 10;
  double _EuclideanFitnessEpsilon        = -std::numeric_limits<double>::max ();
  double _TransformationEpsilon          = 0;
  double _MaxCorrespondenceDistance      = (std::sqrt (std::numeric_limits<double>::max ()));
  // gicp
  double _RotationEpsilon            = 0.002;
  int    _CorrespondenceRandomness   = 20;
  int    _MaximumOptimizerIterations = 20;

  if (n > 4 && argv[4] != NIL) {
    _RANSACIterations = intval(argv[4]);
  }
  if (n > 5 && argv[5] != NIL) {
    _RANSACOutlierRejectionThreshold = fltval(argv[5]);
  }
  if (n > 6 && argv[6] != NIL) {
    _MaximumIterations = intval(argv[6]);
  }
  if (n > 7 && argv[7] != NIL) {
    _EuclideanFitnessEpsilon = fltval(argv[7]);
  }
  if (n > 8 && argv[8] != NIL) {
    _TransformationEpsilon = fltval(argv[8]);
  }
  if (n > 9 && argv[9] != NIL) {
    _MaxCorrespondenceDistance = fltval(argv[9]);
  }
  //
  if (n > 10 && argv[10] != NIL) {
    _RotationEpsilon = fltval(argv[10]);
  }
  if (n > 11 && argv[11] != NIL) {
    _CorrespondenceRandomness = intval(argv[11]);
  }
  if (n > 12 && argv[12] != NIL) {
    _MaximumOptimizerIterations = intval(argv[12]);
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
    icp->setRANSACIterations (_RANSACIterations);
    icp->setRANSACOutlierRejectionThreshold (_RANSACOutlierRejectionThreshold);
    icp->setMaximumIterations (_MaximumIterations);
    icp->setEuclideanFitnessEpsilon (_EuclideanFitnessEpsilon);
    icp->setTransformationEpsilon (_TransformationEpsilon);
    icp->setMaxCorrespondenceDistance (_MaxCorrespondenceDistance);
#if DEBUG
    std::cout << "RANSACIterations               " << icp->getRANSACIterations () << std::endl;
    std::cout << "RANSACOutlierRejectionThreshold" << icp->getRANSACOutlierRejectionThreshold () << std::endl;
    std::cout << "MaximumIterations              " << icp->getMaximumIterations () << std::endl;
    std::cout << "EuclideanFitnessEpsilon        " << icp->getEuclideanFitnessEpsilon () << std::endl;
    std::cout << "TransformationEpsilon          " << icp->getTransformationEpsilon () << std::endl;
    std::cout << "MaxCorrespondenceDistance      " << icp->getMaxCorrespondenceDistance () << std::endl;
#endif
    break;
  case REGIST_GICP:
    GeneralizedIterativeClosestPoint< Point, Point > *gicp;
    gicp = new GeneralizedIterativeClosestPoint< Point, Point >();
    gicp->setRotationEpsilon (_RotationEpsilon);
    gicp->setCorrespondenceRandomness (_CorrespondenceRandomness);
    gicp->setMaximumOptimizerIterations (_MaximumOptimizerIterations);
#if DEBUG
    std::cout << "getRotationEpsilon        " << gicp->getRotationEpsilon () << std::endl;
    std::cout << "CorrespondenceRandomness  " << gicp->getCorrespondenceRandomness () << std::endl;
    std::cout << "MaximumOptimizerIterations" << gicp->getMaximumOptimizerIterations () << std::endl;
#endif
    icp.reset (gicp);
    icp->setRANSACIterations (_RANSACIterations);
    icp->setRANSACOutlierRejectionThreshold (_RANSACOutlierRejectionThreshold);
    icp->setMaximumIterations (_MaximumIterations);
    icp->setEuclideanFitnessEpsilon (_EuclideanFitnessEpsilon);
    icp->setTransformationEpsilon (_TransformationEpsilon);
    icp->setMaxCorrespondenceDistance (_MaxCorrespondenceDistance);
    break;
  case REGIST_NDT:
    icp.reset (new NormalDistributionsTransform< Point, Point >());
    break;
  default:
    // warning
    break;
  }

  icp->setInputSource (a_ptr);
  icp->setInputTarget (b_ptr);

  Points Final;
  if (use_guess) {
    icp->align (Final, guess_mat);
  } else {
    icp->align (Final);
  }

  Eigen::Matrix4f emat (icp->getFinalTransformation ());
  ret = convert_eigenmatrix_to_coordinates (ctx, emat);

  { // add fitness score to coordinates
    pointer lst;
    vpush (lst); pc++;
    lst = rawcons (ctx, icp->hasConverged()?T:NIL, NIL);
    eusfloat_t sc = icp->getFitnessScore();
    lst = rawcons (ctx, makeflt(sc), lst);
    vpush (lst); pc++;
    set_property (ctx, ret, lst, K_EUSPCL_RESULT);
  }

  while (pc-- > 0) vpop();
  return ret;
}
