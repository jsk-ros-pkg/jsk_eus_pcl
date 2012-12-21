#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_recognition.h"

// applications
pointer PCL_ISM_TRAINING (register context *ctx, int n, pointer *argv) {
  //
  

  unsigned int number_of_training_clouds = 1;

  pcl::NormalEstimation< Point, PNormal > normal_estimator;
  normal_estimator.setRadiusSearch (25.0);

  std::vector< Points::Ptr >  training_clouds;
  std::vector< Normals::Ptr > training_normals;
  std::vector<unsigned int>   training_classes;

  pcl::FPFHEstimation< Point, PNormal, pcl::Histogram<153> >::Ptr fpfh
    (new pcl::FPFHEstimation < Point, PNormal, pcl::Histogram<153> >);
  fpfh->setRadiusSearch (30.0);
  pcl::Feature< Point, pcl::Histogram<153> >::Ptr feature_estimator (fpfh);

  pcl::ism::ImplicitShapeModelEstimation<153, Point, PNormal> ism;
  ism.setFeatureEstimator (feature_estimator);
  ism.setTrainingClouds  (training_clouds);
  ism.setTrainingNormals (training_normals);
  ism.setTrainingClasses (training_classes);
  ism.setSamplingSize (2.0f);

  pcl::ism::ImplicitShapeModelEstimation<153, Point, PNormal>::ISMModelPtr model
    = boost::shared_ptr<pcl::features::ISMModel> (new pcl::features::ISMModel);

  ism.trainISM (model);

  std::string file ("trained_ism_model.txt"); // filename
  model->saveModelToFile (file);

  return NIL;
}

pointer PCL_ISM_DETECTION (register context *ctx, int n, pointer *argv) {
  pointer eus_in_cloud;
  pointer ret = NIL;
  int pc = 0;
  numunion nu;
  std::string fname;

  fname.assign ((const char *)argv[0]->c.str.chars);
  unsigned int testing_class = intval (argv[1]);

  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
  }
  eus_in_cloud = argv[0];

  int width = intval (get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_WIDTH));
  int height = intval (get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_HEIGHT));
  pointer points = get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_POINTS);
  //pointer colors = get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_COLORS);
  //pointer normals = get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_NORMALS);
  //pointer curvatures = get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_CURVATURES);
  Points::Ptr testing_cloud =
    make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);

  pcl::ism::ImplicitShapeModelEstimation<153, Point, PNormal>::ISMModelPtr model
    = boost::shared_ptr<pcl::features::ISMModel> (new pcl::features::ISMModel);

  model->loadModelFromfile (fname);

  pcl::NormalEstimation< Point, PNormal > normal_estimator;
  normal_estimator.setRadiusSearch (25.0);

  Normals::Ptr testing_normals
    = (new Normals)->makeShared ();
  normal_estimator.setInputCloud (testing_cloud);
  normal_estimator.compute (*testing_normals);

  pcl::ism::ImplicitShapeModelEstimation<153, Point, PNormal> ism;
  boost::shared_ptr< pcl::features::ISMVoteList< Point > > vote_list =
    ism.findObjects (model, testing_cloud, testing_normals, testing_class);

  double radius = model->sigmas_[testing_class] * 10.0;
  double sigma = model->sigmas_[testing_class];

  std::vector< pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak> > strongest_peaks;
  vote_list->findStrongestPeaks (strongest_peaks, testing_class, radius, sigma);

#if 0
  PointsC::Ptr colored_cloud = (new PointsC)->makeShared ();
  colored_cloud->height = 0;
  colored_cloud->width = 1;

  pcl::PointXYZRGB point;
  point.r = 255;
  point.g = 255;
  point.b = 255;

  for (size_t i_point = 0;
       i_point < testing_cloud->points.size ();
       i_point++)
    {
      point.x = testing_cloud->points[i_point].x;
      point.y = testing_cloud->points[i_point].y;
      point.z = testing_cloud->points[i_point].z;
      colored_cloud->points.push_back (point);
    }
  colored_cloud->height += testing_cloud->points.size ();

  point.r = 255;
  point.g = 0;
  point.b = 0;
  for (size_t i_vote = 0;
       i_vote < strongest_peaks.size ();
       i_vote++)
    {
      point.x = strongest_peaks[i_vote].x;
      point.y = strongest_peaks[i_vote].y;
      point.z = strongest_peaks[i_vote].z;
      colored_cloud->points.push_back (point);
    }
  colored_cloud->height += strongest_peaks.size ();
#endif
  return NIL;
}


