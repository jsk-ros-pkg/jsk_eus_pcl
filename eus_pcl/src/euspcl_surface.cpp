#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_surface.h"

#if __PCL_SELECT == 0
using namespace pcl;
#elif __PCL_SELECT == 17
using namespace pcl17;
#endif

#if 0
int concave_plane(eusfloat_t *src, int ssize,
                  eusfloat_t *coeff, eusfloat_t alpha, eusfloat_t *ret) {

  typedef PointXYZ Point;
  PointCloud<Point>::Ptr cloud_projected (new PointCloud<Point>);
  PointCloud<Point>::Ptr cloud_filtered (new PointCloud<Point>);
  floatvector2pointcloud(src, ssize, 1, *cloud_filtered);

  ModelCoefficients::Ptr coefficients (new ModelCoefficients);
  coefficients->values.resize(4);
  coefficients->values[0] = coeff[0];
  coefficients->values[1] = coeff[1];
  coefficients->values[2] = coeff[2];
  coefficients->values[3] = coeff[3]/1000.0;

  // Project the model inliers
  ProjectInliers<Point> proj;
  proj.setModelType (SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Create a Concave Hull representation of the projected inliers
  PointCloud<Point>::Ptr cloud_hull (new PointCloud<Point>);
  ConcaveHull<Point> chull;

  chull.setInputCloud (cloud_projected);
  chull.setAlpha (alpha);
  chull.reconstruct (*cloud_hull);

  for(int i = 0; i < cloud_hull->points.size(); i++) {
    *ret++ = cloud_hull->points[i].x * 1000.0;
    *ret++ = cloud_hull->points[i].y * 1000.0;
    *ret++ = cloud_hull->points[i].z * 1000.0;
  }

  return cloud_hull->points.size();
}

int convex_plane(eusfloat_t *src, int ssize,
                 eusfloat_t *coeff, eusfloat_t *ret) {

  typedef PointXYZ Point;
  PointCloud<Point>::Ptr cloud_projected (new PointCloud<Point>);
  PointCloud<Point>::Ptr cloud_filtered (new PointCloud<Point>);
  floatvector2pointcloud(src, ssize, 1, *cloud_filtered);

  ModelCoefficients::Ptr coefficients (new ModelCoefficients);
  coefficients->values.resize(4);
  coefficients->values[0] = coeff[0];
  coefficients->values[1] = coeff[1];
  coefficients->values[2] = coeff[2];
  coefficients->values[3] = coeff[3] / 1000.0;

  // Project the model inliers
  ProjectInliers<Point> proj;
  proj.setModelType (SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Create a Concave Hull representation of the projected inliers
  PointCloud<Point>::Ptr cloud_hull (new PointCloud<Point>);
  ConvexHull<Point> chull;

  chull.setInputCloud (cloud_projected);
  //chull.setAlpha (alpha);
  chull.reconstruct (*cloud_hull);

  for(int i = 0; i < cloud_hull->points.size(); i++) {
    *ret++ = cloud_hull->points[i].x * 1000.0;
    *ret++ = cloud_hull->points[i].y * 1000.0;
    *ret++ = cloud_hull->points[i].z * 1000.0;
  }

  return cloud_hull->points.size();
}

int compute_mesh (eusfloat_t *src, eusfloat_t *nm, eusfloat_t *cvt,
                  int ssize, eusfloat_t radius, eusinteger_t *ret) {
  // read points
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  floatvector2pointnormal(src, nm, cvt, ssize, 1, *cloud_with_normals);

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (radius);

  // Set typical values for the parameters
  gp3.setMu (2.5); // ??
  gp3.setMaximumNearestNeighbors (100); // ??
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true); // ??

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree);
  gp3.reconstruct (triangles);

  // return triangles
  // Additional vertex information
  //std::vector<int> parts = gp3.getPartIDs();
  //std::vector<int> states = gp3.getPointStates();
  printf("lsize = %ld\n", triangles.polygons.size());
#if 0
  std::vector<int> sfn = gp3.getSFN();
  std::vector<int> ffn = gp3.getFFN();
  printf("ssize = %ld\n", sfn.size());
  for( size_t i = 0; i < sfn.size(); i++) {
    printf(" %d", sfn[i]);
  }
  printf("fsize = %ld\n", ffn.size());
  for( size_t i = 0; i < sfn.size(); i++) {
    printf(" %d", ffn[i]);
  }
#endif

  size_t lim = triangles.polygons.size();
  if ( lim > (ssize * 3) ) lim = ssize * 3;
  for(size_t i = 0; i < lim; i++) {
    //printf("vsize = %ld\n", triangles.polygons[i].vertices.size());
    *ret++ = triangles.polygons[i].vertices[0];
    *ret++ = triangles.polygons[i].vertices[1];
    *ret++ = triangles.polygons[i].vertices[2];
  }

  return triangles.polygons.size();
}

int compute_lms (eusfloat_t *src, int ssize,
                 eusfloat_t *dst, eusfloat_t *nm, eusfloat_t *cvt, eusfloat_t radius) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  floatvector2pointcloud(src, ssize, 1, *cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  // Output has the same type as the input one, it will be only smoothed
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (radius/1000.0);

#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 6 )
  // Reconstruct
  mls.process (mls_points);
#else
  // mls.reconstruct (mls_points);
#endif

  printf("size = %ld\n", mls_points.points.size());
  for(size_t i = 0; i < mls_points.points.size(); i++) {
    *dst++ = mls_points.points[i].x * 1000.0;
    *dst++ = mls_points.points[i].y * 1000.0;
    *dst++ = mls_points.points[i].z * 1000.0;

    *nm++ = mls_points.points[i].normal[0];
    *nm++ = mls_points.points[i].normal[1];
    *nm++ = mls_points.points[i].normal[2];

    *cvt++ = mls_points.points[i].curvature;
  }

  return mls_points.points.size();
}
#endif

pointer PCL_CONVEX_HULL (register context *ctx, int n, pointer *argv) {
  pointer in_cloud = argv[0];

  int width = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_HEIGHT));
  pointer points = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_POINTS);

  PointCloud< Point >::Ptr ptr =
    make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);

  PointCloud< Point >::Ptr cloud_hull (new PointCloud<Point>);
  ConvexHull< Point > chull;

  chull.setInputCloud (ptr);
  chull.reconstruct (*cloud_hull);

  return make_pointcloud_from_pcl (ctx, *cloud_hull);
}

pointer PCL_CONVEX_HULL_PLANE
(register context *ctx, int n, pointer *argv) {

}

pointer PCL_CONCAVE_HULL (register context *ctx, int n, pointer *argv) { }

pointer PCL_CONCAVE_HULL_PLANE (register context *ctx, int n, pointer *argv) { }
