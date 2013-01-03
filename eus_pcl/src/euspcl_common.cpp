#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_common.h"

#if __PCL_SELECT == 0
using namespace pcl;
#elif __PCL_SELECT == 17
using namespace pcl17;
#endif

pointer PCL_PCA (register context *ctx, int n, pointer *argv) {
  pointer in_cloud;
  ckarg(1);
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
  }
  in_cloud = argv[0];

  int width = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_HEIGHT));
  pointer points = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_POINTS);
  int pc = 0;

  PointCloud< Point >::Ptr ptr =
    make_pcl_pointcloud< Point > (ctx, points, NULL, NULL, NULL, width, height);

  // process
  PCA< Point > pca;
  pca.setInputCloud (ptr);

  //pca.getCoefficients();
  pointer ret = NIL;
  {
    pointer tmp = makevector(C_FLTVECTOR, 3);
    vpush (tmp); pc++;
    // mean
    Eigen::Vector4f mean = pca.getMean();
    //printf("%f %f %f %f\n", mean[0], mean[1], mean[2], mean[3]);
    eusfloat_t *fv = tmp->c.fvec.fv;
    fv[0] = mean[0]; fv[1] = mean[1];  fv[2] = mean[2];
    ret = rawcons (ctx, tmp, ret);
    vpush (ret); pc++;
  }
  {
    pointer tmp = makevector(C_FLTVECTOR, 3);
    vpush (tmp); pc++;
    // eigen value
    Eigen::Vector3f vec = pca.getEigenValues();
    //printf("%f %f %f\n", vec[0], vec[1], vec[2]);
    eusfloat_t *fv = tmp->c.fvec.fv;
    fv[0] = vec[0]; fv[1] = vec[1];  fv[2] = vec[2];
    ret = rawcons (ctx, tmp, ret);
    vpush (ret); pc++;
  }
  {
    pointer tmp = makematrix(ctx, 3, 3);
    vpush (tmp); pc++;
    // eigen vec
    Eigen::Matrix3f mat = pca.getEigenVectors();
    float *data = mat.data();
    //printf("%f %f %f\n%f %f %f\n%f %f %f\n",
    //data[0], data[1], data[2],
    //data[3], data[4], data[5],
    //data[6], data[7], data[8]);
    eusfloat_t *fv = tmp->c.ary.entity->c.fvec.fv;
    fv[0] = data[0]; fv[1] = data[3]; fv[2] = data[6];
    fv[3] = data[1]; fv[4] = data[4]; fv[5] = data[7];
    fv[6] = data[2]; fv[7] = data[5]; fv[8] = data[8];
    ret = rawcons (ctx, tmp, ret);
    vpush (ret); pc++;
  }

  while (pc-- > 0) vpop();
  return ret;
}
