#ifndef __EUSPCL_REGISTRATION__
#define __EUSPCL_REGISTRATION__

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

// eus functions
extern pointer PCL_REGISTRATION_RAW (register context *ctx, int n, pointer *argv);

enum EUS_REGIST_TYPE {
  REGIST_SVD = 0,
  REGIST_NL = 1,
  REGIST_GICP = 2,
  REGIST_NDT = 3,
};

#endif
