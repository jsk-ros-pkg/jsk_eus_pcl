#ifndef __EUSPCL_REGISTRATION__
#define __EUSPCL_REGISTRATION__

#if __PCL_SELECT == 0
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#elif __PCL_SELECT == 17
#include <pcl17/registration/icp.h>
#include <pcl17/registration/icp_nl.h>
#include <pcl17/registration/gicp.h>
#include <pcl17/registration/ndt.h>
#endif

// eus functions
extern pointer PCL_REGISTRATION_RAW (register context *ctx, int n, pointer *argv);

enum EUS_REGIST_TYPE {
  REGIST_SVD = 0,
  REGIST_NL = 1,
  REGIST_GICP = 2,
  REGIST_NDT = 3,
};

#endif
