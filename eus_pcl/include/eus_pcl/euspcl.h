#ifndef __EUSPCL_H__
#define __EUSPCL_H__

//
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <setjmp.h>
#include <errno.h>

#include <list>
#include <vector>
#include <set>
#include <string>
#include <map>
#include <sstream>
#include <cstdio>

#ifndef __PCL_SELECT
#define __PCL_SELECT 0 // 0: pcl_trunk / 17: perception_pcl_unstable
#endif

// For PCL
#if __PCL_SELECT == 0
#define __PCL_NS pcl
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#elif __PCL_SELECT == 17
#define __PCL_NS pcl17
#include <pcl17/pcl_base.h>
#include <pcl17/point_cloud.h>
#include <pcl17/point_types.h>
#include <pcl17/point_representation.h>
#endif

#include <boost/make_shared.hpp>

// euspcl_io.h
//#include <pcl/io/pcd_io.h>
// euspcl_filters.h
//#include <pcl/filters/voxel_grid.h>
// euspcl_segmentation.h
//#include <pcl/segmentation/extract_clusters.h>

// for eus.h
#define class    eus_class
#define throw    eus_throw
#define export   eus_export
#define vector   eus_vector
#define string   eus_string
#define iostream eus_iostream
#define complex  eus_complex

#include "eus.h"

#undef class
#undef throw
#undef export
#undef vector
#undef string
#undef iostream
#undef complex

typedef __PCL_NS::PointXYZ    Point;
typedef __PCL_NS::Normal      PNormal;
typedef __PCL_NS::PointXYZRGB PointC;
typedef __PCL_NS::PointNormal PointN;
typedef __PCL_NS::PointXYZRGBNormal PointCN;

typedef __PCL_NS::PointCloud< Point >   Points;
typedef __PCL_NS::PointCloud< PNormal > Normals;
typedef __PCL_NS::PointCloud< PointN >  PointsN;
typedef __PCL_NS::PointCloud< PointC >  PointsC;
typedef __PCL_NS::PointCloud< PointCN > PointsCN;

namespace __PCL_NS {
  typedef std::vector< int > Indices;
}

extern pointer K_EUSPCL_INIT, K_EUSPCL_POINTS, K_EUSPCL_COLORS, K_EUSPCL_NORMALS, K_EUSPCL_CURVATURES;
extern pointer K_EUSPCL_WIDTH, K_EUSPCL_HEIGHT, K_EUSPCL_SIZE_CHANGE;
extern pointer K_EUSPCL_POS, K_EUSPCL_ROT;
extern pointer EUSPCL_CLS_PTS;

extern pointer eval_c_string (register context *ctx, const char *strings);
extern pointer make_eus_pointcloud (register context *ctx, pointer pos, pointer col,
                                    pointer nom, pointer cuv,
                                    int width = 0, int height = 0);
extern pointer make_eus_pointcloud (register context *ctx, pointer pcloud,
                                    pointer pos, pointer col, pointer nom, pointer cuv,
                                    int width = 0, int height = 0);

extern pointer make_eus_coordinates (register context *ctx, pointer pos, pointer rot);

extern pointer make_pointcloud_from_pcl (register context *ctx, const Points &pt, pointer pcloud = NULL);
extern pointer make_pointcloud_from_pcl (register context *ctx, const PointsC &pt, pointer pcloud = NULL);
extern pointer make_pointcloud_from_pcl (register context *ctx, const PointsN &pt, pointer pcloud = NULL);
extern pointer make_pointcloud_from_pcl (register context *ctx, const PointsCN &pt, pointer pcloud = NULL);
extern pointer make_pointcloud_from_pcl (register context *ctx, const Points &pt,
                                         const Normals &nm, pointer pcloud = NULL);
extern pointer make_pointcloud_from_pcl (register context *ctx, const PointsC &pt,
                                         const Normals &nm, pointer pcloud = NULL);

static inline void fvector2pcl_pointcloud(eusfloat_t *src, eusfloat_t *rgb, eusfloat_t *nm,
                                          eusfloat_t *cvt, int width, int height, Points &pt) {
  pt.width    = width;
  pt.height   = height;
  pt.is_dense = false;
  pt.points.resize (width*height);

  if ( src != NULL ) {
    for (size_t i=0; i < pt.points.size(); i++) {
      pt.points[i].x = (*src++)/1000.0;
      pt.points[i].y = (*src++)/1000.0;
      pt.points[i].z = (*src++)/1000.0;
    }
  }
}

static inline void fvector2pcl_pointcloud(eusfloat_t *src, eusfloat_t *rgb, eusfloat_t *nm,
                                          eusfloat_t *cvt, int width, int height, Normals &pt) {
  pt.width    = width;
  pt.height   = height;
  pt.is_dense = false;
  pt.points.resize (width*height);

  if ( nm != NULL ) {
    for (size_t i=0; i < pt.points.size(); i++) {
      pt.points[i].normal[0] = *src++;
      pt.points[i].normal[1] = *src++;
      pt.points[i].normal[2] = *src++;
      if (cvt != NULL) pt.points[i].curvature = *cvt++;
    }
  }
}

static inline void fvector2pcl_pointcloud(eusfloat_t *src, eusfloat_t *rgb, eusfloat_t *nm,
                                          eusfloat_t *cvt, int width, int height, PointsN &pt) {
  pt.width    = width;
  pt.height   = height;
  pt.is_dense = false;
  pt.points.resize (width*height);

  for (size_t i=0; i < pt.points.size(); i++) {
    if ( src != NULL ) {
      pt.points[i].x = (*src++)/1000.0;
      pt.points[i].y = (*src++)/1000.0;
      pt.points[i].z = (*src++)/1000.0;
    }

    if ( nm != NULL ) {
      pt.points[i].normal[0] = *nm++;
      pt.points[i].normal[1] = *nm++;
      pt.points[i].normal[2] = *nm++;
      if (cvt != NULL) pt.points[i].curvature = *cvt++;
    }
  }
}

static inline void fvector2pcl_pointcloud(eusfloat_t *src, eusfloat_t *rgb, eusfloat_t *nm,
                                          eusfloat_t *cvt, int width, int height, PointsC &pt) {
  pt.width    = width;
  pt.height   = height;
  pt.is_dense = false;
  pt.points.resize (width*height);

  unsigned char r, g, b;
  unsigned int int_rgb;

  for (size_t i=0; i < pt.points.size(); i++) {
    if ( src != NULL ) {
      pt.points[i].x = (*src++)/1000.0;
      pt.points[i].y = (*src++)/1000.0;
      pt.points[i].z = (*src++)/1000.0;
    }
    if ( rgb != NULL ) {
      r = round(255.0 * (*rgb++));
      g = round(255.0 * (*rgb++));
      b = round(255.0 * (*rgb++));
      int_rgb = (r << 16) | (g << 8) | b;
      pt.points[i].rgb = *(float *)(&int_rgb);
    }
  }
}
static inline void fvector2pcl_pointcloud(eusfloat_t *src, eusfloat_t *rgb, eusfloat_t *nm,
                                          eusfloat_t *cvt, int width, int height, PointsCN &pt) {
  pt.width    = width;
  pt.height   = height;
  pt.is_dense = false;
  pt.points.resize (width*height);

  unsigned char r, g, b;
  unsigned int int_rgb;

  for (size_t i=0; i < pt.points.size(); i++) {
    if ( src != NULL ) {
      pt.points[i].x = (*src++)/1000.0;
      pt.points[i].y = (*src++)/1000.0;
      pt.points[i].z = (*src++)/1000.0;
    }

    if ( rgb != NULL ) {
      r = round(255.0 * (*rgb++));
      g = round(255.0 * (*rgb++));
      b = round(255.0 * (*rgb++));
      int_rgb = (r << 16) | (g << 8) | b;
      pt.points[i].rgb = *(float *)(&int_rgb);
    }

    if ( nm != NULL ) {
      pt.points[i].normal[0] = *nm++;
      pt.points[i].normal[1] = *nm++;
      pt.points[i].normal[2] = *nm++;
      if (cvt != NULL) pt.points[i].curvature = *cvt++;
    }
  }
}

template < typename PTS >
inline typename __PCL_NS::PointCloud<PTS>::Ptr
make_pcl_pointcloud (register context *ctx,
                     pointer points, pointer colors, pointer normals,
                     pointer curvatures, int width, int height) {

  typename __PCL_NS::PointCloud< PTS >::Ptr pcl_cloud ( new  __PCL_NS::PointCloud< PTS > );

  fvector2pcl_pointcloud(points == NULL ? NULL :
                         (points == NIL ? NULL : points->c.ary.entity->c.fvec.fv),
                         colors == NULL ? NULL :
                         (colors == NIL ? NULL : colors->c.ary.entity->c.fvec.fv),
                         normals == NULL ? NULL :
                         (normals == NIL ? NULL : normals->c.ary.entity->c.fvec.fv),
                         curvatures == NULL ? NULL :
                         (curvatures == NIL ? NULL : curvatures->c.fvec.fv),
                         width, height, *pcl_cloud);

  return pcl_cloud;
}

inline pointer get_from_pointcloud(register context *ctx,
                                   pointer pointcloud,
                                   pointer key) {
  register pointer *local = ctx->vsp, w;

  local[0] = pointcloud;
  local[1] = key;
  ctx->vsp = local + 2;
  w = (pointer)SEND(ctx, 2, local);
  ctx->vsp = local;

  return w;
}

inline pointer set_to_pointcloud(register context *ctx,
                                 pointer pointcloud,
                                 pointer key, pointer obj) {
  register pointer *local = ctx->vsp, w;

  local[0] = pointcloud;
  local[1] = key;
  local[2] = obj;
  ctx->vsp = local + 3;
  w = (pointer)SEND(ctx, 3, local);
  ctx->vsp = local;

  return w;
}

inline pointer set_to_pointcloud(register context *ctx,
                                 pointer pointcloud,
                                 pointer key, pointer obj0, pointer obj1) {
  register pointer *local = ctx->vsp, w;

  local[0] = pointcloud;
  local[1] = key;
  local[2] = obj0;
  local[3] = obj1;
  ctx->vsp = local + 4;
  w = (pointer)SEND(ctx, 4, local);
  ctx->vsp = local;

  return w;
}

inline bool isPointCloud (pointer p) {
  if (!ispointer(p)) return false;
  if (classof(p)->c.cls.name == EUSPCL_CLS_PTS)
    return true;
  else
    return false;
}

inline pointer convert_eigenmatrix_to_coordinates (register context *ctx, Eigen::Matrix4f &tma) {
  pointer pos, rot;
  int pc = 0;
  pos = makefvector (3);
  vpush (pos); pc++;
  pos->c.fvec.fv[0] = tma (0, 3);
  pos->c.fvec.fv[1] = tma (1, 3);
  pos->c.fvec.fv[2] = tma (2, 3);

  rot = makematrix (ctx, 3, 3);
  vpush (rot); pc++;
  {
    eusfloat_t *fv = rot->c.ary.entity->c.fvec.fv;
    fv[0] = tma (0, 0); fv[1] = tma (0, 1); fv[2] = tma (0, 2);
    fv[3] = tma (1, 0); fv[4] = tma (1, 1); fv[5] = tma (1, 2);
    fv[6] = tma (2, 0); fv[7] = tma (2, 1); fv[8] = tma (2, 2);
  }

  pointer ret = make_eus_coordinates (ctx, pos, rot);
  while (pc-- > 0) vpop();

  return ret;
}

inline Eigen::Matrix4f convert_coordinates_to_eigenmatrix (register context *ctx, pointer coords) {
  pointer pos = get_from_pointcloud(ctx, coords, K_EUSPCL_POS);
  pointer rot = get_from_pointcloud(ctx, coords, K_EUSPCL_ROT);

  Eigen::Matrix4f tma;

  tma (0, 3) = pos->c.fvec.fv[0];
  tma (1, 3) = pos->c.fvec.fv[1];
  tma (2, 3) = pos->c.fvec.fv[2];

  {
    eusfloat_t *fv = rot->c.ary.entity->c.fvec.fv;
    tma (0, 0) = fv[0]; tma (0, 1) = fv[1]; tma (0, 2) = fv[2];
    tma (1, 0) = fv[3]; tma (1, 1) = fv[4]; tma (1, 2) = fv[5];
    tma (2, 0) = fv[6]; tma (2, 1) = fv[7]; tma (2, 2) = fv[8];
  }

  tma(3, 0) = 0.0; tma(3, 1) = 0.0; tma(3, 2) = 0.0; tma(3, 3) = 1.0;

  return tma;
}

#include "euspcl_pcl_utils.h"

#endif
