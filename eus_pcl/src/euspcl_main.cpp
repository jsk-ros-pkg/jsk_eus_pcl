#include "eus_pcl/euspcl.h"
//
#include "eus_pcl/euspcl_common.h"
#include "eus_pcl/euspcl_features.h"
#include "eus_pcl/euspcl_filters.h"
#include "eus_pcl/euspcl_io.h"
#include "eus_pcl/euspcl_octree.h"
#include "eus_pcl/euspcl_range_image.h"
#include "eus_pcl/euspcl_registration.h"
#include "eus_pcl/euspcl_sample_consensus.h"
#include "eus_pcl/euspcl_segmentation.h"
#include "eus_pcl/euspcl_surface.h"
#include "eus_pcl/euspcl_tracking.h"

extern "C" {
  pointer ___eus_pcl(register context *ctx, int n, pointer *argv, pointer env);
  void register_eus_pcl(){
    char modname[] = "___eus_pcl";
    return add_module_initializer(modname, (pointer (*)())___eus_pcl);}
}

using namespace pcl;

pointer K_EUSPCL_INIT, K_EUSPCL_POINTS, K_EUSPCL_COLORS, K_EUSPCL_NORMALS, K_EUSPCL_WIDTH, K_EUSPCL_HEIGHT;
pointer K_EUSPCL_POS, K_EUSPCL_ROT;
pointer EUSPCL_CLS_PTS;

pointer eval_c_string(register context *ctx, const char *strings) {
  pointer p, ret, qstream, qstring;
  int total_length;

  total_length=strlen (strings);
  qstring = makebuffer (total_length);
  vpush (qstring);

  {
    const char *s = strings;
    byte *d =qstring->c.str.chars;
    while (*s) *d++ = *s++;
    *d++= '\n'; /* newline is needed to ensure the reader to stop ignoring comments */
  }

  qstream = (pointer)mkstream (ctx, K_IN, qstring);
  vpush (qstream);

  qstream->c.stream.tail = makeint (total_length);
  while ((p = reader (ctx, qstream, NIL)) != (pointer)(-1)) {
     if (debug) { prinx (ctx,p,STDOUT); terpri(STDOUT);  }
     ret = eval (ctx, p);
  }

  vpop();
  vpop();
  return (ret);
}

pointer make_eus_pointcloud(register context *ctx,
                            pointer pos, pointer col, pointer nom) {
  register pointer *local = ctx->vsp;
  pointer w, name;
  int pc;
  local[0] = eval_c_string (ctx, "user::pointcloud");
  ctx->vsp = local + 1;

  w = (pointer)INSTANTIATE (ctx, 1, local);  /*instantiate*/

  pc = 2;
  local[0] = w; // ??
  local[1] = K_EUSPCL_INIT;

  if (pos != NIL) {
    local[pc++] = K_EUSPCL_POINTS;
    local[pc++] = pos;
  }
  if (col != NIL) {
    local[pc++] = K_EUSPCL_COLORS;
    local[pc++] = col;
  }
  if (nom != NIL) {
    local[pc++] = K_EUSPCL_NORMALS;
    local[pc++] = nom;
  }
  ctx->vsp = local + pc;
  w = (pointer)SEND (ctx, pc, local); /* send :init */

  ctx->vsp = local;
  return (w);
}

pointer make_eus_coordinates (register context *ctx,
                              pointer pos, pointer rot) {
  register pointer *local = ctx->vsp;
  pointer w, name;
  int pc;
  local[0] = eval_c_string (ctx, "user::coordinates");
  ctx->vsp = local + 1;

  w = (pointer)INSTANTIATE (ctx, 1, local);  /*instantiate*/

  pc = 2;
  local[0] = w; // ??
  local[1] = K_EUSPCL_INIT;

  if (pos != NIL) {
    local[pc++] = K_EUSPCL_POS;
    local[pc++] = pos;
  }
  if (rot != NIL) {
    local[pc++] = K_EUSPCL_ROT;
    local[pc++] = rot;
  }

  ctx->vsp = local + pc;
  w = (pointer)SEND (ctx, pc, local); /* send :init */

  ctx->vsp = local;
  return (w);
}

pointer make_pointcloud_from_pcl ( register context *ctx, const Points &pt ) {
  int pc = 0;
  size_t len = pt.points.size();
  pointer pos = NIL;

  pos = makematrix (ctx, len, 3);
  vpush (pos); pc++;
  {
    eusfloat_t *fv = pos->c.ary.entity->c.fvec.fv;
    for (Points::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      *fv++ = 1000 * it->x;
      *fv++ = 1000 * it->y;
      *fv++ = 1000 * it->z;
    }
  }

  pointer retp = make_eus_pointcloud (ctx, pos, NIL, NIL);
  while (pc-- > 0) vpop();

  return retp;
}

pointer make_pointcloud_from_pcl ( register context *ctx, const PointsC &pt ) {
  int pc = 0;
  size_t len = pt.points.size();
  pointer pos = NIL, col = NIL;

  pos = makematrix (ctx, len, 3);
  vpush (pos); pc++;
  {
    eusfloat_t *fv = pos->c.ary.entity->c.fvec.fv;
    for (PointsC::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      *fv++ = 1000 * it->x;
      *fv++ = 1000 * it->y;
      *fv++ = 1000 * it->z;
    }
  }

  col = makematrix (ctx, len, 3);
  vpush (col); pc++;
  {
    eusfloat_t *fv = col->c.ary.entity->c.fvec.fv;
    for (PointsC::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      const unsigned int int_rgb = *reinterpret_cast<const unsigned int *>( &(it->rgb) );

      *fv++ = (( int_rgb & 0x00FF0000 ) >> 16) / 255.0;
      *fv++ = (( int_rgb & 0x0000FF00 ) >> 8 ) / 255.0;
      *fv++ = (( int_rgb & 0x000000FF ) >> 0 ) / 255.0;
    }
  }

  pointer retp = make_eus_pointcloud (ctx, pos, col, NIL);
  while (pc-- > 0) vpop();

  return retp;
}

pointer make_pointcloud_from_pcl ( register context *ctx, const PointsN &pt ) {
  int pc = 0;
  size_t len = pt.points.size();
  pointer pos = NIL, nom = NIL;

  pos = makematrix (ctx, len, 3);
  vpush (pos); pc++;
  {
    eusfloat_t *fv = pos->c.ary.entity->c.fvec.fv;
    for (PointsN::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      *fv++ = 1000 * it->x;
      *fv++ = 1000 * it->y;
      *fv++ = 1000 * it->z;
    }
  }

  nom = makematrix (ctx, len, 3);
  vpush (nom); pc++;
  {
    eusfloat_t *fv = nom->c.ary.entity->c.fvec.fv;
    for (PointsN::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      *fv++ = it->normal_x;
      *fv++ = it->normal_y;
      *fv++ = it->normal_z;
    }
  }

  pointer retp = make_eus_pointcloud (ctx, pos, NIL, nom);
  while (pc-- > 0) vpop();

  return retp;
}

pointer make_pointcloud_from_pcl ( register context *ctx, const PointsCN &pt ) {
  int pc = 0;
  size_t len = pt.points.size();
  pointer pos = NIL, col = NIL, nom = NIL;

  pos = makematrix (ctx, len, 3);
  vpush (pos); pc++;
  {
    eusfloat_t *fv = pos->c.ary.entity->c.fvec.fv;
    for (PointsCN::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      *fv++ = 1000 * it->x;
      *fv++ = 1000 * it->y;
      *fv++ = 1000 * it->z;
    }
  }

  col = makematrix (ctx, len, 3);
  vpush (col); pc++;
  {
    eusfloat_t *fv = col->c.ary.entity->c.fvec.fv;
    for (PointsCN::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      const unsigned int int_rgb = *reinterpret_cast<const unsigned int *>( &(it->rgb) );

      *fv++ = (( int_rgb & 0x00FF0000 ) >> 16) / 255.0;
      *fv++ = (( int_rgb & 0x0000FF00 ) >> 8 ) / 255.0;
      *fv++ = (( int_rgb & 0x000000FF ) >> 0 ) / 255.0;
    }
  }

  nom = makematrix (ctx, len, 3);
  vpush (nom); pc++;
  {
    eusfloat_t *fv = nom->c.ary.entity->c.fvec.fv;
    for (PointsCN::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      *fv++ = it->normal_x;
      *fv++ = it->normal_y;
      *fv++ = it->normal_z;
    }
  }

  pointer retp = make_eus_pointcloud (ctx, pos, col, nom);
  while (pc-- > 0) vpop();

  return retp;
}

pointer make_pointcloud_from_pcl (register context *ctx, const Points &pt,
                                  const Normals &nm) {
  int pc = 0;
  size_t len = pt.points.size();
  pointer pos = NIL, col = NIL, nom = NIL;

  pos = makematrix (ctx, len, 3);
  vpush (pos); pc++;
  {
    eusfloat_t *fv = pos->c.ary.entity->c.fvec.fv;
    for (Points::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      *fv++ = 1000 * it->x;
      *fv++ = 1000 * it->y;
      *fv++ = 1000 * it->z;
    }
  }

  nom = makematrix (ctx, len, 3);
  vpush (nom); pc++;
  {
    eusfloat_t *fv = nom->c.ary.entity->c.fvec.fv;
    for (Normals::const_iterator it = nm.begin();
         it != nm.end(); it++) {
      *fv++ = it->normal_x;
      *fv++ = it->normal_y;
      *fv++ = it->normal_z;
    }
  }

  pointer retp = make_eus_pointcloud (ctx, pos, col, nom);
  while (pc-- > 0) vpop();

  return retp;
}

pointer make_pointcloud_from_pcl (register context *ctx, const PointsC &pt,
                                  const Normals &nm) {
  int pc = 0;
  size_t len = pt.points.size();
  pointer pos = NIL, col = NIL, nom = NIL;

  pos = makematrix (ctx, len, 3);
  vpush (pos); pc++;
  {
    eusfloat_t *fv = pos->c.ary.entity->c.fvec.fv;
    for (PointsC::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      *fv++ = 1000 * it->x;
      *fv++ = 1000 * it->y;
      *fv++ = 1000 * it->z;
    }
  }

  col = makematrix (ctx, len, 3);
  vpush (col); pc++;
  {
    eusfloat_t *fv = col->c.ary.entity->c.fvec.fv;
    for (PointsC::const_iterator it = pt.begin();
         it != pt.end(); it++) {
      const unsigned int int_rgb = *reinterpret_cast<const unsigned int *>( &(it->rgb) );

      *fv++ = (( int_rgb & 0x00FF0000 ) >> 16) / 255.0;
      *fv++ = (( int_rgb & 0x0000FF00 ) >> 8 ) / 255.0;
      *fv++ = (( int_rgb & 0x000000FF ) >> 0 ) / 255.0;
    }
  }

  nom = makematrix (ctx, len, 3);
  vpush (nom); pc++;
  {
    eusfloat_t *fv = nom->c.ary.entity->c.fvec.fv;
    for (Normals::const_iterator it = nm.begin();
         it != nm.end(); it++) {
      *fv++ = it->normal_x;
      *fv++ = it->normal_y;
      *fv++ = it->normal_z;
    }
  }

  pointer retp = make_eus_pointcloud (ctx, pos, col, nom);
  while (pc-- > 0) vpop();

  return retp;
}

#if 0
#define MACRO_TEMPLATE_ (PTYPE, args)
{
  pcl::PointCloud< PTYPE >::Ptr ptr =
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, width, height);

  // process

  //
  ret = make_pointcloud_from_pcl ( ctx, pcl_cloud_filtered );
  vpush (ret); pc++;
}

inline pointer process (register context *ctx, pointer in_cloud) {
  int width = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
  pointer points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);
  pointer colors = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_COLORS);
  pointer normals = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_NORMALS);
  pointer ret = NIL;
  int pc = 0;

  // parse
  if ( points != NIL && colors != NIL && normals != NIL ) {
    MACRO_TEMPLATE_(PointCN, );
  } else if ( points != NIL && colors != NIL ) {
    MACRO_TEMPLATE_(PointC, );
  } else if ( points != NIL && normals != NIL ) {
    MACRO_TEMPLATE_(PointN, );
  } else if ( points != NIL ) {
    MACRO_TEMPLATE_(Point, );
  } else {
    // warning there is no points.
  }

  while (pc-- > 0) vpop();
  return ret;
}
#endif

#define USE_PACKAGE 1
pointer ___eus_pcl(register context *ctx, int n, pointer *argv, pointer env)
{
#ifdef USE_PACKAGE
  // using package
  pointer rospkg, p = Spevalof (PACKAGE);
  rospkg = findpkg (makestring ((char *)"PCL",3));
  if (rospkg == 0) rospkg = makepkg (ctx, makestring((char *)"PCL", 3), NIL, NIL);
  Spevalof (PACKAGE) = rospkg;
#endif
  // euspcl_common.cpp
  defun (ctx, (char *)"PCL-PCA", argv[0], (pointer (*)())PCL_PCA);

  // euspcl_io.cpp
  defun (ctx, (char *)"READ-PCD", argv[0], (pointer (*)())PCL_READ_PCD);
  defun (ctx, (char *)"WRITE-PCD", argv[0], (pointer (*)())PCL_WRITE_PCD);

  // euspcl_filters.cpp
  defun (ctx, (char *)"VOXEL-GRID", argv[0], (pointer (*)())PCL_VOXEL_GRID);

  // euspcl_features.cpp
  defun (ctx, (char *)"ADD_NORMAL", argv[0], (pointer (*)())PCL_ADD_NORMAL);

  // euspcl_sample_consensus.cpp
  defun (ctx, (char *)"SAC-SEGMENTATION", argv[0], (pointer (*)())PCL_SAC_SEGMENTATION);

  // euspcl_segmentation.cpp
  defun (ctx, (char *)"EXTRACT-EUCLIDEAN-CLUSTERS", argv[0],
         (pointer (*)())EXTRACT_EUCLIDEAN_CLUSTERS);

#ifdef USE_PACKAGE
  // reset package
  pointer_update (Spevalof (PACKAGE), p);
#endif
  EUSPCL_CLS_PTS = intern(ctx, (char *)"POINTCLOUD", 10, userpkg);

  K_EUSPCL_INIT = defkeyword (ctx, (char *)"INIT");
  K_EUSPCL_POINTS = defkeyword (ctx, (char *)"POINTS");
  K_EUSPCL_COLORS = defkeyword (ctx, (char *)"COLORS");
  K_EUSPCL_NORMALS = defkeyword (ctx, (char *)"NORMALS");
  K_EUSPCL_WIDTH = defkeyword (ctx, (char *)"WIDTH");
  K_EUSPCL_HEIGHT = defkeyword (ctx, (char *)"HEIGHT");
  K_EUSPCL_POS = defkeyword (ctx, (char *)"POS");
  K_EUSPCL_ROT = defkeyword (ctx, (char *)"ROT");
  return 0;
}
