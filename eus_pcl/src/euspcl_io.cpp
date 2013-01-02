#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_io.h"

using namespace pcl;

#define READ_PCD_(PTYPE) \
  PointCloud < PTYPE > pt;                \
  int res = rd.read< PTYPE > (fname, pt); \
  if ( res != 0 ) {                       \
    return NIL;                           \
  }                                        \
  ret = make_pointcloud_from_pcl(ctx, pt); \
  vpush(ret); pc++;

pointer PCL_READ_PCD (register context *ctx, int n, pointer *argv) {
  /* filename */
  PCDReader rd;
  pointer ret = NIL;
  int pc = 0;
  bool have_rgb=false, have_normal=false, have_position=false;
  if (!isstring(argv[0])) {
    error(E_NOSTRING);
  }

  std::string fname;
  fname.assign ((char *)get_string (argv[0]));

  {
    sensor_msgs::PointCloud2 hdr;
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 6 )
    int ret = rd.readHeader(fname, hdr);
#else
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int pcd_version; int data_type; int data_index;
    int ret = rd.readHeader (fname, hdr, origin, orientation,
                             pcd_version, data_type, data_index);
#endif
    if (ret != 0) {
      std::cerr << ";; read header error" << std::endl;
    } else {
      for (size_t i = 0; i < hdr.fields.size(); i++) {
        if (hdr.fields[i].name == "rgb") {
          have_rgb = true;
        }
        if (hdr.fields[i].name == "normal_x") {
          have_normal = true;
        }
        if (hdr.fields[i].name == "x") {
          have_position = true;
        }
      }
    }
  }

  if (have_position && have_rgb && have_normal) {
    READ_PCD_(PointCN);
  } else if (have_position && have_rgb) {
    READ_PCD_(PointC);
  } else if (have_position && have_normal) {
    READ_PCD_(PointN);
  } else if (have_position) {
    READ_PCD_(Point);
  } else {
    // no points
  }

  while (pc-- > 0) vpop();
  return ret;
}

#define WRITE_PCD_(PTYPE, fname)                                        \
  PointCloud< PTYPE >::Ptr pcl_cloud =                                  \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, curvatures, width, height); \
  PCDWriter wt;                                                         \
  wt.write < PTYPE > (fname, *pcl_cloud, binary);

pointer PCL_WRITE_PCD (register context *ctx, int n, pointer *argv) {
  /* pointcloud fname &optional (binary t) */
  bool binary = true;
  pointer in_cloud;
  pointer points, colors, normals, curvatures;
  numunion nu;
  int pc = 0;
  std::string fname;

  ckarg2(2, 3);

  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
  }
  in_cloud = argv[0];

  if (!isstring(argv[1])) error(E_NOSTRING);

  fname.assign ((char *)(argv[1]->c.str.chars));

  if (n > 2) {
    if (argv[2] == NIL) binary = false;
  }

  int width = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval (get_from_pointcloud (ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_POINTS);
  colors = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_COLORS);
  normals = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_NORMALS);
  curvatures = get_from_pointcloud (ctx, in_cloud, K_EUSPCL_CURVATURES);

  if (points != NIL && colors != NIL && normals != NIL) {
    WRITE_PCD_(PointCN, fname);
  } else if (points != NIL && colors != NIL) {
    WRITE_PCD_(PointC, fname);
  } else if (points != NIL && normals != NIL) {
    WRITE_PCD_(PointN, fname);
  } else if (points != NIL) {
    WRITE_PCD_(Point, fname);
  } else {
    // warning there is no points.
  }

  while (pc-- > 0) vpop();
  return T;
}

#define STEP_POINTCLOUD_(PTYPE)                                         \
  PointCloud< PTYPE >::Ptr ptr =                                        \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, curvatures, width, height); \
  PointCloud< PTYPE > ret_cloud;                                        \
  stepPointCloud< PTYPE > (*ptr, ret_cloud, remove_nan, keep_organized, \
                           x_offset, y_offset, x_step, y_step);         \
  if (create) {                                                         \
    ret = make_pointcloud_from_pcl (ctx, ret_cloud);                    \
    vpush(ret); pc++;                                                   \
  } else {                                                              \
    ret = make_pointcloud_from_pcl (ctx, ret_cloud, eus_in_cloud);      \
    vpush (ret); pc++;                                                  \
  }

pointer PCL_STEP_POINTCLOUD (register context *ctx, int n, pointer *argv) {
  /* (pointcloud &optional (create) (remove_nan) (keep_orgatized t) (x_step 2) (y_step 2)
                 (x_offset 0) (y_offset 0)) */
  pointer eus_in_cloud;
  pointer ret = NIL;
  int pc = 0;
  numunion nu;
  bool create = false;
  bool remove_nan = false, keep_organized = true;
  eusinteger_t x_offset = 0, y_offset = 0;
  eusinteger_t x_step = 2, y_step = 2;

  std::cerr << "n = " << n << std::endl;
  if (!isPointCloud (argv[0])) {
    error(E_TYPEMISMATCH);
  }
  eus_in_cloud = argv[0];

  if (n > 1) {
    if (argv[1] != NIL) create = true;
  }
  if (n > 2) {
    if (argv[2] != NIL) remove_nan = true;
  }
  if (n > 3) {
    if (argv[3] == NIL) keep_organized = false;
  }
  if (n > 4) {
    x_step = intval (argv[4]);
  }
  if (n > 5) {
    y_step = intval (argv[5]);
  }
  if (n > 6) {
    x_offset = intval (argv[6]);
  }
  if (n > 7) {
    y_offset = intval (argv[7]);
  }

  int width = intval (get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_WIDTH));
  int height = intval (get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_HEIGHT));
  pointer points = get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_POINTS);
  pointer colors = get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_COLORS);
  pointer normals = get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_NORMALS);
  pointer curvatures = get_from_pointcloud (ctx, eus_in_cloud, K_EUSPCL_CURVATURES);

  // parse
  if (points != NIL && colors != NIL && normals != NIL) {
    STEP_POINTCLOUD_(PointCN);
  } else if (points != NIL && colors != NIL) {
    STEP_POINTCLOUD_(PointC);
  } else if (points != NIL && normals != NIL) {
    STEP_POINTCLOUD_(PointN);
  } else if (points != NIL) {
    STEP_POINTCLOUD_(Point);
  } else {
    // warning there is no points.
  }

  while (pc-- > 0) vpop();
  return ret;
}
