#include "eus_pcl/euspcl.h"
#include "eus_pcl/euspcl_io.h"

#define READ_PCD_(PTYPE) \
  pcl::PointCloud < PTYPE > pt;  \
  int res = rd.read< PTYPE > (fname, pt); \
  if ( res != 0 ) { \
    return NIL;     \
  }                 \
  ret = make_pointcloud_from_pcl(ctx, pt); \
  vpush(ret); pc++;

pointer PCL_READ_PCD (register context *ctx, int n, pointer *argv) {
  /* filename */
  pcl::PCDReader rd;
  pointer ret = NIL;
  int pc = 0;
  bool have_rgb=false, have_normal=false, have_position=false;
  if ( !isstring(argv[0]) ) {
    error(E_NOSTRING);
  }

  std::string fname;
  fname.assign( (char *) get_string(argv[0]));

  {
    sensor_msgs::PointCloud2 hdr;
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 6 )
    int ret = rd.readHeader(fname, hdr);
#else
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int pcd_version; int data_type; int data_index;
    int ret = rd.readHeader(fname, hdr, origin, orientation, pcd_version, data_type, data_index);
#endif
    if ( ret != 0 ) {
      std::cerr << "read header error" << std::endl;
    } else {
      for (size_t i = 0; i < hdr.fields.size(); i++) {
        if ( hdr.fields[i].name == "rgb" ) {
          have_rgb = true;
        }
        if ( hdr.fields[i].name == "normal_x" ) {
          have_normal = true;
        }
        if ( hdr.fields[i].name == "x" ) {
          have_position = true;
        }
      }
    }
  }

  if ( have_position && have_rgb && have_normal ) {
    READ_PCD_ ( PointCN );
  } else if ( have_position && have_rgb ) {
    READ_PCD_ ( PointC );
  } else if ( have_position && have_normal ) {
    READ_PCD_ ( PointN );
  } else if ( have_position ) {
    READ_PCD_ ( Point );
  } else {
    // no points
  }

  while ( pc-- > 0) vpop();
  return ret;
}

#define WRITE_PCD_(PTYPE, fname)                  \
  pcl::PointCloud< PTYPE >::Ptr pcl_cloud = \
    make_pcl_pointcloud< PTYPE > (ctx, points, colors, normals, width, height); \
  pcl::PCDWriter wt;                                                    \
  wt.write < PTYPE > ( fname, *pcl_cloud, binary );

pointer PCL_WRITE_PCD (register context *ctx, int n, pointer *argv) {
  /* fname pointcloud &optional (binary t) */
  bool binary = true;
  pointer in_cloud;
  pointer points,colors,normals;
  numunion nu;
  int pc = 0;
  std::string fname;

  ckarg2(2, 3);
  if ( ! isstring(argv[0]) ) error(E_NOSTRING);

  fname.assign( (char *)( argv[0]->c.str.chars ) );
  in_cloud = argv[1];

  if ( n > 2 ) {
    if ( argv[2] == NIL) binary = false;
  }

  int width = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_WIDTH));
  int height = intval(get_from_pointcloud(ctx, in_cloud, K_EUSPCL_HEIGHT));
  points = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_POINTS);
  colors = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_COLORS);
  normals = get_from_pointcloud(ctx, in_cloud, K_EUSPCL_NORMALS);

  if ( points != NIL && colors != NIL && normals != NIL ) {
    WRITE_PCD_(PointCN, fname);
  } else if ( points != NIL && colors != NIL ) {
    WRITE_PCD_(PointC, fname);
  } else if ( points != NIL && normals != NIL ) {
    WRITE_PCD_(PointN, fname);
  } else if ( points != NIL ) {
    WRITE_PCD_(Point, fname);
  } else {
    // warning there is no points.
  }

  while ( pc-- > 0) vpop();

  return T;
}
