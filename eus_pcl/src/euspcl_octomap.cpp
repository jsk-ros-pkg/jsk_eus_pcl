#include "eus_pcl/euspcl.h"

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <pcl/point_types_conversion.h>

#if __PCL_SELECT == 0
using namespace pcl;
#elif __PCL_SELECT == 17
using namespace pcl17;
#endif

#define define_octomap_func(name)                                     \
  pointer OCTOMAP_##name (register context *ctx, int n, pointer *argv) { \
    numunion nu;                                                        \
    if (n > 0) {                                                        \
      octomap::AbstractOccupancyOcTree *ptr;                            \
      ptr = (octomap::AbstractOccupancyOcTree *)(ckintval(argv[0]));    \
      std::string tp = ptr->getTreeType();                              \
      if (tp == "OcTree") {                                             \
        return _O_OCTOMAP_##name(ctx, n, argv, dynamic_cast<octomap::OcTree *>(ptr)); \
      } else if (tp == "ColorOcTree") {                                 \
        return _C_OCTOMAP_##name(ctx, n, argv, dynamic_cast<octomap::ColorOcTree *>(ptr)); \
      } else {                                                          \
        return NIL;                                                     \
      }                                                                 \
    } else {                                                            \
      return NIL;                                                       \
    }                                                                   \
  }

pointer OCTOMAP_CREATE (register context *ctx, int n, pointer *argv) {
  /* (resolution) -> octree_pointer */
  numunion nu;
  double resolution = 0.05;
  ckarg2(0, 1);
  if (n > 0) {
    resolution = ckfltval(argv[0]) * 0.001;
  }
  octomap::OcTree *tree_ptr;
  tree_ptr = new octomap::OcTree ( resolution );

  return makeint((eusinteger_t)tree_ptr);
}
pointer OCTOMAP_CREATE_COLOR (register context *ctx, int n, pointer *argv) {
  /* (resolution) -> octree_pointer */
  numunion nu;
  double resolution = 0.05;
  ckarg2(0, 1);
  if (n > 0) {
    resolution = ckfltval(argv[0]) * 0.001;
  }
  octomap::ColorOcTree *tree_ptr;
  tree_ptr = new octomap::ColorOcTree ( resolution );

  return makeint((eusinteger_t)tree_ptr);
}

typedef octomap::ColorOcTree ColorOcTreeT;
typedef octomap::OcTree OcTreeT;

#include "euspcl_octomap.template.cpp"

#define USE_COLOR_OCTOMAP
#define OcTreeT ColorOcTreeT
#include "euspcl_octomap.template.cpp"
#undef USE_COLOR_OCTOMAP

define_octomap_func(DELETE)
define_octomap_func(READ_BINARY)
define_octomap_func(RESOLUTION)
define_octomap_func(BOUNDING_BOX)
define_octomap_func(BOUNDING_BOX_INFO)
define_octomap_func(CLAMPING_THRESHOLD)
define_octomap_func(METRIC_INFO)
define_octomap_func(OCCUPANCY_THRESHOLD)
define_octomap_func(PROBABILITY)
define_octomap_func(BBX_SET)
define_octomap_func(CLEAR)
define_octomap_func(CLEAR_KEY_RAYS)
define_octomap_func(EXPAND)
define_octomap_func(PRUNE)
define_octomap_func(TO_MAX_LIKELIHOOD)
define_octomap_func(UPDATE_INNER_OCCUPANCY)
define_octomap_func(GET_TREE_DEPTH)

define_octomap_func(GET_TREE_INFO)
define_octomap_func(USE_BBX_LIMIT)
define_octomap_func(NODE_NUM)
define_octomap_func(READ_NODES)
define_octomap_func(READ_UNKNOWN)
define_octomap_func(ADD_POINTS)
define_octomap_func(SEARCH_RAY)
define_octomap_func(DUMP_DATA)

#if 0 // methods in OcTree
//  bool bbxSet ();
  size_t calcNumNodes ();

//  void clear ();
//  void clearKeyRays ();

//  virtual void expand ();
  point3d getBBXBounds ();
  point3d getBBXCenter ();
  point3d getBBXMax ();
  point3d getBBXMin ();
  double getClampingThresMax ();
  float getClampingThresMaxLog ();
  double getClampingThresMin ();
  float getClampingThresMinLog ();
  size_t getNumLeafNodes ();
  double getOccupancyThres ();
  float getOccupancyThresLog ();
  double getProbHit ();
  float getProbHitLog ();
  double getProbMiss ();
  float getProbMissLog ();
  double getResolution ();
  OcTreeNode * getRoot ();
//  unsigned int getTreeDepth ();
  std::string getTreeType ();
  bool isChangeDetectionEnabled ();

  unsigned long long memoryFullGrid ();
  virtual size_t memoryUsage ();
  virtual size_t memoryUsageNode ();
  size_t numChangesDetected ();
//  virtual void prune ();
  void resetChangeDetection ();

  virtual size_t size ();
//  void toMaxLikelihood();
//  void updateInnerOccupancy ();
  double volume ();
#endif
