#ifndef __EUSPCL_OCTOMAP__
#define __EUSPCL_OCTOMAP__

// eus functions
extern pointer OCTOMAP_CREATE (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_CREATE_COLOR (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_DELETE (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_READ_BINARY (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_READ_NODES (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_READ_UNKNOWN (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_DUMP_DATA (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_NODE_NUM (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_ADD_POINTS (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_RESOLUTION (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_BOUNDING_BOX (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_BOUNDING_BOX_INFO (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_CLAMPING_THRESHOLD (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_METRIC_INFO (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_OCCUPANCY_THRESHOLD (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_PROBABILITY (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_GET_TREE_INFO (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_USE_BBX_LIMIT (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_SEARCH_RAY (register context *ctx, int n, pointer *argv);

// no arg methods
extern pointer OCTOMAP_BBX_SET (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_CLEAR (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_CLEAR_KEY_RAYS (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_EXPAND (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_PRUNE (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_TO_MAX_LIKELIHOOD (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_UPDATE_INNER_OCCUPANCY (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_GET_TREE_DEPTH (register context *ctx, int n, pointer *argv);

#endif
