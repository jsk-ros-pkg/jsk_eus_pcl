#ifndef __EUSPCL_OCTOMAP__
#define __EUSPCL_OCTOMAP__

// eus functions
extern pointer OCTOMAP_CREATE (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_DELETE (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_READ_NODES (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_READ_UNKNOWN (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_DUMP_BINARY (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_NODE_NUM (register context *ctx, int n, pointer *argv);
extern pointer OCTOMAP_ADD_POINTS (register context *ctx, int n, pointer *argv);
#endif
