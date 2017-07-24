#include "fcl/collision_data.h"
#include "fcl/collision.h"

struct CollisionData
{
  CollisionData()
  {
    done = false;
  }

  /// @brief Collision request
  fcl::CollisionRequest request;

  /// @brief Collision result
  fcl::CollisionResult result;

  /// @brief Whether the collision iteration can stop
  bool done;
};

bool hrpsysCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_);
