#include "fcl/collision_data.h"
#include "fcl/collision.h"
#include <iostream>

namespace fcl
{

struct CollisionData
{
  CollisionData()
  {
    done = false;
  }

  /// @brief Collision request
  CollisionRequest request;

  /// @brief Collision result
  CollisionResult result;

  /// @brief Whether the collision iteration can stop
  bool done;
};

bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2, void* cdata_);
}