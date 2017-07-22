#include "FCLCollision.h"

namespace fcl {

bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2, void* cdata_)
{
  // std::cerr << "\n\nChecking collision...\n\n\n\n\n\n" << std::endl;
  // BUG HERE: Never enter this function!!!!!!
  CollisionData* cdata = static_cast<CollisionData*>(cdata_);
  const CollisionRequest& request = cdata->request;
  CollisionResult& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

}