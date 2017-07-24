#include "FCLCollision.h"
#include <iostream>

bool hrpsysCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_)
{
  std::cout << "\n\nChecking collision...\n\n\n\n\n\n" << std::endl;
  // BUG HERE: Never enter this function!!!!!!
  CollisionData* cdata = static_cast<CollisionData*>(cdata_);
  const fcl::CollisionRequest& request = cdata->request;
  fcl::CollisionResult& result = cdata->result;

  if(cdata->done) return true;

  fcl::collide(o1, o2, request, result);

  std::cout << "num collision: " << result.numContacts() << std::endl;

  if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

