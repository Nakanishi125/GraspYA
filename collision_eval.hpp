#ifndef COLLISION_EVAL_H
#define COLLISION_EVAL_H

#include <string>
#include <map>

#include <pcl/surface/convex_hull.h>
#include <pcl/impl/point_types.hpp>

#include "dhApplication.h"
#include "dhMath.h"
#include "dhArmature.h"
#include "dhMoCapSequence.h"
#include "dhFeaturePoint.h"
#include "dhSkeletalSubspaceDeformation.h"
#include "dhcontact.h"

void extract_contactPoints(dhSkeletalSubspaceDeformation* ssd, dhMesh* objMesh,
                           dhPointCloudAsVertexRef* &bodyPoints, dhPointCloudAsVertexRef* &objectPoints);

double collision_eval(dhArmature* arm, dhPointCloudAsVertexRef* &bodyPoints,
                      dhPointCloudAsVertexRef* &objectPoints);

double hand_length(dhArmature* arm);

#endif // COLLISION_EVAL_H
