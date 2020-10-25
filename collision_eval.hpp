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
#include "segment.h"

double handcollision_eval(dhSkeletalSubspaceDeformation* mesh1, dhMesh* mesh2, dhArmature* arm);

double hand_length(dhArmature* arm);

#endif // COLLISION_EVAL_H
