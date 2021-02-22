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


void prepare_colleval(dhArmature* arm, double &size, dhPointCloud* internal, dhMesh* objMesh,
                      vector<vector<QString>> &input_set);

void generate_points_inobject(dhPointCloud* &internal, dhMesh* objMesh, vector<vector<QString>> input_set);

void extract_contactPoints(dhSkeletalSubspaceDeformation* ssd, dhPointCloud* pts,
                           dhPointCloudAsVertexRef* &bodyPoints, dhPointCloudAsVertexRef* &objectPoints);

double collision_eval(dhArmature* arm, dhPointCloudAsVertexRef* &bodyPoints,
                      dhPointCloudAsVertexRef* &objectPoints, double hand_size);

double hand_length(dhArmature* arm);

#endif // COLLISION_EVAL_H
