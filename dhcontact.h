#ifndef DHCONTACT_H
#define DHCONTACT_H

#include "dhSkeletalSubspaceDeformation.h"
#include "dhMesh.h"
#include "dhCollisionDetection.h"
#include "dhArmature.h"
#include "segment.h"



void computeContactRegion(dhPointCloudAsVertexRef*& contactPoints1,
                          dhPointCloudAsVertexRef*& contactPoints2,
                          dhSkeletalSubspaceDeformation* mesh1,dhMesh* mesh2,
                          double tolIn=10.0,double tolOut=1.0);

void getArmatureStructure(dhArmature* arm,vector<QString>& bones,vector<int>& depths,dhBone* root=NULL);

void traceArmature(dhBone* parent,int depth,vector<QString>& bones,vector<int>& depths);

void getPointsFromCloud(dhPointCloudAsVertexRef* pointCloud,vector<dhVec3>& points);

double getDistance(dhVec3 point,dhVec3 orig,dhVec3 tail);

void segmentObjectPoints(dhPointCloudAsVertexRef* objectPoints,segment* segm,dhArmature* arm,
                         vector<QString> &bones);






#endif // DHCONTACT_H
