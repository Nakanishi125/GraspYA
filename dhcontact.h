#ifndef DHCONTACT_H
#define DHCONTACT_H

#include "dhSkeletalSubspaceDeformation.h"
#include "dhMesh.h"
#include "dhCollisionDetection.h"
#include "dhArmature.h"


struct segment
{
    vector<int> BodyColor;
    vector<int> BodyPointsID;
    dhVec3 BodyCoG;     //CoG (Center of G ?)   点群の中心座標
    dhVec3 BodyCoG_Normal;

    vector<dhVec3> ObjectPoints;
    int ObjectPointsID;
    dhVec3 ObjectCoG;
    dhVec3 ObjectNormal;

};

//「mesh1を構成する点群」と「点群データ」が重なる領域内の点群を各々抽出する関数
void computeContactRegion(dhPointCloudAsVertexRef*& contactPoints1,
                          dhPointCloudAsVertexRef*& contactPoints2,
                          dhSkeletalSubspaceDeformation* mesh1, dhPointCloud* pts,
                          double tolIn=1.0, double tolOut=1.0);

////「mesh1を構成する点群」と「mesh2を構成する点群」が重なる領域内の点群を各々抽出する関数        //使わないけど念のため
//void computeContactRegion(dhPointCloudAsVertexRef*& contactPoints1,
//                          dhPointCloudAsVertexRef*& contactPoints2,
//                          dhSkeletalSubspaceDeformation* mesh1,dhMesh* mesh2,
//                          double tolIn=10.0,double tolOut=1.0);

void getArmatureStructure(dhArmature* arm, dhBone* bone,
                          vector<QString>& bones, vector<int>& depths, QString root = "ROOT");


void traceArmature(dhBone* parent,int depth,vector<QString>& bones,vector<int>& depths);

void getPointsFromCloud(dhPointCloudAsVertexRef* pointCloud,vector<dhVec3>& points);

double getDistance(dhVec3 point,dhVec3 orig,dhVec3 tail);

//点群objectPointsの所属骨を導出し，segment型のメンバに適切な値を入力していく関数
void segmentObjectPoints(dhPointCloudAsVertexRef* objectPoints,segment* segm,dhArmature* arm,
                         vector<QString> &bones);

void segmentBodyPoints_muscle(dhPointCloudAsVertexRef*& contactPoints,
                              dhSkeletalSubspaceDeformation* bodySSD, dhMesh* bodyMesh,
                              dhMesh* objMesh, vector<vector<QString>> color_def,
                              segment* segm);


#endif // DHCONTACT_H
