#include<vector>
#include<limits>
#include<memory>

#include "dhcontact.h"
#include "collision_eval.hpp"
#include "dhSkeletalSubspaceDeformation.h"
#include "dhMesh.h"
#include "dhCollisionDetection.h"
#include "dhArmature.h"
#include "dhSpatialSearching.h"
#include "segment.h"


void computeContactRegion(dhPointCloudAsVertexRef*& contactPoints1,dhPointCloudAsVertexRef*& contactPoints2,
                          dhSkeletalSubspaceDeformation* mesh1,dhMesh* mesh2,double tolIn,double tolOut){

    dhCollisionDetection::Desc cdesc;
    cdesc.pc[0] = mesh1;  cdesc.pc[1] = mesh2;

    //あらかじめPointCloudを入れる変数を用意し，これを上書きしてもらう形をとる
    cdesc.pcCol[0] = contactPoints1;  cdesc.pcCol[1] = contactPoints2;

    cdesc.createContactPointCloud[0] = false;   //dhCollisionDetection内でPointCloudを作るかどうか(True or False)
    cdesc.createContactPointCloud[1] = false;   //Trueは作る，Falseは作らない
    cdesc.tolIn = tolIn;    cdesc.tolOut = tolOut;

    dhCollisionDetection* collision;
    collision = dhnew<dhCollisionDetection>();

    collision->initialize(cdesc);
    collision->doCollisionDetection();

    dhdelete(collision);

    dhApp::updateAllWindows();

}

void getArmatureStructure(dhArmature* arm,vector<QString>& bones,vector<int>& depths,dhBone* root){
    root = arm->bone(0);
    traceArmature(root,0,bones,depths);
}

void traceArmature(dhBone* parent,int depth,vector<QString>& bones,vector<int>& depths){
    bones.push_back(parent->Name());
    depths.push_back(depth);
    for(int j=0; j < parent->childBones.size(); j++){
        traceArmature(parent->childBones[j],depth+1,bones,depths);
    }
}

void getPointsFromCloud(dhPointCloudAsVertexRef* pointCloud,vector<dhVec3>& points){

    for(int j=0; j<pointCloud->PointCount(); j++){
        points.push_back((pointCloud->Position(j)).toVec3());
    }
}

double getDistance(dhVec3 point,dhVec3 orig,dhVec3 tail){
    double length = sqrt(orig*tail)/2;
    dhVec3 center((orig[0] + tail[0])/2 ,(orig[1] + tail[1])/2 ,(orig[2] + tail[2])/2);
    dhVec3 unit((tail[0] - center[0])/length, (tail[1] - center[1])/length, (tail[2] - center[2])/length);

    double scale = ( (point - center)* unit )/length;
    if(scale > 1.0) scale = 1.0;
    if(scale < -1.0)scale = -1.0;

    dhVec3 foot = center + scale*length*unit;

    return sqrt((point-foot)*(point-foot));
}

void segmentObjectPoints(dhPointCloudAsVertexRef* objectPoints,segment* segm,dhArmature* arm,
                         vector<QString> &bones){

    vector<dhVec3> points;
    getPointsFromCloud(objectPoints,points);

//#########################################################################
    dhSpatialSearching* spatialSearching = new dhSpatialSearching;
    for(int i=0; i<points.size(); i++){
        dhVec4 tmp_point = dhVec4(points[i]);
        spatialSearching->addPoint(tmp_point);
    }
    spatialSearching->buildTree();


//#########################################################################
//最寄りリンク探索
    for(int i=0; i<points.size(); i++){
        float dist = std::numeric_limits<float>::infinity();
        int bone;
        for(int j=0; j<bones.size(); j++){
            dhVec3 orig,tail;

            orig = arm->bone(bones[j])->Porigin().toVec3();
            tail = arm->bone(bones[j])->Ptail.toVec3();
            double tmpDist = getDistance(points[i],orig,tail);

            if(tmpDist < dist){
                dist = tmpDist;
                bone = j;
            }
        }
        segm[bone].ObjectPoints.push_back(points[i]);
    }


//############################################################################

    for(int k=0; k<bones.size(); k++){
        if(segm[k].ObjectPoints.empty())    continue;

        //ObjectPointsが空でなければCoGとNormalを導出し，代入
        double x,y,z;
        for(int arr=0; arr<segm[k].ObjectPoints.size(); arr++){
            x += (segm[k].ObjectPoints[arr])[0];
            y += (segm[k].ObjectPoints[arr])[1];
            z += (segm[k].ObjectPoints[arr])[2];
        }
        x = x/(segm[k].ObjectPoints.size());
        y = y/(segm[k].ObjectPoints.size());
        z = z/(segm[k].ObjectPoints.size());
        dhVec3 tmpcog(x,y,z);
        dhVec4 covcog(tmpcog);
        int index = spatialSearching->searchNearestNeighbor(covcog);

        dhVec3 cog = points[index];
        segm[k].ObjectCoG = cog;

        dhVec3 normal_sub = objectPoints->Normal(index).toVec3();
        double norm = sqrt(normal_sub*normal_sub);
        double a,b,c;
        a = normal_sub[0]/norm;
        b = normal_sub[1]/norm;
        c = normal_sub[2]/norm;
        dhVec3 normal(a,b,c);
        segm[k].ObjectNormal = normal;
    }

    delete spatialSearching;
}


