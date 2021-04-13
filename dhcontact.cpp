#include<vector>
#include<array>
#include<limits>
#include<memory>

#include "dhcontact.h"
#include "collision_eval.hpp"
#include "dhSkeletalSubspaceDeformation.h"
#include "dhMesh.h"
#include "dhCollisionDetection.h"
#include "dhArmature.h"
#include "dhSpatialSearching.h"
#include "csv.hpp"

//「mesh1を構成する点群」と「mesh2を構成する点群」が重なる領域内の点群を各々抽出する関数
void computeContactRegion(dhPointCloudAsVertexRef*& contactPoints1,dhPointCloudAsVertexRef*& contactPoints2,
                          dhSkeletalSubspaceDeformation* mesh1, dhPointCloud* pts,
                          double tolIn, double tolOut){

    dhCollisionDetection::Desc cdesc;
    cdesc.pc[0] = mesh1;  cdesc.pc[1] = pts;


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




////「mesh1を構成する点群」と「mesh2を構成する点群」が重なる領域内の点群を各々抽出する関数        // 一応残しておく
//void computeContactRegion(dhPointCloudAsVertexRef*& contactPoints1,dhPointCloudAsVertexRef*& contactPoints2,
//                          dhSkeletalSubspaceDeformation* mesh1, dhMesh* mesh2,
//                          double tolIn, double tolOut){

//    dhCollisionDetection::Desc cdesc;
//    cdesc.pc[0] = mesh1;  cdesc.pc[1] = mesh2;


//    //あらかじめPointCloudを入れる変数を用意し，これを上書きしてもらう形をとる
//    cdesc.pcCol[0] = contactPoints1;  cdesc.pcCol[1] = contactPoints2;

//    cdesc.createContactPointCloud[0] = false;   //dhCollisionDetection内でPointCloudを作るかどうか(True or False)
//    cdesc.createContactPointCloud[1] = false;   //Trueは作る，Falseは作らない
//    cdesc.tolIn = tolIn;    cdesc.tolOut = tolOut;

//    dhCollisionDetection* collision;
//    collision = dhnew<dhCollisionDetection>();

//    collision->initialize(cdesc);
//    collision->doCollisionDetection();

//    dhdelete(collision);

//    dhApp::updateAllWindows();

//}


void getArmatureStructure(dhArmature* arm, dhBone* link,
                          vector<QString>& bones, vector<int>& depths, QString root){

    if(root == "ROOT"){
        link = arm->bone(0);
    }
    else{
        link = arm->bone(root);
    }

    traceArmature(link, 0, bones, depths);

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
    double length = sqrt(orig*tail)/2;      // bone_length/2
    dhVec3 center((orig[0] + tail[0])/2 ,(orig[1] + tail[1])/2 ,(orig[2] + tail[2])/2);
    dhVec3 unit((tail[0] - center[0])/length, (tail[1] - center[1])/length, (tail[2] - center[2])/length);

    double scale = ( (point - center)* unit )/length;
    if(scale > 1.0) scale = 1.0;
    if(scale < -1.0)scale = -1.0;

    dhVec3 foot = center + scale*length*unit;

    return sqrt((point-foot)*(point-foot));
}


//点群objectPointsの所属骨を導出し，segment型のメンバに適切な値を入力していく関数
//ここでのsegmは，骨に所属する．
void segmentObjectPoints(dhPointCloudAsVertexRef* objectPoints, segment* segm, dhArmature* arm,
                         vector<QString> &bones){

    vector<dhVec3> points;
    getPointsFromCloud(objectPoints,points);

    dhSpatialSearching* spatialSearching = new dhSpatialSearching;

    for(int i=0; i<points.size(); i++){
        dhVec4 tmp_point = dhVec4(points[i]);
        spatialSearching->addPoint(tmp_point);
    }
    spatialSearching->buildTree();


//=============
//最寄りリンク探索
//=============
    for(int i=0; i<points.size(); i++){
        float dist = std::numeric_limits<float>::infinity();    //初期値として無限大代入
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

//areaに所属するsegmentのメンバ変数を埋める
void segmentBodyPoints_muscle(dhPointCloudAsVertexRef*& contactPoints,
                              dhSkeletalSubspaceDeformation* bodySSD, dhMesh* bodyMesh,
                              dhMesh* objMesh, vector<vector<QString>> color_def, segment* segm){


    for(size_t area=0; area<color_def.size(); area++){     //メンバ変数BodyColorへの代入
        segm[area].BodyColor.push_back(color_def[area][1].toInt());
        segm[area].BodyColor.push_back(color_def[area][2].toInt());
        segm[area].BodyColor.push_back(color_def[area][3].toInt());
    }

    for(int i=0; i<contactPoints->PointCount(); i++){           //全接触点を適切な領域に所属させていく
        int PointID = contactPoints->refID(i);                      //以降，メンバ変数BodyPointsへの代入

        dhVec3 point_color(bodyMesh->Vi(PointID)->color(0),         //頂点が接触しているbodyMeshの色を抽出する
                           bodyMesh->Vi(PointID)->color(1),
                           bodyMesh->Vi(PointID)->color(2));        

        int flag = 0;
        for(int area=0; area<color_def.size(); area++){
//            DH_LOG("Pointcolor:"+QString::number((int)(255*point_color[0]))+","+QString::number((int)(255*point_color[1]))+","+QString::number((int)(255*point_color[2])),0);
//            DH_LOG("Bodycolor:"+QString::number(segm[area].BodyColor[0])+","+QString::number(segm[area].BodyColor[1])+","+QString::number(segm[area].BodyColor[2]),0);

//            if( (((int)(255*point_color[0])-5 < segm[area].BodyColor[0]) &&
//                 (segm[area].BodyColor[0]<(int)(255*point_color[0])+5))
//                    &&
//                (((int)(255*point_color[1])-5 < segm[area].BodyColor[1]) &&
//                 (segm[area].BodyColor[1]<(int)(255*point_color[1])+5))
//                    &&
//                (((int)(255*point_color[2])-5 < segm[area].BodyColor[2]) &&
//                                 (segm[area].BodyColor[2]<(int)(255*point_color[2])+5)) )

            if((segm[area].BodyColor[0] == (int)(255*point_color[0])) &&
               (segm[area].BodyColor[1] == (int)(255*point_color[1])) &&
               (segm[area].BodyColor[2] == (int)(255*point_color[2])) )
            {
                segm[area].BodyPointsID.push_back(PointID);
                flag = 1;
                break;
            }
        }
        if(flag == 0){
            DH_LOG("The color is not defined.",0);
//            for(int area=0; area<color_def.size(); area++){   //ハンドモデルに存在しないRGB(150,150,255)が混ざっている．
//                DH_LOG("Pointcolor:"+QString::number((int)(255*point_color[0]))+","+QString::number((int)(255*point_color[1]))+","+QString::number((int)(255*point_color[2])),0);
//                DH_LOG("Bodycolor:"+QString::number(segm[area].BodyColor[0])+","+QString::number(segm[area].BodyColor[1])+","+QString::number(segm[area].BodyColor[2]),0);
//            }
        }
    }

    for(int area=0; area<color_def.size(); area++)
    {
        if(segm[area].BodyPointsID.size() <= 5){    //接触点数が5個以内なら，空にする
            segm[area].BodyPointsID.clear();
        }

        dhVec3 sum_points;          //ここからメンバ変数BodyCoGへの代入
        int points_num = segm[area].BodyPointsID.size();
        if(points_num){
            for(int j=0; j<points_num; j++){
                dhVec3 point_pos = bodySSD->V(segm[area].BodyPointsID[j]).toVec3();
                sum_points = sum_points + point_pos;
            }

            //(1領域における全座標の重心)
            dhVec3 cogs(sum_points[0]/points_num, sum_points[1]/points_num, sum_points[2]/points_num);
            double mindist = DBL_MAX;
            dhVec3 nearcogs, nearcogs_norm;

            for(int j=0; j<points_num; j++){        //重心に一番近い点をCoGとし，メンバ変数BodyCoGとBodyCog_Normalに代入
                double dist = ( bodySSD->V(segm[area].BodyPointsID[j]).toVec3() - cogs ).norm();
                if(dist < mindist){
                    nearcogs      = bodySSD->Vi(segm[area].BodyPointsID[j])->pt.toVec3();
                    nearcogs_norm = bodySSD->Vi(segm[area].BodyPointsID[j])->normal.toVec3();
                    mindist = dist;
                }
            }
            segm[area].BodyCoG = nearcogs;
            segm[area].BodyCoG_Normal = nearcogs_norm;

            double dist_min = std::numeric_limits<float>::infinity();   //初期値に無限大を代入
            for(int m=0; m<objMesh->Nv(); m++){
                dhVec3 obj_pos = objMesh->V(m).toVec3();
                double dist = (cogs-obj_pos).norm();

                if(dist_min>dist){      //メンバ変数Object～への代入
                    dist_min = dist;
                    segm[area].ObjectPointsID = m;
                    segm[area].ObjectCoG = obj_pos;
                    segm[area].ObjectNormal = objMesh->Vnorm(m).toVec3();
                }
            }
        }

    }

}


void extract_nearpoints(segment segm, dhSkeletalSubspaceDeformation* bodySSD, int &minID1, int &minID2)
{

    double min_dist1 = DBL_MAX;
    double min_dist2 = DBL_MAX;
    for(int id=0; id<segm.BodyPointsID.size(); id++){   // have debugged

        double dist = ( bodySSD->V(segm.BodyPointsID[id]).toVec3() - segm.BodyCoG ).norm();

        if(dist != 0)
        {
            if(min_dist1 > dist){
                min_dist2 = min_dist1;
                minID2 = minID1;

                min_dist1 = dist;
                minID1 = segm.BodyPointsID[id];
            }
            else if(min_dist2 > dist){
                min_dist2 = dist;
                minID2 = segm.BodyPointsID[id];
            }
        }
    }

}




