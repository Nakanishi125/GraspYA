#include<iostream>
#include<fstream>
#include<vector>
#include<time.h>
#include<sstream>
#include<cmath>
#include<cstring>
#include"csv.hpp"
#include"Vector2D.hpp"
#include"dhArmature.h"
#include"dhMath.h"
#include"dhBone.h"

#include"finalpos.h"

using namespace std;


double calc_distance(vector<Vector2D> xy, Vector2D pos_pos){
    double min_pos_to_edge_length;

	for(int m1=0; m1<xy.size()-2; m1++){
		int m2 = m1 + 1;

        Vector2D position_edge1 = xy[m1];
        Vector2D position_edge2 = xy[m2];

		Vector2D u = position_edge2 - position_edge1;
		Vector2D v = pos_pos - position_edge1;

        double pos_to_edge_length = ((u%v)/(u.norm())) > 0 ? ((u%v)/(u.norm())) : -((u%v)/(u.norm()));

		Vector2D position_H = position_edge1 + (u*v)/(u.norm()*u.norm())*u;

		Vector2D min = position_edge1.x < position_edge2.x ? position_edge1 : position_edge2;
		Vector2D max = position_edge1.x > position_edge2.x ? position_edge1 : position_edge2;

        int flag;

        if(min.x < position_H.x && position_H.x < max.x){
			flag = 0;
		}
		else if(max.x < position_H.x){
            pos_to_edge_length = sqrt( (max.x-pos_pos.x)*(max.x-pos_pos.x) + (max.y-pos_pos.y)*(max.y-pos_pos.y) );
			flag = 1;;
        }
		else if(min.x > position_H.x){
			pos_to_edge_length = (min - pos_pos).norm();
			flag = 1;
		}


        if(m1 == 0){
			min_pos_to_edge_length = pos_to_edge_length;
		}
		else{
			if(min_pos_to_edge_length > pos_to_edge_length){
				min_pos_to_edge_length = pos_to_edge_length;
			}
		}
	}

	return min_pos_to_edge_length;
}


void in_or_out(vector<Vector2D> xy, Vector2D pos_pos,int& inside_num,int& in_out){
	for(int l1=0; l1<xy.size()-2; l1++){
		int l2 = l1 + 1;

        Vector2D position_edge1 = xy[l1];
        Vector2D position_edge2 = xy[l2];

//============================
//Crossing Number Algorithm
//============================

		if(position_edge1.x != position_edge2.x){
			Vector2D min = position_edge1.x < position_edge2.x ? position_edge1 : position_edge2;
			Vector2D max = position_edge1.x > position_edge2.x ? position_edge1 : position_edge2;

            if(min.x < pos_pos.x && pos_pos.x < max.x){		//x方向に見て，対象辺内に現在点がある
				if( (position_edge1.y+(position_edge2.y-position_edge1.y)/(position_edge2.x-position_edge1.x)
                        *(pos_pos.x-position_edge1.x)-pos_pos.y) > 0){  //yが増える方向に探索
					inside_num++;
				}
				else if((position_edge1.y+(position_edge2.y-position_edge1.y)/(position_edge2.x-position_edge1.x)
						*(pos_pos.x-position_edge1.x)-pos_pos.y) == 0){
					in_out = 0;
				}
			}
            else if(pos_pos.x == position_edge2.x){
                if(pos_pos.y < position_edge2.y){
                    if(position_edge1.x > position_edge2.x){    //rule1:左向きの矢印の場合，
                        inside_num++;                           //      開始点を含まず，終着点を含む
					}
//                    else if(position_edge1.x < position_edge2.x){
//                        //rule2:右向きの矢印の場合，開始点を含み，終着点を含まない．
//                    }
				}
                else if(pos_pos.y == position_edge2.y){
                    in_out = 0;
				}
			}
            else if(pos_pos.x == position_edge1.x){
                if(pos_pos.y < position_edge1.y){
                    if(position_edge1.x < position_edge2.x){    //rule2:右向き矢印の場合，
                        inside_num++;                           //      開始点を含み，終着点を含まない
					}
//                    else if(position_edge1.x > position_edge2.x){
//                        //rule1:左向きの矢印の場合，開始点を含まず，終着点を含む
//                    }
				}
				else if(pos_pos.y == position_edge1.y){
					in_out = 0;
				}
			}
		}
        else{   //yが増える方向への探索線と辺が平行で重なる場合，交差数は増えない
			if(pos_pos.x == position_edge1.x){
				Vector2D min2 = position_edge1.y < position_edge2.y ? position_edge1 : position_edge2;
				Vector2D max2 = position_edge1.y > position_edge2.y ? position_edge1 : position_edge2;
				if(min2.y <= pos_pos.y && pos_pos.y <= max2.y){
					in_out = 0;
				}
			}
		}	



	}
}

double rom_eval(dhArmature* arm){
    QString bones[] = {"TMCP","TPP","TDP","IPP","IMP","IDP","MPP","MMP","MDP","RPP","RMP","RDP","PPP","PMP","PDP"};
//    string bones[] = {"TMCP","TPP","TDP","IPP","IMP","IDP","MPP","MMP","MDP","RPP","RMP","RDP","PPP","PMP","PDP"};
//    clock_t t1,t2,t3,t4,t5,t6,t7,t8;
//    t1 = clock();

	vector<vector<string>> rotation;
    double min_length_sum = 0.0;


    for(int boneIndex=0; boneIndex < sizeof(bones)/sizeof(bones[0]); boneIndex++){
        dhBone *bone = arm->bone(bones[boneIndex]);
        dhVec4 r[2];
        dhMat44 mat=bone->R;        //初期姿勢→現在姿勢の変換行列
        mat.getRPYAngle(r[0],r[1]); //解が二つ出るが，ここでは0の方のみ書き出すことにする

        double deg_x = r[0].p[0] / (3.141592) * 180.0;
        double deg_y = r[0].p[1] / (3.141592) * 180.0;
        double deg_z = r[0].p[2] / (3.141592) * 180.0;


		vector<string> line;

        line.push_back(bones[boneIndex].toStdString());
		line.push_back(to_string(deg_x));
		line.push_back(to_string(deg_y));
		line.push_back(to_string(deg_z));

        rotation.push_back(line);       //骨別の現在の関節角度

	}

//    t2 = clock();
//    double etime = (double)(t2-t1)/1000;
//    DH_LOG("section1 is "+QString::number(etime,'f',5),0);


// ********************************************************************************************
//joint_relation.csvの読み込み

    string list = "C:\\kenkyu\\GraspYA\\data\\joint_relation.csv";
	vector<vector<string>> joints_list;

	Csv objJL(list);
	if(!objJL.getCsv(joints_list)){
        DH_LOG("failed to read the file",0);
		return 1;
	}

	joints_list.erase(joints_list.begin());		//先頭行削除


// ********************************************************************************************
// joint_bone.csvの読み込み

    string bone = "C:\\kenkyu\\GraspYA\\data\\joint_bone.csv";
	vector<vector<string>> joint_bone;

	Csv objJB(bone);
	if(!objJB.getCsv(joint_bone)){
        DH_LOG("failed to read the file",0);
		return 1;
	}

	joint_bone.erase(joint_bone.begin());

// ***********************************************************************************************

//    t3 = clock();
//    etime = (double)(t3-t2)/1000;
//    DH_LOG("section2 is "+QString::number(etime,'f',5),0);

#pragma omp parallel for default(private) reduction(+:min_length_sum)
	for(int i=0; i<joints_list.size(); i++){
//        t5 = clock();
		string joint1 = joints_list[i][0];
		string joint2 = joints_list[i][1];

        double rot1,rot2;

		for(int j=0; j<joint_bone.size(); j++){
			if(joint_bone[j][0] == joint1){
				for(int k=0; k<rotation.size(); k++){
					if(rotation[k][0] == joint_bone[j][1]){
						int axis = stoi(joint_bone[j][2]);
						rot1 = stof(rotation[k][axis+1]);
					}
				}
			}
			if(joint_bone[j][0] == joint2){
				for(int k=0; k<rotation.size(); k++){
					if(rotation[k][0] == joint_bone[j][1]){
						int axis = stoi(joint_bone[j][2]);
						rot2 = stof(rotation[k][axis+1]);
					}
				}
			}
        }

//        t6 = clock();
//        etime = (double)(t6-t5)/1000;
//        DH_LOG("subsec1 is "+QString::number(etime,'f',5),0);

        Vector2D position_pos(rot1,rot2);   //joints_listによって定められた関節の組み合わせに応じて
                                            //現在の関節角度組み合わせを定義する．

//        DH_LOG(QString::number(rot1,'f',2)+" , "+QString::number(rot2,'f',2),0);      //clear

        string ashape_frame = "C:\\kenkyu\\GraspYA\\ashape\\simple\\ashape_simple_";
		string extension = ".csv";
		string ashape_file = ashape_frame + joint1 + '_' + joint2 + extension;
		vector<vector<string>> ashape_string;
        vector<Vector2D> ashape;

		Csv obj_ashape(ashape_file);
		if(!obj_ashape.getCsv(ashape_string)){
            DH_LOG("failed to read the file",0);
			return 1;
		}

        for(unsigned int row=1; row<ashape_string.size(); row++){
            vector<string> buf = ashape_string[row];
            Vector2D rec(stof(buf[1]),stof(buf[2]));
            ashape.push_back(rec);
        }

//        t7 = clock();
//        etime = (double)(t7-t6)/1000;
//        DH_LOG("subsec2 is "+QString::number(etime,'f',5),0);

        double min_pos_to_edge_length;

		min_pos_to_edge_length = calc_distance(ashape,position_pos);

		int inside_num = 0;
		int in_out = 1;

//        DH_LOG(QString::number(min_pos_to_edge_length,'f',2),0);      //clear

		in_or_out(ashape,position_pos,inside_num,in_out);

		if(in_out != 0 && inside_num % 2 == 0){
			in_out = 0;
		}

        double min_length;

		if(in_out == 0){
			min_length = min_pos_to_edge_length + 5.0;
		}
		else{
			if(min_pos_to_edge_length < 5.0){
				min_length = 5.0 - min_pos_to_edge_length;
			}
			else{
				min_length = 0;
			}
		}

//        t8 = clock();
//        etime = (double)(t8-t7)/1000;
//        DH_LOG("subsec3 is "+QString::number(etime,'f',5),0);

		min_length_sum += min_length * min_length;

//        DH_LOG("min_length_sum= "+QString::number(min_length_sum,'f',2),0);       //clear

	}
//    t4 = clock();
//    etime = (double)(t4-t3)/1000;
//    DH_LOG("section3 is "+QString::number(etime,'f',5),0);

//===================================
//先にassembledActiveDF01.csvから
//最大値と最小値を抜き出したファイルを読み込む
//===================================

    string add = "C:\\kenkyu\\GraspYA\\data\\assembledActiveDF01_maxmin.csv";
    vector<vector<string>> DF;

    Csv objDF(add);
    if(!objDF.getCsv(DF)){
        DH_LOG("failed to read the file",0);
        return 1;
    }

#pragma omp for default(private) reduction(+:min_length_sum)
    for(int col=0; col<joint_bone.size(); col++){
        double min_length;
        double min,max,rot;

        for(int col2=0; col2<DF.size(); col2++){
            if(joint_bone[col][0] == DF[col2][0]){
                max = stof(DF[col2][1]);
                min = stof(DF[col2][2]);
            }
        }

        string name = joint_bone[col][1];;
        int axis = stoi(joint_bone[col][2]);


        for(int ii=0; ii<rotation.size(); ii++){        //現在の関節角度をrotに格納
            if(rotation[ii][0] == name){
                rot = stof(rotation[ii][axis+1]);
            }
        }
	
        double ave = (max + min)/2;

        if(ave <= rot){
            if(max < rot + 5.0){
                min_length = rot + 5.0 - max;
            }
            else{
                min_length = 0.0;
            }
        }
        if(ave >= rot){
            if(min > rot - 5.0){
                min_length = min - (rot - 5.0);
            }
            else{
                min_length = 0.0;
            }
        }

//        DH_LOG(QString::number(max,'f',2)+","+QString::number(min,'f',2)+","+QString::number(rot,'f',2),0);

		min_length_sum += min_length * min_length;

	} 

//    t5 = clock();
//    etime = (double)(t5-t4)/1000;
//    DH_LOG("section4 is "+QString::number(etime,'f',5),0);

    double Rom_eval = min_length_sum/(30+25);

    return Rom_eval;
}

