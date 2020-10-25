#include<iostream>
#include<fstream>
#include<vector>
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

double calc_distance(vector<vector<double>> xy, Vector2D pos_pos){
    double min_pos_to_edge_length;

	for(int m1=0; m1<xy.size()-2; m1++){
		int m2 = m1 + 1;
		Vector2D position_edge1(xy[m1][0],xy[m1][1]);
		Vector2D position_edge2(xy[m2][0],xy[m2][1]);

		Vector2D u = position_edge2 - position_edge1;
		Vector2D v = pos_pos - position_edge1;

        double pos_to_edge_length = ((u%v)/(u.norm())) > 0 ? ((u%v)/(u.norm())) : -((u%v)/(u.norm()));

		Vector2D position_H = position_edge1 + (u*v)/(u.norm()*u.norm())*u;

		Vector2D min = position_edge1.x < position_edge2.x ? position_edge1 : position_edge2;
		Vector2D max = position_edge1.x > position_edge2.x ? position_edge1 : position_edge2;

        int flag;

        if(min.x < position_H.x && position_H.x < max.x){
			flag = 0;
//            DH_LOG("0",0);
		}
		else if(max.x < position_H.x){
            pos_to_edge_length = sqrt( (max.x-pos_pos.x)*(max.x-pos_pos.x) + (max.y-pos_pos.y)*(max.y-pos_pos.y) );
			flag = 1;
//            DH_LOG("1",0);
//            DH_LOG(QString::number(max.x,'f',2)+','+QString::number(max.y,'f',2),0);
//            DH_LOG(QString::number(pos_pos.x,'f',2)+','+QString::number(pos_pos.y,'f',2),0);
        }
		else if(min.x > position_H.x){
			pos_to_edge_length = (min - pos_pos).norm();
			flag = 1;
//            DH_LOG("2",0);
		}

//        DH_LOG(QString::number(pos_to_edge_length,'f',2),0);
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

void in_or_out(vector<vector<double>> xy, Vector2D pos_pos,int& inside_num,int& in_out){
	for(int l1=0; l1<xy.size()-2; l1++){
		int l2 = l1 + 1;
		Vector2D position_edge1(xy[l1][0],xy[l1][1]);
		Vector2D position_edge2(xy[l2][0],xy[l2][1]);

		if(position_edge1.x != position_edge2.x){
			Vector2D min = position_edge1.x < position_edge2.x ? position_edge1 : position_edge2;
			Vector2D max = position_edge1.x > position_edge2.x ? position_edge1 : position_edge2;

			if(min.x < pos_pos.x && pos_pos.x < max.x){				
				if( (position_edge1.y+(position_edge2.y-position_edge1.y)/(position_edge2.x-position_edge1.x)
						*(pos_pos.x-position_edge1.x)-pos_pos.y) > 0){
					inside_num++;
				}
				else if((position_edge1.y+(position_edge2.y-position_edge1.y)/(position_edge2.x-position_edge1.x)
						*(pos_pos.x-position_edge1.x)-pos_pos.y) == 0){
					in_out = 0;
				}
			}
			else if(pos_pos.x == position_edge2.x){
				if(pos_pos.y < position_edge2.y){
					if(position_edge1.x > position_edge2.x){
						inside_num++;
					}
				}
				else if(pos_pos.y == position_edge2.y){
					in_out = 0;
				}
			}
			else if(pos_pos.x == position_edge1.x){
				if(pos_pos.y < position_edge1.y){
					if(position_edge1.x < position_edge2.x){
						inside_num++;
					}
				}
				else if(pos_pos.y == position_edge1.y){
					in_out = 0;
				}
			}
		}
		else{
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

double rom_evaluation(dhArmature* arm){		//同次変換行列から角度算出
//double rom_evaluation(){
    QString bones[] = {"TMCP","TPP","TDP","IPP","IMP","IDP","MPP","MMP","MDP","RPP","RMP","RDP","PPP","PMP","PDP"};
//    string bones[] = {"TMCP","TPP","TDP","IPP","IMP","IDP","MPP","MMP","MDP","RPP","RMP","RDP","PPP","PMP","PDP"};

//    string extension = ".csv";
	vector<vector<string>> rotation;
    double min_length_sum = 0.0;


    for(int boneIndex=0; boneIndex < sizeof(bones)/sizeof(bones[0]); boneIndex++){
        dhBone *bone = arm->bone(bones[boneIndex]);
        dhVec4 r[2];
        dhMat44 mat=bone->R;
        mat.getRPYAngle(r[0],r[1]); //解が二つ出るが，ここでは0の方のみ書き出すことにする
        double deg_x = r[0].p[0] / (3.141592) * 180.0;
        double deg_y = r[0].p[1] / (3.141592) * 180.0;
        double deg_z = r[0].p[2] / (3.141592) * 180.0;

        QString deg_xx = QString::number(deg_x,'f',2);
        QString deg_yy = QString::number(deg_y,'f',2);
        QString deg_zz = QString::number(deg_z,'f',2);

//        DH_LOG(bones[boneIndex],0);     //Pythonと一致
//        DH_LOG("deg_x="+deg_xx,0);
//        DH_LOG("deg_y="+deg_yy,0);
//        DH_LOG("deg_z="+deg_zz,0);


//    for(int ii=0; ii<(sizeof(bones)/sizeof(bones[0])); ii++){			//関節毎に
//        string frame = "C:\\kenkyu\\GraspYA\\rotation_pos\\";
//        string csv_file = frame + bones[ii] + extension;
//        vector<vector<string>> rotation_matrix;
//        vector<vector<double>> rotation_matrix_cov;
////		cout << csv_file << endl;


//        Csv objCsv(csv_file);	//create Csv Object
//        if(!objCsv.getCsv(rotation_matrix)){
//            cout << "cannot read" << endl;
//            return 1;
//        }

//        for(unsigned int row=0; row<rotation_matrix.size(); row++){	    //stringからdoubleへ
//            vector<string> rec = rotation_matrix[row];		//1 line
//            vector<double> cov;
//            for(unsigned int col=0; col<rec.size(); col++){
//                cov.push_back(stof(rec[col]));
////					cout << cov[col];
////					if(col < rec.size()-1)	cout << ",";
//            }
//            rotation_matrix_cov.push_back(cov);
////				cout << endl;
//        }


		
//        double theta_x,theta_y,theta_z;
//        double deg_x,deg_y,deg_z;

//        theta_x = atan2(rotation_matrix_cov[2][1],rotation_matrix_cov[2][2]);
//        theta_y = asin(-rotation_matrix_cov[2][0]);
//        theta_z = atan2(rotation_matrix_cov[1][0],rotation_matrix_cov[0][0]);

//        deg_x = theta_x/M_PI*180;
//        deg_y = theta_y/M_PI*180;
//        deg_z = theta_z/M_PI*180;
 
/*  		cout << "deg_x:" << deg_x << endl;
        cout << "deg_y:" << deg_y << endl;
        cout << "deg_z:" << deg_z << endl; */

		vector<string> line;

        line.push_back(bones[boneIndex].toStdString());
//        line.push_back(bones[ii]);
		line.push_back(to_string(deg_x));
		line.push_back(to_string(deg_y));
		line.push_back(to_string(deg_z));

        rotation.push_back(line);       //現在の関節角度

	}

	/* for(int row=0; row < rotation.size(); row++){		//Matrixの中身を除くテンプレ
		vector<string> rec = rotation[row];
		for(int col=0; col < rec.size(); col++){
			cout << rec[col];
			if(col < rec.size() -1)	cout << ",";
		}
			cout << endl;
	} */


// ********************************************************************************************
//joint_relation.csvの読み込み

    string list = "C:\\kenkyu\\GraspYA\\data\\joint_relation.csv";
	vector<vector<string>> joints_list;

	Csv objJL(list);
	if(!objJL.getCsv(joints_list)){
		cout << "cannot read" << endl;
		return 1;
	}

	joints_list.erase(joints_list.begin());		//先頭行削除

	/* for(int row=0; row < joints_list.size(); row++){		//Matrixの中を見る
		vector<string> rec = joints_list[row];
		for(int col=0; col < rec.size(); col++){
			cout << rec[col];
			if(col < rec.size() -1)	cout << ",";
		}
			cout << endl;
	} */

// ********************************************************************************************
// joint_bone.csvの読み込み

    string bone = "C:\\kenkyu\\GraspYA\\data\\joint_bone.csv";
	vector<vector<string>> joint_bone;

	Csv objJB(bone);
	if(!objJB.getCsv(joint_bone)){
		cout << "cannot read" << endl;
		return 1;
	}

	joint_bone.erase(joint_bone.begin());

	/* for(int row=0; row < joint_bone.size(); row++){		//Matrixの中を見る
		vector<string> rec = joint_bone[row];
		for(int col=0; col < rec.size(); col++){
			cout << rec[col];
			if(col < rec.size() -1)	cout << ",";
		}
			cout << endl;
	} */

// ***********************************************************************************************

	for(int i=0; i<joints_list.size(); i++){
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

//		cout << rot1 << "," << rot2 << endl;

		Vector2D position_pos(rot1,rot2);
//        DH_LOG(QString::number(rot1,'f',2)+" , "+QString::number(rot2,'f',2),0);      //clear

        string ashape_frame = "C:\\kenkyu\\GraspYA\\ashape\\simple\\ashape_simple_";
		string extension = ".csv";
		string ashape_file = ashape_frame + joint1 + '_' + joint2 + extension;
		vector<vector<string>> ashape_string;
        vector<vector<double>> ashape;

//		cout << ashape_file << endl;
		Csv obj_ashape(ashape_file);
		if(!obj_ashape.getCsv(ashape_string)){
			cout << "cannot read" << endl;
			return 1;
		}

		for(unsigned int row=0; row<ashape_string.size(); row++){
			vector<string> buf = ashape_string[row];
            vector<double> rec;
			for(unsigned int col=1; col<buf.size(); col++){
				rec.push_back(stof(buf[col]));
			}
			ashape.push_back(rec);
		}

		ashape.erase(ashape.begin());

    	/* for(int row=0; row<ashape.size(); row++){		//Matrixの中を見る
            vector<double> rec = ashape[row];
        	for(int col=0; col < rec.size(); col++){
       		    cout << rec[col];
            	if(col < rec.size() -1)	cout << ",";
       		}
        	cout << endl;
    	} */

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

		min_length_sum += min_length * min_length;
//        DH_LOG("min_length_sum= "+QString::number(min_length_sum,'f',2),0);       //clear
//		cout << "min_length_sum1=" << min_length_sum << endl;

	}

//*******************************************************************************************
    string add = "C:\\kenkyu\\GraspYA\\data\\assembledActiveDF01.csv";
	vector<vector<string>> DF;

	Csv objDF(add);
	if(!objDF.getCsv(DF)){
		cout << "cannot read" << endl;
		return 1;
	}

/* 	for(int row=0; row < DF.size(); row++){		//Matrixの中を見る
		vector<string> rec = DF[row];
		for(int col=0; col < rec.size(); col++){
			cout << rec[col];
			if(col < rec.size() -1)	cout << ",";
		}
			cout << endl;
	} */

    for(int col=0; col<joint_bone.size(); col++){
        double min_length;
        double buf,min,max,rot;

        for(int col2=0; col2<DF[0].size(); col2++){         //max and min
            if(joint_bone[col][0] == DF[0][col2]){
                for(int row=1; row<DF.size(); row++){
                    buf = stof(DF[row][col2]);
                    if(row == 1){
                        min = buf;
                        max = buf;
                    }
                    else{
                        if(min>buf)	min=buf;
                        if(max<buf)	max=buf;
                    }
                }
            }
        }

        string name;
        int axis;

        name = joint_bone[col][1];
        axis = stoi(joint_bone[col][2]);

        for(int ii=0; ii<rotation.size(); ii++){        //rot
            if(rotation[ii][0] == name){
                rot = stof(rotation[ii][axis+1]);
            }
        }

        /* cout << "max=" << max << endl;
        cout << "min=" << min << endl;
        cout << "rot=" << rot << endl;
        cout << endl; */
	
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
//		cout << "min_length_sum2=" << min_length_sum << endl;
	} 
    double Rom_eval = min_length_sum/(30+25);
//    DH_LOG("Rom evaluation is "+QString::number(Rom_eval,'f',5),0);

    return Rom_eval;
}

