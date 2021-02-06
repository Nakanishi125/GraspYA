#include<iostream>
#include<fstream>
#include<vector>
#include<time.h>
#include<sstream>
#include<array>
#include<string>
#include<cmath>
#include<cstring>
#include"csv.hpp"
#include"Vector2D.hpp"
#include"dhArmature.h"
#include"dhMath.h"
#include"dhBone.h"

#include"rom_eval.hpp"
#include"pgmlib.h"
#include"finalpos.h"

using namespace std;

double cal_res(int age){
    if(age>=30){
        return -0.10*(age-30)/40 + 1.0;   // y = -0.1/40 * (x-30) + 1.0
    }
    else{
        return 1.0;
    }
}

void restrict_ROM(vector<vector<QString>> &DF, double rst){     //一次元ROMの制限
    for(size_t t=0; t<DF.size(); t++){
        double ave,tmp1,tmp2;
        ave = (DF[t][1].toDouble() + DF[t][2].toDouble())/2;
        tmp1 = ave + (DF[t][1].toDouble() - ave) * rst;
        tmp2 = ave + (DF[t][2].toDouble() - ave) * rst;
        DF[t][1] = QString::number(tmp1);
        DF[t][2] = QString::number(tmp2);
    }
}

void csv_to_pgm(vector<vector<int>> &conv, const vector<Vector2D> ashape,
                    double minx, double miny, double maxx, double maxy)
{
    for(size_t i=0; i<ashape.size(); i++)
    {
        int pixx,pixy;
        vector<int> buf;
        pixx =(int)( ((WIDTH-1)-2*MARGIN) * (ashape[i].x-minx)/(maxx-minx)) + MARGIN;
        pixy = ((HEIGHT-1)-MARGIN) - (int)( ((HEIGHT-1)-2*MARGIN) * (ashape[i].y-miny)/(maxy-miny));
        buf.push_back(pixx);
        buf.push_back(pixy);
        conv.push_back(buf);
        buf.clear();
    }

    // for(int i=0; i<ashape.size();i++){
    //     for(int j=0; j<ashape[i].size(); j++){
    //         std::cout << ashape[i][j] << ' ';
    //     }
    //     std::cout << std::endl;
    // }
}

void pgm_to_csv(vector<array<int,2>> edge, vector<Vector2D> &csvize,
                double minx, double miny, double maxx, double maxy, int split_csv)
{
    for(size_t i=0; i<edge.size()/split_csv; i++){      //PGMをCSVに
        double csvx,csvy;
        csvx = (maxx-minx)*(edge[split_csv*i][0] - MARGIN)/((WIDTH-1)-2*MARGIN) + minx;
        csvy = (maxy-miny)*( (HEIGHT-1-MARGIN) - edge[split_csv*i][1])/((HEIGHT-1)-2*MARGIN) + miny;
        Vector2D temporary(csvx,csvy);
        csvize.push_back(temporary);
    }
    csvize.push_back(csvize[0]);
}

void ConnectingPoints(vector<vector<int>> conv, PGM *rom)
{
    for(size_t t=0; t<conv.size()-1; t++){
        int dx,dy;
        int signx = 1;
        int signy = 1;
        double gap;

        dx = conv[t+1][0] - conv[t][0];
        dy = conv[t+1][1] - conv[t][1];

        if(dx == 0)
        {
            if(dy < 0){
                dy = -dy;
                signy = -1;
            }
            for(int i=0; i<=dy; i++){
                rom->image[conv[t][0]][conv[t][1]+signy*i] = 0;
            }
        }
        else
        {
            if(dx<0)
            {
                dx = -dx;
                signx = -1;
            }

            gap = (double)dx/(rom->split);

            for(int i=0; i<=(rom->split); i++){
                double f;
                int px,py;

                f = (double)signx*dy*(i*signx*gap)/dx + conv[t][1];

                px = (int)std::round(conv[t][0] + i*signx*gap);
                py = (int)std::round(f);

                rom->image[px][py] = 0;
            }
        }

    }
}


vector<alphashape> restrict_ashape(vector<alphashape>& ashape_all, double rst)     //ashape ROMの制限
{
    vector<alphashape> ashape_rst_all;
    for(size_t t=0; t<ashape_all.size(); t++)
    {
        double minx,maxx,miny,maxy,seedy;
        minx = miny = DBL_MAX;
        maxx = maxy = DBL_MIN;

        for(size_t v=0; v<ashape_all[t].vertices.size(); v++){
            if(minx > ashape_all[t].vertices[v].x){
                minx = ashape_all[t].vertices[v].x;
                seedy = ashape_all[t].vertices[v].y;
            }
            if(maxx < ashape_all[t].vertices[v].x)  maxx = ashape_all[t].vertices[v].x;
            if(miny > ashape_all[t].vertices[v].y)  miny = ashape_all[t].vertices[v].y;
            if(maxy < ashape_all[t].vertices[v].y)  maxy = ashape_all[t].vertices[v].y;
        }

        //BoundingBox中心からrst倍の制限をかける
        vector<Vector2D> ashape_rst;
        double cenx,ceny;
        cenx = (minx+maxx)/2;
        ceny = (miny+maxy)/2;
        for(size_t i=0; i<ashape_all[t].vertices.size(); i++)
        {
            double rstx,rsty;
            rstx = cenx + rst*(ashape_all[t].vertices[i].x - cenx);
            rsty = ceny + rst*(ashape_all[t].vertices[i].y - ceny);
            Vector2D tmpvec(rstx,rsty);
            ashape_rst.push_back(tmpvec);
        }

        vector<vector<int>> conv;
        vector<vector<int>> conv_rst;

//        for(int i=0; i<ashape_all[t].vertices.size();i++){
//            DH_LOG(QString::number(ashape_all[t].vertices[i].x)+","+QString::number(ashape_all[t].vertices[i].y),0);
//        }

        csv_to_pgm(conv, ashape_all[t].vertices, minx, miny, maxx, maxy);
        csv_to_pgm(conv_rst,ashape_rst, minx, miny, maxx, maxy);

        PGM *OrgROM = new PGM(255,WIDTH,HEIGHT);    //PGMオブジェクト生成
        PGM *ResROM = new PGM(255,WIDTH,HEIGHT);

        ConnectingPoints(conv, OrgROM);     //点群を線でつなぎ，
        ConnectingPoints(conv_rst,ResROM);

        OrgROM->fill_image(0);      //中身を塗りつぶす
        ResROM->fill_image(0);

//　確認するために，画像ファイルへの出力を行う部分．通常時は遅くなるためコメントアウト
//        PGM* OverWrap = new PGM(255,WIDTH,HEIGHT);
//        for(int row=0; row<OverWrap->width; row++){
//            for(int col=0; col<OverWrap->height; col++){
//                if(OrgROM->image[row][col] == 0)    OverWrap->image[row][col] = 0;
//                if(ResROM->image[row][col] == 0)    OverWrap->image[row][col] = 125;
//            }
//        }
//        QString head = "C:\\kenkyu\\GraspYA\\ashape_rst\\pgm\\";
//        QString middle = (ashape_all[t].path.remove("C:\\kenkyu\\GraspYA\\ashape\\simple\\")).remove(".csv");
//        QString tail = ".pgm";
//        QString path_pgm = head+middle+tail;
//        OverWrap->save_image(path_pgm.toStdString());
//        delete OverWrap;
//ここまで

        PGM *ANDROM = new PGM(255,WIDTH,HEIGHT);

        for(int col=0; col<OrgROM->height; col++){
            for(int row=0; row<OrgROM->width; row++){
                if(OrgROM->image[row][col] == 0 && ResROM->image[row][col] == 0)
                    ANDROM->image[row][col] = 0;
            }
        }

        vector<array<int,2>> edge;
        edge = ANDROM->edge_extract();

        vector<Vector2D> csvize;    //この変数に再構成
        int split_csv = edge.size()/vertex_number;

        pgm_to_csv(edge, csvize, minx, miny, maxx, maxy, split_csv);


//　確認するために，csvファイルへの出力を行う部分．通常時は遅くなるためコメントアウト
//        QString top = "C:\\kenkyu\\GraspYA\\ashape_rst\\csv\\";
//        QString bottom = ashape_all[t].path.remove("C:\\kenkyu\\GraspYA\\ashape\\simple\\");

//        string csv_path = (top+bottom+".csv").toStdString();
//        std::ofstream ofs(csv_path);

//        for(size_t ii=0; ii<csvize.size(); ii++){   //制限ROM
//            ofs << std::to_string(csvize[ii].x) << ',' << std::to_string(csvize[ii].y) << std::endl;
//        }
//        ofs << std::endl << std::endl;
//        for(size_t jj=0; jj<ashape_all[t].vertices.size(); jj++){   //元ROM
//            ofs << std::to_string(ashape_all[t].vertices[jj].x) << ',' << std::to_string(ashape_all[t].vertices[jj].y) << std::endl;
//        }
//ここまで


        alphashape tmp;
        tmp.path = ashape_all[t].path;
        tmp.vertices = csvize;
        ashape_rst_all.push_back(tmp);

        delete OrgROM;
        delete ResROM;
        delete ANDROM;
    }

    return ashape_rst_all;
}

void prepare_romeval(vector<vector<QString>>& joints_list, vector<vector<QString>>& joint_bone,
                     vector<vector<QString>>& DF, vector<alphashape>& ashape_rst_all, int age){

    double rst = cal_res(age);  //年齢に応じた制限率計算

// ===========================
// joint_relation.csvの読み込み
// ===========================

    QString list = "C:\\kenkyu\\GraspYA\\data\\joint_relation.csv";

    Csv objJL(list);
    if(!objJL.getCsv(joints_list)){
        DH_LOG("failed to read the file",0);
        return ;
    }

    joints_list.erase(joints_list.begin());		//先頭行削除


// =======================
// joint_bone.csvの読み込み
// =======================

    QString bone = "C:\\kenkyu\\GraspYA\\data\\joint_bone.csv";

    Csv objJB(bone);
    if(!objJB.getCsv(joint_bone)){
        DH_LOG("failed to read the file",0);
        return ;
    }

    joint_bone.erase(joint_bone.begin());

// =======================================
// assembledActiveDF01_maxmin.csvの読み込み
// =======================================

    QString add = "C:\\kenkyu\\GraspYA\\data\\assembledActiveDF01_health_maxmin.csv";

    Csv objDF(add);
    if(!objDF.getCsv(DF)){
        DH_LOG("failed to read the file",0);
        return ;
    }

    restrict_ROM(DF, rst);      //一次元ROMの制限


//==================================
// ashapeの読み込み
//==================================
    vector<alphashape> ashape_all;
    for(int i=0; i<joints_list.size(); i++){
        QString joint1 = joints_list[i][0];
        QString joint2 = joints_list[i][1];

        QString ashape_frame = "C:\\kenkyu\\GraspYA\\ashape\\simple\\ashape_simple_";
        QString extension = ".csv";
        QString ashape_file = ashape_frame + joint1 + '_' + joint2 + extension;
        vector<vector<QString>> ashape_string;
        vector<Vector2D> vertex;

        Csv obj_ashape(ashape_file);
        if(!obj_ashape.getCsv(ashape_string)){
            DH_LOG("failed to read the file5",0);
            return ;
        }

        for(unsigned int row=1; row<ashape_string.size(); row++){
            vector<QString> buf = ashape_string[row];
            Vector2D rec(buf[1].toDouble(),buf[2].toDouble());
            vertex.push_back(rec);
        }

        alphashape tmp;
        tmp.path = ashape_file;
        tmp.vertices = vertex;
        ashape_all.push_back(tmp);
    }

    ashape_rst_all = restrict_ashape(ashape_all,rst);    //ashape ROMの制限
}




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
            flag = 1;
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
    for(int l1=0; l1<=xy.size()-2; l1++){
		int l2 = l1 + 1;

        Vector2D position_edge1 = xy[l1];
        Vector2D position_edge2 = xy[l2];

//============================
// Crossing Number Algorithm
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

double rom_eval(dhArmature* arm, vector<vector<QString>> joints_list,
                vector<vector<QString>> joint_bone, vector<vector<QString>> DF,
                vector<alphashape> ashape_all){
    QString bones[] = {"TMCP","TPP","TDP","IPP","IMP","IDP","MPP","MMP","MDP","RPP","RMP","RDP","PPP","PMP","PDP"};
//    string bones[] = {"TMCP","TPP","TDP","IPP","IMP","IDP","MPP","MMP","MDP","RPP","RMP","RDP","PPP","PMP","PDP"};


    vector<vector<QString>> rotation;
    double min_length_sum = 0.0;


    for(int boneIndex=0; boneIndex < sizeof(bones)/sizeof(bones[0]); boneIndex++){
        dhBone *bone = arm->bone(bones[boneIndex]);
        dhVec4 r[2];
        dhMat44 mat=bone->R;        //初期姿勢→現在姿勢の変換行列
        mat.getRPYAngle(r[0],r[1]); //解が二つ出るが，ここでは0の方のみ書き出すことにする

        double deg_x = r[0].p[0] / (3.141592) * 180.0;
        double deg_y = r[0].p[1] / (3.141592) * 180.0;
        double deg_z = r[0].p[2] / (3.141592) * 180.0;


        vector<QString> line;

        line.push_back(bones[boneIndex]);
        line.push_back(QString::number(deg_x));
        line.push_back(QString::number(deg_y));
        line.push_back(QString::number(deg_z));

        rotation.push_back(line);       //骨別の現在の関節角度

	}


#pragma omp parallel for default(private) reduction(+:min_length_sum)
	for(int i=0; i<joints_list.size(); i++){

        QString joint1 = joints_list[i][0];
        QString joint2 = joints_list[i][1];

        double rot1,rot2;

		for(int j=0; j<joint_bone.size(); j++){
			if(joint_bone[j][0] == joint1){
				for(int k=0; k<rotation.size(); k++){
					if(rotation[k][0] == joint_bone[j][1]){
                        int axis = joint_bone[j][2].toInt();
                        rot1 = rotation[k][axis+1].toDouble();
					}
				}
			}
			if(joint_bone[j][0] == joint2){
				for(int k=0; k<rotation.size(); k++){
					if(rotation[k][0] == joint_bone[j][1]){
                        int axis = joint_bone[j][2].toInt();
                        rot2 = rotation[k][axis+1].toDouble();
					}
				}
			}
        }

        Vector2D position_pos(rot1,rot2);   //joints_listによって定められた関節の組み合わせに応じて
                                            //現在の関節角度組み合わせを定義する．
//        DH_LOG(QString::number(rot1)+","+QString::number(rot2),0);

        vector<Vector2D> ashape;    //ashapeを構成する頂点データ
        for(int a=0; a<ashape_all.size(); a++){
            if(ashape_all[a].path.contains(joint1, Qt::CaseInsensitive) &&
                    ashape_all[a].path.contains(joint2, Qt::CaseInsensitive)){
                ashape = ashape_all[a].vertices;
            }
        }
//        for(int i=0 ;i<ashape.size();i++){
//            DH_LOG(QString::number(ashape[i][0])+","+QString::number(ashape[i][1]),0);
//        }


        double min_pos_to_edge_length;

		min_pos_to_edge_length = calc_distance(ashape,position_pos);

		int inside_num = 0;
		int in_out = 1;

		in_or_out(ashape,position_pos,inside_num,in_out);

		if(in_out != 0 && inside_num % 2 == 0){
			in_out = 0;
//            DH_LOG("Outside ROM",0);
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
//        DH_LOG(joints_list[i][0]+","+joints_list[i][1]+":"+QString::number(min_length),0);
//        DH_LOG("   ",0);

		min_length_sum += min_length * min_length;
	}


#pragma omp for default(private) reduction(+:min_length_sum)
    for(int col=0; col<joint_bone.size(); col++){
        double min_length;
        double min,max,rot;

        for(int col2=0; col2<DF.size(); col2++){
            if(joint_bone[col][0] == DF[col2][0]){
                max = DF[col2][1].toDouble();
                min = DF[col2][2].toDouble();
            }
        }

        QString name = joint_bone[col][1];
        int axis = joint_bone[col][2].toInt();


        for(int ii=0; ii<rotation.size(); ii++){        //現在の関節角度をrotに格納
            if(rotation[ii][0] == name){
                rot = rotation[ii][axis+1].toDouble();
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
//        DH_LOG(joint_bone[col][0]+":"+QString::number(min_length),0);

		min_length_sum += min_length * min_length;
	} 

    double Rom_eval = min_length_sum/(joints_list.size() + joint_bone.size());

    return Rom_eval;
}

