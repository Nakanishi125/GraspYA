    else if(cmd == "coordinate evaluation"){
        bool isOK;
        IDHElement* e=dhApp::elementSelectionDialog(dhFeaturePoints::type,&isOK);
        dhFeaturePoints* Fp = dynamic_cast<dhFeaturePoints*>(e);
        float co_eva;
        clock_t time1,time2;
        time1 = clock();
        co_eva = coord_eval(Fp);
        time2 = clock();
        double etime = (double)(time2-time1)/1000;
        DH_LOG("elapsed time is "+QString::number(etime,'f',5),0);
    }
    else if(cmd == "collision evaluation"){
        clock_t time1,time2;
        time1 = clock();
        bool isOK;
        IDHElement* e1 = dhApp::elementSelectionDialog(dhSkeletalSubspaceDeformation::type,&isOK);
        dhSkeletalSubspaceDeformation* mesh1 = dynamic_cast<dhSkeletalSubspaceDeformation*>(e1);
        IDHElement* e2 = dhApp::elementSelectionDialog(dhMesh::type,&isOK);
        dhMesh* mesh2 = dynamic_cast<dhMesh*>(e2);
        IDHElement* e3 = dhApp::elementSelectionDialog(dhArmature::type,&isOK);
        dhArmature* arm = dynamic_cast<dhArmature*>(e3);

        map<int,string> bone_index;
        bone_index[0] = "ROOT";     bone_index[1] = "CP";       bone_index[2] = "TMCP";
        bone_index[3] = "TPP";      bone_index[4] = "TDP";      bone_index[5] = "IMCP";
        bone_index[6] = "IPP";      bone_index[7] = "IMP";      bone_index[8] = "IDP";
        bone_index[9] = "MMCP";     bone_index[10] = "MPP";     bone_index[11] = "MMP";
        bone_index[12] = "MDP";     bone_index[13] = "RMCP";    bone_index[14] = "RPP";
        bone_index[15] = "RMP";     bone_index[16] = "RDP";     bone_index[17] = "PMCP";
        bone_index[18] = "PPP";     bone_index[19] = "PMP";     bone_index[20] = "PDP";

        int points1 = 10000;
        int points2 = 10000;

        dhMat44 M1,M2,M3,M4;        //中指の長さをhand_lengthとする
        M1 = arm->bone(9)->Tpj0;    //MMCP
        M2 = arm->bone(10)->Tpj0;   //MPP
        M3 = arm->bone(11)->Tpj0;   //MMP
        M4 = arm->bone(12)->Tpj0;   //MDP

        dhVec3 m1(M1[12],M1[13],M1[14]);    //右三つ(0~2行 3列目)抽出
        dhVec3 m2(M2[12],M2[13],M2[14]);    //右三つ(0~2行 3列目)抽出
        dhVec3 m3(M3[12],M3[13],M3[14]);    //右三つ(0~2行 3列目)抽出
        dhVec3 m4(M4[12],M4[13],M4[14]);    //右三つ(0~2行 3列目)抽出

        double hand_length = m1.norm() + m2.norm() + m3.norm() + m4.norm();
//        DH_LOG("hand_lenth is "+QString::number(hand_length,'f',5),0);

        dhPointCloudAsVertexRef* bodyPoints;
        dhPointCloudAsVertexRef* objectPoints;
        bodyPoints = dhnew<dhPointCloudAsVertexRef>();
        objectPoints = dhnew<dhPointCloudAsVertexRef>();

        computeContactRegion(bodyPoints,objectPoints,mesh1,mesh2,points1,points2);

        dhBone* root;
        root = dhnew<dhBone>();
        vector<QString> bones;
        vector<int>     depths;
        getArmatureStructure(arm,bones,depths,root);

        segment* points_obj;
        points_obj = new segment[bones.size()];
        vector<dhVec3> cog_obj;
        vector<int> keys_obj;

        segmentObjectPoints(objectPoints,points_obj,arm,bones);

        for(int index=0; index<bones.size(); index++){      //getCoGsに相当
            points_obj[index].getCoGs(points_obj,cog_obj,index,keys_obj);
        }

//        for(int k=0; k<cog_obj.size(); k++){          // CoGとそのbone名を出力(物体側)
//            dhVec3 tmp = cog_obj[k];
//            QString s = QString::fromStdString(bone_index[keys_obj[k]]);
//            QString x = QString::number(tmp[0],'f',5);
//            QString y = QString::number(tmp[1],'f',5);
//            QString z = QString::number(tmp[2],'f',5);
//            DH_LOG("bone:"+s+" x="+x+" y="+y+" z="+z,0);
//        }

        segment* points_hand;
        points_hand = new segment[bones.size()];
        vector<dhVec3> cog_hand;
        vector<int> keys_hand;

        segmentObjectPoints(bodyPoints,points_hand,arm,bones);

        for(int index=0; index<bones.size(); index++){      //getCoGsに相当
            points_hand[index].getCoGs(points_hand,cog_hand,index,keys_hand);
        }

//        for(int k=0; k<cog_hand.size(); k++){         // CoGとそのbone名を出力(ハンドモデル側)
//            dhVec3 tmp = cog_hand[k];
//            QString s = QString::fromStdString(bone_index[keys_obj[k]]);
//            QString x = QString::number(tmp[0],'f',5);
//            QString y = QString::number(tmp[1],'f',5);
//            QString z = QString::number(tmp[2],'f',5);
//            DH_LOG("bone:"+s+" x="+x+" y="+y+" z="+z,0);
//        }


        vector<int> keys_hand_orig = keys_hand;

        int sub=0;
        //ハンドモデルについて干渉点が10点以下の場合，keys_handとcog_handから除外する
        for(int bone_num=0; bone_num < keys_hand_orig.size(); bone_num++){
            if(points_hand[keys_hand_orig[bone_num]].ObjectPoints.size() < 10){
                int inc = bone_num - sub;
                sub++;
                keys_hand.erase(keys_hand.begin() + inc);
                cog_hand.erase(cog_hand.begin() + inc);
//                for(int b =0; b<keys_hand.size(); b++){       //削除したbone_numと削除後に残ったkeys_handの確認
//                    DH_LOG(QString::number(bone_num)+" "+QString::number(keys_hand[b]),0);
//                }
            }
        }


        //ハンドモデルについて，干渉点から物体表面までの距離が10mm未満の場合にはその点を除外する
        vector<vector<dhVec3>> points_hand2;

        for(int i=0; i<keys_hand.size(); i++){
            double min_length;
            int sub2 = 0;
            vector<dhVec3> points = points_hand[keys_hand[i]].ObjectPoints;
            for(int j=0; j<points.size(); j++){
                dhVec3 point = points[j];
                for(int k=0; k<keys_obj.size(); k++){
                    vector<dhVec3> obj_vecs = points_obj[keys_obj[k]].ObjectPoints;
                    for(int l=0; l<obj_vecs.size(); l++){
                        dhVec3 obj_vec = obj_vecs[l];
                        double length = (obj_vec - point).norm();
                        if(l == 0 && k == 0){     min_length = length;}
                        else{
                            if(min_length > length){
                                min_length = length;
                            }
                        }
                    }
                }

                if(min_length < 5){
                    int inc = j - sub2++;
                    points.erase(points.begin() + inc);
                }
            }
            points_hand2.push_back(points);
        }


        for(int cnt=0; cnt<points_hand2.size(); cnt++){       //points_hand2の中身チェック
            DH_LOG("Size is "+QString::number(points_hand2[cnt].size()),0);
            for(int jj=0; jj<points_hand2[cnt].size(); jj++){
                dhVec3 tmp = points_hand2[cnt][jj];
                QString x = QString::number(tmp[0],'f',7);
                QString y = QString::number(tmp[1],'f',7);
                QString z = QString::number(tmp[2],'f',7);
               // DH_LOG("x="+x+" y="+y+" z="+z,0);
                DH_LOG("["+x+","+y+","+z+"],",0);
            }
        }

//========================================
//PCLを使ってConvexHull作成
//========================================


        for(int bone=0; bone<points_hand2.size(); bone++){
            pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);

            for(int j=0; j<points_hand2[bone].size(); j++){
                pcl::PointXYZ buf(points_hand2[bone][j][0],points_hand2[bone][j][1],points_hand2[bone][j][2]);
                Cloud->push_back(buf);
            }

//            for(int cnt =0; cnt<Cloud->size(); cnt++){        //中身確認
//                QString x = QString::number(Cloud->points[cnt].x,'f',7);
//                QString y = QString::number(Cloud->points[cnt].y,'f',7);
//                QString z = QString::number(Cloud->points[cnt].z,'f',7);
//                DH_LOG("x="+x+" y="+y+" z="+z,0);
//            }

            pcl::ConvexHull<pcl::PointXYZ>::Ptr CHull(new pcl::ConvexHull<pcl::PointXYZ>);
            CHull->setInputCloud(Cloud);
            CHull->setDimension(3);
            CHull->setComputeAreaVolume(true);

            pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
            CHull->reconstruct(*hull);

//            DH_LOG("          ",0);
//            for(int cnt =0; cnt<hull->size(); cnt++){        //中身確認
//                QString x = QString::number(hull->points[cnt].x,'f',7);
//                QString y = QString::number(hull->points[cnt].y,'f',7);
//                QString z = QString::number(hull->points[cnt].z,'f',7);
//                DH_LOG("x="+x+" y="+y+" z="+z,0);
//            }

            double Volume = CHull->getTotalVolume();
            DH_LOG("Volume is"+QString::number(Volume,'f',5),0);

        }





//================================================
//Qhull.orgを用いてpoints_hand2からvertexBuffer生成
//================================================
//        double Volume = 0;

        for(int bone=0; bone<points_hand2.size(); bone++){
            double Volume = 0;
            orgQhull::Qhull qhull;
            orgQhull::PointCoordinates points;
            vector<double> m_convex_vertexes;

            //3次元を扱う
            points.setDimension(3);

            //qhullに渡す頂点情報を作成
            //配列の中身は{x0, y0, z0, x1, y1, z1, x2, y2, ...}となる
            //QhullではcoorT型を使うが、実質double
            std::vector<coordT> allpoint;

            for(int cnt=0; cnt<points_hand2[bone].size(); cnt++){
                allpoint.push_back(points_hand2[bone][cnt][0]);
                allpoint.push_back(points_hand2[bone][cnt][1]);
                allpoint.push_back(points_hand2[bone][cnt][2]);
            }

            points.append( allpoint);

            //qhullの実行
            qhull.runQhull(points.comment().c_str(), points.dimension(),
                           points.count(), points.coordinates(), "Qt");

            //qhullの結果から面を取得
            orgQhull::QhullFacetList faceList = qhull.facetList();

            for (auto itr = faceList.begin(); itr != faceList.end(); itr++) {

                //面から頂点情報を取得
                orgQhull::QhullVertexSet vset = (*itr).vertices();

                //点の数に対してループ
                for (auto vitr = vset.begin(); vitr != vset.end(); vitr++) {
                    orgQhull::QhullPoint p = (*vitr).point();
                    double * coords = p.coordinates();
                    m_convex_vertexes.push_back(coords[0]);     //x
                    m_convex_vertexes.push_back(coords[1]);     //y
                    m_convex_vertexes.push_back(coords[2]);     //z
                }
            }

            set<vector<double>> vertexBuffer;
            for(int c=0; c<(m_convex_vertexes.size())/3; c++){
                vector<double> tmp = {m_convex_vertexes[3*c],m_convex_vertexes[3*c+1],m_convex_vertexes[3*c+2]};
                vertexBuffer.insert(tmp);
            }


            DH_LOG("QSize is "+QString::number(vertexBuffer.size()),0);    //QHull後の点群座標出力
            for(set<vector<double>>::iterator itr = vertexBuffer.begin();
                itr != vertexBuffer.end(); itr++){
                vector<double> tmpvec = *itr;
                QString x = QString::number(tmpvec[0],'f',7);
                QString y = QString::number(tmpvec[1],'f',7);
                QString z = QString::number(tmpvec[2],'f',7);
//                DH_LOG("x="+x+" y="+y+" z="+z,0);
                DH_LOG("["+x+","+y+","+z+"],",0);
            }

//#####################################################################################################################################
////=====================================
////points_hand2からConvexHull形成(**)
////=====================================

////        double Volume = 0;
//        for(int bone=0; bone<points_hand2.size(); bone++){
//            double Volume = 0;
//            vector<quickhull::Vector3<float>> pointCloud;
//            for(int i=0; i<points_hand2[bone].size(); i++){       //QHullスタイルに合わせてpoints_hand2をqhullに合ったベクトルクラスに
//                quickhull::Vector3<float> tmpvec(points_hand2[bone][i][0],
//                                                 points_hand2[bone][i][1],
//                                                 points_hand2[bone][i][2]);
//                pointCloud.push_back(tmpvec);
//            }

////            for(int cnt=0; cnt<pointCloud.size(); cnt++){     //pointCloud中身チェック
////                uintptr_t pt = (uintptr_t)&pointCloud[cnt];
////                QString s = QString::number(pt);
////                DH_LOG(s,0);
////            }

//            quickhull::QuickHull<float> qh;
//            quickhull::ConvexHull<float> hull = qh.getConvexHull(pointCloud,true,false);

////            const vector<size_t>& indexBuffer = hull.getIndexBuffer();      //採用されたindexを取得
////            for(int i=0; i<indexBuffer.size(); i++){                        //採用されたindexの表示
////                DH_LOG(QString::number(indexBuffer[i]),0);
////            }

////=====================================
////凸多面体化後の頂点データがvertexBufferに
////=====================================

//            const quickhull::VertexDataSource<float>& vertexBuffer = hull.getVertexBuffer();


//            DH_LOG("QSize is "+QString::number(vertexBuffer.size()),0);    //QHull後の点群座標出力
//            for(size_t j=0; j<vertexBuffer.size(); j++){
//                quickhull::Vector3<float> tmpvec = vertexBuffer[j];
//                QString x = QString::number(tmpvec.x);
//                QString y = QString::number(tmpvec.y);
//                QString z = QString::number(tmpvec.z);
//                DH_LOG("x="+x+" y="+y+" z="+z,0);
//            }
//#######################################################################################################################################


//==================
//  ここからDelaunay
//==================

            set<Delaunay::Vector> tmpVertexSet;

            double minX, minY, minZ;
            double maxX, maxY, maxZ;
            minX = minY = minZ = DBL_MAX;
            maxX = maxY = maxZ = DBL_MIN;

//            for(size_t j=0; j<vertexBuffer.size(); j++){       //   型変換     //ここ(**)
//                vector<double> tmpvec = vertexBuffer[j];                      //ここ(**)
            for(set<vector<double>>::iterator itr = vertexBuffer.begin();
                    itr != vertexBuffer.end(); itr++){

                vector<double> tmpvec = *itr;

                Delaunay::Vector dlnvec;
//                dlnvec.x = tmpvec.x;                  //  ここ(**)
//                dlnvec.y = tmpvec.y;                  //　ここ(**)
//                dlnvec.z = tmpvec.z;                  //　ここ(**)
                dlnvec.x = tmpvec[0];
                dlnvec.y = tmpvec[1];
                dlnvec.z = tmpvec[2];


                tmpVertexSet.insert(dlnvec);          //頂点データどんどん代入
            }


            Delaunay::Delaunay3d::ConstVertexSet pVertexSet = tmpVertexSet;
            Delaunay::Delaunay3d::TetraSet tetraSet;

            Delaunay::Delaunay3d::getDelaunayTriangles(pVertexSet,tetraSet);


            DH_LOG("size of tetrahedron is"+QString::number(tetraSet.size()),0);
            for(Delaunay::Delaunay3d::TetraIter tIter = tetraSet.begin();
               tIter != tetraSet.end(); tIter++){

                for(int i=0; i<4; i++){            //分割四面体の中身
                    QString s = QString::number(i);
                    QString x = QString::number((*tIter).p[i]->x,'f',5);
                    QString y = QString::number((*tIter).p[i]->y,'f',5);
                    QString z = QString::number((*tIter).p[i]->z,'f',5);
                    DH_LOG("vertex["+s+"]:x="+x+" y="+y+" z="+z,0);
                }

                //ここから四面体の体積求める
                Delaunay::Vector OA = *((*tIter).p[1]) - *((*tIter).p[0]);
                Delaunay::Vector OB = *((*tIter).p[2]) - *((*tIter).p[0]);
                Delaunay::Vector OC = *((*tIter).p[3]) - *((*tIter).p[0]);

                Volume += abs(OA.dot(OB.cross(OC))) / 6;
                DH_LOG("volume is "+QString::number(Volume,'f',5),0);
            }

        DH_LOG("final volume is "+QString::number(Volume,'f',5),0);
//        pointCloud.clear();                               //ここ(**)
        }




        delete[] points_obj;
        delete[] points_hand;

        dhdelete(root);

        dhdelete(bodyPoints);
        dhdelete(objectPoints);

        time2 = clock();
        double etime = (double)(time2-time1)/1000;
        DH_LOG("elapsed time is "+QString::number(etime,'f',5),0);




    }