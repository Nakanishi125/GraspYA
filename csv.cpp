#include<iostream>
#include<string>
#include<vector>
#include<fstream>
#include<sstream>
#include"csv.hpp"

#include<QString>
#include<QFile>
#include<QTextStream>
#include<QList>

using namespace std;

Csv::Csv(string name){
   this -> Csv_file = name;
}

Csv::Csv(QString qname){
    this -> Csv_file2 = qname;
}

bool Csv::getCsv(vector<vector<string>>& mat, const char delim){
    ifstream ifs(Csv_file);
    if(!ifs.is_open())   return false;

    string buf; //buffer 1 line
    while(getline(ifs,buf)){
        vector<string> rec;     // 1 line vector
        istringstream iss(buf); //Convert to stringstream
        string field;           // string in 1 line

        while(getline(iss,field,delim)){    // ','まで読み込む
            rec.push_back(field);           // rec に　fieldを追加
        }
        
        if(rec.size() != 0){
            mat.push_back(rec);     //add 1 line
        }
    }
    
    return true;
}

bool Csv::getCsv(vector<vector<QString>>& mat, const char delim){
    QFile file(Csv_file2);

    if(!file.open(QIODevice::ReadOnly)){
        DH_LOG("cannot open the input file",0);
        return false;
    }

    QTextStream in(&file);
    while(!in.atEnd()){
        QString tmp = in.readLine();
        vector<QString> tmpMat;
        QStringList tmplst = tmp.split(',');
        for(int index=0;index<tmplst.size();index++){
            tmpMat.push_back(tmplst[index]);
        }
        mat.push_back(tmpMat);
    }

    file.close();

    return true;
}

bool Csv::getCsvdb(vector<vector<double>>& mat2, const char delim){
    ifstream ifs(Csv_file);
    if(!ifs.is_open())   return false;

    string buf; //buffer 1 line
    while(getline(ifs,buf)){
        vector<double> rec;     // 1 line vector
        istringstream iss(buf); //string stream 
        string field;           // 1 line string
        double num;

        while(getline(iss,field,delim)){
            stringstream fid(field);
            fid >> num;
            rec.push_back(num);
        }
        
        if(rec.size() != 0){
            mat2.push_back(rec);     //add 1line
        }
    }
    
    return true;
}

template <class Type> void Confirm_Matrix(vector<vector<Type>> Mat){
    for(int row=0; row<Mat.size(); row++){		//Matrixの中を見る
        vector<Type> rec = Mat[row];
        for(int col=0; col < rec.size(); col++){
            cout << rec[col];
            if(col < rec.size() -1)	cout << ",";
        }
        cout << endl;
    }
}
