#ifndef INCLUDED_CSV_HPP
#define INCLUDED_CSV_HPP

#include<string>
#include<vector>

#include<QString>
#include<QFile>
#include<QTextStream>
#include<QList>

class Csv{
    std::string Csv_file;
    QString Csv_file2;

public:
    Csv(std::string name);      //constructor

    Csv(QString qname);

    bool getCsv(std::vector<std::vector<std::string>>& mat, const char delim = ',');    //csvより文字列で読み込み

    bool getCsv(std::vector<std::vector<QString>>& mat, const char delim = ',');    //csvより文字列で読み込み

    bool getCsvdb(std::vector<std::vector<double>>& mat2, const char delim = ',');      //csvよりdouble型で読み込み

    template <class Type> void Confirm_Matrix(std::vector<std::vector<Type>> Mat);
    
};

#endif
