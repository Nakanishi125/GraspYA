#ifndef PGMLIB_H
#define PGMLIB_H

#include<string>
#include<vector>
#include<array>
#include<stack>

//定数宣言
const int MAX_IMAGESIZE = 3000; // 想定する縦・横の最大画素数 
const int MAX_BRIGHTNESS = 255; // 想定する最大階調値 
const int GRAYLEVEL = 256; // 想定する階調数(=最大階調値+1)
const int MAX_FILENAME = 256; // 想定するファイル名の最大長 
const int MAX_BUFFERSIZE = 256; // 利用するバッファ最大長 

class PGM
{    
public:
    // 画像データ image[x][y] 
    unsigned char image[MAX_IMAGESIZE][MAX_IMAGESIZE];
    // image の横幅・縦幅  
    int width, height;
    //　背景色
    unsigned char back_col;

    int split;
    //コンストラクタ
    PGM( unsigned char value, int row, int col);

    // 階調画像を入力する関数 
    // void load_image( std::string path );

    // 階調画像を出力する関数
    void save_image( std::string path );

    //　輪郭線の内部を塗りつぶす関数
    void fill_image(unsigned char color);

    // エッジ検出する関数
    std::vector<std::array<int,2>> edge_extract();

private:

    void solve_coordinates(int cx, int cy, int &dx, int &dy, int n);

    void eight_direction_search(int cx, int cy, int &nx, int &ny, int olddir, int &newdir);

    void scan_line(unsigned char color, int lx, int rx, int y, std::stack<std::array<int,2>> &stk);

    void find_seed(int &seedx, int &seedy);

};


#endif
