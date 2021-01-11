#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<vector>
#include<array>
#include<stack>

#include "pgmlib.h"
   
PGM::PGM( unsigned char value, int row, int col)
:width(row), height(col), back_col(value)
{
    split = (width>height)?width:height;

    for(int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            image[x][y] = back_col;
        }
    }

}
   
// void PGM::load_image( int n, std::string path )
// {   
//     char buffer[MAX_BUFFERSIZE];  /* データ読み込み用作業変数 */
    
    
//     int max_gray; /* 最大階調値 */

//     // /* 入力ファイルのオープン */
//     // if ( name[0] == '\0' ){
//     //     printf("入力ファイル名 (*.pgm) : ");
//     //     scanf("%s",file_name);
//     // } else strcpy( file_name, name );
//     // if ( (fp = fopen( file_name, "rb" ))==NULL ){
//     //     printf("その名前のファイルは存在しません．\n");
//     //     exit(1);
//     // }

//     /* ファイルタイプ(=P5)の確認 */
//     std::ifstream ifs(path);
//     std::string P5;
//     getline(ifs,P5);
//     if(P5!="P5")
//     {
//         std::cout << "This filetype is not P5" << std::endl;
//         exit(1);
//     }

//     /* width[n], height[n] の代入（#から始まるコメントは読み飛ばす） */
//     width[n] = 0;    height[n] = 0;
//     while ( width[n] == 0 || height[n] == 0 ){
//         fgets( buffer, MAX_BUFFERSIZE, fp );
//         if ( buffer[0] != '#' )
//             sscanf( buffer, "%d %d", &width[n], &height[n] );
//     }
//     /* max_gray の代入（#から始まるコメントは読み飛ばす） */
//     max_gray = 0;
//     while ( max_gray == 0 ){
//         fgets( buffer, MAX_BUFFERSIZE, fp );
//         if ( buffer[0] != '#' )
//             sscanf( buffer, "%d", &max_gray );
//     }
//     /* パラメータの画面への表示 */
//     printf("横の画素数 = %d, 縦の画素数 = %d\n", width[n], height[n] );
//     printf("最大階調値 = %d\n",max_gray);
//     if ( width[n] > MAX_IMAGESIZE || height[n] > MAX_IMAGESIZE ){
//         printf("想定値 %d x %d を超えています．\n", 
//             MAX_IMAGESIZE, MAX_IMAGESIZE);
//         printf("もう少し小さな画像を使って下さい．\n");
//         exit(1);
//     }
//     if ( max_gray != MAX_BRIGHTNESS ){
//         printf("最大階調値が不適切です．\n");  exit(1);
//     }
//     /* 画像データを読み込んで画像用配列に代入する */
//     for(y=0;y<height[n];y++)
//         for(x=0;x<width[n];x++)
//             image[n][x][y] = (unsigned char)fgetc( fp );
//     fclose(fp);
//     printf("画像は正常に読み込まれました．\n");
//}

// 階調画像を出力する関数
void PGM::save_image( std::string path )
{
    // 出力ファイルのオープン 
    std::ofstream ofs(path);
    if (!ofs)
    {
        std::cout << "ファイルが開けませんでした." << std::endl;
        return;
    }

    ofs << "P5" << std::endl;

    // 画像の横幅，縦幅の出力 
    ofs << width << ' ' << height << std::endl;
    // 最大階調値の出力 
    ofs << MAX_BRIGHTNESS << std::endl;

    // 画像データの出力 
    for(int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            ofs << image[x][y];
        }
    }

    std::cout<< path << " is created." << std::endl;

}



void PGM::fill_image(unsigned char color)
{
    int x,y;
    find_seed(x,y);
    int lx,rx,nowy;
    std::stack<std::array<int,2>> stk;
    std::array<int,2> tmp = {x,y};
    stk.push(tmp);

    do
    {
        std::array<int,2> pick = stk.top();
        lx = rx = pick[0];
        nowy = pick[1];
        stk.pop();

        if(image[lx][nowy] != back_col)
        {
            continue;
        }

        while(lx > 0)     //Min側の探索
        {
            if(image[lx-1][nowy] != back_col)    break;
            lx--;
        }
        while(rx < width)   //Max側の探索
        {
            if(image[rx+1][nowy] != back_col)    break;
            rx++;
        }   

        for(int tmpx=lx; tmpx<=rx; tmpx++)    //横一列塗りつぶし
        {
            image[tmpx][nowy] = color;
        }

        if(nowy-1 > 0){
            scan_line(back_col, lx, rx, nowy-1, stk);
        }
        if(nowy+1 < height){
            scan_line(back_col, lx, rx, nowy+1, stk);
        }

    }while(!stk.empty());
}




std::vector<std::array<int,2>> PGM::edge_extract()
{

    //最初の点を見つける
    int inirow,inicol;
    std::vector<std::array<int,2>> edge; 
    int escape=0;
    for(int col=0; col<height; col++){
        for(int row=0; row<width; row++){
            if(image[row][col] == 0)
            {
                inirow = row;
                inicol = col;
                escape = 1;
                break;
            }
        }
        if(escape)  break;
    }

    //初回だけ処理が異なる
    std::array<int,2> arr={inirow,inicol};
    edge.push_back(arr);
    
    int nowx,nowy,nowdir;
    int aftx,afty,aftdir;

    eight_direction_search(inirow,inicol,aftx,afty,5,aftdir);
    arr[0] = aftx;
    arr[1] = afty;
    edge.push_back(arr);

    nowx = aftx;
    nowy = afty;
    if(aftdir%2)
    {
        nowdir = (aftdir+6)%8;
    }
    else
    {
        nowdir = (aftdir+7)%8;
    }

    //繰り返し探索
    while(1){

        eight_direction_search(nowx,nowy,aftx,afty,nowdir,aftdir);

        if(aftx == inirow && afty == inicol)    //一周
        {
            break;
        }

        arr[0] = aftx;
        arr[1] = afty;
        edge.push_back(arr);

        nowx = aftx;
        nowy = afty;
        if(aftdir%2)
        {
            nowdir = (aftdir+6)%8;
        }
        else
        {
            nowdir = (aftdir+7)%8;
        }
    }

    return edge;
}

void PGM::solve_coordinates(int cx, int cy, int &dx, int &dy, int n)
{
    switch(n)
    {
        case 0: dx = cx+1;  dy = cy;    break;
        case 1: dx = cx+1;  dy = cy-1;  break;
        case 2: dx = cx;    dy = cy-1;  break;
        case 3: dx = cx-1;  dy = cy-1;  break;
        case 4: dx = cx-1;  dy = cy;    break;
        case 5: dx = cx-1;  dy = cy+1;  break;
        case 6: dx = cx;    dy = cy+1;  break;
        case 7: dx = cx+1;  dy = cy+1;  break;
    }
}

void PGM::eight_direction_search(int cx, int cy, int &nx, int &ny, int olddir, int &newdir)
{
    for(int t=0; t<8; t++){
        int dx,dy;
        solve_coordinates(cx,cy,dx,dy,(olddir+t)%8);
        if(image[dx][dy] == 0)
        {
            nx = dx;
            ny = dy;
            newdir = (olddir+t)%8;
            break;
        }
    }
}

void PGM::scan_line(unsigned char inter_color, int lx, int rx, int y, std::stack<std::array<int,2>> &stk)
{
    while(lx <= rx)
    {
        for(;lx<=rx; lx++){     //輪郭を構成する画素の右端までスキップ
            if(image[lx][y] == inter_color) break;
        }
        if(image[lx][y] != inter_color)     break;  //上or下の境界線で探索を終了するためのif文
        
        for(;lx<=rx; lx++){     //輪郭内部を構成する画素の右端までスキップ
            if(image[lx][y] != inter_color) break;
        }

        std::array<int,2> temporary={lx-1,y};
        stk.push(temporary);

    }
}


void PGM::find_seed(int &seedx, int &seedy)
{
    int lf = 0;
    int y = height/2;
    while(1){
        int flag = 0;
        std::vector<int> crossx;

        while(lf < width)
        {
            for(; lf < width; lf++){
                if(image[lf][y] != back_col)   break;
            }
            for(; lf < width; lf++){
                if(image[lf][y] == back_col){
                    flag++;
                    crossx.push_back(lf-1);
                    break;
                }
            }
        }

        if(flag%2 == 0){
            seedx = (crossx[0] + crossx[1])/2;
            seedy = y;
            break;
        }
        if(flag%2 == 1){
            crossx.clear();
            y = y - 1;
        }
    }

}
