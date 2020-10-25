#ifndef VECTOR2D
#define VECTOR2D

class Vector2D{
public:
        //メンバ変数
        float x;
        float y;
        
        //constructor
        Vector2D();
        Vector2D(float x,float y);
        //代入演算子
        Vector2D& operator=(const Vector2D& v);
        //単項演算子
        Vector2D& operator+=(const Vector2D& v);
        Vector2D& operator-=(const Vector2D& v);
        Vector2D& operator*=(float k);
        Vector2D& operator/=(float k);
        Vector2D operator+()const;
        Vector2D operator-()const;

        //添え字演算子
        float& operator[](int i);
        //比較演算子
        bool operator==(const Vector2D& v) const;
        bool operator!=(const Vector2D& v) const;
        //ノルム
        float norm()const;
        //正規化
        void normalize();

};
//ベクトル演算
Vector2D operator+(const Vector2D& u,const Vector2D& v);
Vector2D operator-(const Vector2D& u,const Vector2D& v);
Vector2D operator*(const Vector2D& u,float k);
Vector2D operator/(const Vector2D& u,float k);
float operator*(const Vector2D& u,const Vector2D& v);    //内積
float operator%(const Vector2D& u,const Vector2D& v); //外積
float angle(const Vector2D& u,const Vector2D& v);

//*****************************************************************************************//
//ここからは，上で宣言した関数の定義


#include<cmath>
//コンストラクタ
inline Vector2D::Vector2D(){
    this -> x = 0;
    this -> y = 0;
}
inline Vector2D::Vector2D(float x,float y){
    this -> x = x;
    this -> y = y;
}

//代入演算子
inline Vector2D& Vector2D::operator=(const Vector2D& v){
    this -> x = v.x;
    this -> y = v.y;
    return *this;
}

//単項演算子
inline Vector2D& Vector2D::operator+=(const Vector2D& v){
    this -> x += v.x;
    this -> y += v.y;
    return *this;
}
inline Vector2D& Vector2D::operator-=(const Vector2D& v){
    this -> x -= v.x;
    this -> y -= v.y;
    return *this;
}
inline Vector2D& Vector2D::operator*=(float k){
    this -> x *= k;
    this -> y *= k;
    return *this;
}
inline Vector2D& Vector2D::operator/=(float k){
    this -> x /= k;
    this -> y /= k;
    return *this;
}
inline Vector2D Vector2D::operator+()const{
    return *this;
}
inline Vector2D Vector2D::operator-()const{
    return Vector2D(-x,-y);
}

//添え字演算子
inline float& Vector2D::operator[](int i){
    if(i == 0){
        return x;
    }
    else if(i == 1){
        return y;
    }
    else{
        return x;
    }
}

//比較演算子
inline bool Vector2D::operator==(const Vector2D& v) const{
    return (x == v.x) && (y == v.y);
}
inline bool Vector2D::operator!=(const Vector2D& v) const{
    return !(*this == v);
}

//ベクトルの長さ
inline float Vector2D::norm()const{
    return pow(x*x+y*y,0.5f);
}

//正規化
inline void Vector2D::normalize(){
    *this /= norm();
}

//二項演算子
inline Vector2D operator+(const Vector2D& u,const Vector2D& v){
    Vector2D w;
    w.x = u.x + v.x;
    w.y = u.y + v.y;
    return w;
}
inline Vector2D operator-(const Vector2D& u,const Vector2D& v){
    Vector2D w;
    w.x = u.x - v.x;
    w.y = u.y - v.y;
    return w;
}
inline Vector2D operator*(float k,const Vector2D& u){
    Vector2D w;
    w.x = k*u.x;
    w.y = k*u.y;
    return w;
}
inline Vector2D operator*(const Vector2D& u,float k){
    Vector2D w;
    w.x = k*u.x;
    w.y = k*u.y;
    return w;
}
inline Vector2D operator/(const Vector2D& u,float k){
    Vector2D w;
    w.x = u.x/k;
    w.y = u.y/k;
    return w;
}
inline float operator*(const Vector2D& u,const Vector2D& v){        //内積
    float n;
    n = u.x*v.x + u.y*v.y;
    return n;
}
inline float operator%(const Vector2D& u,const Vector2D& v){        //外積
    float n;
    n = u.x*v.y - u.y*v.x;
    return n;
}
inline float angle(const Vector2D& u,const Vector2D& v){
    float cos;
    cos = u*v/(u.norm()*v.norm());
    return float(acos(cos)/M_PI*180);
}

#endif
