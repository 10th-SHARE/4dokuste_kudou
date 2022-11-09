#pragma once
//参考<https://robostepwiki.sakuratan.com/wp/4%E8%BC%AA%E7%8B%AC%E7%AB%8B%E3%82%B9%E3%83%86%E3%82%A2%E3%83%AA%E3%83%B3%E3%82%B0%E6%A9%9F%E6%A7%8B%E3%81%AE%E5%88%B6%E5%BE%A1/>
//三輪を動かすためのクラス
//機体座標のx軸正を右向き、y軸を上向きに取ったとき
//タイヤ0:右上、タイヤ1:左上、タイヤ2:左下、タイヤ３:右下
#include <math.h>
class WheelOmega
{
public:
    //コンストラクタの定義 main関数の前に必ず一度宣言する
    //dist_wheel: 機体中心からタイヤへの距離[mm]
    //r_wheel: タイヤの半径[mm]
    WheelOmega(float dist_wheel, float r_wheel): vx_(0), vy_(0), aimtheta_(0) //初期化
    {
        for(int i =0; i<4; i++) {
            omega[i]=0;
            k[i] = 1;
        }
        dist_wheel_ = dist_wheel;
        inv_r_wheel_ = 1.0f / r_wheel; //割り算は重い処理なので逆数をかける
    }
    //vx,vy: 機体座標での移動速度[mm/s]
    //aimtheta: 機体の回転速度[degree/s] 時計回りが正
    void setVxy(double vx,double vy,double aimtheta)
    {
        vx_ = vx;
        vy_ = vy;
        aimtheta_ = aimtheta * (3.14 / 180);
    }
    void calOmega() //タイヤそれぞれの目標角速度[rad/s]を計算
    {
        omega[0] = sqrt(vx_*vy_+vy_*vy_+dist_wheel_*dist_wheel_*aimtheta_*aimtheta_-2*vx_*aimtheta_*dist_wheel_*sin(aimtheta_+theta_rad)+2*vy_*aimtheta_*dist_wheel_*cos(aimtheta_+4/theta_rad))*inv_r_wheel_*k[0];;
        omega[1] = sqrt(vx_*vy_+vy_*vy_+dist_wheel_*dist_wheel_*aimtheta_*aimtheta_-2*vx_*aimtheta_*dist_wheel_*cos(aimtheta_+theta_rad)-2*vy_*aimtheta_*dist_wheel_*sin(aimtheta_+4/theta_rad))*inv_r_wheel_*k[1];;
        omega[2] = sqrt(vx_*vy_+vy_*vy_+dist_wheel_*dist_wheel_*aimtheta_*aimtheta_+2*vx_*aimtheta_*dist_wheel_*sin(aimtheta_+theta_rad)-2*vy_*aimtheta_*dist_wheel_*cos(aimtheta_+4/theta_rad))*inv_r_wheel_*k[2];;
        omega[3] = sqrt(vx_*vy_+vy_*vy_+dist_wheel_*dist_wheel_*aimtheta_*aimtheta_+2*vx_*aimtheta_*dist_wheel_*cos(aimtheta_+theta_rad)+2*vy_*aimtheta_*dist_wheel_*sin(aimtheta_+4/theta_rad))*inv_r_wheel_*k[3];;
    };
    double getOmega(int i) //タイヤの目標角速度[rad/s]返す
    {
        return omega[i];
    }
    /*
    以下はクラスのコンストラクタで初期設定されている
    設定を変更したいときのみ呼び出す
    */
    //タイヤがエンコーダ正の時に反時計回りなら1、時計回りなら-1 (初期設定は全て1)
    void set_k(int k0, int k1, int k2, int k3)
    {
        k[0] = k0;
        k[1] = k1;
        k[2] = k2;
        k[3] = k3;
    }
private:
    double vx_, vy_, aimtheta_;
    double omega[4];
    static const double theta_rad = 45.0 / 180 * 3.14;
    float dist_wheel_;
    float inv_r_wheel_;
    int k[4];
};
