#include "fksolver.h"
#include "rollpitchyaw.h"
#include "robot.h"
#include <cmath>

namespace cnoid{
namespace vnoid{

void FkSolver::CompLegFk(double l1, double l2, const double* q, Vector3* pos, Quaternion* ori, Vector3* axis){
    Vector3  pos_local[6];
    Vector3  axis_local[6];
    pos_local[0] = Vector3(0.0, 0.0, 0.0);
    pos_local[1] = Vector3(0.0, 0.0, 0.0);
    pos_local[2] = Vector3(0.0, 0.0, 0.0);
    pos_local[3] = Vector3(0.0, 0.0, -l1);
    pos_local[4] = Vector3(0.0, 0.0, -l2);
    pos_local[5] = Vector3(0.0, 0.0, 0.0);
    axis_local[0] = Vector3::UnitZ();
    axis_local[1] = Vector3::UnitX();
    axis_local[2] = Vector3::UnitY();
    axis_local[3] = Vector3::UnitY();
    axis_local[4] = Vector3::UnitY();
    axis_local[5] = Vector3::UnitX();

    Vector3    pbase(0.0, 0.0, 0.0);
    Quaternion qbase(1.0, 0.0, 0.0, 0.0);
    for(int i = 0; i < 6; i++){
        axis[i] = (i == 0 ? qbase : ori[i-1])*axis_local[i];
        pos [i] = (i == 0 ? pbase : pos[i-1]) + (i == 0 ? qbase : ori[i-1])*pos_local[i];
        ori [i] = (i == 0 ? qbase : ori[i-1])*AngleAxis(q[i], axis_local[i]);
    }
}

void FkSolver::CompLegFk(double l1, double l2, const double* q, Vector3& pos, Quaternion& ori){
    Vector3    _pos[6];
    Quaternion _ori[6];
    Vector3    _axis[6];

    CompLegFk(l1, l2, q, _pos, _ori, _axis);

    pos = _pos[5];
    ori = _ori[5];
}

void FkSolver::CompLegJacobian(double l1, double l2, const double* q, Eigen::Matrix<double,6,6>& J){
    Vector3    _pos[6];
    Quaternion _ori[6];
    Vector3    _axis[6];

    CompLegFk(l1, l2, q, _pos, _ori, _axis);

    for(int j = 0; j < 6; j++){
        Vector3 w = _axis[j];
        Vector3 v = _axis[j].cross(_pos[5] - _pos[j]);
        
        J(0,j) = v[0];
        J(1,j) = v[1];
        J(2,j) = v[2];
        J(3,j) = w[0];
        J(4,j) = w[1];
        J(5,j) = w[2];
    }
}

void FkSolver::CompArmFk(double l1, double l2, const double* q, Vector3* pos, Quaternion* ori, Vector3* axis){
    Vector3  pos_local[7];
    Vector3  axis_local[7];
    pos_local[0] = Vector3(0.0, 0.0, 0.0);
    pos_local[1] = Vector3(0.0, 0.0, 0.0);
    pos_local[2] = Vector3(0.0, 0.0, 0.0);
    pos_local[3] = Vector3(0.0, 0.0, -l1);
    pos_local[4] = Vector3(0.0, 0.0, -l2);
    pos_local[5] = Vector3(0.0, 0.0, 0.0);
    pos_local[6] = Vector3(0.0, 0.0, 0.0);
    axis_local[0] = Vector3::UnitY();
    axis_local[1] = Vector3::UnitX();
    axis_local[2] = Vector3::UnitZ();
    axis_local[3] = Vector3::UnitY();
    axis_local[4] = Vector3::UnitZ();
    axis_local[5] = Vector3::UnitY();
    axis_local[6] = Vector3::UnitX();

    Vector3    pbase(0.0, 0.0, 0.0);
    Quaternion qbase(1.0, 0.0, 0.0, 0.0);
    for(int i = 0; i < 7; i++){
        axis[i] = (i == 0 ? qbase : ori[i-1])*axis_local[i];
        pos [i] = (i == 0 ? pbase : pos[i-1]) + (i == 0 ? qbase : ori[i-1])*pos_local[i];
        ori [i] = (i == 0 ? qbase : ori[i-1])*AngleAxis(q[i], axis_local[i]);
    }
}

void FkSolver::CompArmFk(double l1, double l2, const double* q, Vector3& pos, Quaternion& ori){
    Vector3    _pos[7];
    Quaternion _ori[7];
    Vector3    _axis[7];

    CompArmFk(l1, l2, q, _pos, _ori, _axis);

    pos = _pos[6];
    ori = _ori[6];
}

void FkSolver::Comp(const Param& param, const vector<Joint>& joint, const Base& base, Centroid& centroid, vector<Hand>& hand, vector<Foot>& foot){
    Vector3     arm_pos [2][7];
    Quaternion  arm_ori [2][7];
    Vector3     arm_axis[2][7];
    Vector3     leg_pos [2][6];
    Quaternion  leg_ori [2][6];
    Vector3     leg_axis[2][6];
    double      q[7];

    // comp arm fk
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 7; j++){
            q[j] = joint[param.arm_joint_index[i] + j].q;
        }

        CompArmFk(param.upper_arm_length, param.lower_arm_length, q, &arm_pos[i][0], &arm_ori[i][0], &arm_axis[i][0]);

        hand[i].ori   = base.ori*arm_ori[i][6];
        hand[i].angle = ToRollPitchYaw(hand[i].ori);
        hand[i].pos   = base.pos + base.ori*(arm_pos[i][6] + param.base_to_shoulder[i]) + hand[i].ori*param.wrist_to_hand[i];
    }

    // comp leg fk
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 6; j++){
            q[j] = joint[param.leg_joint_index[i] + j].q;
        }

        CompLegFk(param.upper_leg_length, param.lower_leg_length, q, &leg_pos[i][0], &leg_ori[i][0], &leg_axis[i][0]);

        foot[i].ori   = base.ori*leg_ori[i][5];
        foot[i].angle = ToRollPitchYaw(foot[i].ori);
        foot[i].pos   = base.pos + base.ori*(leg_pos[i][5] + param.base_to_hip[i]) + foot[i].ori*param.ankle_to_foot[i];
    }

    // comp center_of_mass
    double total_mass = 0.0;
    Vector3  com(0.0, 0.0, 0.0);

    total_mass += param.trunk_mass;
    com += param.trunk_mass * param.trunk_com;

    for(int i = 0; i < 2; i++){
        for (int j = 0; j < 7; j++) {
            total_mass += param.arm_mass[j];
            com += param.arm_mass[j] * (param.base_to_shoulder[i] + arm_pos[i][j] + arm_ori[i][j] * param.arm_com[j]);
        }
    }
    for(int i = 0; i < 2; i++){
        for (int j = 0; j < 6; j++) {
            total_mass += param.leg_mass[j];
            com += param.leg_mass[j] * (param.base_to_hip[i] + leg_pos[i][j] + leg_ori[i][j] * param.leg_com[j]);
        }
    }

    com /= total_mass;

    centroid.com_pos = base.pos + base.ori * com;
}

// 支持脚基準の地面点群の高さを計算する関数
//vector<Vector3> FkSolver::FootToGroundFK(MyRobot* robot){
vector<Vector3> FkSolver::FootToGroundFK(const Param& param, const vector<Joint>& joint, const Base& base,  vector<Foot>& foot, vector<Vector3>& points_convex){
    // Define variables
    Vector3    FootToGround;       // Ground coordinate from foot
    Vector3    pos_HipToAncle;    // Ancle coordinate from base-link
    Quaternion qua_HipToAncle;    // Ancle quaternion from base-link
    Quaternion qua_AncleToFoot;    // Foot quaternion from ancle
    Vector3    pos_AncleToFoot;    // Foot coordinate from ancle
    Vector3    pos_HipToHead;     // Head coordinate from base-link
    Vector3    pos_HeadToCamera;   // Camera coordinate from head
    Vector3    pos_CameraToGround; // Ground coordinate from camera
    Quaternion qua_HipToCamera;   // Camera quaternion from base-link

    // comp leg fkに用いる
    Vector3     leg_pos [2][6];
    Quaternion  leg_ori [2][6];
    Vector3     leg_axis[2][6];
    double      q[7];
    Vector3     angle_HipToCamera;

    // comp leg fk
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 6; j++){
            q[j] =  joint[param.leg_joint_index[i] + j].q;
        }

        CompLegFk(param.upper_leg_length, param.lower_leg_length, q, &leg_pos[i][0], &leg_ori[i][0], &leg_axis[i][0]);
    }
    // 右足で考える（本当は支持脚としたいが、簡単のため両足揃えて止まっているときに撮影するという前提）
    pos_HipToAncle = leg_pos[0][5];
    qua_HipToAncle = leg_ori[0][5];

    //qua_AncleToFoot = qua_BaseToAncle; // 足首と足の角度は同じ
    pos_AncleToFoot = Vector3(0.0, 0.0, -0.05); // 固定値

    pos_HipToHead = Vector3(0.0, 0.1, 0.5+0.1); // 固定値（回転なし）

    pos_HeadToCamera = Vector3(0.2, 0.0, 0.0); // 固定値（回転なし）
    
    // 頭はベースに対して傾いていないのでCameraのプロパティから頭とカメラの傾き関係を取得でOK
    // カメラの角度をラジアンで設定（固定値）
    angle_HipToCamera = Vector3(M_PI*5/180, 0, -M_PI/2);

    // クォータニオンに変換
    qua_HipToCamera = FromRollPitchYaw(angle_HipToCamera);

    //printf("STRAT\n");
    std::vector<Vector3> groundFromFoot;
    for (const Vector3& p :  points_convex){
       // 最後に支持脚基準の地面点群の座標を計算
        FootToGround = qua_HipToAncle.conjugate() * (pos_HipToHead + pos_HeadToCamera + qua_HipToCamera*p - pos_HipToAncle) - pos_AncleToFoot;
        //printf("%lf,%lf,%lf\n", FootToGround[0], FootToGround[1], FootToGround[2]);
        groundFromFoot.push_back(FootToGround);
    }
    //printf("END\n");

    // 長方形の四隅の座標を計算
    double avgZ = std::accumulate(groundFromFoot.begin(), groundFromFoot.end(), 0.0, [](double sum, const Vector3& v) {
       return sum + v.z();
    }) / groundFromFoot.size();
    auto minmaxX = std::minmax_element(groundFromFoot.begin(), groundFromFoot.end(), [](const Vector3& a, const Vector3& b) {
        return a.x() < b.x();
    });
    auto minmaxY = std::minmax_element(groundFromFoot.begin(), groundFromFoot.end(), [](const Vector3& a, const Vector3& b) {
        return a.y() < b.y();
    });
    double minX = std::round(minmaxX.first->x() * 1000.0) / 1000.0;
    double maxX = std::round(minmaxX.second->x() * 1000.0) / 1000.0;
    double minY = std::round(minmaxY.first->y() * 1000.0) / 1000.0;
    double maxY = std::round(minmaxY.second->y() * 1000.0) / 1000.0;
    double avgZRounded = std::round(avgZ * 1000.0) / 1000.0;
    std::vector<Vector3> groundRectangle;   // 時計回りに定義している
    groundRectangle.push_back(Vector3(maxX, maxY, avgZ));
    groundRectangle.push_back(Vector3(maxX, minY, avgZ));
    groundRectangle.push_back(Vector3(minX, minY, avgZ));
    groundRectangle.push_back(Vector3(minX, maxY, avgZ));
    
    return groundRectangle;
}

}
}