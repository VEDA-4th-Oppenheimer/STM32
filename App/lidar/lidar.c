#include "lidar.h"
#include <math.h>

struct point3d LiDAR_Calc3DSpace(float dist_meter, int16_t cur_theta_ddeg, int16_t cur_phi_ddeg) {
    struct point3d position = {0.0f, 0.0f, 0.0f};

    // TOFSense-F2 측정 한계 및 하드웨어 예외 처리
    if (dist_meter < 0.1f || dist_meter > 25.0f) {
        return position;
    }

    // deci-degree 정수 각도를 삼각함수용 Radian 단위로 환산
    double rad_th = ((double)cur_theta_ddeg / ENCODER_SCALE) * (M_PI / 180.0);
    double rad_ph = ((double)cur_phi_ddeg / ENCODER_SCALE) * (M_PI / 180.0);

    // 구면 좌표계 -> 3D 직교 벡터 공간 정사영
    position.x = (float)(dist_meter * cos(rad_ph) * cos(rad_th));
    position.y = (float)(dist_meter * cos(rad_ph) * sin(rad_th));
    position.z = (float)(dist_meter * sin(rad_ph));

    return position;
}