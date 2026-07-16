// #ifndef APP_LIDAR_LIDAR_H
// #define APP_LIDAR_LIDAR_H
//
// #include <stdint.h>
//
// #define ENCODER_SCALE     10.0f  // 0.1도 단위 제어 스케일
//
// #ifndef M_PI
// #define M_PI 3.14159265358979323846
// #endif
//
// // 터렛 원점 기준의 3D 공간 절대 위치 좌표 구조체 (m 단위)
// struct point3d {
//     float x;  // 전후축 오프셋 (+: 앞, -: 뒤)
//     float y;  // 좌우축 오프셋 (+: 우, -: 좌)
//     float z;  // 상하축 오프셋 (+: 위, -: 아래)
// };
//
// /**
//  * @brief 1D 라이다 실측 거리와 현재 모터 엔코더 각도를 결합해 3차원 물리 공간 벡터(X, Y, Z)를 역산합니다.
//  */
// struct point3d LiDAR_Calc3DSpace(float dist_meter, int16_t cur_theta_ddeg, int16_t cur_phi_ddeg);
//
// #endif // APP_LIDAR_LIDAR_H