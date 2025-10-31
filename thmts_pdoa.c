#include "thmts_pdoa.h"

/**
 * 计算两路 CIR 三点向量的相位差（角度，范围(-180, 180]）
 * 使用 [0][0], [0][1], [0][2] 三个采样点
 * 所有权重均为 1
 * 计算出来的结果是pdoa(b-a)的结果
 */
double cal_pdoa(const uint32_t CIR_a[3][3], const uint32_t CIR_b[3][3]) {
    c64 va[3], vb[3];

    // 组装三点复向量（只用第0行）
    for (int k = 0; k < 3; ++k) {
        va[k] = iq_from_u32(CIR_a[0][k]);
        vb[k] = iq_from_u32(CIR_b[0][k]);
    }

    // 计算 z = v_a^H * v_b = sum(conj(a[i]) * b[i])
    double zr = 0.0, zi = 0.0;
    for (int i = 0; i < 3; ++i) {
        double ar = va[i].re, ai = va[i].im;
        double br = vb[i].re, bi = vb[i].im;

        zr += ( ar*br + ai*bi );     // 实部
        zi += (-ar*bi + ai*br );     // 虚部
    }

    if (zr == 0.0 && zi == 0.0)
        return 0.0;

    return (57.2958*atan2(zi, zr)); 
}

/**
 * 根据三组 PDOA 计算 AOA（角度度数，范围[0, 360)）
 * phi1_0 = pdoa(1-0)
 * phi2_0 = pdoa(2-0)
 * phi2_1 = pdoa(2-1)
 */
 double aoa_compute_from_pdoa(double phi1_0, double phi2_0, double phi2_1) 
 {    
    // 执行公式计算
    double numerator = 1.7321 * phi1_0;
    double denominator = phi2_0 - phi2_1;
    
    // 使用atan2计算角度，并转换为度
    double angle_rad = atan2(numerator, denominator);
    double angle_deg = angle_rad * 180.0 / M_PI;
    
    return angle_deg;
 }
