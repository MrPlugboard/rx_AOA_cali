#include "thmts_pdoa.h"

int32_t AOA_index = 0;
uint32_t CIR_data0[3][CIR_FOR_PDOA_LENGTH];         // 通道0的CIR数据，采集CIR_FOR_PDOA_LENGTH个点，第一径点以及对应前后两个点的数据
uint32_t CIR_data1[3][CIR_FOR_PDOA_LENGTH];         // 通道1的CIR数据，采集CIR_FOR_PDOA_LENGTH个点，第一径点以及对应前后两个点的数据
uint32_t CIR_data2[3][CIR_FOR_PDOA_LENGTH];         // 通道2的CIR数据，采集CIR_FOR_PDOA_LENGTH个点，第一径点以及对应前后两个点的数据


static inline int wrap_idx(int i) {
    i %= CIR_N;
    return (i < 0) ? (i + CIR_N) : i;
}

/**
 在firstPeakIdx附近采集CIR数据
/* src: 源CIR数组（如 channel0_CIR）
 * dst: 目标二维数组（如 CIR_data0）
 * subfrm_idx: 要写入的子帧行号
 * firstPeakIdx : 第一径位置
 */
void collect_cir_from(const uint32_t* src,
                      uint32_t (*dst)[CIR_FOR_PDOA_LENGTH],
                      int subfrm_idx,
					  uint16_t firstPeakIdx
					  )
{
    if (!src || !dst) return;
    if (subfrm_idx < 0 || subfrm_idx >= 3) return;   // 你的CIR_data0有3行

    const int L = CIR_FOR_PDOA_LENGTH;
    const int left  = L / 2;                         // floor(L/2)
    const int start = wrap_idx((int)firstPeakIdx - left);

    uint32_t* out = dst[subfrm_idx];                 // 指向要写入的那一行

    /* 第1段：从 start 到数组末尾 */
    int chunk1 = CIR_N - start;
    if (chunk1 > L) chunk1 = L;
    memcpy(out, src + start, (size_t)chunk1 * sizeof(uint32_t));

    /* 第2段：从头部拷贝剩余 */
    int remain = L - chunk1;
    if (remain > 0) {
        memcpy(out + chunk1, src, (size_t)remain * sizeof(uint32_t));
    }
}


/**
 * 计算两路 CIR 多点向量的相位差（角度，范围约为 (-180, 180]）
 * 只用第 0 行：CIR_a[0][k] 与 CIR_b[0][k]，k=0..CIR_FOR_PDOA_LENGTH-1
 * 所有权重为 1；返回的是 pdoa(b - a)
 */
double cal_pdoa(const uint32_t CIR_a[3][CIR_FOR_PDOA_LENGTH],
                const uint32_t CIR_b[3][CIR_FOR_PDOA_LENGTH])
{
    double zr = 0.0, zi = 0.0;     // z = v_a^H * v_b 的实部/虚部

    // 逐样本累加：z = sum_k conj(a_k) * b_k
    for (int k = 0; k < CIR_FOR_PDOA_LENGTH; ++k) {
        c64 a = iq_from_u32(CIR_a[0][k]);
        c64 b = iq_from_u32(CIR_b[0][k]);

        // conj(a) * b = (ar - j ai) * (br + j bi)
        // 实： ar*br + ai*bi
        // 虚：-ar*bi + ai*br
        zr += ( a.re * b.re + a.im * b.im );
        zi += (-a.re * b.im + a.im * b.re );
    }

    if (zr == 0.0 && zi == 0.0) return 0.0;

    // atan2 返回 (-pi, pi]，换算为度；如需严格 (-180,180] 可再 wrap 一次
    double deg = atan2(zi, zr) * 57.29577951308232; // 180/pi
    if (deg <= -180.0) deg += 360.0;
    if (deg  >  180.0) deg -= 360.0;
    return deg;
}

// ---- 小工具：wrap 到 (-pi, pi] 和 (-180, 180] ----
static inline double wrap_to_pi(double x_rad)
{
    x_rad = fmod(x_rad + M_PI, 2.0 * M_PI);
    if (x_rad < 0) x_rad += 2.0 * M_PI;
    return x_rad - M_PI;           // (-pi, pi]
}
static inline double wrap_to_180(double x_deg)
{
    x_deg = fmod(x_deg + 180.0, 360.0);
    if (x_deg < 0) x_deg += 360.0;
    return x_deg - 180.0;          // (-180, 180]
}

double aoa_compute_from_pdoa(double phi1_0, double phi2_0, double phi2_1, uwb_config_t *uwb_config)
{
    // 检查指针是否有效
    if (uwb_config == NULL) {
        // 可以返回错误值或使用默认配置
        printf("Error: uwb_config pointer is NULL\n");
        return 0.0;
    }

    // 1) 偏置：count -> rad
    // 假定：pdoa1 = b21, pdoa2 = b31；则 b32 = b31 - b21
    const double b21_rad = ((double)uwb_config->pdoa1) * (M_PI / COUNT_PER_PI);
    const double b31_rad = ((double)uwb_config->pdoa2) * (M_PI / COUNT_PER_PI);
    const double b32_rad = b31_rad - b21_rad;

    // 2) 输入：deg -> rad
    const double phi21_rad = phi1_0 * (M_PI / 180.0);
    const double phi31_rad = phi2_0 * (M_PI / 180.0);
    const double phi32_rad = phi2_1 * (M_PI / 180.0);

    // 3) 去偏 + wrap 到 (-pi, pi]
    const double phi12_corr = wrap_to_pi(-phi21_rad - b21_rad);
    const double phi13_corr = wrap_to_pi(-phi31_rad - b31_rad);
    // 用测得的 φ32 减去理论偏置 b32，再 wrap
    const double phi23_corr = wrap_to_pi(-phi32_rad - b32_rad);
    // （等价地也可：wrap_to_pi(phi31_corr - phi21_corr)）

    // 4) AOA 计算（与 MATLAB 一致）
    const double num = SQRT3 * (phi13_corr-phi12_corr);
    const double den = phi12_corr + phi13_corr ;
    double angle_deg = atan2(num, den) * (180.0 / M_PI);

    // 5) 叠加整体零位（度）并 wrap 到 (-180, 180]
    angle_deg = wrap_to_180(angle_deg + (double)uwb_config->aoaOffset);

    return angle_deg;
}
