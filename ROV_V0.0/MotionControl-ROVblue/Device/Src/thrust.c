#include "thrust.h"

/*
 * 函数名: ThrustAllocate
 * 描述  : 推力分配
 * 输入  : askedthrust 0~5 对应 x y z rx ry rz
 *         motorthrust 电机被分配的推力 0~5 对应电机 0~5
 * 输出  : /
 * 备注  : /
 */
void ThrustAllocate(float *askedthrust, float *motorthrust)
{
    // // motorthrust[0] = -0.4125f *askedthrust[0] + 0.3480f * askedthrust[2] + 2.6630f * askedthrust[4] + 1.9254f * askedthrust[5];
    // // motorthrust[1] = 0.4119f *askedthrust[0] + 0.3481f * askedthrust[2] + 2.6652f * askedthrust[4] - 1.9218f * askedthrust[5];
    // // motorthrust[2] = 0.1065f *askedthrust[0] + 0.5003f * askedthrust[1] + 2.4164f * askedthrust[5] - 4.7794f * askedthrust[3];
    // // motorthrust[3] = -0.1065f *askedthrust[0] + 0.4997f * askedthrust[1] - 2.4164f * askedthrust[5] + 4.7794f * askedthrust[3];
    // // motorthrust[4] = -0.4598f *askedthrust[0] - 0.2623f * askedthrust[2] + 2.6652f * askedthrust[4] - 1.9218f * askedthrust[5];
    // // motorthrust[5] = 0.4593f *askedthrust[0] - 0.2624f * askedthrust[2] + 2.6630f * askedthrust[4] + 1.9254f * askedthrust[5];

    // motorthrust[0] = -0.4125f *askedthrust[0] + 0.3480f * askedthrust[2] + 2.6630f * askedthrust[4] ;
    // motorthrust[1] =  0.4119f *askedthrust[0] + 0.3481f * askedthrust[2] + 2.6652f * askedthrust[4] ;
    // motorthrust[2] =  0.1065f *askedthrust[0] + 0.5003f * askedthrust[1] + 2.4164f * askedthrust[5] - 4.7794f * askedthrust[3];
    // motorthrust[3] = -0.1065f *askedthrust[0] + 0.4997f * askedthrust[1] - 2.4164f * askedthrust[5] + 4.7794f * askedthrust[3];
    // motorthrust[4] = -0.4598f *askedthrust[0] - 0.3480f * askedthrust[2] + 2.6652f * askedthrust[4] ;
    // motorthrust[5] =  0.4593f *askedthrust[0] - 0.3481f * askedthrust[2] + 2.6630f * askedthrust[4] ;

    motorthrust[0] = -__A_2 * askedthrust[0] / __B + __C * askedthrust[1] + askedthrust[5] / __B;
    motorthrust[1] = __A_2 * askedthrust[0] / __B + __C * askedthrust[1] - askedthrust[5] / __B;
    motorthrust[2] = -0.5 * askedthrust[2] + askedthrust[4] / __2b;
    motorthrust[3] = -0.5 * askedthrust[2] - askedthrust[4] / __2b;
    motorthrust[4] = -__A_1 * askedthrust[0] / __B - __C * askedthrust[1] - askedthrust[5] / __B;
    motorthrust[5] = __A_1 * askedthrust[0] / __B - __C * askedthrust[1] + askedthrust[5] / __B;

}

