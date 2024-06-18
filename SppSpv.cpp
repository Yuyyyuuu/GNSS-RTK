#include"MyHeadFile.h"


/*
**************************************************************************
函数名：单点定位
参数：Epoch   当前历元的观测数据
      GPSEph  GPS星历
      BDSEph  BDS星历
      Res     用户定位结果
             （传入时为上一历元的定位结果，函数运行后为本次历元的定位结果）
返回值：单点定位是否完成 true-成功 false-失败
函数功能：进行SPP解算并将结果存入Res
**************************************************************************
*/
bool SPP(EPOCHOBS* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res)
{
    Res->Time = Epoch->Time;
    // 设定初始位置
    // 第一个历元的初始位置设置为0（构造函数已做）
    // 后续历元的初始位置设置为上一历元解算结果
    double X_0[3] = { Res->Pos[0],Res->Pos[1], Res->Pos[2] };
    double dt_0[2] = { 0,0 };// 接收机钟差（GPS+BDS）
    // 迭代标志
    bool flag = true;
    // 迭代计数器
    int calcu_num = 0;
    // 迭代解算，直至收敛
    do {
        // 计算信号发射时刻的卫星位置、钟差、地球自转改正和对流层延迟
        ComputeGPSSatOrbitAtSignalTrans(Epoch, GPSEph, BDSEph, X_0);
        // 对所有卫星的观测数据进行线性化
        // 以初始位置为参考，对观测方程线性化，得B和W矩阵，统计参与定位的各系统卫星数和所有卫星数
        int satnum[2] = { 0,0 };// 第一个元素为GPS可用卫星数，第二个元素为BDS可用卫星数
         // 总的可用卫星数计数
        int sum_satnum = 0;
        double B[MAXCHANNUM * 5];
        memset(B, 0, sizeof(B));
        double w[MAXCHANNUM * 1];
        memset(w, 0, sizeof(w));
        // 权阵
        double P[MAXCHANNUM * MAXCHANNUM];
        memset(P, 0, sizeof(P));
        for (int i = 0; i < Epoch->SatNum; i++)
        {
            sum_satnum = satnum[0] + satnum[1];
            // 观测数据不完整或有粗差、卫星位置计算失败，不参与定位计算
            if (Epoch->SatObs[i].Valid == false || Epoch->SatPVT[i].Valid == false) continue;
            double rou = sqrt((Epoch->SatPVT[i].SatPos[0] - X_0[0]) * (Epoch->SatPVT[i].SatPos[0] - X_0[0]) +
                (Epoch->SatPVT[i].SatPos[1] - X_0[1]) * (Epoch->SatPVT[i].SatPos[1] - X_0[1]) +
                (Epoch->SatPVT[i].SatPos[2] - X_0[2]) * (Epoch->SatPVT[i].SatPos[2] - X_0[2]));
            B[sum_satnum * 5] = (X_0[0] - Epoch->SatPVT[i].SatPos[0]) / rou;
            B[sum_satnum * 5 + 1] = (X_0[1] - Epoch->SatPVT[i].SatPos[1]) / rou;
            B[sum_satnum * 5 + 2] = (X_0[2] - Epoch->SatPVT[i].SatPos[2]) / rou;
            if (Epoch->SatObs[i].System == GPS)
            {
                B[sum_satnum * 5 + 3] = 1;
                B[sum_satnum * 5 + 4] = 0;
                w[sum_satnum] = Epoch->ComObs[i].PIF - (rou + dt_0[0] - C_Light * Epoch->SatPVT[i].SatClkOft + Epoch->SatPVT[i].TropCorr);
                satnum[0]++;// GPS可用卫星数加一
            }
            else if (Epoch->SatObs[i].System == BDS)
            {

                B[sum_satnum * 5 + 3] = 0;
                B[sum_satnum * 5 + 4] = 1;
                // 注意BDS的IF组合的观测方程会多一个tgd硬件延迟
                w[sum_satnum] = Epoch->ComObs[i].PIF - (rou + dt_0[1] - C_Light * Epoch->SatPVT[i].SatClkOft + Epoch->SatPVT[i].TropCorr + C_Light * (FG1_BDS * FG1_BDS * Epoch->SatPVT[i].Tgd1) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS));
                satnum[1]++;// BDS可用卫星数加一
            }
            P[sum_satnum + sum_satnum * MAXCHANNUM] = 1;// 等权法
            //P[sum_satnum + sum_satnum * MAXCHANNUM] = sin(Epoch->SatPVT[i].Elevation) * sin(Epoch->SatPVT[i].Elevation);// 高度角定权
        }
        sum_satnum = satnum[0] + satnum[1];
        // 如果卫星数不足，直接返回定位失败
        int x_num = 3;// 参数个数
        for (int i = 0; i < 2; i++)
        {
            if (satnum[i] != 0) x_num++;
        }
        // 可用卫星数不足以解算，则退出函数
        if (sum_satnum < x_num) return 0;
        // 建立法方程N，W
        double N[5 * 5];
        double W[5];
        double BT[5 * MAXCHANNUM];
        double BTP[5 * MAXCHANNUM];
        MatrixTranspose(MAXCHANNUM, 5, B, BT);
        MatrixMultiply(5, MAXCHANNUM, MAXCHANNUM, MAXCHANNUM, BT, P, BTP);
        MatrixMultiply(5, MAXCHANNUM, MAXCHANNUM, 5, BTP, B, N);
        MatrixMultiply(5, MAXCHANNUM, MAXCHANNUM, 1, BTP, w, W);
        double x[5];
        double v[MAXCHANNUM * 1];
        double vT[1 * MAXCHANNUM];
        double vTv[1] = { 0 };
        double Bx[MAXCHANNUM * 1];

        // GPS\BDS双系统均有数据
        if (satnum[0] != 0 && satnum[1] != 0)
        {
            // 最小二乘求解 
            double N_inv[5 * 5];
            if (MatrixInv(5, N, N_inv) == 0) return 0;
            MatrixMultiply(5, 5, 5, 1, N_inv, W, x);
            double x_norm = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3] + x[4] * x[4]);//sign
            if (x_norm < 1e-5)  flag = false;
            X_0[0] = X_0[0] + x[0];
            X_0[1] = X_0[1] + x[1];
            X_0[2] = X_0[2] + x[2];
            // 定位精度评价，计算PDOP
            Res->PDOP = sqrt(N_inv[0] + N_inv[6] + N_inv[12]);
        }
        // 只有GPS或BDS单个系统的数据
        else
        {
            // 重构N，W
            if (satnum[0] == 0)
            {
                // 如果GPS卫星数为0，则第4行和第4列均为0，可以删除，将矩阵缩减为4 * 4
                deleteRowAndColumn(5, 5, 4, 4, N);
                // 将W矩阵删除第4行
                deleteRow(5, 4, W);
            }
            else if (satnum[1] == 0)
            {
                // BDS卫星数为0，将第5行和第5列删除
                deleteRowAndColumn(5, 5, 5, 5, N);
                // 将W矩阵删除第5行
                deleteRow(5, 5, W);
            }
            // 最小二乘求解
            double N_inv[4 * 4];
            if (MatrixInv(4, N, N_inv) == 0) return 0;
            MatrixMultiply(4, 4, 4, 1, N_inv, W, x);
            // 重构x
            if (satnum[0] == 0)
            {
                x[4] = x[3];
                x[3] = 0;
            }
            else if (satnum[1] == 0) x[4] = 0;
            double x_norm = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3] + x[4] * x[4]);
            if (x_norm < 1e-5)  flag = false;
            X_0[0] = X_0[0] + x[0];
            X_0[1] = X_0[1] + x[1];
            X_0[2] = X_0[2] + x[2];
            // 定位精度评价，计算PDOP
            Res->PDOP = sqrt(N_inv[0] + N_inv[5] + N_inv[10]);
        }

        /*
        if (satnum[0] == 0)
        {
            // 如果GPS卫星数为0，则第4行和第4列均为0，可以删除，将矩阵缩减为4 * 4
            deleteRowAndColumn(5, 5, 4, 4, N);
            // 将W矩阵删除第4行
            deleteRow(5, 4, W);
            // 最小二乘求解
            double N_inv[4 * 4];
            if (MatrixInv(4, N, N_inv)==0) return 0;
            MatrixMultiply(4, 4, 4, 1, N_inv, W, x);
            // 重构x
            x[4] = x[3];
            x[3] = 0;
            double x_norm = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3] + x[4] * x[4]);
            if (x_norm < 1e-5)  flag = false;
            X_0[0] = X_0[0] + x[0];
            X_0[1] = X_0[1] + x[1];
            X_0[2] = X_0[2] + x[2];
            // 定位精度评价，计算PDOP
            Res->PDOP = sqrt(N_inv[0] + N_inv[5] + N_inv[10]);
        }
        else if (satnum[1] == 0)
        {
            // BDS卫星数为0，将第5行和第5列删除
            deleteRowAndColumn(5, 5, 5, 5, N);
            // 将W矩阵删除第5行
            deleteRow(5, 5, W);
            // 最小二乘求解
            double N_inv[4 * 4];
            if (MatrixInv(4, N, N_inv)==0) return 0;
            MatrixMultiply(4, 4, 4, 1, N_inv, W, x);
            x[4] = 0;
            double x_norm = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3] + x[4] * x[4]);
            if (x_norm < 1e-5)  flag = false;
            X_0[0] = X_0[0] + x[0];
            X_0[1] = X_0[1] + x[1];
            X_0[2] = X_0[2] + x[2];
            // 定位精度评价，计算PDOP
            Res->PDOP = sqrt(N_inv[0] + N_inv[5] + N_inv[10]);
        }
        // 如果GPS和BDS卫星数均不为0，N和W矩阵不变
        else
        {
            // 最小二乘求解 
            double N_inv[5 * 5];
            if (MatrixInv(5, N, N_inv)==0) return 0;
            MatrixMultiply(5, 5, 5, 1, N_inv, W, x);
            double x_norm = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3] + x[4] * x[4]);//sign
            if (x_norm < 1e-5)  flag = false;
            X_0[0] = X_0[0] + x[0];
            X_0[1] = X_0[1] + x[1];
            X_0[2] = X_0[2] + x[2];
            // 定位精度评价，计算PDOP
            Res->PDOP = sqrt(N_inv[0] + N_inv[6] + N_inv[12]);
        }
        */

        // 更新钟差
        dt_0[0] = x[3];
        dt_0[1] = x[4];
        // 定位精度评价，计算验后单位权中误差
        MatrixMultiply(MAXCHANNUM, 5, 5, 1, B, x, Bx);
        MatrixSubtraction(MAXCHANNUM, 1, Bx, w, v);
        MatrixTranspose(MAXCHANNUM, 1, v, vT);
        MatrixMultiply(1, MAXCHANNUM, MAXCHANNUM, 1, vT, v, vTv);
        Res->SigmaPos = sqrt(vTv[0] / (sum_satnum - x_num));
        calcu_num++;
        Res->SatNum = sum_satnum;
        if (calcu_num > 15) flag = false;
    } while (flag);

    Res->Pos[0] = X_0[0];
    Res->Pos[1] = X_0[1];
    Res->Pos[2] = X_0[2];

    return true;

}

/*
***************************************
函数名：单点测速
参数：Epoch   当前历元的观测数据
      Res     用户定位结果
函数功能：进行SPV解算并将结果存入Res
注：只有当SPP成功才能进行SPV
***************************************
*/
void SPV(EPOCHOBS* Epoch, POSRES* Res)
{
    double B[MAXCHANNUM * 4];
    memset(B, 0, sizeof(B));
    double w[MAXCHANNUM * 1];
    memset(w, 0, sizeof(w));
    int valid_satnum = 0;
    for (int i = 0; i < Epoch->SatNum; i++)
    {
        // 观测数据不完整或有粗差、卫星位置计算失败，不参与定位计算
        if (Epoch->SatObs[i].Valid == false || Epoch->SatPVT[i].Valid == false) continue;
        double xsr = Res->Pos[0] - Epoch->SatPVT[i].SatPos[0];
        double ysr = Res->Pos[1] - Epoch->SatPVT[i].SatPos[1] ;
        double zsr = Res->Pos[2] - Epoch->SatPVT[i].SatPos[2];
        double rou = sqrt(xsr * xsr + ysr * ysr + zsr * zsr);
        double roudot = -(xsr* Epoch->SatPVT[i].SatVel[0]+ ysr * Epoch->SatPVT[i].SatVel[1] + zsr * Epoch->SatPVT[i].SatVel[2]) / rou;
        B[0 + i * 4] = xsr / rou;
        B[1 + i * 4] = ysr / rou;
        B[2 + i * 4] = zsr / rou;
        B[3 + i * 4] = 1;
        w[i] = Epoch->SatObs[i].d[0] - (roudot - C_Light * Epoch->SatPVT[i].SatClkSft);
        valid_satnum++;
    }
    // 可用卫星数不足以解算，则退出函数
    if (valid_satnum < 4) return;
    double X[4];
    double N[4 * 4];
    double W[4];
    double BT[4 * MAXCHANNUM];
    MatrixTranspose(MAXCHANNUM, 4, B, BT);
    MatrixMultiply(4, MAXCHANNUM, MAXCHANNUM, 4, BT, B, N);
    MatrixMultiply(4, MAXCHANNUM, MAXCHANNUM, 1, BT, w, W);
    // 最小二乘求解 
    double N_inv[4 * 4];
    int iserror = MatrixInv(4, N, N_inv);
    if (iserror == 0) return; // 求逆遇到致命性错误，则退出函数
    MatrixMultiply(4, 4, 4, 1, N_inv, W, X);
    // 存储测速结果
    Res->Vel[0] = X[0];
    Res->Vel[1] = X[1];
    Res->Vel[2] = X[2];
    // 计算验后单位权中误差
    double v[MAXCHANNUM * 1];
    double vT[1 * MAXCHANNUM];
    double vTv[1];
    double Bx[MAXCHANNUM * 1];
    MatrixMultiply(MAXCHANNUM, 4, 4, 1, B, X, Bx);
    MatrixSubtraction(MAXCHANNUM, 1, Bx, w, v);
    MatrixTranspose(MAXCHANNUM, 1, v, vT);
    MatrixMultiply(1, MAXCHANNUM, MAXCHANNUM, 1, vT, v, vTv);
    Res->SigmaVel = sqrt(vTv[0] / (valid_satnum - 4));
    
}


/*
*******************************************
函数名：结果输出函数
参数：pres   接收机定位结果
      res    解算结果
      enu    定位误差
函数功能：将结果进行形式转换并打印在控制台
*******************************************
*/
void OutputResult(const POSRES* pres, const POSRES* res, double enu[])
{
    double pres_xyz[3]; // 接收机定位结果（XYZ形式）
    printf("%d %7.0f 卫星数:%d   解算位置:%14.4f %13.4f %13.4f  PDOP:%6.4f  sigmaPos:%6.4f   解算速度: %6.3f %6.3f %6.3f  sigmaV:%5.3f\n",
        res->Time.Week, res->Time.SecOfWeek, res->SatNum, res->Pos[0], res->Pos[1], res->Pos[2], res->PDOP, res->SigmaPos, res->Vel[0], res->Vel[1], res->Vel[2], res->SigmaVel);
    // 将接收机定位结果转为XYZ形式
    BLHToXYZ(pres->Pos, pres_xyz, R_CGS2K, F_CGS2K);
    printf("卫星数:%d   精确位置:%14.4f %13.4f %13.4f\n", pres->SatNum, pres_xyz[0], pres_xyz[1], pres_xyz[2]);
    // 计算定位误差
    Comp_dEnu(2, pres_xyz, res->Pos, enu);
    printf("定位误差:%5.3f %5.3f %5.3f\n\n", enu[0], enu[1], enu[2]);
}
















