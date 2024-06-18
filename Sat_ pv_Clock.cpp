#include"MyHeadFile.h"


/*
*********************************************************************
函数名：计算GPS中间量（卫星的位置、速度，钟差、钟速，硬件延迟）的函数
参数：Prn     卫星号
      t       需知中间量对应的时刻（GPS时）
      eph     GPS广播星历
      Mid     所得中间量结果存储体
返回值：true-计算成功 false-计算失败
函数功能：根据卫星广播星历来计算中间量
*********************************************************************
*/
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* eph, SATMIDRES* Mid)
{
    // 星历是否过期或健康判断
    if (eph->SVHealth == 1 || (fabs(GetDiffTime(t, &eph->TOE)) > 7500))
    {
        Mid->Valid = false;
        return false;
    }
    Mid->Valid = true;
    // 计算卫星位置
    // 计算轨道长半轴
    double A = eph->SqrtA * eph->SqrtA;
    // 计算平均运动角速度
    double n0 = sqrt(GM_WGS / (A * A * A));// rad/s
    // 计算相对于星历参考历元的时间
    double tk = GetDiffTime(t, &eph->TOE);
    // 对平均运动角速度进行改正
    double n = n0 + eph->DeltaN;// rad/s
    // 计算平近点角
    double Mk = eph->M0 + n * tk;// rad
    // 计算偏近点角（迭代）
    double Ek = 0;// rad 
    double Et = Mk;// 迭代器
    while (fabs(Et - Ek) > 1e-12)
    {
        Et = Ek;
        Ek = Mk + eph->e * sin(Ek);
    }
    // 计算真近点角
    double vk = atan2(sqrt(1 - eph->e * eph->e) * sin(Ek), cos(Ek) - eph->e);
    // 计算升交角距
    double faik = vk + eph->omega;
    // 计算二阶调和改正数
    double duk = eph->Cus * sin(2 * faik) + eph->Cuc * cos(2 * faik);// 升交角距改正数
    double drk = eph->Crs * sin(2 * faik) + eph->Crc * cos(2 * faik);// 向径改正数
    double dik = eph->Cis * sin(2 * faik) + eph->Cic * cos(2 * faik);// 轨道倾角改正数
    // 计算经过改正的升交角距
    double uk = faik + duk;
    // 计算经过改正的向径
    double rk = A * (1 - eph->e * cos(Ek)) + drk;
    // 计算经过改正的轨道倾角
    double ik = eph->i0 + dik + eph->iDot * tk;
    // 计算卫星在轨道平面上的位置
    double xk1 = rk * cos(uk);
    double yk1 = rk * sin(uk);
    // 计算改正后的升交点经度
    double OMEGAk = eph->OMEGA + (eph->OMEGADot - Omega_WGS) * tk - Omega_WGS * eph->TOE.SecOfWeek;
    // 计算在地固坐标系下的位置
    double xk = xk1 * cos(OMEGAk) - yk1 * cos(ik) * sin(OMEGAk);
    double yk = xk1 * sin(OMEGAk) + yk1 * cos(ik) * cos(OMEGAk);
    double zk = yk1 * sin(ik);
    Mid->SatPos[0] = xk;
    Mid->SatPos[1] = yk;
    Mid->SatPos[2] = zk;

    // 计算卫星运动速度
    double Ekdot = n / (1 - eph->e * cos(Ek));
    double faikdot = sqrt((1 + eph->e) / (1 - eph->e)) * (cos(vk / 2) * cos(vk / 2) / (cos(Ek / 2) * cos(Ek / 2))) * Ekdot;
    double ukdot = 2 * (eph->Cus * cos(2 * faik) - eph->Cuc * sin(2 * faik)) * faikdot + faikdot;
    double rkdot = A * eph->e * sin(Ek) * Ekdot + 2 * (eph->Crs * cos(2 * faik) - eph->Crc * sin(2 * faik)) * faikdot;
    double ikdot = eph->iDot + 2 * (eph->Cis * cos(2 * faik) - eph->Cic * sin(2 * faik)) * faikdot;
    double OMEGAkdot = eph->OMEGADot - Omega_WGS;
    // 列出Rdot矩阵
    double Rdot[12] = { cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk1 * sin(OMEGAk) + yk1 * cos(OMEGAk) * cos(ik)), yk1 * sin(OMEGAk) * sin(ik),
        sin(OMEGAk), cos(OMEGAk) * cos(ik), (xk1 * cos(OMEGAk) - yk1 * sin(OMEGAk) * cos(ik)), -yk1 * cos(OMEGAk) * sin(ik),
        0, sin(ik), 0, yk1 * cos(ik) };
    // 列出辅助量
    double xk1dot = rkdot * cos(uk) - rk * ukdot * sin(uk);
    double yk1dot = rkdot * sin(uk) + rk * ukdot * cos(uk);
    double xyomegai[4] = { xk1dot, yk1dot, OMEGAkdot, ikdot };
    MatrixMultiply(3, 4, 4, 1, Rdot, xyomegai, Mid->SatVel);

    // 计算钟差
    double dtr = -4.442807633e-10 * eph->e * eph->SqrtA * sin(Ek); // 相对论效应改正
    double dtsv = eph->ClkBias + eph->ClkDrift * (GetDiffTime(t, &eph->TOC)) + eph->ClkDriftRate * (GetDiffTime(t, &eph->TOC) * GetDiffTime(t, &eph->TOC))+dtr;
    Mid->SatClkOft = dtsv;

    // 计算钟速
    double dtrdot = -4.442807633e-10 * eph->e * eph->SqrtA * cos(Ek) * Ekdot; // 相对论效应改正
    double ddsv = eph->ClkDrift + 2 * eph->ClkDriftRate * (GetDiffTime(t, &eph->TOC))+ dtrdot;
    Mid->SatClkSft = ddsv;

    // 硬件延迟项
    Mid->Tgd1 = eph->TGD1;
    Mid->Tgd2 = eph->TGD2;

    return true;
}

/*
*********************************************************************
函数名：计算BDS中间量（卫星的位置、速度，钟差、钟速，硬件延迟）的函数
参数：Prn     卫星号
      t       需知中间量对应的时刻（GPS时）
      eph     BDS广播星历
      Mid     所得中间量结果存储体
返回值：true-计算成功 false-计算失败
函数功能：根据卫星广播星历来计算中间量
*********************************************************************
*/
bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* eph, SATMIDRES* Mid)
{
    // 把t转化到BDS时下
    GPSTIME tim;
    tim.Week = t->Week - 1356;
    tim.SecOfWeek = t->SecOfWeek - 14;
    // 星历是否过期或健康判断
    if (eph->SVHealth == 1 || (fabs(GetDiffTime(&tim, &eph->TOE)) > 3900))
    {
        Mid->Valid = false;
        return false;
    }
    Mid->Valid = true;


    // 计算卫星位置
    // 计算轨道长半轴
    double A = eph->SqrtA * eph->SqrtA;
    // 计算平均运动角速度
    double n0 = sqrt(GM_BDS / (A * A * A));// rad/s
    // 计算相对于星历参考历元的时间
    double tk = GetDiffTime(&tim, &eph->TOE);
    // 对平均运动角速度进行改正
    double n = n0 + eph->DeltaN;// rad/s
    // 计算平近点角
    double Mk = eph->M0 + n * tk;// rad
    // 计算偏近点角（迭代）
    double Ek = 0;// rad 
    double Et = Mk;//迭代器
    while (fabs(Et - Ek) > 1e-12)
    {
        Et = Ek;
        Ek = Mk + eph->e * sin(Ek);
    }
    // 计算真近点角
    double vk = atan2(sqrt(1 - eph->e * eph->e) * sin(Ek), (cos(Ek) - eph->e));
    // 计算升交角距
    double faik = vk + eph->omega;
    // 计算二阶调和改正数
    double duk = eph->Cus * sin(2 * faik) + eph->Cuc * cos(2 * faik);//升交角距改正数
    double drk = eph->Crs * sin(2 * faik) + eph->Crc * cos(2 * faik);//向径改正数
    double dik = eph->Cis * sin(2 * faik) + eph->Cic * cos(2 * faik);//轨道倾角改正数
    // 计算经过改正的升交角距
    double uk = faik + duk;
    // 计算经过改正的向径
    double rk = A * (1 - eph->e * cos(Ek)) + drk;
    // 计算经过改正的轨道倾角
    double ik = eph->i0 + dik + eph->iDot * tk;
    // 计算卫星在轨道平面上的位置
    double xk1 = rk * cos(uk);
    double yk1 = rk * sin(uk);
    double OMEGAk;

    // 根据卫星种类不同进行选择
    // GEO卫星
    if ((Prn >= 1 && Prn <= 5) || (Prn >= 59 && Prn <= 63))
    {
        // 计算历元升交点经度
        OMEGAk = eph->OMEGA + eph->OMEGADot * tk - Omega_BDS * eph->TOE.SecOfWeek;
        // 卫星在自定义的坐标系中的坐标
        double xgk = xk1 * cos(OMEGAk) - yk1 * cos(ik) * sin(OMEGAk);
        double ygk = xk1 * sin(OMEGAk) + yk1 * cos(ik) * cos(OMEGAk);
        double zgk = yk1 * sin(ik);
        double gk[3] = { xgk,ygk,zgk };
        // 投影矩阵
        double Rx[9] = { 1,0,0,0,cos(-5 * Rad),sin(-5 * Rad),0,-sin(-5 * Rad),cos(-5 * Rad) };
        double Rz[9] = { cos(Omega_BDS * tk),sin(Omega_BDS * tk),0,-sin(Omega_BDS * tk),cos(Omega_BDS * tk),0,0,0,1 };
        double Rzx[9];
        MatrixMultiply(3, 3, 3, 3, Rz, Rx, Rzx);
        MatrixMultiply(3, 3, 3, 1, Rzx, gk, Mid->SatPos);

        // 计算卫星运动速度
        double Ekdot = n / (1 - eph->e * cos(Ek));
        double faikdot = sqrt((1 + eph->e) / (1 - eph->e)) * (cos(vk / 2) * cos(vk / 2) / (cos(Ek / 2) * cos(Ek / 2))) * Ekdot;
        double ukdot = 2 * (eph->Cus * cos(2 * faik) - eph->Cuc * sin(2 * faik)) * faikdot + faikdot;
        double rkdot = A * eph->e * sin(Ek) * Ekdot + 2 * (eph->Crs * cos(2 * faik) - eph->Crc * sin(2 * faik)) * faikdot;
        double ikdot = eph->iDot + 2 * (eph->Cis * cos(2 * faik) - eph->Cic * sin(2 * faik)) * faikdot;

        double OMEGAkdot = eph->OMEGADot;
        // 列出Rdot矩阵
        double Rdot[12] = { cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk1 * sin(OMEGAk) + yk1 * cos(OMEGAk) * cos(ik)), yk1 * sin(OMEGAk) * sin(ik),
            sin(OMEGAk), cos(OMEGAk) * cos(ik), (xk1 * cos(OMEGAk) - yk1 * sin(OMEGAk) * cos(ik)), -yk1 * cos(OMEGAk) * sin(ik),
            0, sin(ik), 0, yk1 * cos(ik) };
        // 列出辅助量
        double xk1dot = rkdot * cos(uk) - rk * ukdot * sin(uk);
        double yk1dot = rkdot * sin(uk) + rk * ukdot * cos(uk);
        double xyomegai[4] = { xk1dot, yk1dot, OMEGAkdot, ikdot };
        double Rzdot[9] = { -sin(Omega_BDS * tk) * Omega_BDS ,cos(Omega_BDS * tk) * Omega_BDS ,0,-cos(Omega_BDS * tk) * Omega_BDS ,-sin(Omega_BDS * tk) * Omega_BDS ,0,0,0,0 };
        double RzdotRx[9];
        MatrixMultiply(3, 3, 3, 3, Rzdot, Rx, RzdotRx);
        double Rzxr[12];
        MatrixMultiply(3, 3, 3, 4, Rzx, Rdot, Rzxr);
        double R1[3];
        double R2[3];
        MatrixMultiply(3, 4, 4, 1, Rzxr, xyomegai, R1);
        MatrixMultiply(3, 3, 3, 1, RzdotRx, gk, R2);
        MatrixAddition(3, 1, R1, R2, Mid->SatVel);

        // 计算钟差
        double dtr = -4.442807633e-10 * eph->e * eph->SqrtA * sin(Ek);// 相对论效应改正
        double dtsv = eph->ClkBias + eph->ClkDrift * (GetDiffTime(&tim, &eph->TOC)) + eph->ClkDriftRate * (GetDiffTime(&tim, &eph->TOC) * GetDiffTime(&tim, &eph->TOC))+dtr;
        Mid->SatClkOft = dtsv;

        // 计算钟速
        double dtrdot = -4.442807633e-10 * eph->e * eph->SqrtA * cos(Ek) * Ekdot;// 相对论效应改正
        double ddsv = eph->ClkDrift + 2 * eph->ClkDriftRate * (GetDiffTime(&tim, &eph->TOC))+ dtrdot;
        Mid->SatClkSft = ddsv;
    }
    // MEO或IGSO卫星
    else if(Prn>5&&Prn<59)
    {
        // 计算改正后的升交点经度
        OMEGAk = eph->OMEGA + (eph->OMEGADot - Omega_BDS) * tk - Omega_BDS * eph->TOE.SecOfWeek;
        // 计算在地固坐标系下的位置
        double xk = xk1 * cos(OMEGAk) - yk1 * cos(ik) * sin(OMEGAk);
        double yk = xk1 * sin(OMEGAk) + yk1 * cos(ik) * cos(OMEGAk);
        double zk = yk1 * sin(ik);
        Mid->SatPos[0] = xk;
        Mid->SatPos[1] = yk;
        Mid->SatPos[2] = zk;

        // 计算卫星运动速度
        double Ekdot = n / (1 - eph->e * cos(Ek));
        double faikdot = sqrt((1 + eph->e) / (1 - eph->e)) * (cos(vk / 2) * cos(vk / 2) / (cos(Ek / 2) * cos(Ek / 2))) * Ekdot;
        double ukdot = 2 * (eph->Cus * cos(2 * faik) - eph->Cuc * sin(2 * faik)) * faikdot + faikdot;
        double rkdot = A * eph->e * sin(Ek) * Ekdot + 2 * (eph->Crs * cos(2 * faik) - eph->Crc * sin(2 * faik)) * faikdot;
        double ikdot = eph->iDot + 2 * (eph->Cis * cos(2 * faik) - eph->Cic * sin(2 * faik)) * faikdot;
        double OMEGAkdot = eph->OMEGADot - Omega_BDS;
        // 列出Rdot矩阵
        double Rdot[12] = { cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk1 * sin(OMEGAk) + yk1 * cos(OMEGAk) * cos(ik)), yk1 * sin(OMEGAk) * sin(ik),
            sin(OMEGAk), cos(OMEGAk) * cos(ik), (xk1 * cos(OMEGAk) - yk1 * sin(OMEGAk) * cos(ik)), -yk1 * cos(OMEGAk) * sin(ik),
            0, sin(ik), 0, yk1 * cos(ik) };
        // 列出辅助量
        double xk1dot = rkdot * cos(uk) - rk * ukdot * sin(uk);
        double yk1dot = rkdot * sin(uk) + rk * ukdot * cos(uk);
        double xyomegai[4] = { xk1dot, yk1dot, OMEGAkdot, ikdot };
        MatrixMultiply(3, 4, 4, 1, Rdot, xyomegai, Mid->SatVel);

        // 计算钟差
        double dtr = -4.442807633e-10 * eph->e * eph->SqrtA * sin(Ek);// 相对论效应改正
        double dtsv = eph->ClkBias + eph->ClkDrift * (GetDiffTime(&tim, &eph->TOC)) + eph->ClkDriftRate * (GetDiffTime(&tim, &eph->TOC) * GetDiffTime(&tim, &eph->TOC))+dtr ;
        Mid->SatClkOft = dtsv;

        // 计算钟速
        double dtrdot = -4.442807633e-10 * eph->e * eph->SqrtA * cos(Ek) * Ekdot;// 相对论效应改正
        double ddsv = eph->ClkDrift + 2 * eph->ClkDriftRate * (GetDiffTime(&tim, &eph->TOC))+ dtrdot;
        Mid->SatClkSft = ddsv;
    }
    // 硬件延迟项
    Mid->Tgd1 = eph->TGD1;
    Mid->Tgd2 = eph->TGD2;

    return true;

}

/*
*********************************************************************
函数名：计算信号发射时刻的卫星位置、速度、钟差、钟速
参数：Epk     当前历元观测数据
      GPSEph  GPS星历
      BDSEph  BDS星历
      RcvPos  用户的XYZ坐标 m
函数功能：同函数名
*********************************************************************
*/
void ComputeGPSSatOrbitAtSignalTrans(EPOCHOBS* Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double RcvPos[3])
{
    GPSTIME t_clock = Epk->Time;
    for (int i = 0; i < Epk->SatNum; i++)
    {
        GPSTIME t_AtSignalTrans;
        double dt = 0;// 迭代计算卫星钟差阈值判断
        if (Epk->SatObs[i].System == GPS)
        {
            GPSEPHREC* eph = GPSEph + Epk->SatObs[i].Prn - 1;// 取用本组观测值对应的卫星的星历
            Epk->SatPVT[i].SatClkOft = 0;// 初始钟差为0
            do
            {   // 计算卫星信号发射时刻
               t_AtSignalTrans.Week = t_clock.Week;
               t_AtSignalTrans.SecOfWeek = t_clock.SecOfWeek - Epk->SatObs[i].p[0] / C_Light - Epk->SatPVT[i].SatClkOft ;
                // 计算卫星钟差
                double st_tmp= eph->ClkBias + eph->ClkDrift * (GetDiffTime(&t_AtSignalTrans, &eph->TOC)) + eph->ClkDriftRate * (GetDiffTime(&t_AtSignalTrans, &eph->TOC) * GetDiffTime(&t_AtSignalTrans, &eph->TOC));
                // 更新判断值
                dt = fabs(st_tmp - Epk->SatPVT[i].SatClkOft);
                // 更新卫星钟差
                Epk->SatPVT[i].SatClkOft = st_tmp;
            } while (dt > 1e-12);
            // 卫星钟差迭代计算完毕
            // 计算卫星位置、速度、钟差和钟速
            CompGPSSatPVT(Epk->SatObs[i].Prn, &t_AtSignalTrans, eph, Epk->SatPVT + i);
            // 计算信号传播时间
            double t_trans = sqrt((Epk->SatPVT[i].SatPos[0]- RcvPos[0]) * (Epk->SatPVT[i].SatPos[0] - RcvPos[0])+ (Epk->SatPVT[i].SatPos[1] - RcvPos[1]) * (Epk->SatPVT[i].SatPos[1] - RcvPos[1])+ (Epk->SatPVT[i].SatPos[2] - RcvPos[2]) * (Epk->SatPVT[i].SatPos[2] - RcvPos[2])) / C_Light;
            // 地球自转改正
            double alpha = Omega_WGS * t_trans;
            double Rz[9] = { cos(alpha), sin(alpha), 0, -sin(alpha), cos(alpha), 0, 0, 0, 1 };
            double pos[3] = { Epk->SatPVT[i].SatPos[0] ,Epk->SatPVT[i].SatPos[1],Epk->SatPVT[i].SatPos[2] };
            double vel[3] = { Epk->SatPVT[i].SatVel[0] ,Epk->SatPVT[i].SatVel[1],Epk->SatPVT[i].SatVel[2] };
            // 位置改正
            MatrixMultiply(3, 3, 3, 1, Rz, pos, Epk->SatPVT[i].SatPos);
            // 速度改正
            MatrixMultiply(3, 3, 3, 1, Rz, vel, Epk->SatPVT[i].SatVel);
            // 计算卫星的高度角和方位角
            CompSatElAz(1,RcvPos, Epk->SatPVT[i].SatPos, &Epk->SatPVT[i].Elevation, &Epk->SatPVT[i].Azimuth);
            // 计算对流层改正
            double blh[3];// 测站坐标
            XYZToBLH(RcvPos, blh, R_WGS84, F_WGS84);
            Epk->SatPVT[i].TropCorr = hopfield(blh[2], Epk->SatPVT[i].Elevation);

        }
        else if (Epk->SatObs[i].System == BDS)
        {
           
            GPSEPHREC* eph = BDSEph + Epk->SatObs[i].Prn - 1;// 取用本组观测值对应的卫星的星历
            Epk->SatPVT[i].SatClkOft = 0;// 初始钟差为0
            do
            {   // 计算卫星信号发射时刻
                t_AtSignalTrans.Week = t_clock.Week;
                t_AtSignalTrans.SecOfWeek = t_clock.SecOfWeek - Epk->SatObs[i].p[0] / C_Light - Epk->SatPVT[i].SatClkOft;
                // 计算卫星钟差
                GPSTIME tim;
                tim.Week = t_AtSignalTrans.Week - 1356;
                tim.SecOfWeek = t_AtSignalTrans.SecOfWeek - 14;
                double st_tmp = eph->ClkBias + eph->ClkDrift * (GetDiffTime(&tim, &eph->TOC)) + eph->ClkDriftRate * (GetDiffTime(&tim, &eph->TOC) * GetDiffTime(&tim, &eph->TOC));
                // 更新判断值
                dt = fabs(st_tmp - Epk->SatPVT[i].SatClkOft);
                // 更新卫星钟差
                Epk->SatPVT[i].SatClkOft = st_tmp;
            } while (dt > 1e-12);
            // 卫星钟差迭代计算完毕
            // 计算卫星位置、速度、钟差和钟速
            CompBDSSatPVT(Epk->SatObs[i].Prn, &t_AtSignalTrans, eph, Epk->SatPVT + i);
            // 计算信号传播时间
            double t_trans = sqrt((Epk->SatPVT[i].SatPos[0] - RcvPos[0]) * (Epk->SatPVT[i].SatPos[0] - RcvPos[0]) + (Epk->SatPVT[i].SatPos[1] - RcvPos[1]) * (Epk->SatPVT[i].SatPos[1] - RcvPos[1]) + (Epk->SatPVT[i].SatPos[2] - RcvPos[2]) * (Epk->SatPVT[i].SatPos[2] - RcvPos[2])) / C_Light;
            // 地球自转改正
            double alpha = Omega_BDS * t_trans;
            double Rz[9] = { cos(alpha), sin(alpha), 0, -sin(alpha), cos(alpha), 0, 0, 0, 1 };
            double pos[3] = { Epk->SatPVT[i].SatPos[0] ,Epk->SatPVT[i].SatPos[1],Epk->SatPVT[i].SatPos[2] };
            double vel[3] = { Epk->SatPVT[i].SatVel[0] ,Epk->SatPVT[i].SatVel[1],Epk->SatPVT[i].SatVel[2] };
            // 位置改正
            MatrixMultiply(3, 3, 3, 1, Rz, pos, Epk->SatPVT[i].SatPos);
            // 速度改正
            MatrixMultiply(3, 3, 3, 1, Rz, vel, Epk->SatPVT[i].SatVel);
            // 计算卫星的高度角和方位角
            CompSatElAz(2,RcvPos, Epk->SatPVT[i].SatPos, &Epk->SatPVT[i].Elevation, &Epk->SatPVT[i].Azimuth);
            // 计算对流层改正
            double blh[3];// 测站的坐标
            XYZToBLH(RcvPos, blh, R_CGS2K, F_CGS2K);
            Epk->SatPVT[i].TropCorr = hopfield(blh[2], Epk->SatPVT[i].Elevation);
        }
    }
}



