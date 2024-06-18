#include"MyHeadFile.h"


/*
*********************************************************************
������������GPS�м��������ǵ�λ�á��ٶȣ��Ӳ���٣�Ӳ���ӳ٣��ĺ���
������Prn     ���Ǻ�
      t       ��֪�м�����Ӧ��ʱ�̣�GPSʱ��
      eph     GPS�㲥����
      Mid     �����м�������洢��
����ֵ��true-����ɹ� false-����ʧ��
�������ܣ��������ǹ㲥�����������м���
*********************************************************************
*/
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* eph, SATMIDRES* Mid)
{
    // �����Ƿ���ڻ򽡿��ж�
    if (eph->SVHealth == 1 || (fabs(GetDiffTime(t, &eph->TOE)) > 7500))
    {
        Mid->Valid = false;
        return false;
    }
    Mid->Valid = true;
    // ��������λ��
    // ������������
    double A = eph->SqrtA * eph->SqrtA;
    // ����ƽ���˶����ٶ�
    double n0 = sqrt(GM_WGS / (A * A * A));// rad/s
    // ��������������ο���Ԫ��ʱ��
    double tk = GetDiffTime(t, &eph->TOE);
    // ��ƽ���˶����ٶȽ��и���
    double n = n0 + eph->DeltaN;// rad/s
    // ����ƽ�����
    double Mk = eph->M0 + n * tk;// rad
    // ����ƫ����ǣ�������
    double Ek = 0;// rad 
    double Et = Mk;// ������
    while (fabs(Et - Ek) > 1e-12)
    {
        Et = Ek;
        Ek = Mk + eph->e * sin(Ek);
    }
    // ����������
    double vk = atan2(sqrt(1 - eph->e * eph->e) * sin(Ek), cos(Ek) - eph->e);
    // ���������Ǿ�
    double faik = vk + eph->omega;
    // ������׵��͸�����
    double duk = eph->Cus * sin(2 * faik) + eph->Cuc * cos(2 * faik);// �����Ǿ������
    double drk = eph->Crs * sin(2 * faik) + eph->Crc * cos(2 * faik);// �򾶸�����
    double dik = eph->Cis * sin(2 * faik) + eph->Cic * cos(2 * faik);// �����Ǹ�����
    // ���㾭�������������Ǿ�
    double uk = faik + duk;
    // ���㾭����������
    double rk = A * (1 - eph->e * cos(Ek)) + drk;
    // ���㾭�������Ĺ�����
    double ik = eph->i0 + dik + eph->iDot * tk;
    // ���������ڹ��ƽ���ϵ�λ��
    double xk1 = rk * cos(uk);
    double yk1 = rk * sin(uk);
    // ���������������㾭��
    double OMEGAk = eph->OMEGA + (eph->OMEGADot - Omega_WGS) * tk - Omega_WGS * eph->TOE.SecOfWeek;
    // �����ڵع�����ϵ�µ�λ��
    double xk = xk1 * cos(OMEGAk) - yk1 * cos(ik) * sin(OMEGAk);
    double yk = xk1 * sin(OMEGAk) + yk1 * cos(ik) * cos(OMEGAk);
    double zk = yk1 * sin(ik);
    Mid->SatPos[0] = xk;
    Mid->SatPos[1] = yk;
    Mid->SatPos[2] = zk;

    // ���������˶��ٶ�
    double Ekdot = n / (1 - eph->e * cos(Ek));
    double faikdot = sqrt((1 + eph->e) / (1 - eph->e)) * (cos(vk / 2) * cos(vk / 2) / (cos(Ek / 2) * cos(Ek / 2))) * Ekdot;
    double ukdot = 2 * (eph->Cus * cos(2 * faik) - eph->Cuc * sin(2 * faik)) * faikdot + faikdot;
    double rkdot = A * eph->e * sin(Ek) * Ekdot + 2 * (eph->Crs * cos(2 * faik) - eph->Crc * sin(2 * faik)) * faikdot;
    double ikdot = eph->iDot + 2 * (eph->Cis * cos(2 * faik) - eph->Cic * sin(2 * faik)) * faikdot;
    double OMEGAkdot = eph->OMEGADot - Omega_WGS;
    // �г�Rdot����
    double Rdot[12] = { cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk1 * sin(OMEGAk) + yk1 * cos(OMEGAk) * cos(ik)), yk1 * sin(OMEGAk) * sin(ik),
        sin(OMEGAk), cos(OMEGAk) * cos(ik), (xk1 * cos(OMEGAk) - yk1 * sin(OMEGAk) * cos(ik)), -yk1 * cos(OMEGAk) * sin(ik),
        0, sin(ik), 0, yk1 * cos(ik) };
    // �г�������
    double xk1dot = rkdot * cos(uk) - rk * ukdot * sin(uk);
    double yk1dot = rkdot * sin(uk) + rk * ukdot * cos(uk);
    double xyomegai[4] = { xk1dot, yk1dot, OMEGAkdot, ikdot };
    MatrixMultiply(3, 4, 4, 1, Rdot, xyomegai, Mid->SatVel);

    // �����Ӳ�
    double dtr = -4.442807633e-10 * eph->e * eph->SqrtA * sin(Ek); // �����ЧӦ����
    double dtsv = eph->ClkBias + eph->ClkDrift * (GetDiffTime(t, &eph->TOC)) + eph->ClkDriftRate * (GetDiffTime(t, &eph->TOC) * GetDiffTime(t, &eph->TOC))+dtr;
    Mid->SatClkOft = dtsv;

    // ��������
    double dtrdot = -4.442807633e-10 * eph->e * eph->SqrtA * cos(Ek) * Ekdot; // �����ЧӦ����
    double ddsv = eph->ClkDrift + 2 * eph->ClkDriftRate * (GetDiffTime(t, &eph->TOC))+ dtrdot;
    Mid->SatClkSft = ddsv;

    // Ӳ���ӳ���
    Mid->Tgd1 = eph->TGD1;
    Mid->Tgd2 = eph->TGD2;

    return true;
}

/*
*********************************************************************
������������BDS�м��������ǵ�λ�á��ٶȣ��Ӳ���٣�Ӳ���ӳ٣��ĺ���
������Prn     ���Ǻ�
      t       ��֪�м�����Ӧ��ʱ�̣�GPSʱ��
      eph     BDS�㲥����
      Mid     �����м�������洢��
����ֵ��true-����ɹ� false-����ʧ��
�������ܣ��������ǹ㲥�����������м���
*********************************************************************
*/
bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* eph, SATMIDRES* Mid)
{
    // ��tת����BDSʱ��
    GPSTIME tim;
    tim.Week = t->Week - 1356;
    tim.SecOfWeek = t->SecOfWeek - 14;
    // �����Ƿ���ڻ򽡿��ж�
    if (eph->SVHealth == 1 || (fabs(GetDiffTime(&tim, &eph->TOE)) > 3900))
    {
        Mid->Valid = false;
        return false;
    }
    Mid->Valid = true;


    // ��������λ��
    // ������������
    double A = eph->SqrtA * eph->SqrtA;
    // ����ƽ���˶����ٶ�
    double n0 = sqrt(GM_BDS / (A * A * A));// rad/s
    // ��������������ο���Ԫ��ʱ��
    double tk = GetDiffTime(&tim, &eph->TOE);
    // ��ƽ���˶����ٶȽ��и���
    double n = n0 + eph->DeltaN;// rad/s
    // ����ƽ�����
    double Mk = eph->M0 + n * tk;// rad
    // ����ƫ����ǣ�������
    double Ek = 0;// rad 
    double Et = Mk;//������
    while (fabs(Et - Ek) > 1e-12)
    {
        Et = Ek;
        Ek = Mk + eph->e * sin(Ek);
    }
    // ����������
    double vk = atan2(sqrt(1 - eph->e * eph->e) * sin(Ek), (cos(Ek) - eph->e));
    // ���������Ǿ�
    double faik = vk + eph->omega;
    // ������׵��͸�����
    double duk = eph->Cus * sin(2 * faik) + eph->Cuc * cos(2 * faik);//�����Ǿ������
    double drk = eph->Crs * sin(2 * faik) + eph->Crc * cos(2 * faik);//�򾶸�����
    double dik = eph->Cis * sin(2 * faik) + eph->Cic * cos(2 * faik);//�����Ǹ�����
    // ���㾭�������������Ǿ�
    double uk = faik + duk;
    // ���㾭����������
    double rk = A * (1 - eph->e * cos(Ek)) + drk;
    // ���㾭�������Ĺ�����
    double ik = eph->i0 + dik + eph->iDot * tk;
    // ���������ڹ��ƽ���ϵ�λ��
    double xk1 = rk * cos(uk);
    double yk1 = rk * sin(uk);
    double OMEGAk;

    // �����������಻ͬ����ѡ��
    // GEO����
    if ((Prn >= 1 && Prn <= 5) || (Prn >= 59 && Prn <= 63))
    {
        // ������Ԫ�����㾭��
        OMEGAk = eph->OMEGA + eph->OMEGADot * tk - Omega_BDS * eph->TOE.SecOfWeek;
        // �������Զ��������ϵ�е�����
        double xgk = xk1 * cos(OMEGAk) - yk1 * cos(ik) * sin(OMEGAk);
        double ygk = xk1 * sin(OMEGAk) + yk1 * cos(ik) * cos(OMEGAk);
        double zgk = yk1 * sin(ik);
        double gk[3] = { xgk,ygk,zgk };
        // ͶӰ����
        double Rx[9] = { 1,0,0,0,cos(-5 * Rad),sin(-5 * Rad),0,-sin(-5 * Rad),cos(-5 * Rad) };
        double Rz[9] = { cos(Omega_BDS * tk),sin(Omega_BDS * tk),0,-sin(Omega_BDS * tk),cos(Omega_BDS * tk),0,0,0,1 };
        double Rzx[9];
        MatrixMultiply(3, 3, 3, 3, Rz, Rx, Rzx);
        MatrixMultiply(3, 3, 3, 1, Rzx, gk, Mid->SatPos);

        // ���������˶��ٶ�
        double Ekdot = n / (1 - eph->e * cos(Ek));
        double faikdot = sqrt((1 + eph->e) / (1 - eph->e)) * (cos(vk / 2) * cos(vk / 2) / (cos(Ek / 2) * cos(Ek / 2))) * Ekdot;
        double ukdot = 2 * (eph->Cus * cos(2 * faik) - eph->Cuc * sin(2 * faik)) * faikdot + faikdot;
        double rkdot = A * eph->e * sin(Ek) * Ekdot + 2 * (eph->Crs * cos(2 * faik) - eph->Crc * sin(2 * faik)) * faikdot;
        double ikdot = eph->iDot + 2 * (eph->Cis * cos(2 * faik) - eph->Cic * sin(2 * faik)) * faikdot;

        double OMEGAkdot = eph->OMEGADot;
        // �г�Rdot����
        double Rdot[12] = { cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk1 * sin(OMEGAk) + yk1 * cos(OMEGAk) * cos(ik)), yk1 * sin(OMEGAk) * sin(ik),
            sin(OMEGAk), cos(OMEGAk) * cos(ik), (xk1 * cos(OMEGAk) - yk1 * sin(OMEGAk) * cos(ik)), -yk1 * cos(OMEGAk) * sin(ik),
            0, sin(ik), 0, yk1 * cos(ik) };
        // �г�������
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

        // �����Ӳ�
        double dtr = -4.442807633e-10 * eph->e * eph->SqrtA * sin(Ek);// �����ЧӦ����
        double dtsv = eph->ClkBias + eph->ClkDrift * (GetDiffTime(&tim, &eph->TOC)) + eph->ClkDriftRate * (GetDiffTime(&tim, &eph->TOC) * GetDiffTime(&tim, &eph->TOC))+dtr;
        Mid->SatClkOft = dtsv;

        // ��������
        double dtrdot = -4.442807633e-10 * eph->e * eph->SqrtA * cos(Ek) * Ekdot;// �����ЧӦ����
        double ddsv = eph->ClkDrift + 2 * eph->ClkDriftRate * (GetDiffTime(&tim, &eph->TOC))+ dtrdot;
        Mid->SatClkSft = ddsv;
    }
    // MEO��IGSO����
    else if(Prn>5&&Prn<59)
    {
        // ���������������㾭��
        OMEGAk = eph->OMEGA + (eph->OMEGADot - Omega_BDS) * tk - Omega_BDS * eph->TOE.SecOfWeek;
        // �����ڵع�����ϵ�µ�λ��
        double xk = xk1 * cos(OMEGAk) - yk1 * cos(ik) * sin(OMEGAk);
        double yk = xk1 * sin(OMEGAk) + yk1 * cos(ik) * cos(OMEGAk);
        double zk = yk1 * sin(ik);
        Mid->SatPos[0] = xk;
        Mid->SatPos[1] = yk;
        Mid->SatPos[2] = zk;

        // ���������˶��ٶ�
        double Ekdot = n / (1 - eph->e * cos(Ek));
        double faikdot = sqrt((1 + eph->e) / (1 - eph->e)) * (cos(vk / 2) * cos(vk / 2) / (cos(Ek / 2) * cos(Ek / 2))) * Ekdot;
        double ukdot = 2 * (eph->Cus * cos(2 * faik) - eph->Cuc * sin(2 * faik)) * faikdot + faikdot;
        double rkdot = A * eph->e * sin(Ek) * Ekdot + 2 * (eph->Crs * cos(2 * faik) - eph->Crc * sin(2 * faik)) * faikdot;
        double ikdot = eph->iDot + 2 * (eph->Cis * cos(2 * faik) - eph->Cic * sin(2 * faik)) * faikdot;
        double OMEGAkdot = eph->OMEGADot - Omega_BDS;
        // �г�Rdot����
        double Rdot[12] = { cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk1 * sin(OMEGAk) + yk1 * cos(OMEGAk) * cos(ik)), yk1 * sin(OMEGAk) * sin(ik),
            sin(OMEGAk), cos(OMEGAk) * cos(ik), (xk1 * cos(OMEGAk) - yk1 * sin(OMEGAk) * cos(ik)), -yk1 * cos(OMEGAk) * sin(ik),
            0, sin(ik), 0, yk1 * cos(ik) };
        // �г�������
        double xk1dot = rkdot * cos(uk) - rk * ukdot * sin(uk);
        double yk1dot = rkdot * sin(uk) + rk * ukdot * cos(uk);
        double xyomegai[4] = { xk1dot, yk1dot, OMEGAkdot, ikdot };
        MatrixMultiply(3, 4, 4, 1, Rdot, xyomegai, Mid->SatVel);

        // �����Ӳ�
        double dtr = -4.442807633e-10 * eph->e * eph->SqrtA * sin(Ek);// �����ЧӦ����
        double dtsv = eph->ClkBias + eph->ClkDrift * (GetDiffTime(&tim, &eph->TOC)) + eph->ClkDriftRate * (GetDiffTime(&tim, &eph->TOC) * GetDiffTime(&tim, &eph->TOC))+dtr ;
        Mid->SatClkOft = dtsv;

        // ��������
        double dtrdot = -4.442807633e-10 * eph->e * eph->SqrtA * cos(Ek) * Ekdot;// �����ЧӦ����
        double ddsv = eph->ClkDrift + 2 * eph->ClkDriftRate * (GetDiffTime(&tim, &eph->TOC))+ dtrdot;
        Mid->SatClkSft = ddsv;
    }
    // Ӳ���ӳ���
    Mid->Tgd1 = eph->TGD1;
    Mid->Tgd2 = eph->TGD2;

    return true;

}

/*
*********************************************************************
�������������źŷ���ʱ�̵�����λ�á��ٶȡ��Ӳ����
������Epk     ��ǰ��Ԫ�۲�����
      GPSEph  GPS����
      BDSEph  BDS����
      RcvPos  �û���XYZ���� m
�������ܣ�ͬ������
*********************************************************************
*/
void ComputeGPSSatOrbitAtSignalTrans(EPOCHOBS* Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double RcvPos[3])
{
    GPSTIME t_clock = Epk->Time;
    for (int i = 0; i < Epk->SatNum; i++)
    {
        GPSTIME t_AtSignalTrans;
        double dt = 0;// �������������Ӳ���ֵ�ж�
        if (Epk->SatObs[i].System == GPS)
        {
            GPSEPHREC* eph = GPSEph + Epk->SatObs[i].Prn - 1;// ȡ�ñ���۲�ֵ��Ӧ�����ǵ�����
            Epk->SatPVT[i].SatClkOft = 0;// ��ʼ�Ӳ�Ϊ0
            do
            {   // ���������źŷ���ʱ��
               t_AtSignalTrans.Week = t_clock.Week;
               t_AtSignalTrans.SecOfWeek = t_clock.SecOfWeek - Epk->SatObs[i].p[0] / C_Light - Epk->SatPVT[i].SatClkOft ;
                // ���������Ӳ�
                double st_tmp= eph->ClkBias + eph->ClkDrift * (GetDiffTime(&t_AtSignalTrans, &eph->TOC)) + eph->ClkDriftRate * (GetDiffTime(&t_AtSignalTrans, &eph->TOC) * GetDiffTime(&t_AtSignalTrans, &eph->TOC));
                // �����ж�ֵ
                dt = fabs(st_tmp - Epk->SatPVT[i].SatClkOft);
                // ���������Ӳ�
                Epk->SatPVT[i].SatClkOft = st_tmp;
            } while (dt > 1e-12);
            // �����Ӳ�����������
            // ��������λ�á��ٶȡ��Ӳ������
            CompGPSSatPVT(Epk->SatObs[i].Prn, &t_AtSignalTrans, eph, Epk->SatPVT + i);
            // �����źŴ���ʱ��
            double t_trans = sqrt((Epk->SatPVT[i].SatPos[0]- RcvPos[0]) * (Epk->SatPVT[i].SatPos[0] - RcvPos[0])+ (Epk->SatPVT[i].SatPos[1] - RcvPos[1]) * (Epk->SatPVT[i].SatPos[1] - RcvPos[1])+ (Epk->SatPVT[i].SatPos[2] - RcvPos[2]) * (Epk->SatPVT[i].SatPos[2] - RcvPos[2])) / C_Light;
            // ������ת����
            double alpha = Omega_WGS * t_trans;
            double Rz[9] = { cos(alpha), sin(alpha), 0, -sin(alpha), cos(alpha), 0, 0, 0, 1 };
            double pos[3] = { Epk->SatPVT[i].SatPos[0] ,Epk->SatPVT[i].SatPos[1],Epk->SatPVT[i].SatPos[2] };
            double vel[3] = { Epk->SatPVT[i].SatVel[0] ,Epk->SatPVT[i].SatVel[1],Epk->SatPVT[i].SatVel[2] };
            // λ�ø���
            MatrixMultiply(3, 3, 3, 1, Rz, pos, Epk->SatPVT[i].SatPos);
            // �ٶȸ���
            MatrixMultiply(3, 3, 3, 1, Rz, vel, Epk->SatPVT[i].SatVel);
            // �������ǵĸ߶ȽǺͷ�λ��
            CompSatElAz(1,RcvPos, Epk->SatPVT[i].SatPos, &Epk->SatPVT[i].Elevation, &Epk->SatPVT[i].Azimuth);
            // ������������
            double blh[3];// ��վ����
            XYZToBLH(RcvPos, blh, R_WGS84, F_WGS84);
            Epk->SatPVT[i].TropCorr = hopfield(blh[2], Epk->SatPVT[i].Elevation);

        }
        else if (Epk->SatObs[i].System == BDS)
        {
           
            GPSEPHREC* eph = BDSEph + Epk->SatObs[i].Prn - 1;// ȡ�ñ���۲�ֵ��Ӧ�����ǵ�����
            Epk->SatPVT[i].SatClkOft = 0;// ��ʼ�Ӳ�Ϊ0
            do
            {   // ���������źŷ���ʱ��
                t_AtSignalTrans.Week = t_clock.Week;
                t_AtSignalTrans.SecOfWeek = t_clock.SecOfWeek - Epk->SatObs[i].p[0] / C_Light - Epk->SatPVT[i].SatClkOft;
                // ���������Ӳ�
                GPSTIME tim;
                tim.Week = t_AtSignalTrans.Week - 1356;
                tim.SecOfWeek = t_AtSignalTrans.SecOfWeek - 14;
                double st_tmp = eph->ClkBias + eph->ClkDrift * (GetDiffTime(&tim, &eph->TOC)) + eph->ClkDriftRate * (GetDiffTime(&tim, &eph->TOC) * GetDiffTime(&tim, &eph->TOC));
                // �����ж�ֵ
                dt = fabs(st_tmp - Epk->SatPVT[i].SatClkOft);
                // ���������Ӳ�
                Epk->SatPVT[i].SatClkOft = st_tmp;
            } while (dt > 1e-12);
            // �����Ӳ�����������
            // ��������λ�á��ٶȡ��Ӳ������
            CompBDSSatPVT(Epk->SatObs[i].Prn, &t_AtSignalTrans, eph, Epk->SatPVT + i);
            // �����źŴ���ʱ��
            double t_trans = sqrt((Epk->SatPVT[i].SatPos[0] - RcvPos[0]) * (Epk->SatPVT[i].SatPos[0] - RcvPos[0]) + (Epk->SatPVT[i].SatPos[1] - RcvPos[1]) * (Epk->SatPVT[i].SatPos[1] - RcvPos[1]) + (Epk->SatPVT[i].SatPos[2] - RcvPos[2]) * (Epk->SatPVT[i].SatPos[2] - RcvPos[2])) / C_Light;
            // ������ת����
            double alpha = Omega_BDS * t_trans;
            double Rz[9] = { cos(alpha), sin(alpha), 0, -sin(alpha), cos(alpha), 0, 0, 0, 1 };
            double pos[3] = { Epk->SatPVT[i].SatPos[0] ,Epk->SatPVT[i].SatPos[1],Epk->SatPVT[i].SatPos[2] };
            double vel[3] = { Epk->SatPVT[i].SatVel[0] ,Epk->SatPVT[i].SatVel[1],Epk->SatPVT[i].SatVel[2] };
            // λ�ø���
            MatrixMultiply(3, 3, 3, 1, Rz, pos, Epk->SatPVT[i].SatPos);
            // �ٶȸ���
            MatrixMultiply(3, 3, 3, 1, Rz, vel, Epk->SatPVT[i].SatVel);
            // �������ǵĸ߶ȽǺͷ�λ��
            CompSatElAz(2,RcvPos, Epk->SatPVT[i].SatPos, &Epk->SatPVT[i].Elevation, &Epk->SatPVT[i].Azimuth);
            // ������������
            double blh[3];// ��վ������
            XYZToBLH(RcvPos, blh, R_CGS2K, F_CGS2K);
            Epk->SatPVT[i].TropCorr = hopfield(blh[2], Epk->SatPVT[i].Elevation);
        }
    }
}



