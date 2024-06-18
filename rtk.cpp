#include"MyHeadFile.h"

extern ROVERCFGINFO cfginfo;

/*
****************************************************************************
������������ͬ���������ļ���
������FBas ָ���վ�����ļ���ָ��
      FRov ָ������վ�����ļ���ָ��
      Raw  ͬ�����ݴ洢��������վ�Ĺ۲�ֵ���������ο���λ�����
����ֵ������ͬ������ı�־  1-����ͬ���ɹ�  0-����ͬ��ʧ�� -1-�ļ����ݽ���
�������ܣ���ȡ����������վ��Ӧ�ļ��е����ݣ����������ʱ��ͬ��
****************************************************************************
*/
int GetSynObsPP(FILE* FBas, FILE* FRov, RAWDAT* Raw)
{
    // ����һ��Ԫ����վ���ݴ洢��
    memcpy(&Raw->BasEpk0, &Raw->BasEpk, sizeof(Raw->BasEpk0));
    memcpy(&Raw->RovEpk0, &Raw->RovEpk, sizeof(Raw->RovEpk0));
	// ��������վ����
    static unsigned char Rbuf[MAXRAWLEN];
    static int Rlen = 0;
    static int Rflag = 0;
    while (!feof(FRov))
    {
        int obtain_byte = fread(Rbuf + Rlen, 1, MAXRAWLEN - Rlen, FRov);
        if (obtain_byte < MAXRAWLEN - Rlen) return -1; // ����վ�����ļ�����
        Rlen = obtain_byte + Rlen;
        Rflag = DecodeNovOem7Dat(Rbuf, Rlen, &Raw->RovEpk, Raw->GpsEph, Raw->BdsEph, &Raw->RovPres);
        if (Rflag == 1) break; // ���뵽�˹۲�ֵ������ѭ��
    }
  
    // ����վʱ���ֵ(��λΪs)
    double dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);

    if (fabs(dt) < 0.1) return 1; // ʱ��ͬ��
    else if (dt < 0) return 0; // ����վ�����ͺ��޷�����
    else // ��վ�����ͺ���Ҫ��ȡ������ʱ�̵Ļ�վ����(ѭ��ִ��ֱ��ʱ��ͬ��Ϊֹ)
    {
        static unsigned char Bbuf[MAXRAWLEN];
        static int Blen = 0;
        static int Bflag = 0;
        bool flag = true;
        do
        {
            while (!feof(FBas))
            {
                int obtain_byte = fread(Bbuf + Blen, 1, MAXRAWLEN - Blen, FBas);
                if (obtain_byte < MAXRAWLEN - Blen)  return -1; // ��վ�����ļ�����
                Blen = obtain_byte + Blen;
                Bflag = DecodeNovOem7Dat(Bbuf, Blen, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph, &Raw->BasPres);
                if (Bflag == 1) break; // ���뵽�˹۲�ֵ������ѭ��
            }

            dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);

            if (dt < 0) return 0; // ��վ��Ԫ���ݶ�ʧ���޷�����

            if (fabs(dt) < 0.1)  flag = false;// ʱ�����ͬ��

        } while (flag);
       return 1; 
    }
	
}

/*
****************************************************************************
������������ͬ��������ʵʱ��
������BasSock ��վ�����ļ�������
      RovSock ����վ�����ļ�������
      Raw     ͬ�����ݴ洢��������վ�Ĺ۲�ֵ���������ο���λ�����
����ֵ������ͬ������ı�־  1-����ͬ���ɹ�  0-����ͬ��ʧ�� 
�������ܣ���ȡ����������վ��Ӧ�����е����ݣ����������ʱ��ͬ��
****************************************************************************
*/
int GetSynObsRT(SOCKET& BasSock, SOCKET& RovSock, RAWDAT* Raw)
{
    // ����һ��Ԫ����վ���ݴ洢��
    memcpy(&Raw->BasEpk0, &Raw->BasEpk, sizeof(Raw->BasEpk0));
    memcpy(&Raw->RovEpk0, &Raw->RovEpk, sizeof(Raw->RovEpk0));

    Sleep(900);

    // ��������վ����
    static unsigned char RBuf[MAXRAWLEN * 2];
    static int RlenB = 0; // RBuf�ĳ��ȣ��ֽ�����
    unsigned char Rbuf[MAXRAWLEN]; // �м�洢��
    int RlenT = 0; // �м�洢�صĳ���
    if ((RlenT = recv(RovSock, (char*)Rbuf, MAXRAWLEN, 0)) > 0)
    {
        if ((RlenB + RlenT) > MAXRAWLEN * 2)  RlenB = 0; // �����ζ������ݼ����BUFF��ᳬ������󳤶ȣ����Ƚ�ԭ�����е���������
        memcpy(RBuf + RlenB, Rbuf, RlenT);
        RlenB = RlenT + RlenB;
        DecodeNovOem7Dat(RBuf, RlenB, &Raw->RovEpk, Raw->GpsEph, Raw->BdsEph, &Raw->RovPres);

    }

    // �����վ����
    static unsigned char BBuf[MAXRAWLEN * 2];
    static int BlenB = 0; // BBuf�ĳ��ȣ��ֽ�����
    unsigned char Bbuf[MAXRAWLEN];
    int BlenT = 0; // �м�洢�صĳ���
    if ((BlenT = recv(BasSock, (char*)Bbuf, MAXRAWLEN, 0)) > 0)
    {
        if ((BlenB + BlenT) > MAXRAWLEN * 2)  BlenB = 0;
        memcpy(BBuf + BlenB, Bbuf, BlenT);
        BlenB = BlenT + BlenB;
        DecodeNovOem7Dat(BBuf, BlenB, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph, &Raw->BasPres);
    }

    // ����վʱ���ֵ(��λΪs)
    double dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);

    if (fabs(dt)<10 && (Raw->RovEpk.Time.SecOfWeek> Raw->RovEpk0.Time.SecOfWeek)) return 1; // ʱ��ͬ��
    else return 0; // ʱ�䲻ͬ��
}



/*
******************************************************************
���������۲�������Ч�Ա�Ǻ���
������EpkA ָ����һ��Ԫ�۲����ݵ�ָ��
      EpkB ָ��ǰ��Ԫ�۲����ݵ�ָ��
�������ܣ����ý��ջ����������ݣ�����������ʱ��locktime��
          �Ե�ǰ��Ԫ�Ĺ۲����ݽ�����Ч�Ա��
******************************************************************
*/
void MarkValidPre(const EPOCHOBS* EpkA, EPOCHOBS* EpkB)
{
    // ���׼��ֻ�е�˫Ƶ��locktime��ͨ������ű��Ϊtrue
    
    // GPS��L1-0��L2-1   BDS��B1I-0, B3I-1
    bool flag[2]; // �Ƿ��������ٵı�־��locktime��
   
    // ��������Ԫ�۲������е�ÿһ������
    for (int i = 0; i < EpkB->SatNum; i++)
    {
        // ���ڱ���Ԫ��ÿ�����ǣ��������������ݵı�־����ʼ��Ϊfalse
        flag[0] = flag[1] = false;

        // Ѱ�ұ�����������һ��Ԫ�е�����
        for (int j = 0; j < EpkA->SatNum; j++) 
        {
            if (EpkA->SatObs[j].System == EpkB->SatObs[i].System && EpkA->SatObs[j].Prn == EpkB->SatObs[i].Prn)
            {
                // ������Ƶ�ʵ���������������м���
                for (int n = 0; n < 2; n++)
                {
                    if (EpkA->SatObs[i].LockTime[n]==0.0)
                    {
                        // ����һ��Ԫ��������ʧ��״̬������Ԫ����������ʱ�����ڵ���6s����Ϊͨ������
                        if (EpkB->SatObs[i].LockTime[n] > 6 || EpkB->SatObs[i].LockTime[n] == 6) flag[n] = true;
                        else flag[n] = false;
                    }
                    else
                    {
                        double dt = EpkB->SatObs[i].LockTime[n] - EpkA->SatObs[j].LockTime[n];
                        if (dt > 0 || dt == 0) flag[n] = true;
                        else  flag[n] = false;
                    }                    
                }
                break;
            }
            else continue;
        }
        
        // ���й۲����ݵ���Ч�Ա�ǣ�ֻ��˫Ƶ��ͨ���������������ű��Ϊtrue
        if (EpkB->SatObs[i].Valid == true) // �Ե�վ��������̽����Ϊ����ٽ�һ�����ý��ջ��ڲ�ָ����
        {
            if (flag[0] == true && flag[1] == true) EpkB->SatObs[i].Valid = true;
            else EpkB->SatObs[i].Valid = false;
        }
        else; // Ĭ��Ϊfalse
      
    }               
}

/*
**************************************************************
��������վ�䵥���
������EpkR  ָ������վ�۲����ݵ�ָ��
      EpkB  ָ���վ�۲����ݵ�ָ��
      SDObs �������洢��
�������ܣ�������վ���վ�Ĺ۲��������,ͬʱ�����ܱ�Ǵ�������
**************************************************************
*/
void FormSDEpochObs(const EPOCHOBS* EpkR, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs)
{
    memset(SDObs->SdSatObs, 0.0, sizeof(SDObs->SdSatObs));

    SDObs->Time = EpkR->Time;

    int satnum = 0; // ����۲����ݵĿ���������

    // ����������վ�Ĺ۲�����
    for (int i = 0; i < EpkR->SatNum; i++)
    {
        // �����ջ����ڲ�����ָ�겻�ϸ���������������������������������ݣ����������
        if (EpkR->SatObs[i].Valid == false || EpkR->SatPVT[i].Valid == false) continue;
        else;
        // �ڲ������վ�Ĺ۲�����
        for (int j = 0; j < EpkB->SatNum; j++)
        {
            // �����ջ����ڲ�����ָ�겻�ϸ���������������������������������ݣ����������
            if (EpkB->SatObs[j].Valid == false || EpkB->SatPVT[i].Valid == false) continue;
            // ��û�����������ƥ��
            else if (EpkB->SatObs[j].System == EpkR->SatObs[i].System && EpkB->SatObs[j].Prn == EpkR->SatObs[i].Prn)
            {
                SDObs->SdSatObs[satnum].System = EpkR->SatObs[i].System;
                SDObs->SdSatObs[satnum].Prn = EpkR->SatObs[i].Prn;
                SDObs->SdSatObs[satnum].nRov = i;
                SDObs->SdSatObs[satnum].nBas = j;

                for (int k = 0; k < 2; k++)
                {
                    // ֻ������վ��α�඼��ֵ�����䵥�����˵�����ݶ�ʧ->�����㴦��
                    if (fabs(EpkR->SatObs[i].p[k]) > 1e-8 && fabs(EpkB->SatObs[j].p[k]) > 1e-8)
                    {
                        SDObs->SdSatObs[satnum].dP[k] = EpkR->SatObs[i].p[k] - EpkB->SatObs[j].p[k];
                    }
                    else SDObs->SdSatObs[satnum].dP[k] = 0;
                    // ֻ������վ����λ����ֵ�����䵥�����˵�����ݶ�ʧ->�����㴦��
                    if (fabs(EpkR->SatObs[i].l[k]) > 1e-8 && fabs(EpkB->SatObs[j].l[k]) > 1e-8)
                    {
                        SDObs->SdSatObs[satnum].dL[k] = EpkR->SatObs[i].l[k] - EpkB->SatObs[j].l[k];
                    }
                    else SDObs->SdSatObs[satnum].dL[k] = 0;    

                    // ���ݰ��ܱ��
                    if (EpkR->SatObs[i].half[k] == 1 && EpkB->SatObs[j].half[k] == 1) SDObs->SdSatObs[satnum].half[k] = 1; // �����ڰ�������
                    else SDObs->SdSatObs[satnum].half[k] = 0; // ���ܴ��ڰ�������
                }

                satnum++;
                break;
            }
            else continue;
        }
    }

    SDObs->SatNum = satnum;
}

/*
**************************************************************
������������̽�⺯��
������Obs  ָ��ÿ����Ԫ�ĵ���۲����ݵ�ָ��
�������ܣ�����ÿ����ԪMW��GF��Ϲ۲�ֵ����������������̽�⣬
          Ȼ���ÿ�����ǵĵ���۲����ݽ�����Ч�Ա��
**************************************************************
*/
void DetectCycleSlip(SDEPOCHOBS* Obs)
{
/* ˼·��Obs�еĵ�����Ϲ۲�ֵ�����ڽ��뱾����ʱΪ��һ��Ԫ�����ݣ��ں����������ű���Ԫ�ĵ�
         ��������ݣ�����������������ʱ��ŵ�ǰ��Ԫ�ļ������Ա������Ԫ�����̽��������   */
    
    MWGF sdcomobs[MAXCHANNUM]; // ��ŵ�ǰ��Ԫ�ĵ�����Ϲ۲�ֵ
    for (int i = 0; i < Obs->SatNum; i++)
    {
        // ������ǵĵ��������Ƿ�����
        if (fabs(Obs->SdSatObs[i].dL[0]) < 1e-8 || fabs(Obs->SdSatObs[i].dL[1]) < 1e-8 || fabs(Obs->SdSatObs[i].dP[0]) < 1e-8 || fabs(Obs->SdSatObs[i].dP[1]) < 1e-8)
        {
            // ������������������̽������������Ϊ��Ч
            Obs->SdSatObs[i].Valid = 0;
            continue;
        }
        else; // ���ǵ�˫Ƶα�ࡢ��λ�������ݾ�����
        
        sdcomobs[i].Sys = Obs->SdSatObs[i].System;
        sdcomobs[i].Prn = Obs->SdSatObs[i].Prn;
        sdcomobs[i].GF = Obs->SdSatObs[i].dL[0] - Obs->SdSatObs[i].dL[1];
        if (sdcomobs[i].Sys == GPS)
        {
            sdcomobs[i].MW = (FG1_GPS * Obs->SdSatObs[i].dL[0] - FG2_GPS * Obs->SdSatObs[i].dL[1]) / (FG1_GPS - FG2_GPS) -
                (FG1_GPS * Obs->SdSatObs[i].dP[0] + FG2_GPS * Obs->SdSatObs[i].dP[1]) / (FG1_GPS + FG2_GPS);
        }
        else if (sdcomobs[i].Sys == BDS)
        {
            sdcomobs[i].MW = (FG1_BDS * Obs->SdSatObs[i].dL[0] - FG3_BDS * Obs->SdSatObs[i].dL[1]) / (FG1_BDS - FG3_BDS) -
                (FG1_BDS * Obs->SdSatObs[i].dP[0] + FG3_BDS * Obs->SdSatObs[i].dP[1]) / (FG1_BDS + FG3_BDS);
        }

        sdcomobs[i].n = 1; // ����MW��ƽ������,�˴���ֵΪ1��Ϊ�ݴ�������������

        // ���ϸ���Ԫ����������в��ұ������ǵ�GF��MW���ֵ
        for (int j = 0; j < MAXCHANNUM; j++)
        {
            if (Obs->SdCObs[j].Sys == sdcomobs[i].Sys && Obs->SdCObs[j].Prn == sdcomobs[i].Prn)
            {
                // ����GF��MW���ֵ����Ԫ����
                double dGF = fabs(sdcomobs[i].GF - Obs->SdCObs[j].GF);
                double dMW = fabs(sdcomobs[i].MW - Obs->SdCObs[j].MW);
                // �ж��Ƿ��ޣ�δ������δ�������������Ϊtrue
                if (dGF < 0.05 && dMW < 3)
                {
                    Obs->SdSatObs[i].Valid = 1;
                    // ��MW���ֵ����ƽ��
                    sdcomobs[i].MW = (Obs->SdCObs[j].MW * Obs->SdCObs[j].n + sdcomobs[i].MW) / (Obs->SdCObs[j].n + 1);
                    sdcomobs[i].n = Obs->SdCObs[j].n + 1; // ����MWƽ������
                }
                else Obs->SdSatObs[i].Valid = 0; // ���ޣ�����Ϊfalse

                break;
            }
            else continue;
        }
    }
    // ������������Ϲ۲�ֵ�Ŀ�����Obs��
    memcpy(Obs->SdCObs, sdcomobs, sizeof(sdcomobs));
}


/*
*************************************************************************************
��������ȷ����׼��
������EpkR   ָ������վ�۲����ݵ�ָ��
      EpkB   ָ���վ�۲����ݵ�ָ��
      SDObs  ָ�򵥲����ݵ�ָ��
      DDObs  ָ��˫�����ݵ�ָ��
�������ܣ�ѡȡ��׼�ǣ�˫ϵͳ��һ����������PRN�������������˫��������
����ֵ��������ϵͳ�Ĳο��Ǿ�ѡȡ�ɹ��򷵻�true�����򷵻�false
ע�⣺�����EpkR��EpkB��Ҫ�ڱ���������ǰ����һ��SPP���õ�����λ�á��߶Ƚǵ��������
*************************************************************************************
*/
bool DetRefSat(const EPOCHOBS* EpkR, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
    /*
    ÿ������ϵͳ��ѡȡһ��������Ϊ�ο��ǣ�0=GPS; 1=BDS
    ������1��α����ز���λͨ������̽�⣬û�дֲ�������Լ����ܱ��
          2��������������������λ�ü���ɹ�
          3�����Ǹ߶Ƚ����
    */
    short sys = 0; // 0=GPS; 1=BDS
    double MaxEle[2] = { 0 , 0 }; // �߶Ƚǣ�Ѱ��������ã�
    // ������������
    for (int i = 0; i < SDObs->SatNum; i++)
    {
        if (SDObs->SdSatObs[i].Valid == false|| SDObs->SdSatObs[i].half[0]== 0 || SDObs->SdSatObs[i].half[1] == 0) continue; // δͨ������̽�����ܼ�飬���ɳ�Ϊ�ο���
        else if (EpkR->SatPVT[SDObs->SdSatObs[i].nRov].Valid == false || EpkB->SatPVT[SDObs->SdSatObs[i].nBas].Valid == false) continue; // ���������쳣�����ɳ�Ϊ�ο���
        else;
        // Ѱ������ϵͳ�и߶Ƚ���������
        // ȷ���洢����
        if (SDObs->SdSatObs[i].System == GPS) sys = 0;
        else sys = 1;
        // Ѱ�����߶Ƚ�,������վΪ׼����վ����Ϊ�������ڿ����ĺû����£�����Ҫ�������վ��
        if (EpkR->SatPVT[SDObs->SdSatObs[i].nRov].Elevation > MaxEle[sys])
        {
            MaxEle[sys] = EpkR->SatPVT[SDObs->SdSatObs[i].nRov].Elevation;
            DDObs->RefPrn[sys] = SDObs->SdSatObs[i].Prn;
            DDObs->RefPos[sys] = i;
        }
        else continue;
    }
    if (DDObs->RefPos[0] > -1 && DDObs->RefPos[1] > -1) return true;
    else return false;
}


/*
*****************************************************************************************
����������ȡ��Զ�λ�����
������Raw    ָ����Ԫ�����ݵ�ָ��
      Rov    ����վ��SPP��������ڳ�ʼ�����꣩
      Fres   �������
�������ܣ����и������㣬����˫����ģ���ȼ���Э�������󡢻����������侫��ָ�굽Fres
����ֵ�������ɹ��򷵻�true��ʧ���򷵻�false
*****************************************************************************************
*/
bool RTKFloat(RAWDAT* Raw, POSRES* Rov, FloatResult* Fres)
{
    /* ������ñ��� */
    double BasPos[3]; // ��ȡ��վ����(ʹ�ý�������׼ȷ����)
    BLHToXYZ(Raw->BasPres.Pos, BasPos, R_WGS84, F_WGS84);   
    double RovPos[3] = { Rov->Pos[0],Rov->Pos[1] ,Rov->Pos[2] }; // ��������վ�����ֵ(��SPP�Ľ��)
    short DDsatnum[2] = { 0 ,0 }; // GPS��BDS��ϵͳ������˫��������  0=GPS, 1=BDS
    short totalsatnum = 0; // �ܵĿ���˫��������(��׼�ǲ�����)
    double RovToRefSat[2] = {0,0};  // ����վ����ο��ǵļ��ξ��� 0=GPS, 1=BDS   
    double BasToSats[MAXCHANNUM];  // ��վ���굽�������ǵļ��ξ��루�±����վԭʼ�۲����ݵ�����һ�£�
    double DDN[MAXCHANNUM * 2]; // ����˫��ģ����
    double x[3 + MAXCHANNUM * 2]; // ���������ĸ�����
    double B[(4 * MAXCHANNUM) * (3 + MAXCHANNUM * 2)]; // ��ƾ���
    double w[4 * MAXCHANNUM]; // �в���
    double P[(4 * MAXCHANNUM) * (4 * MAXCHANNUM)]; // Ȩ��
    double N_inv[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)]; // ����������Э�����󣬶��������Ա�����洢

    // ��ȡ�ο�����λ����ԭʼ�۲������е����� 0=GPS, 1=BDS
    int RefSatOfRov[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nRov,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nRov };
    int RefSatOfBas[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nBas,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nBas };
    // �ο����ǵ�λ�ã���������վ�ͻ�վ��˵������ʱ�䲻һ����ȫͬ��,�ʸ����ø����������õ�����λ�ã�
    double RefSatPosOfGPSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[2] };
    double RefSatPosOfBDSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[2] };
    double RefSatPosOfGPSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[2] };
    double RefSatPosOfBDSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[2] };

    /* ������ʼ�� */
    // �����ʼ������
    memset(DDN, 0.0, sizeof(DDN));
    memset(x, 0.0, sizeof(x));
    memset(B, 0.0, sizeof(B));
    memset(w, 0.0, sizeof(w));
    memset(P, 0.0, sizeof(P));
    memset(BasToSats, 0, sizeof(BasToSats));

    // ��ʼ��˫��ģ���ȣ��±������ù۲ⷽ�̵���������˳��һ�� 
    int ii=0; // �洢����
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {      
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // �ο��ǲ��������˫��������
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // ���ڴֲ�Ͱ���������
        else; // �������ǿ���

        // ȷ������������ϵͳ
        int sys = -1;
        if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0;
        else sys = 1;

        // ����˫��۲�ֵ
        double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
        double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
        // ��ʼ��˫��ģ����
        DDN[ii] = (sys == 0) ? (ddl[0] - ddp[0]) / WL1_GPS : (ddl[0] - ddp[0]) / WL1_BDS;
        DDN[ii + 1] = (sys == 0) ? (ddl[1] - ddp[1]) / WL2_GPS : (ddl[1] - ddp[1]) / WL3_BDS;
        ii = ii + 2;
     
    }

    // �����վ���굽�������ǵļ��ξ���
    for (int i = 0; i < Raw->BasEpk.SatNum; i++)
    {
        BasToSats[i] = sqrt((BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) * (BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) + (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) * (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) + (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]) * (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]));
    }


    /* ������С���˵��� */
    bool flag = true; // ����������
    int count = 0; // ��������
    do
    {
        // ÿ�ε���ʱ�Ƚ���ؾ��������㣬��������������
        memset(x, 0, sizeof(x));
        memset(B, 0, sizeof(B));
        memset(w, 0, sizeof(w));
        memset(P, 0, sizeof(P));
        memset(N_inv, 0, sizeof(N_inv));

        // ÿ�ε���ʱ�������������㣬��������������
        DDsatnum[0] = DDsatnum[1] = 0;
        totalsatnum = 0; 

        // ��������վ����ο��ǵļ��ξ��� 0=GPS, 1=BDS       
        RovToRefSat[0] = sqrt((RovPos[0] - RefSatPosOfGPSR[0]) * (RovPos[0] - RefSatPosOfGPSR[0]) + (RovPos[1] - RefSatPosOfGPSR[1]) * (RovPos[1] - RefSatPosOfGPSR[1]) + (RovPos[2] - RefSatPosOfGPSR[2]) * (RovPos[2] - RefSatPosOfGPSR[2]));
        RovToRefSat[1] = sqrt((RovPos[0] - RefSatPosOfBDSR[0]) * (RovPos[0] - RefSatPosOfBDSR[0]) + (RovPos[1] - RefSatPosOfBDSR[1]) * (RovPos[1] - RefSatPosOfBDSR[1]) + (RovPos[2] - RefSatPosOfBDSR[2]) * (RovPos[2] - RefSatPosOfBDSR[2]));
        
        // ��������۲�ֵ�����B��w����ͬʱͳ�Ƹ�ϵͳ�Ŀ���˫��������
        for (int i = 0; i < Raw->SdObs.SatNum; i++)
        {
            
            if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // �ο��ǲ��������˫��������
            else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // ���ڴֲ�Ͱ���
            else; // �������ǿ���

            // ��ǰ���ǵ�λ�ã�������վԭʼ�۲������еģ�����
            short satindex = Raw->SdObs.SdSatObs[i].nRov;
            // ��������վ���������ǵļ��ξ���
            double RovToSat = sqrt((RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) * (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) + (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) * (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) + (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) * (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]));
            // ȷ��������������ϵͳ
            int sys = -1;
            if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0; // GPS
            else sys = 1; // BDS

            // ����B����м���
            double l = (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) / RovToSat - (RovPos[0] - ((sys == 0) ? RefSatPosOfGPSR[0] : RefSatPosOfBDSR[0])) / RovToRefSat[sys];
            double m = (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) / RovToSat - (RovPos[1] - ((sys == 0) ? RefSatPosOfGPSR[1] : RefSatPosOfBDSR[1])) / RovToRefSat[sys];
            double n = (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) / RovToSat - (RovPos[2] - ((sys == 0) ? RefSatPosOfGPSR[2] : RefSatPosOfBDSR[2])) / RovToRefSat[sys];
            // ���B�����������
            for (int j = 0; j < 4; j++)
            {
                B[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j)] = l;
                B[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 1] = m;
                B[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 2] = n;
                if (j == 2) B[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 3 + 2 * totalsatnum] = (sys == 0) ? WL1_GPS : WL1_BDS;
                else if (j == 3) B[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 4 + 2 * totalsatnum] = (sys == 0) ? WL2_GPS : WL3_BDS;
                else continue;
            }
           
            // ����˫��۲�ֵ
            double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            double ddrou = RovToSat - RovToRefSat[sys] + BasToSats[RefSatOfBas[sys]] - BasToSats[Raw->SdObs.SdSatObs[i].nBas];
            // ���w�����������
            w[4 * totalsatnum] = ddp[0] - ddrou;
            w[4 * totalsatnum + 1] = ddp[1] - ddrou;
            w[4 * totalsatnum + 2] = (sys == 0) ? (ddl[0] - ddrou - DDN[totalsatnum * 2] * WL1_GPS) : (ddl[0] - ddrou - DDN[totalsatnum * 2] * WL1_BDS);
            w[4 * totalsatnum + 3] = (sys == 0) ? (ddl[1] - ddrou - DDN[totalsatnum * 2 + 1] * WL2_GPS) : (ddl[1] - ddrou - DDN[totalsatnum * 2 + 1] * WL3_BDS);
            
            // ��Ӧϵͳ��������������
            DDsatnum[sys]++;
            totalsatnum = DDsatnum[0] + DDsatnum[1];    
        }
        

        // ���P��,GP��BP�Ǵ�������ֵ   
        double GP[4] = { DDsatnum[0] / (DDsatnum[0] + 1.0),1000 * DDsatnum[0] / (DDsatnum[0] + 1.0),-1 / (DDsatnum[0] + 1.0),-1000 / (DDsatnum[0] + 1.0) };
        double BP[4] = { DDsatnum[1] / (DDsatnum[1] + 1.0),1000 * DDsatnum[1] / (DDsatnum[1] + 1.0),-1 / (DDsatnum[1] + 1.0),-1000 / (DDsatnum[1] + 1.0) };
        // ���GPS����
        for (int i = 0; i < 4 * DDsatnum[0]; i++)
        {
            for (int j = 0; j < 4 * DDsatnum[0]; j++)
            {
                // ���Խ���Ԫ��
                if (j == i)
                {
                    if (i % 4 == 0 || i % 4 == 1) P[i * (4 * MAXCHANNUM) + j] = GP[0];
                    else P[i * (4 * MAXCHANNUM) + j] = GP[1];
                }
                // ���ǶԽ���Ԫ��
                else if (j % 4 == i % 4)
                {
                    if (i % 4 == 0 || i % 4 == 1) P[i * (4 * MAXCHANNUM) + j] = GP[2];
                    else P[i * (4 * MAXCHANNUM) + j] = GP[3];
                }
            }
        }
        // ���BDS����
        int startRow = 4 * DDsatnum[0]; // BDS���ֿ�ʼ����
        int startCol = 4 * DDsatnum[0]; // BDS���ֿ�ʼ����
        for (int i = 0; i < 4 * DDsatnum[1]; i++) {
            for (int j = 0; j < 4 * DDsatnum[1]; j++) {
                int rowIndex = startRow + i; // ����������
                int colIndex = startCol + j; // ����������
                // ���Խ���Ԫ��
                if (i == j) {
                    if (i % 4 == 0 || i % 4 == 1) P[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[0];
                    else P[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[1];
                }
                // ���ǶԽ���Ԫ��
                else if (i % 4 == j % 4) {
                    if (i % 4 == 0 || i % 4 == 1) P[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[2];
                    else P[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[3];
                }
            }
        }

        // �����ع���ʹ��ƥ�䱾��Ŀ�ľ������㺯��
        restructureMatrix(4 * totalsatnum, 3 + totalsatnum * 2, 4 * MAXCHANNUM, 3 + MAXCHANNUM * 2, B);
        restructureMatrix(4 * totalsatnum, 4 * totalsatnum, 4 * MAXCHANNUM, 4 * MAXCHANNUM, P);

        // ��˫��۲ⷽ����������δ֪������������
        if (totalsatnum < 5) return false; // �۲ⲻ���Խ���
        else; // �۲��㹻���������

        // ���岢��ʼ���м�������
        double BT[(3 + MAXCHANNUM * 2) * (4 * MAXCHANNUM)];
        MatrixTranspose((4 * totalsatnum), (3 + totalsatnum * 2), B, BT);
        double BTP[(3 + MAXCHANNUM * 2) * (4 * MAXCHANNUM)];
        memset(BTP, 0.0, sizeof(BTP));
        double N[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];
        memset(N, 0.0, sizeof(N));
        double W[(3 + MAXCHANNUM * 2) * 1];
        memset(W, 0.0, sizeof(W));

        // ��������
        MatrixMultiply((3 + totalsatnum * 2), (4 * totalsatnum), (4 * totalsatnum), (4 * totalsatnum), BT, P, BTP);
        MatrixMultiply((3 + totalsatnum * 2), (4 * totalsatnum), (4 * totalsatnum), (3 + totalsatnum * 2), BTP, B, N);       
        int iserror = MatrixInv((3 + totalsatnum * 2), N, N_inv);
        if (iserror == 0) return false; // �������������Դ������˳�����    
        MatrixMultiply((3 + totalsatnum * 2), (4 * totalsatnum), (4 * totalsatnum), 1, BTP, w, W);
        MatrixMultiply((3 + totalsatnum * 2), (3 + totalsatnum * 2), (3 + totalsatnum * 2), 1, N_inv, W, x);

        // ��������վ��λ�ú�˫��ģ���Ȳ���
        RovPos[0] = RovPos[0] + x[0];
        RovPos[1] = RovPos[1] + x[1];
        RovPos[2] = RovPos[2] + x[2];
        for (int i = 0; i < totalsatnum * 2; i++)  DDN[i] = DDN[i] + x[i + 3];
       
        // ��������������
        if ((fabs(x[0]) < 1e-8 && fabs(x[1]) < 1e-8 && fabs(x[2]) < 1e-8) || count > 50) flag = false;
        else flag = true;

        count++;
 
    }while (flag);

    /* �洢��� */
    // ��¼����˫�������������������˫��ṹ����
    Raw->DDObs.DDSatNum[0] = Fres->stanum[0] = DDsatnum[0];
    Raw->DDObs.DDSatNum[1] = Fres->stanum[1] = DDsatnum[1];
    Raw->DDObs.Sats = Fres->totalsatnum = totalsatnum;
    // �洢��������
    Fres->dX[0] = RovPos[0] - BasPos[0];
    Fres->dX[1] = RovPos[1] - BasPos[1];
    Fres->dX[2] = RovPos[2] - BasPos[2];
    // �洢����������DOP
    Fres->DOP[0] = sqrt(N_inv[0]);
    Fres->DOP[1] = sqrt(N_inv[3 + 2 * totalsatnum + 1]);
    Fres->DOP[2] = sqrt(N_inv[2 * (3 + 2 * totalsatnum) + 2]);
    Fres->PDOP = sqrt(N_inv[0] + N_inv[3 + 2 * totalsatnum + 1] + N_inv[2 * (3 + 2 * totalsatnum) + 2]);
    // �洢˫��ģ����
    memcpy(Fres->N, DDN, sizeof(double) * 2 * totalsatnum);
    // �洢˫��ģ���ȵ�Э������
    GetQnn(totalsatnum, N_inv, Fres->Qn );
    
    // �������λȨ�����
    // ���岢��ʼ���м�������
    double V[4 * MAXCHANNUM];
    memset(V, 0, sizeof(V));
    double Bx[4 * MAXCHANNUM];
    memset(Bx, 0, sizeof(Bx));
    double VT[4 * MAXCHANNUM];
    memset(VT, 0, sizeof(VT));   
    double VTP[4 * MAXCHANNUM];
    memset(VTP, 0, sizeof(VTP));
    double VTPV[1] = {0};
    // ��������
    MatrixMultiply(4 * totalsatnum, 3 + totalsatnum * 2, 3 + totalsatnum * 2, 1, B, x, Bx);
    MatrixSubtraction(4 * totalsatnum, 1, Bx, w, V);
    MatrixTranspose(4 * totalsatnum, 1, V, VT);
    MatrixMultiply(1, 4 * totalsatnum, 4 * totalsatnum, 4 * totalsatnum, VT, P, VTP);
    MatrixMultiply(1, 4 * totalsatnum, 4 * totalsatnum, 1, VTP, V, VTPV);

    Fres->sigma = sqrt(VTPV[0] / (2 * totalsatnum - 3));

    return true;
}


/*
************************************************************************************
����������ȡ��Զ�λ�̶���
������Raw    ָ����Ԫ�����ݵ�ָ�루��ʱ�Ѱ����̶��������ģ���ȣ�
      Rov    ����վ��SPP��������ڳ�ʼ�����꣩
�������ܣ��ڹ̶���ģ���ȵ������½���ֻʹ����λ�۲�ֵ�Ľ��㣬�õ�����ȷ�Ķ�λ��
          ���洢��Raw�е�DDObs���dPos
����ֵ���̶���ɹ��򷵻�true��ʧ���򷵻�false
************************************************************************************
*/
bool RTKFix(RAWDAT* Raw, POSRES* Rov) 
{
    /* ������ñ��� */
    double BasPos[3]; // ��ȡ��վ����(ʹ�ý�������׼ȷ����)
    BLHToXYZ(Raw->BasPres.Pos, BasPos, R_WGS84, F_WGS84);
    double RovPos[3] = { Rov->Pos[0],Rov->Pos[1] ,Rov->Pos[2] }; // ��������վ�����ֵ(��SPP�Ľ��)
    double RovToRefSat[2] = { 0, 0 }; // ����վ����ο��ǵļ��ξ��� 0=GPS, 1=BDS
    double BasToSats[MAXCHANNUM]; // ��վ���굽�������ǵļ��ξ��루�±����վԭʼ�۲����ݵ�����һ�£�
    double DDN[MAXCHANNUM * 2]; // ˫��ģ����
    short DDsatnum[2] = { 0 ,0 }; // GPS��BDS��ϵͳ������˫��������  0=GPS, 1=BDS
    short totalsatnum = 0; // �ܵĿ���˫��������(��׼�ǲ�����)
    double x[3]; // ���������ĸ�����
    double B[(2 * MAXCHANNUM) * 3]; // B��
    double w[2 * MAXCHANNUM]; // w��
    double P[(2 * MAXCHANNUM) * (2 * MAXCHANNUM)]; // P��
    double N_inv[3 * 3]; // ����������Э�����󣬶��������Ա�����洢

    // ��ȡ�ο�����λ����ԭʼ�۲������е����� 0=GPS, 1=BDS
    short RefSatOfRov[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nRov,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nRov };
    short RefSatOfBas[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nBas,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nBas };
    // �ο����ǵ�λ�ã���������վ�ͻ�վ��˵������ʱ�䲻һ����ȫͬ��,�ʸ����ø����������õ�����λ�ã�
    double RefSatPosOfGPSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[2] };
    double RefSatPosOfBDSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[2] };
    double RefSatPosOfGPSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[2] };
    double RefSatPosOfBDSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[2] };


    /* ������ʼ�� */
    // �����ʼ������
    memset(x, 0.0, sizeof(x));
    memset(B, 0.0, sizeof(B));
    memset(w, 0.0, sizeof(w));
    memset(P, 0.0, sizeof(P));
    memset(BasToSats, 0.0, sizeof(BasToSats));
    memset(DDN, 0.0, sizeof(DDN));

    // �����վ���굽�������ǵļ��ξ���
    for (int i = 0; i < Raw->BasEpk.SatNum; i++)
    {
        BasToSats[i] = sqrt((BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) * (BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) + (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) * (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) + (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]) * (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]));
    }

    // ��ȡ�ѹ̶���˫��ģ���ȣ��±������ù۲ⷽ�̵���������˳��һ��   
    memcpy(DDN, Raw->DDObs.FixedAmb, sizeof(double) * Raw->DDObs.Sats * 2);
   
    /* ��С���˵��� */
    bool flag = true; // ����������
    int count = 0; // �������� 
    do
    {
        // ÿ�ε���ʱ����ؾ��������㣬��������������
        memset(x, 0, sizeof(x));
        memset(B, 0, sizeof(B));
        memset(w, 0, sizeof(w));
        memset(P, 0, sizeof(P));
        memset(N_inv, 0, sizeof(N_inv));

        // ÿ�ε���ʱ�������������㣬��������������
        DDsatnum[0] = DDsatnum[1] = 0;
        totalsatnum = 0;

        // ��������վ����ο��ǵļ��ξ��� 0=GPS, 1=BDS       
        RovToRefSat[0] = sqrt((RovPos[0] - RefSatPosOfGPSR[0]) * (RovPos[0] - RefSatPosOfGPSR[0]) + (RovPos[1] - RefSatPosOfGPSR[1]) * (RovPos[1] - RefSatPosOfGPSR[1]) + (RovPos[2] - RefSatPosOfGPSR[2]) * (RovPos[2] - RefSatPosOfGPSR[2]));
        RovToRefSat[1] = sqrt((RovPos[0] - RefSatPosOfBDSR[0]) * (RovPos[0] - RefSatPosOfBDSR[0]) + (RovPos[1] - RefSatPosOfBDSR[1]) * (RovPos[1] - RefSatPosOfBDSR[1]) + (RovPos[2] - RefSatPosOfBDSR[2]) * (RovPos[2] - RefSatPosOfBDSR[2]));
        
        // ��������۲�ֵ�����B��w����ͬʱͳ�Ƹ�ϵͳ�Ŀ���˫��������
        for (int i = 0; i < Raw->SdObs.SatNum; i++)
        {
            if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // �ο��ǲ��������˫��������
            else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // ���ڴֲ�Ͱ���
            else; // �������ǿ���

            // ��ǰ���ǵ�λ�ã�������վԭʼ�۲������еģ�����
            short satindex = Raw->SdObs.SdSatObs[i].nRov;
            // ��������վ���������ǵļ��ξ���
            double RovToSat = sqrt((RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) * (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) + (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) * (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) + (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) * (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]));
            // ȷ��������������ϵͳ
            int sys = -1;
            if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0; // GPS
            else sys = 1; // BDS

            // ����B����м���
            double l = (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) / RovToSat - (RovPos[0] - ((sys == 0) ? RefSatPosOfGPSR[0] : RefSatPosOfBDSR[0])) / RovToRefSat[sys];
            double m = (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) / RovToSat - (RovPos[1] - ((sys == 0) ? RefSatPosOfGPSR[1] : RefSatPosOfBDSR[1])) / RovToRefSat[sys];
            double n = (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) / RovToSat - (RovPos[2] - ((sys == 0) ? RefSatPosOfGPSR[2] : RefSatPosOfBDSR[2])) / RovToRefSat[sys];
            // ���B�����������
            for (int j = 0; j < 2; j++)
            {
                B[3 * (2 * totalsatnum + j)] = l;
                B[3 * (2 * totalsatnum + j) + 1] = m;
                B[3 * (2 * totalsatnum + j) + 2] = n;
            }

            // ����˫��۲�ֵ
            double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            double ddrou = RovToSat - RovToRefSat[sys] + BasToSats[RefSatOfBas[sys]] - BasToSats[Raw->SdObs.SdSatObs[i].nBas];
            // ���w�����������
            w[2 * totalsatnum] = (sys == 0) ? (ddl[0] - ddrou - DDN[totalsatnum * 2] * WL1_GPS) : (ddl[0] - ddrou - DDN[totalsatnum * 2] * WL1_BDS);
            w[2 * totalsatnum + 1] = (sys == 0) ? (ddl[1] - ddrou - DDN[totalsatnum * 2 + 1] * WL2_GPS) : (ddl[1] - ddrou - DDN[totalsatnum * 2 + 1] * WL3_BDS);

            // ��Ӧϵͳ��������������
            DDsatnum[sys]++;
            totalsatnum = DDsatnum[0] + DDsatnum[1];
        }
     
        // ���P��,GP��BP�Ǵ�������ֵ   
        double GP[2] = { DDsatnum[0] / (DDsatnum[0] + 1.0),-1.0/ (DDsatnum[0] + 1.0)};
        double BP[2] = { DDsatnum[1] / (DDsatnum[1] + 1.0),-1.0/ (DDsatnum[1] + 1.0)};
        // ���GPS����
        for (int i = 0; i < 2 * DDsatnum[0]; i++)
        {
            for (int j = 0; j < 2 * DDsatnum[0]; j++)
            {
                // ���Խ���Ԫ��
                if (j == i)
                {
                    P[i * (2 * MAXCHANNUM) + j] = GP[0];
                }
                // ���ǶԽ���Ԫ��
                else if (j % 2 == i % 2)
                {
                    P[i * (2 * MAXCHANNUM) + j] = GP[1];
                }
            }
        }
        // ���BDS����
        int startRow = 2 * DDsatnum[0]; // BDS���ֿ�ʼ����
        int startCol = 2 * DDsatnum[0]; // BDS���ֿ�ʼ����
        for (int i = 0; i < 2 * DDsatnum[1]; i++)
        {
            for (int j = 0; j < 2 * DDsatnum[1]; j++) 
            {
                int rowIndex = startRow + i; // ����������
                int colIndex = startCol + j; // ����������
                // ���Խ���Ԫ��
                if (i == j) 
                {
                    P[rowIndex * (2 * MAXCHANNUM) + colIndex] = BP[0];
                }
                // ���ǶԽ���Ԫ��
                else if (i % 2 == j % 2) 
                {
                    P[rowIndex * (2 * MAXCHANNUM) + colIndex] = BP[1];
                }
            }
        }

        // �����ع���ʹ��ƥ�䱾��Ŀ�ľ������㺯��
        restructureMatrix(2 * totalsatnum, 2 * totalsatnum, 2 * MAXCHANNUM, 2 * MAXCHANNUM, P);

        // ���ڸ������У���̶�ʱ���迼�����Ǹ����Ƿ��㹻������
        // ���岢��ʼ���м�������
        double BT[3 * (2 * MAXCHANNUM)];
        MatrixTranspose((2 * totalsatnum),3, B, BT);
        double BTP[3 * (2 * MAXCHANNUM)];
        memset(BTP, 0.0, sizeof(BTP));
        double N[3 * 3];
        memset(N, 0.0, sizeof(N));   
        double W[3 * 1];
        memset(W, 0.0, sizeof(W));

        // ��������
        MatrixMultiply(3, (2 * totalsatnum), (2 * totalsatnum), (2 * totalsatnum), BT, P, BTP);
        MatrixMultiply(3, (2 * totalsatnum), (2 * totalsatnum), 3, BTP, B, N);
        int iserror = MatrixInv(3, N, N_inv);
        if (iserror == 0) return false; // �������������Դ������˳�����
        MatrixMultiply(3, (2 * totalsatnum), (2 * totalsatnum), 1, BTP, w, W);
        MatrixMultiply(3, 3, 3, 1, N_inv, W, x);

        // ��������վ��λ��
        RovPos[0] = RovPos[0] + x[0];
        RovPos[1] = RovPos[1] + x[1];
        RovPos[2] = RovPos[2] + x[2];

        Raw->DDObs.PDOP = sqrt(N_inv[0] + N_inv[4] + N_inv[8]);

        // ��������������
        if (fabs(x[0]) < 1e-8 && fabs(x[1]) < 1e-8 && fabs(x[2]) < 1e-8 || count > 50) flag = false;
        else flag = true;

        count++;

    } while (flag);

    /* �洢��� */
    // �洢�̶���Ļ�������
    Raw->DDObs.dPos[0] = RovPos[0] - BasPos[0];
    Raw->DDObs.dPos[1] = RovPos[1] - BasPos[1];
    Raw->DDObs.dPos[2] = RovPos[2] - BasPos[2];
    
    // ����в�ƽ����
    // ���岢��ʼ���м�������
    double V[2 * MAXCHANNUM];
    memset(V, 0, sizeof(V));
    double Bx[2 * MAXCHANNUM];
    memset(Bx, 0, sizeof(Bx));
    double VT[2 * MAXCHANNUM];
    memset(VT, 0, sizeof(VT));
    double VTP[2 * MAXCHANNUM];
    memset(VTP, 0, sizeof(VTP));
    double VTPV[1] = { 0 };
    // ��������
    MatrixMultiply(2 * totalsatnum, 3, 3 , 1, B, x, Bx);
    MatrixSubtraction(2 * totalsatnum, 1, Bx, w, V);
    MatrixTranspose(2 * totalsatnum, 1, V, VT);
    MatrixMultiply(1, 2 * totalsatnum, 2 * totalsatnum, 2 * totalsatnum, VT, P, VTP);
    MatrixMultiply(1, 2 * totalsatnum, 2 * totalsatnum, 1, VTP, V, VTPV);

    Raw->DDObs.FixRMS = VTPV[0];

    return true;
}


/*
***********************************************************
���������˲���ʼ������
������Raw    ָ����Ԫ�����ݵ�ָ��
      Rov    ����վ��SPP���
      ekf    �˲�����
�������ܣ���ʼ���˲���״̬��X�����Ӧ�ķ���Э�������P
***********************************************************
*/
void InitFilter(RAWDAT* Raw, POSRES* Rov, RTKEKF* ekf)
{
    // ״̬����
    memset(&ekf->X0, 0, sizeof(ekf->X0));
    memset(&ekf->P0, 0, sizeof(ekf->P0));
    memset(&ekf->X, 0, sizeof(ekf->X));
    memset(&ekf->P, 0, sizeof(ekf->P));

    // ��ʼ��ʱ����
    memcpy(&ekf->Time, &Raw->RovEpk.Time, sizeof(ekf->Time));
    // ��ʼ��״̬��λ��Ԫ��(ʹ��SPP������X��ǰ����Ԫ��)
    ekf->X0[0] = Rov->Pos[0];
    ekf->X0[1] = Rov->Pos[1];
    ekf->X0[2] = Rov->Pos[2]; 
    // ��ʼ��״̬��˫��ģ����Ԫ�أ������������ݣ���˳����㲢��䣩
    int ii = 3; // �洢����      ״̬X��Ԫ�ظ���Ϊii --> Ϊ���P���ṩ����
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        // �ο��ǲ��������˫��������
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1])
        {
            ekf->Index[i] = -1;
            continue;
        }
        else;
        // �ж��Ƿ���ڴֲ�Ͱ���
        if (Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)
        {
            ekf->Index[i] = ii; // �������Ƕ�Ӧվ�䵥�����ݲ�����˫Ƶ˫��ģ���ȴ����״̬X�е�����Ϊii��ii+1

            // ȷ���ÿ���������ϵͳ
            int sys = -1;
            if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0;
            else sys = 1;
            // ����˫��۲�ֵ������ģ���ȳ�ʼ��
            double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            ekf->X0[ii] = (sys == 0) ? ((ddl[0] - ddp[0]) / WL1_GPS) : ((ddl[0] - ddp[0]) / WL1_BDS);
            ekf->X0[ii + 1] = (sys == 0) ? ((ddl[1] - ddp[1]) / WL2_GPS) : ((ddl[1] - ddp[1]) / WL3_BDS);
            ii = ii + 2;

        }
        else
        {
            ekf->Index[i] = -1;
            continue;
        }
    }

    // ��ʼ��״̬�ķ���Э������
    for (int j = 0; j < ii; j++)
    {
        if (j < 3) ekf->P0[j + j * ii] = 100;
        else ekf->P0[j + j * ii] = 500;
    }
 
    // �˲���ʼ����ɣ����±��
    ekf->IsInit = true;
}


/*
******************************************************************
��������Ԥ�⺯��
������Raw    ָ����Ԫ�����ݵ�ָ��
      ekf    �˲�����
�������ܣ���ekf�е�״̬�������Э���������Ԥ��
����ֵ��Ԥ��ɹ��򷵻�true������Ϊfalse
ע�⣺ekf�ڽ���ʱӦ������һ��Ԫ������ԪDDOBS�ڵĲο�����Ϣ
      �Լ���һ��Ԫ��SDOBS����������Ϣ
******************************************************************
*/
bool EkfPredict(RAWDAT* Raw, RTKEKF* ekf)
{
    // ʱ�䴫��
    memcpy(&ekf->Time, &Raw->RovEpk.Time, sizeof(ekf->Time));

    // ͳ�Ƶ�ǰ��Ԫ�Ŀ���˫��������
    ekf->CurDDObs.Sats = 0;
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        if (i == ekf->CurDDObs.RefPos[0] || i == ekf->CurDDObs.RefPos[1]) continue; // �ο��ǣ�����
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // ����������ܱ�ǣ�����
        else ekf->CurDDObs.Sats++; // �����ṩ��ǰ��Ԫ״̬�е�˫��ģ����
    }
    // ͳ����һ��Ԫ�Ŀ���˫��������
    ekf->DDObs.Sats = 0;
    for (int i = 0; i < ekf->SDObs.SatNum; i++)
    {
        if (i == ekf->DDObs.RefPos[0] || i == ekf->DDObs.RefPos[1]) continue; // �ο��ǣ�����
        else if (!(ekf->SDObs.SdSatObs[i].Valid == 1 && ekf->SDObs.SdSatObs[i].half[0] == 1 && ekf->SDObs.SdSatObs[i].half[1] == 1)) continue; // ����������ܱ�ǣ�����
        else ekf->DDObs.Sats++; // �����ṩ��ǰ��Ԫ״̬�е�˫��ģ����
    }

    // ����״̬ת�ƾ���Ԥ��״̬�����Э�����󲢳�ʼ����
    double fai[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)], Q[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)];
    memset(fai, 0.0, sizeof(fai));
    memset(Q, 0.0, sizeof(Q));
    // ���λ��Ԫ�ض�Ӧ��fai��Q
    for (int i = 0; i < 3; i++)
    {
        fai[i + i * (3 + 2 * ekf->DDObs.Sats)] = 1; // λ�ü���һ��Ԫ��λ��
        Q[i + i * (3 + 2 * ekf->CurDDObs.Sats)] = 1e-4 * 1e-4; // ��̬����Ԥ������С
    }

    // ��¼˫ϵͳ�Ĳο������޷����ı� 0-GPS��1-BDS
    bool ischange[2] = { false, false }; 
    for (int i = 0; i < 2; i++)
    {
        if (ekf->CurDDObs.RefPrn[i] != ekf->DDObs.RefPrn[i])  ischange[i] = true; // ��¼�仯���
        else continue;
    }

    // ��¼����Ԫ�Ĳο�������һ��Ԫ���Ƿ����
    bool isvalid[2] = { false ,false }; 
    int refindex[2] = { -1,-1 }; // ����Ԫ�ο�������һ��Ԫ���������еĶ�Ӧ�洢λ��
    for (int j = 0; j < ekf->SDObs.SatNum; j++)
    {
        if (!(ekf->SDObs.SdSatObs[j].Valid == 1 && ekf->SDObs.SdSatObs[j].half[0] == 1 && ekf->SDObs.SdSatObs[j].half[1] == 1)) continue;
        else;
        if (ekf->SDObs.SdSatObs[j].System == GPS && ekf->SDObs.SdSatObs[j].Prn == ekf->CurDDObs.RefPrn[0])
        {
            isvalid[0] = true;
            refindex[0] = j;
        }                          
        else if (ekf->SDObs.SdSatObs[j].System == BDS && ekf->SDObs.SdSatObs[j].Prn == ekf->CurDDObs.RefPrn[1])
        {
            isvalid[1] = true;
            refindex[1] = j;
        }
        else continue;
    }

    int xnum = 3; // ����Ԫ��״̬X�Ķ�Ӧfai��Q�Ĵ�������� ����������Index�ṩ��
    int initIndex[MAXCHANNUM]; // ��Ҫ��ʼ�������Ƕ�Ӧ˫��ģ�����ڵ�ǰ��ԪSdobs�е�����
    int ii = 0; // initIndex�������
    memset(initIndex, -1, sizeof(initIndex));

    // �������Է�ӳ��ǰ��Ԫ״̬X��˫��ģ���ȴ��˳���SdObs
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // �ο��ǣ�����
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // ����������ܱ�ǣ�����
        else; // �����ṩ��ǰ��Ԫ״̬�е�˫��ģ����

        bool isfind = false; // ��¼��ǰ��Ԫ�ĸÿ����Ƕ�Ӧ��˫��ģ��������һ��Ԫ���ܷ��ҵ�

        for (int j = 0; j < ekf->SDObs.SatNum; j++) // �������Է�ӳ��һ��Ԫ״̬X��˫��ģ���ȴ��˳���SdObs
        {
            if (ekf->SDObs.SdSatObs[j].System == Raw->SdObs.SdSatObs[i].System && ekf->SDObs.SdSatObs[j].Prn == Raw->SdObs.SdSatObs[i].Prn)
            {
                isfind = true;
                if (ekf->SDObs.SdSatObs[j].Valid == 1 && ekf->SDObs.SdSatObs[j].half[0] == 1 && ekf->SDObs.SdSatObs[j].half[1] == 1)
                {
                    // ����ϵͳ��־
                    short sys = -1;
                    if (ekf->SDObs.SdSatObs[j].System == GPS) sys = 0;
                    else sys = 1;

                    // ���1���ο���δ�ı�
                    if (ischange[sys] == false)
                    {
                        // ��һ��Ԫ�����������ܱ��,�����ֱ�Ӽ̳�
                        fai[xnum * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[j]] = 1;
                        fai[(xnum + 1) * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[j] + 1] = 1;
                        Q[xnum + xnum * (3 + 2 * ekf->CurDDObs.Sats)] = 1e-5 * 1e-5;
                        Q[(xnum + 1) + (xnum + 1) * (3 + 2 * ekf->CurDDObs.Sats)] = 1e-5 * 1e-5;
                    }
                    // ���2���ο����иı�
                    else 
                    {
                        // ������Ԫ�Ĳο�������һ��Ԫ�п��ã�����Լ�Ӽ̳�
                        if (isvalid[sys] == true)
                        {
                            if (ekf->Index[j] != -1) // -1Ҫô����������Ҫô�ǲο��ǣ��˴��Ѿ��ų����������ǹ�һ������ο���
                            {
                                // ������Ԫ��ǰ��������һ��Ԫ�Ĳο��ǣ���ע���ݴ�
                                fai[xnum * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[j]] = 1;
                                fai[(xnum + 1) * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[j] + 1] = 1;
                            }
                            else;

                            fai[xnum * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[refindex[sys]]] = -1;                           
                            fai[(xnum + 1) * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[refindex[sys]]+1] = -1;
                            Q[xnum + xnum * (3 + 2 * ekf->CurDDObs.Sats)] = 1e-5 * 1e-5;
                            Q[(xnum + 1) + (xnum + 1) * (3 + 2 * ekf->CurDDObs.Sats)] = 1e-5 * 1e-5;
                        }
                        // ������Ԫ�Ĳο�������һ��Ԫ�в����ã��򲻿ɼ̳У���Ҫ��ʼ��
                        else
                        {
                            // faiԪ��Ĭ��Ϊ0����ʹ����һ��Ԫ��˫��ģ���ȴ���
                            // QԪ����Ҫ����ʼ����׼����
                            Q[xnum + xnum * (3 + 2 * ekf->CurDDObs.Sats)] = 100;
                            Q[(xnum + 1) + (xnum + 1) * (3 + 2 * ekf->CurDDObs.Sats)] = 500;
                            initIndex[ii] = i; // ��¼���Ժ���г�ʼ��
                            ii++;
                        }

                    }                  
                }
                else // ��һ��Ԫ�ĸÿ����������⣬���޷��̳У���Ҫ��ʼ��
                {
                    // faiԪ��Ĭ��Ϊ0����ʹ����һ��Ԫ��˫��ģ���ȴ���
                    // QԪ����Ҫ����ʼ����׼����
                    Q[xnum + xnum * (3 + 2 * ekf->CurDDObs.Sats)] = 100;
                    Q[(xnum + 1) + (xnum + 1) * (3 + 2 * ekf->CurDDObs.Sats)] = 500;
                    initIndex[ii] = i; // ��¼���Ժ���г�ʼ��
                    ii++;
                }

                break;
            }
            else continue;
        }

        if (isfind == false) 
        {
            // falԪ��Ĭ��Ϊ0����һ��Ԫ��˫��ģ���ȶԸÿ����Ƕ�Ӧ��˫��ģ����û�д�������
            // QԪ����Ҫ����ʼ����׼����
            Q[xnum + xnum * (3 + 2 * ekf->CurDDObs.Sats)] = 100;
            Q[(xnum + 1) + (xnum + 1) * (3 + 2 * ekf->CurDDObs.Sats)] = 500;
            initIndex[ii] = i; // ��¼���Ժ���г�ʼ��
            ii++;
        }

        xnum = xnum + 2; // ÿ�ſ��õ�˫�����Ǵ�������˫��ģ����
    }
   

    /*
    // ����Q�Ĵ�С
    for (int i = 0; i < (3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM); i++)
    {
        Q[i] = Q[i] * 0.001;
    }
    */
    // fai Q ���ݼ��㲿��
    double faiT[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)];
    memset(faiT, 0, sizeof(faiT));
    double faiP[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)];
    memset(faiP, 0, sizeof(faiP));
    double faiPfaiT[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)];
    memset(faiPfaiT, 0, sizeof(faiPfaiT));
    // ��������
    MatrixMultiply(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 1, fai, ekf->X0, ekf->X);
    MatrixTranspose(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->DDObs.Sats, fai, faiT);
    MatrixMultiply(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->DDObs.Sats, fai, ekf->P0, faiP);
    MatrixMultiply(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->CurDDObs.Sats, faiP, faiT, faiPfaiT);
    MatrixAddition(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->CurDDObs.Sats, faiPfaiT, Q, ekf->P);


    // ����Ԫ����˫��ģ���Ȳ�©���֣���Ҫ��ʼ��
    // initIndex������Ϊ��ǰ��Ԫ��������Ҫ����˫��ģ���ȳ�ʼ��������
    xnum = 3;
    ii = 0;
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // �ο��ǣ�����
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // ����������ܱ�ǣ�����
        else; // �����ṩ��ǰ��Ԫ״̬�е�˫��ģ����

        // �ж��Ƿ�Ϊ��Ҫ��ʼ����˫��ģ���ȶ�Ӧ������
        if (i == initIndex[ii])
        {
            // ȷ���ÿ���������ϵͳ
            int sys = -1;
            if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0;
            else sys = 1;
            // ����˫��۲�ֵ����ģ���ȳ�ʼ��
            double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            ekf->X[xnum] = (sys == 0) ? ((ddl[0] - ddp[0]) / WL1_GPS) : ((ddl[0] - ddp[0]) / WL1_BDS);
            ekf->X[xnum + 1] = (sys == 0) ? ((ddl[1] - ddp[1]) / WL2_GPS) : ((ddl[1] - ddp[1]) / WL3_BDS);
            ii++;
        }
        else;
        
        xnum = xnum + 2; // ����״̬�еĴ洢����
    }

    // ����ekf��index���
    ii = 3; // �洢����      
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        // �ο��ǲ��������˫��������
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1])
        {
            ekf->Index[i] = -1;
            continue;
        }
        
        // �ж��Ƿ���ڴֲ�Ͱ���
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1))
        {
            ekf->Index[i] = -1;
            continue;
                     
        }
        else
        {
            ekf->Index[i] = ii; // �������Ƕ�Ӧվ�䵥�����ݲ�����˫Ƶ˫��ģ���ȴ����״̬X�е�����Ϊii��ii+1
            ii = ii + 2;
        }
    }

    return true;
}


/*
************************************************************************************
���������������º���
������Raw    ָ����Ԫ�����ݵ�ָ�루��ʱ�Ѱ����̶��������ģ���ȣ�
      ekf    �˲�����
�������ܣ���һ��Ԥ����ɺ���˲����ݽ��в������£���Ϊ�����֣�����ʹ��˫��
          �۲�ֵ����һ�β������£�֮�����Թ̶�ģ����Ϊ�۲��ٴθ���
����ֵ���̶���ɹ��򷵻�true��ʧ���򷵻�false
************************************************************************************
*/
bool EkfMeasureUpdate(RAWDAT* Raw, RTKEKF* ekf)
{
    /*** ˫��۲�ֵ�������� ***/
    // ����H��V��R�󲢳�ʼ��
    double H[(4 * MAXCHANNUM) * (3 + MAXCHANNUM * 2)], V[4 * MAXCHANNUM], R[(4 * MAXCHANNUM) * (4 * MAXCHANNUM)];
    memset(H, 0.0, sizeof(H));
    memset(V, 0.0, sizeof(V));
    memset(R, 0.0, sizeof(R));

    // ��ȡ�ο�����λ����ԭʼ�۲������е����� 0=GPS, 1=BDS
    short RefSatOfRov[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nRov,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nRov };
    short RefSatOfBas[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nBas,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nBas };
    // �ο����ǵ�λ�ã���������վ�ͻ�վ��˵������ʱ�䲻һ����ȫͬ��,�ʸ����ø����������õ�����λ�ã�
    double RefSatPosOfGPSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[2] };
    double RefSatPosOfBDSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[2] };
    double RefSatPosOfGPSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[2] };
    double RefSatPosOfBDSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[2] };

    // ��ȡ��վ����(ʹ�ý�������׼ȷ����)
    double BasPos[3];
    BLHToXYZ(Raw->BasPres.Pos, BasPos, R_WGS84, F_WGS84);
    // �����վ���굽�������ǵļ��ξ���
    double BasToSats[MAXCHANNUM]; // �±����վԭʼ�۲����ݵ�����һ��
    memset(BasToSats, 0.0, sizeof(BasToSats));
    for (int i = 0; i < Raw->BasEpk.SatNum; i++)
    {
        BasToSats[i] = sqrt((BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) * (BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) + (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) * (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) + (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]) * (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]));
    }

    // ��������վ����ο��ǵļ��ξ��� 0=GPS, 1=BDS    
    double RovToRefSat[2];
    double RovPos[3] = { ekf->X[0],ekf->X[1],ekf->X[2] };
    RovToRefSat[0] = sqrt((RovPos[0] - RefSatPosOfGPSR[0]) * (RovPos[0] - RefSatPosOfGPSR[0]) + (RovPos[1] - RefSatPosOfGPSR[1]) * (RovPos[1] - RefSatPosOfGPSR[1]) + (RovPos[2] - RefSatPosOfGPSR[2]) * (RovPos[2] - RefSatPosOfGPSR[2]));
    RovToRefSat[1] = sqrt((RovPos[0] - RefSatPosOfBDSR[0]) * (RovPos[0] - RefSatPosOfBDSR[0]) + (RovPos[1] - RefSatPosOfBDSR[1]) * (RovPos[1] - RefSatPosOfBDSR[1]) + (RovPos[2] - RefSatPosOfBDSR[2]) * (RovPos[2] - RefSatPosOfBDSR[2]));

    // GPS��BDS��ϵͳ������˫��������  0=GPS, 1=BDS
    int DDsatnum[2] = { 0,0 };
    int totalsatnum = 0; // �ܵĿ���˫��������(��׼�ǲ�����)

    // ��������۲�ֵ�����H��V����ͬʱͳ�Ƹ�ϵͳ�Ŀ���˫��������
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {       
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // �ο��ǣ�����
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // �������������⣬����
        else;

        // ��ǰ���ǵ�λ�ã�������վԭʼ�۲������еģ�����
        int satindex = Raw->SdObs.SdSatObs[i].nRov;
        // ��������վ���������ǵļ��ξ���
        double RovToSat = sqrt((RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) * (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) + (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) * (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) + (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) * (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]));
     
        
        // ȷ��������������ϵͳ
        int sys = -1;
        if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0;
        else sys = 1;

        // ����H����м���
        double l = (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) / RovToSat - (RovPos[0] - ((sys == 0) ? RefSatPosOfGPSR[0] : RefSatPosOfBDSR[0])) / RovToRefSat[sys];
        double m = (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) / RovToSat - (RovPos[1] - ((sys == 0) ? RefSatPosOfGPSR[1] : RefSatPosOfBDSR[1])) / RovToRefSat[sys];
        double n = (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) / RovToSat - (RovPos[2] - ((sys == 0) ? RefSatPosOfGPSR[2] : RefSatPosOfBDSR[2])) / RovToRefSat[sys];
        // ���H�����������
        for (int j = 0; j < 4; j++)
        {
            H[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j)] = l;
            H[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 1] = m;
            H[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 2] = n;
            if (j == 2) H[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 3 + 2 * totalsatnum] = (sys == 0) ? WL1_GPS : WL1_BDS;
            else if (j == 3) H[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 4 + 2 * totalsatnum] = (sys == 0) ? WL2_GPS : WL3_BDS;
            else continue;
        }

        // ����˫��۲�ֵ
        double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
        double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
        double ddrou = RovToSat - RovToRefSat[sys] + BasToSats[RefSatOfBas[sys]] - BasToSats[Raw->SdObs.SdSatObs[i].nBas];
        // ���V����������
        V[4 * totalsatnum] = ddp[0] - ddrou;
        V[4 * totalsatnum + 1] = ddp[1] - ddrou;
        V[4 * totalsatnum + 2] = (sys==0)?(ddl[0] - ddrou - ekf->X[3 + totalsatnum * 2] * WL1_GPS):(ddl[0] - ddrou - ekf->X[3 + totalsatnum * 2] * WL1_BDS);
        V[4 * totalsatnum + 3] = (sys==0)?(ddl[1] - ddrou - ekf->X[3 + totalsatnum * 2 + 1] * WL2_GPS):(ddl[1] - ddrou - ekf->X[3 + totalsatnum * 2 + 1] * WL3_BDS);
        
        // ���¿������Ǽ���
        DDsatnum[sys]++;
        totalsatnum = DDsatnum[0] + DDsatnum[1];
      
    }
    ekf->nSats = totalsatnum;
    
    // ���R��,GP��BP�Ǵ�������ֵ
    double GP[4] = { 4 * cfginfo.CodeNoise * cfginfo.CodeNoise , 4 * cfginfo.CPNoise * cfginfo.CPNoise,  2 * cfginfo.CodeNoise * cfginfo.CodeNoise,2 * cfginfo.CPNoise * cfginfo.CPNoise };
    double BP[4] = { 4 * cfginfo.CodeNoise * cfginfo.CodeNoise , 4 * cfginfo.CPNoise * cfginfo.CPNoise,  2 * cfginfo.CodeNoise * cfginfo.CodeNoise,2 * cfginfo.CPNoise * cfginfo.CPNoise };
   
    // ���GPS����
    for (int i = 0; i < 4 * DDsatnum[0]; i++)
    {
        for (int j = 0; j < 4 * DDsatnum[0]; j++)
        {
            // ���Խ���Ԫ��
            if (j == i)
            {
                if (i % 4 == 0 || i % 4 == 1) R[i * (4 * MAXCHANNUM) + j] = GP[0];
                else R[i * (4 * MAXCHANNUM) + j] = GP[1];
            }
            // ���ǶԽ���Ԫ��
            else if (j % 4 == i % 4)
            {
                if (i % 4 == 0 || i % 4 == 1) R[i * (4 * MAXCHANNUM) + j] = GP[2];
                else R[i * (4 * MAXCHANNUM) + j] = GP[3];
            }
        }
    }
    // ���BDS����
    int startRow = 4 * DDsatnum[0]; // BDS���ֿ�ʼ����
    int startCol = 4 * DDsatnum[0]; // BDS���ֿ�ʼ����
    for (int i = 0; i < 4 * DDsatnum[1]; i++) 
    {
        for (int j = 0; j < 4 * DDsatnum[1]; j++) 
        {
            int rowIndex = startRow + i; // ����������
            int colIndex = startCol + j; // ����������
            // ���Խ���Ԫ��
            if (i == j) {
                if (i % 4 == 0 || i % 4 == 1) R[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[0];
                else R[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[1];
            }
            // ���ǶԽ���Ԫ��
            else if (i % 4 == j % 4) {
                if (i % 4 == 0 || i % 4 == 1) R[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[2];
                else R[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[3];
            }
        }
    }


    // �����ع���ʹ��ƥ�䱾��Ŀ�ľ������㺯��
    restructureMatrix(4 * totalsatnum, 3 + totalsatnum * 2, 4 * MAXCHANNUM, 3 + MAXCHANNUM * 2, H);
    restructureMatrix(4 * totalsatnum, 4 * totalsatnum, 4 * MAXCHANNUM, 4 * MAXCHANNUM, R);

    // ���в�������
    int calflag1 = CalcuUpdate(3 + totalsatnum * 2, 4 * totalsatnum, H, V, R, ekf->X, ekf->P);
    if (calflag1 == 0)  return false;


    /*** �̶��������ģ������Ϊ�۲���в������� ***/
    // ��ȡ˫��ģ���ȶ�Ӧ��P
    double Pn[(MAXCHANNUM * 2) * (MAXCHANNUM * 2)]; // ģ���ȵ�P��
    GetQnn(totalsatnum, ekf->P, Pn);
    
    // ����LAMBDA�̶�˫��ģ����
    double FixedAmb[MAXCHANNUM * 4];
    double ResAmb[2];
    if(lambda(totalsatnum * 2, 2, ekf->X+3, Pn, FixedAmb, ResAmb)!=0) return false;
    ekf->ratio = ResAmb[1] / ResAmb[0];

    ekf->beFixed = false;
    // ���̶�������й̶�ģ���ȹ۲����
    if (ekf->ratio > cfginfo.RatioThres)
    {
        // ����H1��V1��R1�󲢳�ʼ������
        double  H1[(2 * MAXCHANNUM) * (3 + MAXCHANNUM * 2)], V1[2 * MAXCHANNUM], R1[(2 * MAXCHANNUM) * (2 * MAXCHANNUM)];
        memset(H1, 0.0, sizeof(H1));
        memset(V1, 0.0, sizeof(V1));
        memset(R1, 0.0, sizeof(R1));

        // ���H1��V1��R1
        for (int i = 0; i < ekf->CurDDObs.Sats * 2; i++)
        {
            V1[i] = FixedAmb[i] - ekf->X[3 + i];
            R1[i + i * ekf->CurDDObs.Sats * 2] = 1e-8;
            H1[3 + i + i * (3 + ekf->CurDDObs.Sats * 2)] = 1;
        }

        // ��һ�ν��в�������
        int calflag2 = CalcuUpdate(3 + totalsatnum * 2, 2 * totalsatnum, H1, V1, R1, ekf->X, ekf->P);
        if (calflag2 == 0) return false;

        ekf->beFixed = true;
    }
    //else ekf->IsInit = false; // �������³�ʼ��

    ekf->sigmaP = sqrt(ekf->P[0] + ekf->P[3 + totalsatnum * 2 + 1] + ekf->P[2 * (3 + totalsatnum * 2) + 2]);
    
    // ����״̬�������Э������
    memcpy(&ekf->X0, &ekf->X, sizeof(ekf->X));
    memcpy(&ekf->P0, &ekf->P, sizeof(ekf->P));

    // ״̬����
    memset(&ekf->X, 0, sizeof(ekf->X));
    memset(&ekf->P, 0, sizeof(ekf->P));

    return true;
}



/*
*****************************************************************
����������ӡ������
������rows     ���������
      cols     ���������
      matrix   ��ž������ݵ�һά���飨ǰ�������Ϊ��Ч���ݣ�
      filename �洢�ļ���
�������ܣ���ӡ����txt�ļ��Ա���
*****************************************************************
*/
void MatrixDisplay(int rows, int cols, double matrix[], const char* filename) 
{
    FILE* file = NULL;
    errno_t err = fopen_s(&file, filename, "w");
    if (err != 0 || file == NULL) {
        // �ļ���ʧ��
        return ;
    }

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            fprintf(file, "%10.5f ", matrix[i * cols + j]);
        }
        fprintf(file, "\n");
    }

    fclose(file);
}

/*
***********************************************************
����������ȡQNN������
������satnum   ����˫������������������׼�ǣ�
      N_inv    ������ά�������ڵĲ�����Ӧ��Э������
      Q        ģ���Ȳ�����Ӧ��Э������
�������ܣ���N_invȡ��QNN�Ĳ������ں���lambda����
***********************************************************
*/
void GetQnn(const int satnum,const double N_inv[], double Q[]) 
{
    int N_inv_size = (3 + 2 * satnum) * (3 + 2 * satnum); // N_inv�е���Ч��������
    int skip_count = 3 * (3 + 2 * satnum); // ������Ҫ������Ԫ������
    int index_N_inv = skip_count; // ����N_inv�ĵ�ǰ����
    int index_Q = 0; // ����Q�ĵ�ǰ����

    while (index_N_inv + 2 * satnum <= N_inv_size) 
    {
        // ����3��Ԫ��
        index_N_inv += 3;

        // ����2*satnum��Ԫ�ص�Q����
        for (int i = 0; i < 2 * satnum; i++) 
        {
            Q[index_Q] = N_inv[index_N_inv];
            index_Q++;
            index_N_inv++;
        }
    }
}


/*
**************************************************************
���������������˲��еĲ������µĹ�ʽ���㺯��
������n    X/P��ʵ������ ��Ӧ(3 + MAXCHANNUM * 2)
      m    H/V/R��ʵ������ ��Ӧ(4 * MAXCHANNUM)
      H    �۲����m*n
      V    ��Ϣ��m*1
      R    �۲�������m*m
      X    ״̬����n*1
      P    ״̬���ķ���Э������n*n
�������ܣ����в������¼��㣬�õ����º��X��P��
ע�⣺X��P����ʱΪԤ��Ľ��������ִ�н�����Ϊ���º�Ľ��
����ֵ������ɹ��򷵻�1������ʧ���򷵻�0
**************************************************************
*/
int CalcuUpdate(int n, int m, double H[], double V[], double R[], double X[], double P[])
{
    double X1[3 + MAXCHANNUM * 2], P1[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];
    memset(X1, 0, (3 + MAXCHANNUM * 2) * sizeof(double));
    memset(P1, 0, (3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2) * sizeof(double));
    double* HT = new double[n * m];
    double* PHT = new double[n * m];
    double* HPHT = new double[m * m];
    double* HPHTaddR = new double[m * m];
    double* HPHTaddR_inverse = new double[m * m];
    double* K = new double[n * m];
    double* KV = new double[n];
    double* I = new double[n * n];
    double* KH = new double[n * n];
    double* IsubKH = new double[n * n];

    // ��ʼ����̬����Ϊ0
    memset(HT, 0, n * m * sizeof(double)); // ��ʼ�� HT
    memset(PHT, 0, n * m * sizeof(double)); // ��ʼ�� PHT
    memset(HPHT, 0, m * m * sizeof(double)); // ��ʼ�� HPHT
    memset(HPHTaddR, 0, m * m * sizeof(double)); // ��ʼ�� HPHTaddR
    memset(HPHTaddR_inverse, 0, m * m * sizeof(double)); // ��ʼ�� HPHTaddR_inverse
    memset(K, 0, n * m * sizeof(double)); // ��ʼ�� K
    memset(KV, 0, n * sizeof(double)); // ��ʼ�� KV
    memset(I, 0, n * n * sizeof(double)); // ��ʼ�� I
    memset(KH, 0, n * n * sizeof(double)); // ��ʼ�� KH
    memset(IsubKH, 0, n * n * sizeof(double)); // ��ʼ�� IsubKH


    memcpy(P1, P, (3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2) * sizeof(double));
    memcpy(X1, X, (3 + MAXCHANNUM * 2) * sizeof(double));
    for (int i = 0; i < n; i++) I[i + i * n] = 1;

    // ��������
    MatrixTranspose(m, n, H, HT);
    MatrixMultiply(n, n, n, m, P, HT, PHT);
    MatrixMultiply(m, n, n, m, H, PHT, HPHT);
    MatrixAddition(m, m, HPHT, R, HPHTaddR);
    int iserror = MatrixInv(m, HPHTaddR, HPHTaddR_inverse);
    if (iserror == 0)
    {
        // �ͷŶ�̬������ڴ�
        delete[] HT;
        delete[] PHT;
        delete[] HPHT;
        delete[] HPHTaddR;
        delete[] HPHTaddR_inverse;
        delete[] K;
        delete[] KV;
        delete[] I;
        delete[] KH;
        delete[] IsubKH;
        return 0;

    }
    MatrixMultiply(n, m, m, m, PHT, HPHTaddR_inverse, K);
    MatrixMultiply(n, m, m, 1, K, V, KV);
    MatrixAddition(n, 1, X1, KV, X);
    MatrixMultiply(n, m, m, n, K, H, KH);
    MatrixSubtraction(n, n, I, KH, IsubKH);
    MatrixMultiply(n, n, n, n, IsubKH, P1, P);

    // �ͷŶ�̬������ڴ�
    delete[] HT;
    delete[] PHT;
    delete[] HPHT;
    delete[] HPHTaddR;
    delete[] HPHTaddR_inverse;
    delete[] K;
    delete[] KV;
    delete[] I;
    delete[] KH;
    delete[] IsubKH;
    return 1;
}