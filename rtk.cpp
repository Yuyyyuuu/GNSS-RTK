#include"MyHeadFile.h"

extern ROVERCFGINFO cfginfo;

/*
****************************************************************************
函数名：数据同步函数（文件）
参数：FBas 指向基站数据文件的指针
      FRov 指向流动站数据文件的指针
      Raw  同步数据存储处（两个站的观测值、星历、参考定位结果）
返回值：数据同步结果的标志  1-数据同步成功  0-数据同步失败 -1-文件数据结束
函数功能：读取并解码两个站对应文件中的数据，并将其进行时间同步
****************************************************************************
*/
int GetSynObsPP(FILE* FBas, FILE* FRov, RAWDAT* Raw)
{
    // 将上一历元的两站数据存储好
    memcpy(&Raw->BasEpk0, &Raw->BasEpk, sizeof(Raw->BasEpk0));
    memcpy(&Raw->RovEpk0, &Raw->RovEpk, sizeof(Raw->RovEpk0));
	// 解码流动站数据
    static unsigned char Rbuf[MAXRAWLEN];
    static int Rlen = 0;
    static int Rflag = 0;
    while (!feof(FRov))
    {
        int obtain_byte = fread(Rbuf + Rlen, 1, MAXRAWLEN - Rlen, FRov);
        if (obtain_byte < MAXRAWLEN - Rlen) return -1; // 流动站数据文件结束
        Rlen = obtain_byte + Rlen;
        Rflag = DecodeNovOem7Dat(Rbuf, Rlen, &Raw->RovEpk, Raw->GpsEph, Raw->BdsEph, &Raw->RovPres);
        if (Rflag == 1) break; // 解码到了观测值，跳出循环
    }
  
    // 求两站时间差值(单位为s)
    double dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);

    if (fabs(dt) < 0.1) return 1; // 时间同步
    else if (dt < 0) return 0; // 流动站数据滞后，无法解算
    else // 基站数据滞后，需要读取更后面时刻的基站数据(循环执行直至时间同步为止)
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
                if (obtain_byte < MAXRAWLEN - Blen)  return -1; // 基站数据文件结束
                Blen = obtain_byte + Blen;
                Bflag = DecodeNovOem7Dat(Bbuf, Blen, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph, &Raw->BasPres);
                if (Bflag == 1) break; // 解码到了观测值，跳出循环
            }

            dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);

            if (dt < 0) return 0; // 基站历元数据丢失，无法解算

            if (fabs(dt) < 0.1)  flag = false;// 时间完成同步

        } while (flag);
       return 1; 
    }
	
}

/*
****************************************************************************
函数名：数据同步函数（实时）
参数：BasSock 基站数据文件的网口
      RovSock 流动站数据文件的网口
      Raw     同步数据存储处（两个站的观测值、星历、参考定位结果）
返回值：数据同步结果的标志  1-数据同步成功  0-数据同步失败 
函数功能：读取并解码两个站对应网口中的数据，并将其进行时间同步
****************************************************************************
*/
int GetSynObsRT(SOCKET& BasSock, SOCKET& RovSock, RAWDAT* Raw)
{
    // 将上一历元的两站数据存储好
    memcpy(&Raw->BasEpk0, &Raw->BasEpk, sizeof(Raw->BasEpk0));
    memcpy(&Raw->RovEpk0, &Raw->RovEpk, sizeof(Raw->RovEpk0));

    Sleep(900);

    // 解码流动站数据
    static unsigned char RBuf[MAXRAWLEN * 2];
    static int RlenB = 0; // RBuf的长度（字节数）
    unsigned char Rbuf[MAXRAWLEN]; // 中间存储地
    int RlenT = 0; // 中间存储地的长度
    if ((RlenT = recv(RovSock, (char*)Rbuf, MAXRAWLEN, 0)) > 0)
    {
        if ((RlenB + RlenT) > MAXRAWLEN * 2)  RlenB = 0; // 若本次读入数据加入大BUFF后会超过其最大长度，则先将原先已有的数据舍弃
        memcpy(RBuf + RlenB, Rbuf, RlenT);
        RlenB = RlenT + RlenB;
        DecodeNovOem7Dat(RBuf, RlenB, &Raw->RovEpk, Raw->GpsEph, Raw->BdsEph, &Raw->RovPres);

    }

    // 解码基站数据
    static unsigned char BBuf[MAXRAWLEN * 2];
    static int BlenB = 0; // BBuf的长度（字节数）
    unsigned char Bbuf[MAXRAWLEN];
    int BlenT = 0; // 中间存储地的长度
    if ((BlenT = recv(BasSock, (char*)Bbuf, MAXRAWLEN, 0)) > 0)
    {
        if ((BlenB + BlenT) > MAXRAWLEN * 2)  BlenB = 0;
        memcpy(BBuf + BlenB, Bbuf, BlenT);
        BlenB = BlenT + BlenB;
        DecodeNovOem7Dat(BBuf, BlenB, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph, &Raw->BasPres);
    }

    // 求两站时间差值(单位为s)
    double dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);

    if (fabs(dt)<10 && (Raw->RovEpk.Time.SecOfWeek> Raw->RovEpk0.Time.SecOfWeek)) return 1; // 时间同步
    else return 0; // 时间不同步
}



/*
******************************************************************
函数名：观测数据有效性标记函数
参数：EpkA 指向上一历元观测数据的指针
      EpkB 指向当前历元观测数据的指针
函数功能：利用接收机的质量数据（即连续跟踪时间locktime）
          对当前历元的观测数据进行有效性标记
******************************************************************
*/
void MarkValidPre(const EPOCHOBS* EpkA, EPOCHOBS* EpkB)
{
    // 标记准则：只有当双频的locktime均通过检验才标记为true
    
    // GPS：L1-0，L2-1   BDS：B1I-0, B3I-1
    bool flag[2]; // 是否连续跟踪的标志（locktime）
   
    // 遍历本历元观测数据中的每一颗卫星
    for (int i = 0; i < EpkB->SatNum; i++)
    {
        // 对于本历元的每颗卫星，其两个质量数据的标志均初始化为false
        flag[0] = flag[1] = false;

        // 寻找本颗卫星在上一历元中的数据
        for (int j = 0; j < EpkA->SatNum; j++) 
        {
            if (EpkA->SatObs[j].System == EpkB->SatObs[i].System && EpkA->SatObs[j].Prn == EpkB->SatObs[i].Prn)
            {
                // 对两个频率的连续跟踪情况进行检验
                for (int n = 0; n < 2; n++)
                {
                    if (EpkA->SatObs[i].LockTime[n]==0.0)
                    {
                        // 若上一历元的数据是失锁状态，则本历元的连续跟踪时长大于等于6s才视为通过检验
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
        
        // 进行观测数据的有效性标记：只有双频均通过连续跟踪条件才标记为true
        if (EpkB->SatObs[i].Valid == true) // 对单站数据周跳探测结果为真的再进一步利用接收机内部指标标记
        {
            if (flag[0] == true && flag[1] == true) EpkB->SatObs[i].Valid = true;
            else EpkB->SatObs[i].Valid = false;
        }
        else; // 默认为false
      
    }               
}

/*
**************************************************************
函数名：站间单差函数
参数：EpkR  指向流动站观测数据的指针
      EpkB  指向基站观测数据的指针
      SDObs 单差结果存储处
函数功能：将流动站与基站的观测数据求差,同时将半周标记传递下来
**************************************************************
*/
void FormSDEpochObs(const EPOCHOBS* EpkR, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs)
{
    memset(SDObs->SdSatObs, 0.0, sizeof(SDObs->SdSatObs));

    SDObs->Time = EpkR->Time;

    int satnum = 0; // 单差观测数据的可用卫星数

    // 外层遍历流动站的观测数据
    for (int i = 0; i < EpkR->SatNum; i++)
    {
        // 若接收机的内部质量指标不合格或卫星星历不可用则跳过本颗卫星数据，不做单差处理
        if (EpkR->SatObs[i].Valid == false || EpkR->SatPVT[i].Valid == false) continue;
        else;
        // 内层遍历基站的观测数据
        for (int j = 0; j < EpkB->SatNum; j++)
        {
            // 若接收机的内部质量指标不合格或卫星星历不可用则跳过本颗卫星数据，不做单差处理
            if (EpkB->SatObs[j].Valid == false || EpkB->SatPVT[i].Valid == false) continue;
            // 若没有问题则进行匹配
            else if (EpkB->SatObs[j].System == EpkR->SatObs[i].System && EpkB->SatObs[j].Prn == EpkR->SatObs[i].Prn)
            {
                SDObs->SdSatObs[satnum].System = EpkR->SatObs[i].System;
                SDObs->SdSatObs[satnum].Prn = EpkR->SatObs[i].Prn;
                SDObs->SdSatObs[satnum].nRov = i;
                SDObs->SdSatObs[satnum].nBas = j;

                for (int k = 0; k < 2; k++)
                {
                    // 只有两个站的伪距都有值才做其单差，否则说明数据丢失->做置零处理
                    if (fabs(EpkR->SatObs[i].p[k]) > 1e-8 && fabs(EpkB->SatObs[j].p[k]) > 1e-8)
                    {
                        SDObs->SdSatObs[satnum].dP[k] = EpkR->SatObs[i].p[k] - EpkB->SatObs[j].p[k];
                    }
                    else SDObs->SdSatObs[satnum].dP[k] = 0;
                    // 只有两个站的相位都有值才做其单差，否则说明数据丢失->做置零处理
                    if (fabs(EpkR->SatObs[i].l[k]) > 1e-8 && fabs(EpkB->SatObs[j].l[k]) > 1e-8)
                    {
                        SDObs->SdSatObs[satnum].dL[k] = EpkR->SatObs[i].l[k] - EpkB->SatObs[j].l[k];
                    }
                    else SDObs->SdSatObs[satnum].dL[k] = 0;    

                    // 传递半周标记
                    if (EpkR->SatObs[i].half[k] == 1 && EpkB->SatObs[j].half[k] == 1) SDObs->SdSatObs[satnum].half[k] = 1; // 不存在半周问题
                    else SDObs->SdSatObs[satnum].half[k] = 0; // 可能存在半周问题
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
函数名：周跳探测函数
参数：Obs  指向每个历元的单差观测数据的指针
函数功能：计算每个历元MW、GF组合观测值并用其来进行周跳探测，
          然后对每个卫星的单差观测数据进行有效性标记
**************************************************************
*/
void DetectCycleSlip(SDEPOCHOBS* Obs)
{
/* 思路：Obs中的单差组合观测值数组在进入本函数时为上一历元的数据，在函数结束后存放本历元的单
         差组合数据，建立缓冲区用于临时存放当前历元的计算结果以便进行历元间求差探测周跳。   */
    
    MWGF sdcomobs[MAXCHANNUM]; // 存放当前历元的单差组合观测值
    for (int i = 0; i < Obs->SatNum; i++)
    {
        // 检查卫星的单差数据是否正常
        if (fabs(Obs->SdSatObs[i].dL[0]) < 1e-8 || fabs(Obs->SdSatObs[i].dL[1]) < 1e-8 || fabs(Obs->SdSatObs[i].dP[0]) < 1e-8 || fabs(Obs->SdSatObs[i].dP[1]) < 1e-8)
        {
            // 若不正常则不满足周跳探测的条件，标记为无效
            Obs->SdSatObs[i].Valid = 0;
            continue;
        }
        else; // 卫星的双频伪距、相位单差数据均正常
        
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

        sdcomobs[i].n = 1; // 用于MW的平滑计数,此处赋值为1是为容错出现周跳的情况

        // 从上个历元的组合数据中查找本颗卫星的GF和MW组合值
        for (int j = 0; j < MAXCHANNUM; j++)
        {
            if (Obs->SdCObs[j].Sys == sdcomobs[i].Sys && Obs->SdCObs[j].Prn == sdcomobs[i].Prn)
            {
                // 计算GF、MW组合值的历元间差分
                double dGF = fabs(sdcomobs[i].GF - Obs->SdCObs[j].GF);
                double dMW = fabs(sdcomobs[i].MW - Obs->SdCObs[j].MW);
                // 判断是否超限，未超限则未检测出周跳，标记为true
                if (dGF < 0.05 && dMW < 3)
                {
                    Obs->SdSatObs[i].Valid = 1;
                    // 对MW组合值进行平滑
                    sdcomobs[i].MW = (Obs->SdCObs[j].MW * Obs->SdCObs[j].n + sdcomobs[i].MW) / (Obs->SdCObs[j].n + 1);
                    sdcomobs[i].n = Obs->SdCObs[j].n + 1; // 更新MW平滑计数
                }
                else Obs->SdSatObs[i].Valid = 0; // 超限，则标记为false

                break;
            }
            else continue;
        }
    }
    // 将缓冲区的组合观测值的拷贝到Obs里
    memcpy(Obs->SdCObs, sdcomobs, sizeof(sdcomobs));
}


/*
*************************************************************************************
函数名：确定基准星
参数：EpkR   指向流动站观测数据的指针
      EpkB   指向基站观测数据的指针
      SDObs  指向单差数据的指针
      DDObs  指向双差数据的指针
函数功能：选取基准星（双系统各一个），将其PRN号与索引存放在双差数据中
返回值：若两个系统的参考星均选取成功则返回true，否则返回false
注意：传入的EpkR、EpkB需要在本函数外提前经过一次SPP来得到卫星位置、高度角等相关数据
*************************************************************************************
*/
bool DetRefSat(const EPOCHOBS* EpkR, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
    /*
    每个卫星系统各选取一颗卫星作为参考星，0=GPS; 1=BDS
    条件：1、伪距和载波相位通过周跳探测，没有粗差和周跳以及半周标记
          2、卫星星历正常，卫星位置计算成功
          3、卫星高度角最大
    */
    short sys = 0; // 0=GPS; 1=BDS
    double MaxEle[2] = { 0 , 0 }; // 高度角（寻找最大所用）
    // 遍历单差数据
    for (int i = 0; i < SDObs->SatNum; i++)
    {
        if (SDObs->SdSatObs[i].Valid == false|| SDObs->SdSatObs[i].half[0]== 0 || SDObs->SdSatObs[i].half[1] == 0) continue; // 未通过周跳探测或半周检查，不可成为参考星
        else if (EpkR->SatPVT[SDObs->SdSatObs[i].nRov].Valid == false || EpkB->SatPVT[SDObs->SdSatObs[i].nBas].Valid == false) continue; // 卫星星历异常，不可成为参考星
        else;
        // 寻找两个系统中高度角最大的卫星
        // 确定存储索引
        if (SDObs->SdSatObs[i].System == GPS) sys = 0;
        else sys = 1;
        // 寻找最大高度角,以流动站为准（基站会人为地设置在开阔的好环境下，故主要兼顾流动站）
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
函数名：获取相对定位浮点解
参数：Raw    指向历元总数据的指针
      Rov    流动站的SPP结果（用于初始化坐标）
      Fres   浮点解结果
函数功能：进行浮点解计算，保存双差浮点解模糊度及其协因数矩阵、基线向量及其精度指标到Fres
返回值：浮点解成功则返回true，失败则返回false
*****************************************************************************************
*/
bool RTKFloat(RAWDAT* Raw, POSRES* Rov, FloatResult* Fres)
{
    /* 定义待用变量 */
    double BasPos[3]; // 获取基站坐标(使用解码所得准确坐标)
    BLHToXYZ(Raw->BasPres.Pos, BasPos, R_WGS84, F_WGS84);   
    double RovPos[3] = { Rov->Pos[0],Rov->Pos[1] ,Rov->Pos[2] }; // 设置流动站坐标初值(即SPP的结果)
    short DDsatnum[2] = { 0 ,0 }; // GPS和BDS两系统各可用双差卫星数  0=GPS, 1=BDS
    short totalsatnum = 0; // 总的可用双差卫星数(基准星不计入)
    double RovToRefSat[2] = {0,0};  // 流动站到其参考星的几何距离 0=GPS, 1=BDS   
    double BasToSats[MAXCHANNUM];  // 基站坐标到所有卫星的几何距离（下标与基站原始观测数据的索引一致）
    double DDN[MAXCHANNUM * 2]; // 待估双差模糊度
    double x[3 + MAXCHANNUM * 2]; // 待估参数的改正数
    double B[(4 * MAXCHANNUM) * (3 + MAXCHANNUM * 2)]; // 设计矩阵
    double w[4 * MAXCHANNUM]; // 残差阵
    double P[(4 * MAXCHANNUM) * (4 * MAXCHANNUM)]; // 权阵
    double N_inv[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)]; // 待估参数的协因数阵，定义在外以便后续存储

    // 提取参考卫星位置在原始观测数据中的索引 0=GPS, 1=BDS
    int RefSatOfRov[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nRov,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nRov };
    int RefSatOfBas[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nBas,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nBas };
    // 参考卫星的位置（对于流动站和基站来说，由于时间不一定完全同步,故各自用各自星历所得的卫星位置）
    double RefSatPosOfGPSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[2] };
    double RefSatPosOfBDSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[2] };
    double RefSatPosOfGPSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[2] };
    double RefSatPosOfBDSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[2] };

    /* 变量初始化 */
    // 矩阵初始化置零
    memset(DDN, 0.0, sizeof(DDN));
    memset(x, 0.0, sizeof(x));
    memset(B, 0.0, sizeof(B));
    memset(w, 0.0, sizeof(w));
    memset(P, 0.0, sizeof(P));
    memset(BasToSats, 0, sizeof(BasToSats));

    // 初始化双差模糊度，下标与所用观测方程的卫星数据顺序一致 
    int ii=0; // 存储计数
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {      
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // 参考星不计入可用双差卫星数
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // 存在粗差和半周则跳过
        else; // 本颗卫星可用

        // 确定该卫星所属系统
        int sys = -1;
        if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0;
        else sys = 1;

        // 计算双差观测值
        double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
        double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
        // 初始化双差模糊度
        DDN[ii] = (sys == 0) ? (ddl[0] - ddp[0]) / WL1_GPS : (ddl[0] - ddp[0]) / WL1_BDS;
        DDN[ii + 1] = (sys == 0) ? (ddl[1] - ddp[1]) / WL2_GPS : (ddl[1] - ddp[1]) / WL3_BDS;
        ii = ii + 2;
     
    }

    // 计算基站坐标到所有卫星的几何距离
    for (int i = 0; i < Raw->BasEpk.SatNum; i++)
    {
        BasToSats[i] = sqrt((BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) * (BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) + (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) * (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) + (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]) * (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]));
    }


    /* 进入最小二乘迭代 */
    bool flag = true; // 迭代控制器
    int count = 0; // 迭代次数
    do
    {
        // 每次迭代时先将相关矩阵先置零，以免数据填充混乱
        memset(x, 0, sizeof(x));
        memset(B, 0, sizeof(B));
        memset(w, 0, sizeof(w));
        memset(P, 0, sizeof(P));
        memset(N_inv, 0, sizeof(N_inv));

        // 每次迭代时可用卫星数归零，以免数据填充混乱
        DDsatnum[0] = DDsatnum[1] = 0;
        totalsatnum = 0; 

        // 计算流动站到其参考星的几何距离 0=GPS, 1=BDS       
        RovToRefSat[0] = sqrt((RovPos[0] - RefSatPosOfGPSR[0]) * (RovPos[0] - RefSatPosOfGPSR[0]) + (RovPos[1] - RefSatPosOfGPSR[1]) * (RovPos[1] - RefSatPosOfGPSR[1]) + (RovPos[2] - RefSatPosOfGPSR[2]) * (RovPos[2] - RefSatPosOfGPSR[2]));
        RovToRefSat[1] = sqrt((RovPos[0] - RefSatPosOfBDSR[0]) * (RovPos[0] - RefSatPosOfBDSR[0]) + (RovPos[1] - RefSatPosOfBDSR[1]) * (RovPos[1] - RefSatPosOfBDSR[1]) + (RovPos[2] - RefSatPosOfBDSR[2]) * (RovPos[2] - RefSatPosOfBDSR[2]));
        
        // 遍历单差观测值来填充B、w矩阵，同时统计各系统的可用双差卫星数
        for (int i = 0; i < Raw->SdObs.SatNum; i++)
        {
            
            if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // 参考星不计入可用双差卫星数
            else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // 存在粗差和半周
            else; // 本颗卫星可用

            // 当前卫星的位置（在流动站原始观测数据中的）索引
            short satindex = Raw->SdObs.SdSatObs[i].nRov;
            // 计算流动站到本颗卫星的几何距离
            double RovToSat = sqrt((RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) * (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) + (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) * (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) + (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) * (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]));
            // 确定本颗卫星所属系统
            int sys = -1;
            if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0; // GPS
            else sys = 1; // BDS

            // 计算B阵的中间量
            double l = (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) / RovToSat - (RovPos[0] - ((sys == 0) ? RefSatPosOfGPSR[0] : RefSatPosOfBDSR[0])) / RovToRefSat[sys];
            double m = (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) / RovToSat - (RovPos[1] - ((sys == 0) ? RefSatPosOfGPSR[1] : RefSatPosOfBDSR[1])) / RovToRefSat[sys];
            double n = (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) / RovToSat - (RovPos[2] - ((sys == 0) ? RefSatPosOfGPSR[2] : RefSatPosOfBDSR[2])) / RovToRefSat[sys];
            // 填充B阵的四行数据
            for (int j = 0; j < 4; j++)
            {
                B[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j)] = l;
                B[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 1] = m;
                B[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 2] = n;
                if (j == 2) B[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 3 + 2 * totalsatnum] = (sys == 0) ? WL1_GPS : WL1_BDS;
                else if (j == 3) B[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 4 + 2 * totalsatnum] = (sys == 0) ? WL2_GPS : WL3_BDS;
                else continue;
            }
           
            // 计算双差观测值
            double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            double ddrou = RovToSat - RovToRefSat[sys] + BasToSats[RefSatOfBas[sys]] - BasToSats[Raw->SdObs.SdSatObs[i].nBas];
            // 填充w阵的四行数据
            w[4 * totalsatnum] = ddp[0] - ddrou;
            w[4 * totalsatnum + 1] = ddp[1] - ddrou;
            w[4 * totalsatnum + 2] = (sys == 0) ? (ddl[0] - ddrou - DDN[totalsatnum * 2] * WL1_GPS) : (ddl[0] - ddrou - DDN[totalsatnum * 2] * WL1_BDS);
            w[4 * totalsatnum + 3] = (sys == 0) ? (ddl[1] - ddrou - DDN[totalsatnum * 2 + 1] * WL2_GPS) : (ddl[1] - ddrou - DDN[totalsatnum * 2 + 1] * WL3_BDS);
            
            // 对应系统可用卫星数增加
            DDsatnum[sys]++;
            totalsatnum = DDsatnum[0] + DDsatnum[1];    
        }
        

        // 填充P阵,GP和BP是待填充的数值   
        double GP[4] = { DDsatnum[0] / (DDsatnum[0] + 1.0),1000 * DDsatnum[0] / (DDsatnum[0] + 1.0),-1 / (DDsatnum[0] + 1.0),-1000 / (DDsatnum[0] + 1.0) };
        double BP[4] = { DDsatnum[1] / (DDsatnum[1] + 1.0),1000 * DDsatnum[1] / (DDsatnum[1] + 1.0),-1 / (DDsatnum[1] + 1.0),-1000 / (DDsatnum[1] + 1.0) };
        // 填充GPS部分
        for (int i = 0; i < 4 * DDsatnum[0]; i++)
        {
            for (int j = 0; j < 4 * DDsatnum[0]; j++)
            {
                // 填充对角线元素
                if (j == i)
                {
                    if (i % 4 == 0 || i % 4 == 1) P[i * (4 * MAXCHANNUM) + j] = GP[0];
                    else P[i * (4 * MAXCHANNUM) + j] = GP[1];
                }
                // 填充非对角线元素
                else if (j % 4 == i % 4)
                {
                    if (i % 4 == 0 || i % 4 == 1) P[i * (4 * MAXCHANNUM) + j] = GP[2];
                    else P[i * (4 * MAXCHANNUM) + j] = GP[3];
                }
            }
        }
        // 填充BDS部分
        int startRow = 4 * DDsatnum[0]; // BDS部分开始的行
        int startCol = 4 * DDsatnum[0]; // BDS部分开始的列
        for (int i = 0; i < 4 * DDsatnum[1]; i++) {
            for (int j = 0; j < 4 * DDsatnum[1]; j++) {
                int rowIndex = startRow + i; // 计算行索引
                int colIndex = startCol + j; // 计算列索引
                // 填充对角线元素
                if (i == j) {
                    if (i % 4 == 0 || i % 4 == 1) P[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[0];
                    else P[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[1];
                }
                // 填充非对角线元素
                else if (i % 4 == j % 4) {
                    if (i % 4 == 0 || i % 4 == 1) P[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[2];
                    else P[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[3];
                }
            }
        }

        // 矩阵重构，使其匹配本项目的矩阵运算函数
        restructureMatrix(4 * totalsatnum, 3 + totalsatnum * 2, 4 * MAXCHANNUM, 3 + MAXCHANNUM * 2, B);
        restructureMatrix(4 * totalsatnum, 4 * totalsatnum, 4 * MAXCHANNUM, 4 * MAXCHANNUM, P);

        // 若双差观测方程数量大于未知数，则可以求解
        if (totalsatnum < 5) return false; // 观测不足以解算
        else; // 观测足够，进入解算

        // 定义并初始化中间计算矩阵
        double BT[(3 + MAXCHANNUM * 2) * (4 * MAXCHANNUM)];
        MatrixTranspose((4 * totalsatnum), (3 + totalsatnum * 2), B, BT);
        double BTP[(3 + MAXCHANNUM * 2) * (4 * MAXCHANNUM)];
        memset(BTP, 0.0, sizeof(BTP));
        double N[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];
        memset(N, 0.0, sizeof(N));
        double W[(3 + MAXCHANNUM * 2) * 1];
        memset(W, 0.0, sizeof(W));

        // 矩阵运算
        MatrixMultiply((3 + totalsatnum * 2), (4 * totalsatnum), (4 * totalsatnum), (4 * totalsatnum), BT, P, BTP);
        MatrixMultiply((3 + totalsatnum * 2), (4 * totalsatnum), (4 * totalsatnum), (3 + totalsatnum * 2), BTP, B, N);       
        int iserror = MatrixInv((3 + totalsatnum * 2), N, N_inv);
        if (iserror == 0) return false; // 求逆遇到致命性错误，则退出函数    
        MatrixMultiply((3 + totalsatnum * 2), (4 * totalsatnum), (4 * totalsatnum), 1, BTP, w, W);
        MatrixMultiply((3 + totalsatnum * 2), (3 + totalsatnum * 2), (3 + totalsatnum * 2), 1, N_inv, W, x);

        // 更新流动站的位置和双差模糊度参数
        RovPos[0] = RovPos[0] + x[0];
        RovPos[1] = RovPos[1] + x[1];
        RovPos[2] = RovPos[2] + x[2];
        for (int i = 0; i < totalsatnum * 2; i++)  DDN[i] = DDN[i] + x[i + 3];
       
        // 调整迭代控制器
        if ((fabs(x[0]) < 1e-8 && fabs(x[1]) < 1e-8 && fabs(x[2]) < 1e-8) || count > 50) flag = false;
        else flag = true;

        count++;
 
    }while (flag);

    /* 存储结果 */
    // 记录所用双差卫星数到浮点解结果和双差结构体内
    Raw->DDObs.DDSatNum[0] = Fres->stanum[0] = DDsatnum[0];
    Raw->DDObs.DDSatNum[1] = Fres->stanum[1] = DDsatnum[1];
    Raw->DDObs.Sats = Fres->totalsatnum = totalsatnum;
    // 存储基线向量
    Fres->dX[0] = RovPos[0] - BasPos[0];
    Fres->dX[1] = RovPos[1] - BasPos[1];
    Fres->dX[2] = RovPos[2] - BasPos[2];
    // 存储基线向量的DOP
    Fres->DOP[0] = sqrt(N_inv[0]);
    Fres->DOP[1] = sqrt(N_inv[3 + 2 * totalsatnum + 1]);
    Fres->DOP[2] = sqrt(N_inv[2 * (3 + 2 * totalsatnum) + 2]);
    Fres->PDOP = sqrt(N_inv[0] + N_inv[3 + 2 * totalsatnum + 1] + N_inv[2 * (3 + 2 * totalsatnum) + 2]);
    // 存储双差模糊度
    memcpy(Fres->N, DDN, sizeof(double) * 2 * totalsatnum);
    // 存储双差模糊度的协因数阵
    GetQnn(totalsatnum, N_inv, Fres->Qn );
    
    // 计算验后单位权中误差
    // 定义并初始化中间计算矩阵
    double V[4 * MAXCHANNUM];
    memset(V, 0, sizeof(V));
    double Bx[4 * MAXCHANNUM];
    memset(Bx, 0, sizeof(Bx));
    double VT[4 * MAXCHANNUM];
    memset(VT, 0, sizeof(VT));   
    double VTP[4 * MAXCHANNUM];
    memset(VTP, 0, sizeof(VTP));
    double VTPV[1] = {0};
    // 矩阵运算
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
函数名：获取相对定位固定解
参数：Raw    指向历元总数据的指针（此时已包含固定后的整数模糊度）
      Rov    流动站的SPP结果（用于初始化坐标）
函数功能：在固定完模糊度的条件下进行只使用相位观测值的解算，得到更精确的定位，
          并存储在Raw中的DDObs里的dPos
返回值：固定解成功则返回true，失败则返回false
************************************************************************************
*/
bool RTKFix(RAWDAT* Raw, POSRES* Rov) 
{
    /* 定义待用变量 */
    double BasPos[3]; // 获取基站坐标(使用解码所得准确坐标)
    BLHToXYZ(Raw->BasPres.Pos, BasPos, R_WGS84, F_WGS84);
    double RovPos[3] = { Rov->Pos[0],Rov->Pos[1] ,Rov->Pos[2] }; // 设置流动站坐标初值(即SPP的结果)
    double RovToRefSat[2] = { 0, 0 }; // 流动站到其参考星的几何距离 0=GPS, 1=BDS
    double BasToSats[MAXCHANNUM]; // 基站坐标到所有卫星的几何距离（下标与基站原始观测数据的索引一致）
    double DDN[MAXCHANNUM * 2]; // 双差模糊度
    short DDsatnum[2] = { 0 ,0 }; // GPS和BDS两系统各可用双差卫星数  0=GPS, 1=BDS
    short totalsatnum = 0; // 总的可用双差卫星数(基准星不计入)
    double x[3]; // 待估参数的改正数
    double B[(2 * MAXCHANNUM) * 3]; // B阵
    double w[2 * MAXCHANNUM]; // w阵
    double P[(2 * MAXCHANNUM) * (2 * MAXCHANNUM)]; // P阵
    double N_inv[3 * 3]; // 待估参数的协因数阵，定义在外以便后续存储

    // 提取参考卫星位置在原始观测数据中的索引 0=GPS, 1=BDS
    short RefSatOfRov[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nRov,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nRov };
    short RefSatOfBas[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nBas,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nBas };
    // 参考卫星的位置（对于流动站和基站来说，由于时间不一定完全同步,故各自用各自星历所得的卫星位置）
    double RefSatPosOfGPSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[2] };
    double RefSatPosOfBDSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[2] };
    double RefSatPosOfGPSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[2] };
    double RefSatPosOfBDSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[2] };


    /* 变量初始化 */
    // 矩阵初始化置零
    memset(x, 0.0, sizeof(x));
    memset(B, 0.0, sizeof(B));
    memset(w, 0.0, sizeof(w));
    memset(P, 0.0, sizeof(P));
    memset(BasToSats, 0.0, sizeof(BasToSats));
    memset(DDN, 0.0, sizeof(DDN));

    // 计算基站坐标到所有卫星的几何距离
    for (int i = 0; i < Raw->BasEpk.SatNum; i++)
    {
        BasToSats[i] = sqrt((BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) * (BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) + (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) * (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) + (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]) * (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]));
    }

    // 提取已固定的双差模糊度，下标与所用观测方程的卫星数据顺序一致   
    memcpy(DDN, Raw->DDObs.FixedAmb, sizeof(double) * Raw->DDObs.Sats * 2);
   
    /* 最小二乘迭代 */
    bool flag = true; // 迭代控制器
    int count = 0; // 迭代次数 
    do
    {
        // 每次迭代时将相关矩阵先置零，以免数据填充混乱
        memset(x, 0, sizeof(x));
        memset(B, 0, sizeof(B));
        memset(w, 0, sizeof(w));
        memset(P, 0, sizeof(P));
        memset(N_inv, 0, sizeof(N_inv));

        // 每次迭代时可用卫星数归零，以免数据填充混乱
        DDsatnum[0] = DDsatnum[1] = 0;
        totalsatnum = 0;

        // 计算流动站到其参考星的几何距离 0=GPS, 1=BDS       
        RovToRefSat[0] = sqrt((RovPos[0] - RefSatPosOfGPSR[0]) * (RovPos[0] - RefSatPosOfGPSR[0]) + (RovPos[1] - RefSatPosOfGPSR[1]) * (RovPos[1] - RefSatPosOfGPSR[1]) + (RovPos[2] - RefSatPosOfGPSR[2]) * (RovPos[2] - RefSatPosOfGPSR[2]));
        RovToRefSat[1] = sqrt((RovPos[0] - RefSatPosOfBDSR[0]) * (RovPos[0] - RefSatPosOfBDSR[0]) + (RovPos[1] - RefSatPosOfBDSR[1]) * (RovPos[1] - RefSatPosOfBDSR[1]) + (RovPos[2] - RefSatPosOfBDSR[2]) * (RovPos[2] - RefSatPosOfBDSR[2]));
        
        // 遍历单差观测值来填充B、w矩阵，同时统计各系统的可用双差卫星数
        for (int i = 0; i < Raw->SdObs.SatNum; i++)
        {
            if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // 参考星不计入可用双差卫星数
            else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // 存在粗差和半周
            else; // 本颗卫星可用

            // 当前卫星的位置（在流动站原始观测数据中的）索引
            short satindex = Raw->SdObs.SdSatObs[i].nRov;
            // 计算流动站到本颗卫星的几何距离
            double RovToSat = sqrt((RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) * (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) + (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) * (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) + (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) * (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]));
            // 确定本颗卫星所属系统
            int sys = -1;
            if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0; // GPS
            else sys = 1; // BDS

            // 计算B阵的中间量
            double l = (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) / RovToSat - (RovPos[0] - ((sys == 0) ? RefSatPosOfGPSR[0] : RefSatPosOfBDSR[0])) / RovToRefSat[sys];
            double m = (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) / RovToSat - (RovPos[1] - ((sys == 0) ? RefSatPosOfGPSR[1] : RefSatPosOfBDSR[1])) / RovToRefSat[sys];
            double n = (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) / RovToSat - (RovPos[2] - ((sys == 0) ? RefSatPosOfGPSR[2] : RefSatPosOfBDSR[2])) / RovToRefSat[sys];
            // 填充B阵的两行数据
            for (int j = 0; j < 2; j++)
            {
                B[3 * (2 * totalsatnum + j)] = l;
                B[3 * (2 * totalsatnum + j) + 1] = m;
                B[3 * (2 * totalsatnum + j) + 2] = n;
            }

            // 计算双差观测值
            double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            double ddrou = RovToSat - RovToRefSat[sys] + BasToSats[RefSatOfBas[sys]] - BasToSats[Raw->SdObs.SdSatObs[i].nBas];
            // 填充w阵的两行数据
            w[2 * totalsatnum] = (sys == 0) ? (ddl[0] - ddrou - DDN[totalsatnum * 2] * WL1_GPS) : (ddl[0] - ddrou - DDN[totalsatnum * 2] * WL1_BDS);
            w[2 * totalsatnum + 1] = (sys == 0) ? (ddl[1] - ddrou - DDN[totalsatnum * 2 + 1] * WL2_GPS) : (ddl[1] - ddrou - DDN[totalsatnum * 2 + 1] * WL3_BDS);

            // 对应系统可用卫星数增加
            DDsatnum[sys]++;
            totalsatnum = DDsatnum[0] + DDsatnum[1];
        }
     
        // 填充P阵,GP和BP是待填充的数值   
        double GP[2] = { DDsatnum[0] / (DDsatnum[0] + 1.0),-1.0/ (DDsatnum[0] + 1.0)};
        double BP[2] = { DDsatnum[1] / (DDsatnum[1] + 1.0),-1.0/ (DDsatnum[1] + 1.0)};
        // 填充GPS部分
        for (int i = 0; i < 2 * DDsatnum[0]; i++)
        {
            for (int j = 0; j < 2 * DDsatnum[0]; j++)
            {
                // 填充对角线元素
                if (j == i)
                {
                    P[i * (2 * MAXCHANNUM) + j] = GP[0];
                }
                // 填充非对角线元素
                else if (j % 2 == i % 2)
                {
                    P[i * (2 * MAXCHANNUM) + j] = GP[1];
                }
            }
        }
        // 填充BDS部分
        int startRow = 2 * DDsatnum[0]; // BDS部分开始的行
        int startCol = 2 * DDsatnum[0]; // BDS部分开始的列
        for (int i = 0; i < 2 * DDsatnum[1]; i++)
        {
            for (int j = 0; j < 2 * DDsatnum[1]; j++) 
            {
                int rowIndex = startRow + i; // 计算行索引
                int colIndex = startCol + j; // 计算列索引
                // 填充对角线元素
                if (i == j) 
                {
                    P[rowIndex * (2 * MAXCHANNUM) + colIndex] = BP[0];
                }
                // 填充非对角线元素
                else if (i % 2 == j % 2) 
                {
                    P[rowIndex * (2 * MAXCHANNUM) + colIndex] = BP[1];
                }
            }
        }

        // 矩阵重构，使其匹配本项目的矩阵运算函数
        restructureMatrix(2 * totalsatnum, 2 * totalsatnum, 2 * MAXCHANNUM, 2 * MAXCHANNUM, P);

        // 由于浮点解可行，则固定时无需考虑卫星个数是否足够的问题
        // 定义并初始化中间计算矩阵
        double BT[3 * (2 * MAXCHANNUM)];
        MatrixTranspose((2 * totalsatnum),3, B, BT);
        double BTP[3 * (2 * MAXCHANNUM)];
        memset(BTP, 0.0, sizeof(BTP));
        double N[3 * 3];
        memset(N, 0.0, sizeof(N));   
        double W[3 * 1];
        memset(W, 0.0, sizeof(W));

        // 矩阵运算
        MatrixMultiply(3, (2 * totalsatnum), (2 * totalsatnum), (2 * totalsatnum), BT, P, BTP);
        MatrixMultiply(3, (2 * totalsatnum), (2 * totalsatnum), 3, BTP, B, N);
        int iserror = MatrixInv(3, N, N_inv);
        if (iserror == 0) return false; // 求逆遇到致命性错误，则退出函数
        MatrixMultiply(3, (2 * totalsatnum), (2 * totalsatnum), 1, BTP, w, W);
        MatrixMultiply(3, 3, 3, 1, N_inv, W, x);

        // 更新流动站的位置
        RovPos[0] = RovPos[0] + x[0];
        RovPos[1] = RovPos[1] + x[1];
        RovPos[2] = RovPos[2] + x[2];

        Raw->DDObs.PDOP = sqrt(N_inv[0] + N_inv[4] + N_inv[8]);

        // 调整迭代控制器
        if (fabs(x[0]) < 1e-8 && fabs(x[1]) < 1e-8 && fabs(x[2]) < 1e-8 || count > 50) flag = false;
        else flag = true;

        count++;

    } while (flag);

    /* 存储结果 */
    // 存储固定解的基线向量
    Raw->DDObs.dPos[0] = RovPos[0] - BasPos[0];
    Raw->DDObs.dPos[1] = RovPos[1] - BasPos[1];
    Raw->DDObs.dPos[2] = RovPos[2] - BasPos[2];
    
    // 计算残差平方和
    // 定义并初始化中间计算矩阵
    double V[2 * MAXCHANNUM];
    memset(V, 0, sizeof(V));
    double Bx[2 * MAXCHANNUM];
    memset(Bx, 0, sizeof(Bx));
    double VT[2 * MAXCHANNUM];
    memset(VT, 0, sizeof(VT));
    double VTP[2 * MAXCHANNUM];
    memset(VTP, 0, sizeof(VTP));
    double VTPV[1] = { 0 };
    // 矩阵运算
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
函数名：滤波初始化函数
参数：Raw    指向历元总数据的指针
      Rov    流动站的SPP结果
      ekf    滤波数据
函数功能：初始化滤波的状态量X和其对应的方差协方差矩阵P
***********************************************************
*/
void InitFilter(RAWDAT* Raw, POSRES* Rov, RTKEKF* ekf)
{
    // 状态清零
    memset(&ekf->X0, 0, sizeof(ekf->X0));
    memset(&ekf->P0, 0, sizeof(ekf->P0));
    memset(&ekf->X, 0, sizeof(ekf->X));
    memset(&ekf->P, 0, sizeof(ekf->P));

    // 初始化时间标记
    memcpy(&ekf->Time, &Raw->RovEpk.Time, sizeof(ekf->Time));
    // 初始化状态的位置元素(使用SPP结果填充X的前三个元素)
    ekf->X0[0] = Rov->Pos[0];
    ekf->X0[1] = Rov->Pos[1];
    ekf->X0[2] = Rov->Pos[2]; 
    // 初始化状态的双差模糊度元素（遍历单差数据，按顺序计算并填充）
    int ii = 3; // 存储索引      状态X的元素个数为ii --> 为填充P阵提供依据
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        // 参考星不计入可用双差卫星数
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1])
        {
            ekf->Index[i] = -1;
            continue;
        }
        else;
        // 判断是否存在粗差和半周
        if (Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)
        {
            ekf->Index[i] = ii; // 本颗卫星对应站间单差数据产生的双频双差模糊度存放在状态X中的索引为ii、ii+1

            // 确定该颗卫星所属系统
            int sys = -1;
            if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0;
            else sys = 1;
            // 计算双差观测值并进行模糊度初始化
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

    // 初始化状态的方差协方差阵
    for (int j = 0; j < ii; j++)
    {
        if (j < 3) ekf->P0[j + j * ii] = 100;
        else ekf->P0[j + j * ii] = 500;
    }
 
    // 滤波初始化完成，更新标记
    ekf->IsInit = true;
}


/*
******************************************************************
函数名：预测函数
参数：Raw    指向历元总数据的指针
      ekf    滤波数据
函数功能：对ekf中的状态及其误差协方差阵进行预测
返回值：预测成功则返回true，否则为false
注意：ekf在进入时应已有上一历元、本历元DDOBS内的参考星信息
      以及上一历元的SDOBS单差数据信息
******************************************************************
*/
bool EkfPredict(RAWDAT* Raw, RTKEKF* ekf)
{
    // 时间传递
    memcpy(&ekf->Time, &Raw->RovEpk.Time, sizeof(ekf->Time));

    // 统计当前历元的可用双差卫星数
    ekf->CurDDObs.Sats = 0;
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        if (i == ekf->CurDDObs.RefPos[0] || i == ekf->CurDDObs.RefPos[1]) continue; // 参考星，跳过
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // 有周跳或半周标记，跳过
        else ekf->CurDDObs.Sats++; // 可以提供当前历元状态中的双差模糊度
    }
    // 统计上一历元的可用双差卫星数
    ekf->DDObs.Sats = 0;
    for (int i = 0; i < ekf->SDObs.SatNum; i++)
    {
        if (i == ekf->DDObs.RefPos[0] || i == ekf->DDObs.RefPos[1]) continue; // 参考星，跳过
        else if (!(ekf->SDObs.SdSatObs[i].Valid == 1 && ekf->SDObs.SdSatObs[i].half[0] == 1 && ekf->SDObs.SdSatObs[i].half[1] == 1)) continue; // 有周跳或半周标记，跳过
        else ekf->DDObs.Sats++; // 可以提供当前历元状态中的双差模糊度
    }

    // 定义状态转移矩阵，预测状态的误差协方差阵并初始置零
    double fai[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)], Q[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)];
    memset(fai, 0.0, sizeof(fai));
    memset(Q, 0.0, sizeof(Q));
    // 填充位置元素对应的fai和Q
    for (int i = 0; i < 3; i++)
    {
        fai[i + i * (3 + 2 * ekf->DDObs.Sats)] = 1; // 位置即上一历元的位置
        Q[i + i * (3 + 2 * ekf->CurDDObs.Sats)] = 1e-4 * 1e-4; // 静态，故预测误差很小
    }

    // 记录双系统的参考星有无发生改变 0-GPS，1-BDS
    bool ischange[2] = { false, false }; 
    for (int i = 0; i < 2; i++)
    {
        if (ekf->CurDDObs.RefPrn[i] != ekf->DDObs.RefPrn[i])  ischange[i] = true; // 记录变化情况
        else continue;
    }

    // 记录本历元的参考星在上一历元中是否可用
    bool isvalid[2] = { false ,false }; 
    int refindex[2] = { -1,-1 }; // 本历元参考星在上一历元单差数据中的对应存储位置
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

    int xnum = 3; // 本历元的状态X的对应fai和Q的存放行索引 （列索引由Index提供）
    int initIndex[MAXCHANNUM]; // 需要初始化的卫星对应双差模糊度在当前历元Sdobs中的索引
    int ii = 0; // initIndex存放所用
    memset(initIndex, -1, sizeof(initIndex));

    // 遍历可以反映当前历元状态X里双差模糊度存放顺序的SdObs
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // 参考星，跳过
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // 有周跳或半周标记，跳过
        else; // 可以提供当前历元状态中的双差模糊度

        bool isfind = false; // 记录当前历元的该颗卫星对应的双差模糊度在上一历元中能否找到

        for (int j = 0; j < ekf->SDObs.SatNum; j++) // 遍历可以反映上一历元状态X里双差模糊度存放顺序的SdObs
        {
            if (ekf->SDObs.SdSatObs[j].System == Raw->SdObs.SdSatObs[i].System && ekf->SDObs.SdSatObs[j].Prn == Raw->SdObs.SdSatObs[i].Prn)
            {
                isfind = true;
                if (ekf->SDObs.SdSatObs[j].Valid == 1 && ekf->SDObs.SdSatObs[j].half[0] == 1 && ekf->SDObs.SdSatObs[j].half[1] == 1)
                {
                    // 卫星系统标志
                    short sys = -1;
                    if (ekf->SDObs.SdSatObs[j].System == GPS) sys = 0;
                    else sys = 1;

                    // 情况1：参考星未改变
                    if (ischange[sys] == false)
                    {
                        // 上一历元无周跳、半周标记,则可以直接继承
                        fai[xnum * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[j]] = 1;
                        fai[(xnum + 1) * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[j] + 1] = 1;
                        Q[xnum + xnum * (3 + 2 * ekf->CurDDObs.Sats)] = 1e-5 * 1e-5;
                        Q[(xnum + 1) + (xnum + 1) * (3 + 2 * ekf->CurDDObs.Sats)] = 1e-5 * 1e-5;
                    }
                    // 情况2：参考星有改变
                    else 
                    {
                        // 若本历元的参考星在上一历元中可用，则可以间接继承
                        if (isvalid[sys] == true)
                        {
                            if (ekf->Index[j] != -1) // -1要么是问题卫星要么是参考星，此处已经排除过问题卫星故一定代表参考星
                            {
                                // 若本历元当前卫星是上一历元的参考星，则注意容错
                                fai[xnum * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[j]] = 1;
                                fai[(xnum + 1) * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[j] + 1] = 1;
                            }
                            else;

                            fai[xnum * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[refindex[sys]]] = -1;                           
                            fai[(xnum + 1) * (3 + 2 * ekf->DDObs.Sats) + ekf->Index[refindex[sys]]+1] = -1;
                            Q[xnum + xnum * (3 + 2 * ekf->CurDDObs.Sats)] = 1e-5 * 1e-5;
                            Q[(xnum + 1) + (xnum + 1) * (3 + 2 * ekf->CurDDObs.Sats)] = 1e-5 * 1e-5;
                        }
                        // 若本历元的参考星在上一历元中不可用，则不可继承，需要初始化
                        else
                        {
                            // fai元素默认为0，不使用上一历元的双差模糊度传递
                            // Q元素需要按初始化标准设置
                            Q[xnum + xnum * (3 + 2 * ekf->CurDDObs.Sats)] = 100;
                            Q[(xnum + 1) + (xnum + 1) * (3 + 2 * ekf->CurDDObs.Sats)] = 500;
                            initIndex[ii] = i; // 记录，稍后进行初始化
                            ii++;
                        }

                    }                  
                }
                else // 上一历元的该颗卫星有问题，则无法继承，需要初始化
                {
                    // fai元素默认为0，不使用上一历元的双差模糊度传递
                    // Q元素需要按初始化标准设置
                    Q[xnum + xnum * (3 + 2 * ekf->CurDDObs.Sats)] = 100;
                    Q[(xnum + 1) + (xnum + 1) * (3 + 2 * ekf->CurDDObs.Sats)] = 500;
                    initIndex[ii] = i; // 记录，稍后进行初始化
                    ii++;
                }

                break;
            }
            else continue;
        }

        if (isfind == false) 
        {
            // fal元素默认为0，上一历元的双差模糊度对该颗卫星对应的双差模糊度没有传递意义
            // Q元素需要按初始化标准设置
            Q[xnum + xnum * (3 + 2 * ekf->CurDDObs.Sats)] = 100;
            Q[(xnum + 1) + (xnum + 1) * (3 + 2 * ekf->CurDDObs.Sats)] = 500;
            initIndex[ii] = i; // 记录，稍后进行初始化
            ii++;
        }

        xnum = xnum + 2; // 每颗可用的双差卫星带来两个双差模糊度
    }
   

    /*
    // 调整Q的大小
    for (int i = 0; i < (3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM); i++)
    {
        Q[i] = Q[i] * 0.001;
    }
    */
    // fai Q 传递计算部分
    double faiT[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)];
    memset(faiT, 0, sizeof(faiT));
    double faiP[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)];
    memset(faiP, 0, sizeof(faiP));
    double faiPfaiT[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)];
    memset(faiPfaiT, 0, sizeof(faiPfaiT));
    // 矩阵运算
    MatrixMultiply(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 1, fai, ekf->X0, ekf->X);
    MatrixTranspose(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->DDObs.Sats, fai, faiT);
    MatrixMultiply(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->DDObs.Sats, fai, ekf->P0, faiP);
    MatrixMultiply(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->CurDDObs.Sats, faiP, faiT, faiPfaiT);
    MatrixAddition(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->CurDDObs.Sats, faiPfaiT, Q, ekf->P);


    // 本历元部分双差模糊度补漏部分，需要初始化
    // initIndex里面存放为当前历元单差中需要进行双差模糊度初始化的索引
    xnum = 3;
    ii = 0;
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // 参考星，跳过
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // 有周跳或半周标记，跳过
        else; // 可以提供当前历元状态中的双差模糊度

        // 判断是否为需要初始化的双差模糊度对应的卫星
        if (i == initIndex[ii])
        {
            // 确定该颗卫星所属系统
            int sys = -1;
            if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0;
            else sys = 1;
            // 计算双差观测值并对模糊度初始化
            double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            ekf->X[xnum] = (sys == 0) ? ((ddl[0] - ddp[0]) / WL1_GPS) : ((ddl[0] - ddp[0]) / WL1_BDS);
            ekf->X[xnum + 1] = (sys == 0) ? ((ddl[1] - ddp[1]) / WL2_GPS) : ((ddl[1] - ddp[1]) / WL3_BDS);
            ii++;
        }
        else;
        
        xnum = xnum + 2; // 调整状态中的存储索引
    }

    // 更新ekf的index存放
    ii = 3; // 存储索引      
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        // 参考星不计入可用双差卫星数
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1])
        {
            ekf->Index[i] = -1;
            continue;
        }
        
        // 判断是否存在粗差和半周
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1))
        {
            ekf->Index[i] = -1;
            continue;
                     
        }
        else
        {
            ekf->Index[i] = ii; // 本颗卫星对应站间单差数据产生的双频双差模糊度存放在状态X中的索引为ii、ii+1
            ii = ii + 2;
        }
    }

    return true;
}


/*
************************************************************************************
函数名：测量更新函数
参数：Raw    指向历元总数据的指针（此时已包含固定后的整数模糊度）
      ekf    滤波数据
函数功能：对一步预测完成后的滤波数据进行测量更新，分为两部分，首先使用双差
          观测值进行一次测量更新，之后再以固定模糊度为观测再次更新
返回值：固定解成功则返回true，失败则返回false
************************************************************************************
*/
bool EkfMeasureUpdate(RAWDAT* Raw, RTKEKF* ekf)
{
    /*** 双差观测值测量更新 ***/
    // 定义H、V和R阵并初始化
    double H[(4 * MAXCHANNUM) * (3 + MAXCHANNUM * 2)], V[4 * MAXCHANNUM], R[(4 * MAXCHANNUM) * (4 * MAXCHANNUM)];
    memset(H, 0.0, sizeof(H));
    memset(V, 0.0, sizeof(V));
    memset(R, 0.0, sizeof(R));

    // 提取参考卫星位置在原始观测数据中的索引 0=GPS, 1=BDS
    short RefSatOfRov[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nRov,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nRov };
    short RefSatOfBas[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nBas,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nBas };
    // 参考卫星的位置（对于流动站和基站来说，由于时间不一定完全同步,故各自用各自星历所得的卫星位置）
    double RefSatPosOfGPSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[2] };
    double RefSatPosOfBDSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[2] };
    double RefSatPosOfGPSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[0]].SatPos[2] };
    double RefSatPosOfBDSB[3] = { Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[0],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[1],Raw->BasEpk.SatPVT[RefSatOfBas[1]].SatPos[2] };

    // 获取基站坐标(使用解码所得准确坐标)
    double BasPos[3];
    BLHToXYZ(Raw->BasPres.Pos, BasPos, R_WGS84, F_WGS84);
    // 计算基站坐标到所有卫星的几何距离
    double BasToSats[MAXCHANNUM]; // 下标与基站原始观测数据的索引一致
    memset(BasToSats, 0.0, sizeof(BasToSats));
    for (int i = 0; i < Raw->BasEpk.SatNum; i++)
    {
        BasToSats[i] = sqrt((BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) * (BasPos[0] - Raw->BasEpk.SatPVT[i].SatPos[0]) + (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) * (BasPos[1] - Raw->BasEpk.SatPVT[i].SatPos[1]) + (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]) * (BasPos[2] - Raw->BasEpk.SatPVT[i].SatPos[2]));
    }

    // 计算流动站到其参考星的几何距离 0=GPS, 1=BDS    
    double RovToRefSat[2];
    double RovPos[3] = { ekf->X[0],ekf->X[1],ekf->X[2] };
    RovToRefSat[0] = sqrt((RovPos[0] - RefSatPosOfGPSR[0]) * (RovPos[0] - RefSatPosOfGPSR[0]) + (RovPos[1] - RefSatPosOfGPSR[1]) * (RovPos[1] - RefSatPosOfGPSR[1]) + (RovPos[2] - RefSatPosOfGPSR[2]) * (RovPos[2] - RefSatPosOfGPSR[2]));
    RovToRefSat[1] = sqrt((RovPos[0] - RefSatPosOfBDSR[0]) * (RovPos[0] - RefSatPosOfBDSR[0]) + (RovPos[1] - RefSatPosOfBDSR[1]) * (RovPos[1] - RefSatPosOfBDSR[1]) + (RovPos[2] - RefSatPosOfBDSR[2]) * (RovPos[2] - RefSatPosOfBDSR[2]));

    // GPS和BDS两系统各可用双差卫星数  0=GPS, 1=BDS
    int DDsatnum[2] = { 0,0 };
    int totalsatnum = 0; // 总的可用双差卫星数(基准星不计入)

    // 遍历单差观测值来填充H、V矩阵，同时统计各系统的可用双差卫星数
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {       
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue; // 参考星，跳过
        else if (!(Raw->SdObs.SdSatObs[i].Valid == 1 && Raw->SdObs.SdSatObs[i].half[0] == 1 && Raw->SdObs.SdSatObs[i].half[1] == 1)) continue; // 卫星数据有问题，跳过
        else;

        // 当前卫星的位置（在流动站原始观测数据中的）索引
        int satindex = Raw->SdObs.SdSatObs[i].nRov;
        // 计算流动站到本颗卫星的几何距离
        double RovToSat = sqrt((RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) * (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) + (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) * (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) + (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) * (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]));
     
        
        // 确定本颗卫星所属系统
        int sys = -1;
        if (Raw->SdObs.SdSatObs[i].System == GPS) sys = 0;
        else sys = 1;

        // 计算H阵的中间量
        double l = (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) / RovToSat - (RovPos[0] - ((sys == 0) ? RefSatPosOfGPSR[0] : RefSatPosOfBDSR[0])) / RovToRefSat[sys];
        double m = (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) / RovToSat - (RovPos[1] - ((sys == 0) ? RefSatPosOfGPSR[1] : RefSatPosOfBDSR[1])) / RovToRefSat[sys];
        double n = (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) / RovToSat - (RovPos[2] - ((sys == 0) ? RefSatPosOfGPSR[2] : RefSatPosOfBDSR[2])) / RovToRefSat[sys];
        // 填充H阵的四行数据
        for (int j = 0; j < 4; j++)
        {
            H[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j)] = l;
            H[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 1] = m;
            H[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 2] = n;
            if (j == 2) H[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 3 + 2 * totalsatnum] = (sys == 0) ? WL1_GPS : WL1_BDS;
            else if (j == 3) H[(3 + MAXCHANNUM * 2) * (4 * totalsatnum + j) + 4 + 2 * totalsatnum] = (sys == 0) ? WL2_GPS : WL3_BDS;
            else continue;
        }

        // 计算双差观测值
        double ddp[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
        double ddl[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
        double ddrou = RovToSat - RovToRefSat[sys] + BasToSats[RefSatOfBas[sys]] - BasToSats[Raw->SdObs.SdSatObs[i].nBas];
        // 填充V的四行数据
        V[4 * totalsatnum] = ddp[0] - ddrou;
        V[4 * totalsatnum + 1] = ddp[1] - ddrou;
        V[4 * totalsatnum + 2] = (sys==0)?(ddl[0] - ddrou - ekf->X[3 + totalsatnum * 2] * WL1_GPS):(ddl[0] - ddrou - ekf->X[3 + totalsatnum * 2] * WL1_BDS);
        V[4 * totalsatnum + 3] = (sys==0)?(ddl[1] - ddrou - ekf->X[3 + totalsatnum * 2 + 1] * WL2_GPS):(ddl[1] - ddrou - ekf->X[3 + totalsatnum * 2 + 1] * WL3_BDS);
        
        // 更新可用卫星计数
        DDsatnum[sys]++;
        totalsatnum = DDsatnum[0] + DDsatnum[1];
      
    }
    ekf->nSats = totalsatnum;
    
    // 填充R阵,GP和BP是待填充的数值
    double GP[4] = { 4 * cfginfo.CodeNoise * cfginfo.CodeNoise , 4 * cfginfo.CPNoise * cfginfo.CPNoise,  2 * cfginfo.CodeNoise * cfginfo.CodeNoise,2 * cfginfo.CPNoise * cfginfo.CPNoise };
    double BP[4] = { 4 * cfginfo.CodeNoise * cfginfo.CodeNoise , 4 * cfginfo.CPNoise * cfginfo.CPNoise,  2 * cfginfo.CodeNoise * cfginfo.CodeNoise,2 * cfginfo.CPNoise * cfginfo.CPNoise };
   
    // 填充GPS部分
    for (int i = 0; i < 4 * DDsatnum[0]; i++)
    {
        for (int j = 0; j < 4 * DDsatnum[0]; j++)
        {
            // 填充对角线元素
            if (j == i)
            {
                if (i % 4 == 0 || i % 4 == 1) R[i * (4 * MAXCHANNUM) + j] = GP[0];
                else R[i * (4 * MAXCHANNUM) + j] = GP[1];
            }
            // 填充非对角线元素
            else if (j % 4 == i % 4)
            {
                if (i % 4 == 0 || i % 4 == 1) R[i * (4 * MAXCHANNUM) + j] = GP[2];
                else R[i * (4 * MAXCHANNUM) + j] = GP[3];
            }
        }
    }
    // 填充BDS部分
    int startRow = 4 * DDsatnum[0]; // BDS部分开始的行
    int startCol = 4 * DDsatnum[0]; // BDS部分开始的列
    for (int i = 0; i < 4 * DDsatnum[1]; i++) 
    {
        for (int j = 0; j < 4 * DDsatnum[1]; j++) 
        {
            int rowIndex = startRow + i; // 计算行索引
            int colIndex = startCol + j; // 计算列索引
            // 填充对角线元素
            if (i == j) {
                if (i % 4 == 0 || i % 4 == 1) R[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[0];
                else R[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[1];
            }
            // 填充非对角线元素
            else if (i % 4 == j % 4) {
                if (i % 4 == 0 || i % 4 == 1) R[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[2];
                else R[rowIndex * (4 * MAXCHANNUM) + colIndex] = BP[3];
            }
        }
    }


    // 矩阵重构，使其匹配本项目的矩阵运算函数
    restructureMatrix(4 * totalsatnum, 3 + totalsatnum * 2, 4 * MAXCHANNUM, 3 + MAXCHANNUM * 2, H);
    restructureMatrix(4 * totalsatnum, 4 * totalsatnum, 4 * MAXCHANNUM, 4 * MAXCHANNUM, R);

    // 进行测量更新
    int calflag1 = CalcuUpdate(3 + totalsatnum * 2, 4 * totalsatnum, H, V, R, ekf->X, ekf->P);
    if (calflag1 == 0)  return false;


    /*** 固定后的整周模糊度作为观测进行测量更新 ***/
    // 提取双差模糊度对应的P
    double Pn[(MAXCHANNUM * 2) * (MAXCHANNUM * 2)]; // 模糊度的P阵
    GetQnn(totalsatnum, ekf->P, Pn);
    
    // 进行LAMBDA固定双差模糊度
    double FixedAmb[MAXCHANNUM * 4];
    double ResAmb[2];
    if(lambda(totalsatnum * 2, 2, ekf->X+3, Pn, FixedAmb, ResAmb)!=0) return false;
    ekf->ratio = ResAmb[1] / ResAmb[0];

    ekf->beFixed = false;
    // 若固定，则进行固定模糊度观测更新
    if (ekf->ratio > cfginfo.RatioThres)
    {
        // 定义H1、V1和R1阵并初始化置零
        double  H1[(2 * MAXCHANNUM) * (3 + MAXCHANNUM * 2)], V1[2 * MAXCHANNUM], R1[(2 * MAXCHANNUM) * (2 * MAXCHANNUM)];
        memset(H1, 0.0, sizeof(H1));
        memset(V1, 0.0, sizeof(V1));
        memset(R1, 0.0, sizeof(R1));

        // 填充H1、V1和R1
        for (int i = 0; i < ekf->CurDDObs.Sats * 2; i++)
        {
            V1[i] = FixedAmb[i] - ekf->X[3 + i];
            R1[i + i * ekf->CurDDObs.Sats * 2] = 1e-8;
            H1[3 + i + i * (3 + ekf->CurDDObs.Sats * 2)] = 1;
        }

        // 再一次进行测量更新
        int calflag2 = CalcuUpdate(3 + totalsatnum * 2, 2 * totalsatnum, H1, V1, R1, ekf->X, ekf->P);
        if (calflag2 == 0) return false;

        ekf->beFixed = true;
    }
    //else ekf->IsInit = false; // 否则，重新初始化

    ekf->sigmaP = sqrt(ekf->P[0] + ekf->P[3 + totalsatnum * 2 + 1] + ekf->P[2 * (3 + totalsatnum * 2) + 2]);
    
    // 拷贝状态及其误差协方差阵
    memcpy(&ekf->X0, &ekf->X, sizeof(ekf->X));
    memcpy(&ekf->P0, &ekf->P, sizeof(ekf->P));

    // 状态清零
    memset(&ekf->X, 0, sizeof(ekf->X));
    memset(&ekf->P, 0, sizeof(ekf->P));

    return true;
}



/*
*****************************************************************
函数名：打印矩阵函数
参数：rows     矩阵的行数
      cols     矩阵的列数
      matrix   存放矩阵数据的一维数组（前面的数据为有效数据）
      filename 存储文件名
函数功能：打印矩阵到txt文件以便检查
*****************************************************************
*/
void MatrixDisplay(int rows, int cols, double matrix[], const char* filename) 
{
    FILE* file = NULL;
    errno_t err = fopen_s(&file, filename, "w");
    if (err != 0 || file == NULL) {
        // 文件打开失败
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
函数名：获取QNN矩阵函数
参数：satnum   可用双差卫星数（不包括基准星）
      N_inv    包含三维坐标在内的参数对应的协因数阵
      Q        模糊度参数对应的协因数阵
函数功能：从N_inv取出QNN的部分用于后续lambda方法
***********************************************************
*/
void GetQnn(const int satnum,const double N_inv[], double Q[]) 
{
    int N_inv_size = (3 + 2 * satnum) * (3 + 2 * satnum); // N_inv中的有效数据数量
    int skip_count = 3 * (3 + 2 * satnum); // 计算需要跳过的元素数量
    int index_N_inv = skip_count; // 设置N_inv的当前索引
    int index_Q = 0; // 设置Q的当前索引

    while (index_N_inv + 2 * satnum <= N_inv_size) 
    {
        // 跳过3个元素
        index_N_inv += 3;

        // 复制2*satnum个元素到Q数组
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
函数名：卡尔曼滤波中的测量更新的公式计算函数
参数：n    X/P的实际行数 对应(3 + MAXCHANNUM * 2)
      m    H/V/R的实际行数 对应(4 * MAXCHANNUM)
      H    观测矩阵m*n
      V    新息阵m*1
      R    观测噪声阵m*m
      X    状态量阵n*1
      P    状态量的方差协方差阵n*n
函数功能：进行测量更新计算，得到更新后的X和P阵
注意：X、P传入时为预测的结果，函数执行结束后为更新后的结果
返回值：计算成功则返回1，计算失败则返回0
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

    // 初始化动态数组为0
    memset(HT, 0, n * m * sizeof(double)); // 初始化 HT
    memset(PHT, 0, n * m * sizeof(double)); // 初始化 PHT
    memset(HPHT, 0, m * m * sizeof(double)); // 初始化 HPHT
    memset(HPHTaddR, 0, m * m * sizeof(double)); // 初始化 HPHTaddR
    memset(HPHTaddR_inverse, 0, m * m * sizeof(double)); // 初始化 HPHTaddR_inverse
    memset(K, 0, n * m * sizeof(double)); // 初始化 K
    memset(KV, 0, n * sizeof(double)); // 初始化 KV
    memset(I, 0, n * n * sizeof(double)); // 初始化 I
    memset(KH, 0, n * n * sizeof(double)); // 初始化 KH
    memset(IsubKH, 0, n * n * sizeof(double)); // 初始化 IsubKH


    memcpy(P1, P, (3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2) * sizeof(double));
    memcpy(X1, X, (3 + MAXCHANNUM * 2) * sizeof(double));
    for (int i = 0; i < n; i++) I[i + i * n] = 1;

    // 矩阵运算
    MatrixTranspose(m, n, H, HT);
    MatrixMultiply(n, n, n, m, P, HT, PHT);
    MatrixMultiply(m, n, n, m, H, PHT, HPHT);
    MatrixAddition(m, m, HPHT, R, HPHTaddR);
    int iserror = MatrixInv(m, HPHTaddR, HPHTaddR_inverse);
    if (iserror == 0)
    {
        // 释放动态分配的内存
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

    // 释放动态分配的内存
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