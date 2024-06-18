#include"MyHeadFile.h"

// 配置信息变量
ROVERCFGINFO cfginfo;

int main()
{
    // 读取配置信息
    if (!ReadSATODSConfigInfo("Config.txt", &cfginfo))
    {
        printf("读取配置文件失败！\n");
        return 0;
    }
    else printf("读取配置文件成功！\n");

    RAWDAT RawData; // 数据包
    FloatResult Fres; // 浮点解结果
    RTKEKF ekf; // 滤波数据

    // 打开基站和流动站两个数据文件
    FILE* fbas;
    if (fopen_s(&fbas, cfginfo.BasObsDatFile, "rb") != 0)
    {
        printf("无法打开文件！\n");
        return 0;
    }
    FILE* frov;
    if (fopen_s(&frov, cfginfo.RovObsDatFile, "rb") != 0)
    {
        printf("无法打开文件！\n");
        return 0;
    }

    // 打开基站和流动站两个网口
    SOCKET BasNet;
    if (OpenSocket(BasNet, cfginfo.BasNetIP, cfginfo.BasNetPort) == false)
    {
        printf("This ip & port was not opened.\n");
        return 0;
    }
    SOCKET RovNet; 
    if (OpenSocket(RovNet, cfginfo.RovNetIP, cfginfo.RovNetPort) == false)
    {
        printf("This ip & port was not opened.\n");
        return 0;
    }

    POSRES Bres; // 基站的SPP结果
    double Benu[3] = { 0,0,0 }; // 基站的SPP定位误差
    POSRES Rres; // 流动站的SPP结果
    double Renu[3] = { 0,0,0 }; // 流动站的SPP定位误差

    FILE* FileResult = fopen(cfginfo.ResFile, "w"); // 结果存储文件

    while (1)
    {
        // 进行时间同步
        int synflag;
        if (cfginfo.IsFileData == 1) synflag = GetSynObsPP(fbas, frov, &RawData); // 事后时间同步
        else if (cfginfo.IsFileData == 0) synflag = GetSynObsRT(BasNet, RovNet, &RawData); // 实时时间同步
        else printf("处理模式错误！\n");

        if (synflag == -1) break; // 文件结束，跳出循环
        else if (synflag == 0) continue; // 同步失败，继续循环
        else; // 同步成功，进入解算环节

        printf("\nRov-Time:%f \n", RawData.RovEpk.Time.SecOfWeek);
        printf("Bas-Time:%f \n", RawData.BasEpk.Time.SecOfWeek);

        // 基站SPP以获取卫星位置等中间量        
        DetectOutlier(&RawData.BasEpk);
        int f1 = SPP(&RawData.BasEpk, RawData.GpsEph, RawData.BdsEph, &Bres);
        if (f1) SPV(&RawData.BasEpk, &Bres);

        // 流动站SPP以获取卫星位置等中间量        
        DetectOutlier(&RawData.RovEpk);
        int f2 = SPP(&RawData.RovEpk, RawData.GpsEph, RawData.BdsEph, &Rres);
        if (f2) SPV(&RawData.RovEpk, &Rres);

        // 利用接收机内部质量指标进行粗检
        MarkValidPre(&RawData.BasEpk0, &RawData.BasEpk);
        MarkValidPre(&RawData.RovEpk0, &RawData.RovEpk);

        // 计算单差观测值
        FormSDEpochObs(&RawData.RovEpk, &RawData.BasEpk, &RawData.SdObs);

        // 进行周跳探测
        DetectCycleSlip(&RawData.SdObs);

        // 确定基准星
        bool RefFlag = DetRefSat(&RawData.RovEpk, &RawData.BasEpk, &RawData.SdObs, &RawData.DDObs);
        if (RefFlag == false) continue;
        else; // 基准星确定成功，继续解算

        // 获取基站坐标(使用解码所得准确坐标)
        double BasPos[3];
        BLHToXYZ(RawData.BasPres.Pos, BasPos, R_WGS84, F_WGS84);

        if (fabs(RawData.RovEpk.Time.SecOfWeek - 408338) < 0.1)
        {
            printf("daole!");
        }



        switch (cfginfo.RTKProcMode)
        {
            // 迭代最小二乘解算
            case 1:
            {
                // 浮点解
                bool FloatFlag = RTKFloat(&RawData, &Rres, &Fres);
                if (FloatFlag == false) continue;
                else; // 浮点解解算成功，则进入模糊度固定

                if (lambda(RawData.DDObs.Sats * 2, 2, Fres.N, Fres.Qn, RawData.DDObs.FixedAmb, RawData.DDObs.ResAmb) != 0) continue;
                else; // 模糊度固定成功

                RawData.DDObs.Ratio = RawData.DDObs.ResAmb[1] / RawData.DDObs.ResAmb[0]; // 计算ratio值并标记是否为固定解
                if (RawData.DDObs.Ratio > cfginfo.RatioThres) RawData.DDObs.bFixed = true;
                else RawData.DDObs.bFixed = false;

                // 进行固定解解算
                if (!RTKFix(&RawData, &Rres)) continue;
                else; // 固定解解算成功，存储结果数据

                // 计算dENU
                double BasD[3] = { RawData.DDObs.dPos[0] + BasPos[0],RawData.DDObs.dPos[1] + BasPos[1],RawData.DDObs.dPos[2] + BasPos[2] };
                double dENU[3] = { 0,0,0 };
                Comp_dEnu(1, BasPos, BasD, dENU);
                fprintf(FileResult, "%f %f %f %f %f %f %d %f %s %d %d\n", RawData.RovEpk.Time.SecOfWeek, dENU[0], dENU[1], dENU[2], RawData.DDObs.FixRMS, RawData.DDObs.PDOP, RawData.DDObs.Sats, RawData.DDObs.Ratio, RawData.DDObs.bFixed ? "固定解" : "浮点解", RawData.DDObs.RefPrn[0], RawData.DDObs.RefPrn[1]);
                printf("%f dE:%f dN:%f dU:%f 残差平方和:%f PDOP:%f 可用卫星数:%d ratio:%f %s %d %d\n",
                    RawData.RovEpk.Time.SecOfWeek,
                    dENU[0],
                    dENU[1],
                    dENU[2],
                    RawData.DDObs.FixRMS,
                    RawData.DDObs.PDOP,
                    RawData.DDObs.Sats,
                    RawData.DDObs.Ratio,
                    RawData.DDObs.bFixed ? "固定解" : "浮点解",
                    RawData.DDObs.RefPrn[0], 
                    RawData.DDObs.RefPrn[1]);
                break;
            }

            // 卡尔曼滤波解算
            case 2:
            {
                // 拷贝当前历元双差观测值
                memcpy(&ekf.CurDDObs, &RawData.DDObs, sizeof(ekf.CurDDObs));
                // 滤波初始化
                if (ekf.IsInit == false)
                {
                    InitFilter(&RawData, &Rres, &ekf);
                    // 初始化时需要拷贝当前历元的数据到滤波
                    memcpy(&ekf.SDObs, &RawData.SdObs, sizeof(ekf.SDObs));
                    memcpy(&ekf.DDObs, &RawData.DDObs, sizeof(ekf.DDObs));
                    continue;
                }
                else; // 滤波已初始化过，直接进入解算

                // 一步预测
                if (!EkfPredict(&RawData, &ekf)) continue;
                else;

                // 测量更新
                if (!EkfMeasureUpdate(&RawData, &ekf)) continue;
                else;
               
                // 计算dENU
                double dENU[3] = { 0,0,0 };
                Comp_dEnu(1, BasPos, ekf.X0, dENU);
                fprintf(FileResult, "%f %f %f %f %d %f %d\n", ekf.Time.SecOfWeek, dENU[0], dENU[1], dENU[2], ekf.nSats, ekf.sigmaP, ekf.beFixed ? 1 : 0);
                printf("%f dE:%f dN:%f dU:%f 可用卫星数:%d sigmaP:%f %s\n",
                    ekf.Time.SecOfWeek,
                    dENU[0], 
                    dENU[1], 
                    dENU[2],
                    ekf.nSats,
                    ekf.sigmaP,
                    ekf.beFixed ? "固定解" : "浮点解");
                // 拷贝当前单双差数据到滤波
                memcpy(&ekf.SDObs, &RawData.SdObs, sizeof(ekf.SDObs));
                memcpy(&ekf.DDObs, &RawData.DDObs, sizeof(ekf.DDObs));
                break;
            }    

            default: printf("解算方法错误！\n");
        }
       
    }
    


    system("pause");

}

