#ifndef MyHeadFile_H

#include <iostream> 
#include <stdio.h>  
#include"sockets.h"


#define MyHeadFile_H

#define PAI 3.1415926535898
#define Rad (PAI/180.0)                   // �Ƕ�ת���ȵ�ϵ��
#define Deg (180.0/PAI)                   // ����ת�Ƕȵ�ϵ��
#define C_Light 299792458.0               // ���� m/s

#define R_WGS84  6378137.0                // WGS84����ĳ����� m 
#define F_WGS84  1.0/298.257223563        // WGS84����ı���
#define Omega_WGS 7.2921151467e-5         // WGS84�������ת���ٶ� rad/s
#define GM_WGS   398600.5e+9              // WGS84���������������GM m3/s2

#define R_CGS2K  6378137.0                // CGCS2000����ĳ����� m
#define F_CGS2K  1.0/298.257222101        // CGCS2000����ı���
#define Omega_BDS 7.2921150e-5            // CGCS2000�������ת���ٶ� rad/s
#define GM_BDS   398600.4418e+9           // CGCS2000���������������GM m3/s2

// GPS�����źŵ�һЩ����
#define  FG1_GPS  1575.42E6               // L1�ź�Ƶ�� 
#define  FG2_GPS  1227.60E6               // L2�ź�Ƶ��
#define  WL1_GPS  (C_Light/FG1_GPS)       // L1�źŲ���
#define  WL2_GPS  (C_Light/FG2_GPS)       // L2�źŲ���

// BDS�����źŵ�һЩ����
#define  FG1_BDS  1561.098E6              // B1�źŵ�Ƶ�� 
#define  FG3_BDS  1268.520E6              // B3�źŵ�Ƶ�� 
#define  WL1_BDS  (C_Light/FG1_BDS)       // B1�źŲ���
#define  WL3_BDS  (C_Light/FG3_BDS)       // B3�źŲ���


#define MAXCHANNUM 36                     // ÿһ��Ԫ���ܹ۲⵽�����������
#define MAXGPSNUM  32                     // GPSϵͳ�����������
#define MAXBDSNUM 63                      // BDSϵͳ�����������
#define MAXRAWLEN 50000                   // �ļ������ڶ�ȡ�ĵ�������ֽ���

#define POLYCRC32 0xEDB88320u             // У�������ɺ�������
#define MAXBUFF 20000                     // ���뺯��������buff����󳤶�
#define OEM7HLEN 28                       // ����ͷ���ֽ���
#define OEM7SYNC1 0xAA                    // ����ͷ�ı�ʶ1
#define OEM7SYNC2 0x44                    // ����ͷ�ı�ʶ2
#define OEM7SYNC3 0x12                    // ����ͷ�ı�ʶ3
#define ID_RANGE 43                       // �۲�ֵ���͵�message��ID
#define ID_GPSEPH 7                       // GPS�㲥�������͵�message��ID
#define ID_BDSEPH 1696                    // BDS�㲥�������͵�message��ID
#define ID_POS 42                         // NovAtel���ջ���λ������͵�message��ID

#define U1(p) (*((unsigned char *)(p)))   // ��ָ��pָ����ڴ��е�ֵ��Ϊ�޷����ַ�����
#define I1(p) (*((char *)(p)))            // ��ָ��pָ����ڴ��е�ֵ��Ϊ�ַ�����


// ��������ϵͳ���� 
enum GNSSSys { UNKS = 0, GPS, BDS};

// ͨ��ʱ�䶨��
struct COMMONTIME   
{
    short Year;
    unsigned short Month;
    unsigned short Day;
    unsigned short Hour;
    unsigned short Minute;
    double         Second;

    COMMONTIME()
    {
        Year = 0;
        Month = 0;
        Day = 0;
        Hour = 0;
        Minute = 0;
        Second = 0.0;
    }
};

// GPSʱ���壨��+���������ʽ��
struct GPSTIME
{
    unsigned short Week;
    double         SecOfWeek;

    GPSTIME()
    {
        Week = 0;
        SecOfWeek = 0.0;
    }
};

// �������ն���
struct MJDTIME             
{
    int Days;
    double FracDay;

    MJDTIME()
    {
        Days = 0;
        FracDay = 0.0;
    }
};

// GPS+BDS�㲥����
struct GPSEPHREC
{
    unsigned short PRN;
    GNSSSys     Sys;
    // TOC����ȫ���������Ӳ��ʱ������������ӵĲο�ʱ�䣩 TOE����ȫ���ڹ����ϵ�ʱ������������Ĳο�ʱ�䣩
    GPSTIME  	TOC, TOE; // s
    short		SVHealth; // 0-���� 1-������
    double		ClkBias, ClkDrift, ClkDriftRate; // �Ӳ� s  ���� s/s  ��Ư s/s2
    unsigned int	IODE, IODC;//������������ �����ӵ���������
    double      TGD1, TGD2;// �㲥����������ʱ��Ⱥ�ӳ�
    double		SqrtA, e, M0, OMEGA, i0, omega, OMEGADot, iDot, DeltaN;
    double		Crs, Cuc, Cus, Cic, Cis, Crc;// �㶯��
    double		SVAccuracy;

    GPSEPHREC() {
        PRN = SVHealth = 0;
        Sys = UNKS;
        ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
        SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
        Crs = Cuc = Cus = Cic = Cis = Crc = SVAccuracy = 0.0;
    }
};

// ÿ�����ǵĹ۲����ݶ��� 
struct SATOBS
{
    short    Prn;
    GNSSSys  System;
    // GPS��L1-0��L2-1   BDS��B1I-0, B3I-1
    double   p[2], l[2], d[2]; // α�ࡢ��λΪm,������Ϊm/s
    double   cn0[2], LockTime[2]; // ����ȣ�����ʱ��
    unsigned char half[2]; // �Ƿ���ڰ������⣬0-���ܴ��ڡ�1-������
    bool     Valid; // ���壨��˫Ƶ�����ڣ���Ч��
    
    SATOBS()
    {
        Prn = 0;
        System = UNKS;
        p[0] = p[1] = l[0] = l[1] = d[0] = d[1] = 0.0;
        cn0[0] = cn0[1] = 0.0;
        LockTime[0] = LockTime[1] = 0.0;
        half[0] = half[1] = 0;
        Valid = false;
    }
};

// ÿ����Ԫ�Ķ�λ����ṹ�嶨�� 
struct POSRES
{
    GPSTIME Time;
    double Pos[3], Vel[3];
    double PDOP, SigmaPos, SigmaVel;
    int SatNum;

    POSRES()
    {
        Time.Week = 0;
        Time.SecOfWeek = 0.0;
        for (int i = 0; i < 3; i++) Pos[i] = 0.0,Vel[i] = 0.0;
        PDOP = SigmaPos = SigmaVel = 0.0;
        SatNum = 0;
    }

};

// MW��GF��Ϲ۲�ֵ���ݵĶ���(���ڴֲ�̽��)
struct MWGF
{
    short Prn; 
    GNSSSys Sys;
    double MW, GF, PIF;
    int n; // ƽ��MW���ֵ�ļ�����

    MWGF()
    {
        Prn = n = 0;
        Sys = UNKS;
        MW = GF = PIF = 0.0;
    }
};

// ÿ������λ�á��ٶȺ��Ӳ�ȵ��м������ 
struct SATMIDRES
{
    double SatPos[3], SatVel[3];
    double SatClkOft, SatClkSft;
    double Elevation, Azimuth; // �߶Ƚǣ���λ��
    double TropCorr; // �������ӳٸ���
    /*
    GPS��tgd��׼��L1��L2��ɵ��޵������Ϲ۲�ֵ
    BDS��tgd��׼��B3
    */
    double Tgd1, Tgd2; // Ӳ���ӳ�
    bool Valid;  // false=û���������������� true-��������

    SATMIDRES()
    {
        SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
        SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
        Elevation = PAI / 2.0;
        Azimuth = 0.0;
        SatClkOft = SatClkSft = 0.0;
        Tgd1 = Tgd2 = TropCorr = 0.0;
        Valid = false;
    }
};

// ÿ����Ԫ�Ĺ۲����ݶ��� 
struct EPOCHOBS
{
    GPSTIME    Time;
    short      SatNum;
    SATOBS     SatObs[MAXCHANNUM]; // ���ǵĹ۲�����
    SATMIDRES  SatPVT[MAXCHANNUM]; // ����λ�õȼ�����������������SatObs��ͬ
    MWGF       ComObs[MAXCHANNUM]; // ��ǰ��Ԫ����Ϲ۲�ֵ������������SatObs��ͬ
    EPOCHOBS()
    {
        SatNum = 0;
    }
};

// ÿ�����ǵĵ���۲����ݶ��� 
struct SDSATOBS
{
    short    Prn;
    GNSSSys  System;
    short    Valid;
    double   dP[2], dL[2];   // ��λΪm
    unsigned char half[2]; // �Ƿ���ڰ������⣬0-���ܴ��ڡ�1-������

    // վ�䵥�����û�վ������վ�Ĺ۲���������ԭʼ�۲�ֵ�����е�����
    short    nBas, nRov;   

    SDSATOBS()
    {
        Prn = nBas = nRov = 0;
        System = UNKS;
        dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
        half[0] = half[1] = 0;
        Valid = -1;
    }
};

// ÿ����Ԫ�ĵ���۲����ݶ��� 
struct SDEPOCHOBS
{
    GPSTIME    Time;
    short      SatNum;
    SDSATOBS   SdSatObs[MAXCHANNUM];
    MWGF       SdCObs[MAXCHANNUM];

    SDEPOCHOBS()
    {
        SatNum = 0;
    }
};

// ˫����ص����ݶ��� 
struct DDCOBS
{
    int RefPrn[2], RefPos[2];         // �ο������Ǻ������ڵ��������еĴ洢λ��������0=GPS; 1=BDS
    int Sats, DDSatNum[2];            // ������˫��ģ����������0=GPS; 1=BDS
    double FixedAmb[MAXCHANNUM * 4];  // ����˫Ƶ���Ž�[0,AmbNum]�ʹ��Ž�[AmbNum,2*AmbNum]
    double ResAmb[2], Ratio;          // LAMBDA������е�ģ���Ȳв�
    float  FixRMS;                    // �̶��ⶨλ��rms��� ���в�ƽ���ͣ�
    double dPos[3];                   // ��������
    bool bFixed;                      // trueΪ�̶���falseΪδ�̶�
    double PDOP;

    DDCOBS()
    {
        int i;
        PDOP = 0;
        for (i = 0; i < 2; i++) {
            DDSatNum[i] = 0;    // ������ϵͳ��˫������
            RefPos[i] = RefPrn[i] = -1;
        }
        Sats = 0;              // ˫����������
        dPos[0] = dPos[1] = dPos[2] = 0.0;
        ResAmb[0] = ResAmb[1] = FixRMS = Ratio = 0.0;
        bFixed = false;
        for (i = 0; i < MAXCHANNUM * 2; i++)
        {
            FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
        }
    }
};

// RTK��λ�����ݶ���
struct RAWDAT 
{
    EPOCHOBS BasEpk0; // ��һ��Ԫ�Ļ�վ����
    EPOCHOBS RovEpk0; // ��һ��Ԫ������վ����
    EPOCHOBS BasEpk; // ����Ԫ�Ļ�վ����
    EPOCHOBS RovEpk; // ����Ԫ������վ����
    SDEPOCHOBS SdObs; // ����Ԫ�ĵ�������
    DDCOBS DDObs; // ����Ԫ��˫������
    GPSEPHREC GpsEph[MAXGPSNUM], BdsEph[MAXBDSNUM]; 
    POSRES BasPres; // ��վ�Ĳο�λ��
    POSRES RovPres; // ����վ�Ĳο�λ��
};

// RTK��������ݶ���
struct FloatResult
{
    double dX[3]; // ����������dx,dy,dz��Rov-Bas
    double N[MAXCHANNUM * 2]; // ˫��ģ����
    double sigma; // ���λȨ�����
    double DOP[3]; // ����������Ӧ��Э������Qxx,Qyy,Qzz��
    double PDOP;
    double Qn[(MAXCHANNUM * 2) * (MAXCHANNUM * 2)]; // ģ���ȵ�Э�������� 
    int stanum[2],totalsatnum; // ��������˫��������������������ϵͳ���ԵĻ�׼�ǣ�-> ����ȷ��Qn����ЧԪ������

    FloatResult() 
    {
        dX[0] = dX[1] = dX[2] = 0.0;
        PDOP = DOP[0] = DOP[1] = DOP[2] = 0.0;
        sigma = 0.0;
        stanum[0] = stanum[1] = totalsatnum = 0;
        for (int i = 0; i < MAXCHANNUM * 2; i++)
        {
            N[i] = 0.0;
            for (int j = 0; j < MAXCHANNUM * 2; j++)
            {
                Qn[i * MAXCHANNUM * 2 + j] = 0.0;
            }
        }
           
    }

};

// �������˲����ݶ���
struct RTKEKF
{
    GPSTIME Time;
    double X[3 + MAXCHANNUM * 2], P[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];
    int Index[MAXCHANNUM], nSats; // index��ŵ�����һ��ԪSDOBS��ÿ�����ݶ�Ӧ��Xk-1�е�����
    int FixAmb[MAXCHANNUM];          // ʱ����º��ϸ���Ԫ�Ѿ��̶������ݵ�ģ���ȣ� 1=�ѹ̶���-1=δ�̶���������
    DDCOBS DDObs, CurDDObs;           // ��һ����Ԫ�͵�ǰ��Ԫ��˫��۲�ֵ��Ϣ
    SDEPOCHOBS SDObs;                 // ��һ����Ԫ�ĵ���۲�ֵ
    double X0[3 + MAXCHANNUM * 2], P0[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];  // ״̬����
    bool IsInit;                      // �˲��Ƿ��ʼ��
    double ratio;
    double sigmaP;                   // λ�÷���
    bool beFixed;                   // trueΪ�̶���falseΪδ�̶�

    RTKEKF() {
        IsInit = false;
        nSats = 0;
        ratio = 0;
        beFixed = false;
        for (int i = 0; i < MAXCHANNUM; i++)  Index[i] = FixAmb[i] = -1;
        for (int i = 0; i < 3 + MAXCHANNUM * 2; i++) {
            X[i] = X0[i] = 0.0;
            for (int j = 0; j < 3 + MAXCHANNUM * 2; j++) P[i * (3 + MAXCHANNUM * 2) + j] = P0[i * (3 + MAXCHANNUM * 2) + j] = 0.0;
        }
    }
};

// ������Ϣ
struct ROVERCFGINFO   
{
    short  IsFileData;  // 1-�ļ����� 2-����ʵʱ����
    short RTKProcMode;  // 1-������С���˽��� 2-�������˲�����
    char   BasNetIP[20], RovNetIP[20];   // ʵʱ��������IP��ַ
    short  BasNetPort, RovNetPort;       // ʵʱ�������Ķ˿ں�
    char  BasObsDatFile[256], RovObsDatFile[256];    //  �۲����ݵ��ļ���
    double CodeNoise, CPNoise;           // α���������ز�����
    double RatioThres;                   // Ratio������ֵ
    char  ResFile[256];            //  ��������ļ���

    ROVERCFGINFO()
    {
        IsFileData = RTKProcMode = 1;
        BasNetPort = RovNetPort = 0;
        CodeNoise = CPNoise = 0.0;
        RatioThres = 3.0;
    }
};

// ��ȡ�����ļ�
bool ReadSATODSConfigInfo(const char FName[], ROVERCFGINFO* cfg);

// ͨ��ʱ,GPSʱ�ͼ�������֮����໥ת������
void CommonTimeToMJDTime(const COMMONTIME* CT, MJDTIME* MJDT);
void MJDTimeToCommonTime(const MJDTIME* MJDT, COMMONTIME* CT);
void GPSTimeToMJDTime(const GPSTIME* GT, MJDTIME* MJDT);
void MJDTimeToGPSTime(const MJDTIME* MJDT, GPSTIME* GT);
void CommonTimeToGPSTime(const COMMONTIME* CT, GPSTIME* GT);
void GPSTimeToCommonTime(const GPSTIME* GT, COMMONTIME* CT);
// ������GPSʱ֮��Ĳ�ֵ
double GetDiffTime(const GPSTIME* GT2, const GPSTIME* GT1); 


// �ռ�ֱ�����ꡢ���������໥ת������
void XYZToBLH(const double xyz[3], double blh[3], const double R, const double F);
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F);
// ��վ��ƽ����ת��������㺯��
void BLHToENUMat(const double Blh[], double Mat[]); 
// ���Ǹ߶ȽǷ�λ�Ǽ��㺯��
void CompSatElAz(int i,const double Xr[], const double Xs[], double* Elev, double* Azim); 
// ��λ�����㺯��
void Comp_dEnu(int type, const double X0[], const double Xr[], double dENU[]);


// �������
double VectDot(int m, int n, const double A[], const double B[]);
// �������
bool CrossDot(int m, int n, const double A[], const double B[], double C[]);
// �������
bool MatrixAddition(int m, int n, const double M1[], const double M2[], double M3[]);
//�������
bool MatrixSubtraction(int m, int n, const double M1[], const double M2[], double M3[]);
// �������
bool MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[]);
// ����ת��
bool MatrixTranspose(int m, int n, const double M1[], double MT[]);
// ��������
int MatrixInv(int n, double a[], double b[]);
// �����ع�
void deleteRowAndColumn(int m, int n, int m1, int n1, double M[]);// ɾ��ָ����һ�к�һ��
void deleteRow(int rows, int rowToDelete, double vector[]);// ɾ��ָ����һ��
void restructureMatrix(int a, int b, int m, int n, double A[]); // ������ʵ���õ�������


// NovAtel OEM7���ݽ��뺯��
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pres);
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBS* obs);// ��ȡ�۲�ֵ��Ϣ
int decode_gpsephem(unsigned char* buff, GPSEPHREC* eph);// ��ȡGPS�㲥������Ϣ
int decode_bdsephem(unsigned char* buff, GPSEPHREC* eph);// ��ȡBDS�㲥������Ϣ
int decode_psrpos(unsigned char* buff, POSRES* pos);// ��ȡ���ջ���λ�����Ϣ
unsigned int crc32(const unsigned char* buff, int len);// У�������ɺ���

// ���븨��ת������
unsigned short U2(unsigned char* p);
unsigned int U4(unsigned char* p);
int U8(unsigned char* p);
float R4(unsigned char* p);
double R8(unsigned char* p);

// ��������λ�á��ٶȺ��Ӳ���м���
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid);// �������ʱ�̵�GPS����PVT
bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid);// �������ʱ�̵�BDS����PVT
void ComputeGPSSatOrbitAtSignalTrans(EPOCHOBS* Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double RcvPos[3]);// �����źŷ���ʱ�̶�Ӧ���ǵ�PVT
double hopfield(double H, double E);// �������ӳٸ�������

// �ֲ�̽��
void DetectOutlier(EPOCHOBS* obs);

// SPP
bool SPP(EPOCHOBS* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res);
// SPV
void SPV(EPOCHOBS* Epoch, POSRES* Res);

// ����������
void OutputResult(const POSRES* pres, const POSRES* res, double enu[]);
void MatrixDisplay(int rows, int cols, double matrix[], const char* filename); 
void GetQnn(const int satnum, const double N_inv[], double Q[]);

// RTK
int GetSynObsPP(FILE* FBas, FILE* FRov, RAWDAT* Raw); // ����ͬ������(�º�)
int GetSynObsRT(SOCKET& BasSock, SOCKET& RovSock, RAWDAT* Raw); // ����ͬ������(ʵʱ)
void MarkValidPre(const EPOCHOBS* EpkA, EPOCHOBS* EpkB); // �ý��ջ��ڲ��������ݽ�����Ч�Ա��
void FormSDEpochObs(const EPOCHOBS* EpkR, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs); // ��վ�䵥��
void DetectCycleSlip(SDEPOCHOBS* Obs); // ����̽��
bool DetRefSat(const EPOCHOBS* EpkR, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs); // ȷ����׼��
bool RTKFloat(RAWDAT* Raw, POSRES* Rov, FloatResult* Fres); // �����
bool RTKFix(RAWDAT* Raw, POSRES* Rov); // �̶��� 

// EKFRTK
void InitFilter(RAWDAT* Raw, POSRES* Rov, RTKEKF* ekf); // �˲���ʼ������
bool EkfPredict(RAWDAT* Raw, RTKEKF* ekf); // һ��Ԥ�⺯��
bool EkfMeasureUpdate(RAWDAT* Raw, RTKEKF* ekf); // �������º���
int CalcuUpdate(int n, int m, double H[], double L[], double R[], double X[], double P[]); // �������¼��㺯��


// lambda
static int LD(int n, const double* Q, double* L, double* D);
void gauss(int n, double* L, double* Z, int i, int j);
void perm(int n, double* L, double* D, int j, double del, double* Z);
void reduction(int n, double* L, double* D, double* Z);
int search(int n, int m, const double* L, const double* D, const double* zs, double* zn, double* s);
int lambda(int n, int m, const double* a, const double* Q, double* F, double* s); // ��ģ�麯��

#endif
