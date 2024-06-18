#include"MyHeadFile.h"


// ͨ��ʱ To ��������
void CommonTimeToMJDTime(const COMMONTIME* CT, MJDTIME* MJDT)
{
	int y, m;
	double UT = CT->Hour + CT->Minute / 60.0 + CT->Second / 3600;
	if (CT->Month < 2 || CT->Month == 2)
	{
		y = CT->Year - 1;
		m = CT->Month + 12;
	}
	else if (CT->Month > 2)
	{
		y = CT->Year;
		m = CT->Month;
	}
	//��������
	double MJD = int(365.25 * y) + int(30.6001 * (m + 1)) + CT->Day  + 1720981.5 - 2400000.5;
	MJDT->Days = int(MJD);
	MJDT->FracDay = UT/24.0;
}

// �������� To ͨ��ʱ
void MJDTimeToCommonTime(const MJDTIME* MJDT, COMMONTIME* CT)
{
	//������
	double JD = MJDT->Days + MJDT->FracDay + 2400000.5;

	int a = int(JD + 0.5);
	int b = a + 1537;
	int c = int((b - 122.1) / 365.25);
	int d = int(365.25 * c);
	int e = int((b-d) / 30.6001);
	
	CT->Day = b - d - int(30.6001 * e);
	CT->Month = e - 1 - 12 * int(e / 14);
	CT->Year = c -4715 - int((7 + CT->Month) / 10);
	CT->Hour = int(MJDT->FracDay*24);
	CT->Minute = int(MJDT->FracDay*24*60- CT->Hour*60);
	CT->Second = MJDT->FracDay*24*60*60- CT->Hour*24*60*60- CT->Minute*60;
}

// GPSʱ To ��������
void GPSTimeToMJDTime(const GPSTIME* GT, MJDTIME* MJDT)
{
	double MJD = 44244 + GT->Week * 7 + GT->SecOfWeek / 86400;
	MJDT->Days = int(MJD);
	MJDT->FracDay = (GT->SecOfWeek- int(GT->SecOfWeek / 86400)*86400)/86400;
}

// �������� To GPSʱ
void MJDTimeToGPSTime(const MJDTIME* MJDT, GPSTIME* GT)
{
	GT->Week = int((MJDT->Days - 44244) / 7+ MJDT->FracDay/7);
	GT->SecOfWeek = (MJDT->Days+ MJDT->FracDay - 44244 - GT->Week * 7) * 86400;
}

// ͨ��ʱ To GPSʱ
void CommonTimeToGPSTime(const COMMONTIME* CT, GPSTIME* GT)
{
	MJDTIME MJDT;
	CommonTimeToMJDTime(CT, &MJDT);
	MJDTimeToGPSTime(&MJDT, GT);

}

// GPSʱ To ͨ��ʱ
void GPSTimeToCommonTime(const GPSTIME* GT, COMMONTIME* CT)
{
	MJDTIME MJDT;
	GPSTimeToMJDTime(GT, &MJDT);
	MJDTimeToCommonTime(&MJDT, CT);
}

// ������GPSʱ֮��Ĳ�ֵ������ʱ�λΪ��
double GetDiffTime(const GPSTIME* GT2, const GPSTIME* GT1)
{
 
	double second = (GT2->Week - GT1->Week) * 7 * 24 * 60 * 60 + GT2->SecOfWeek - GT1->SecOfWeek;
	return second;
}


/*
****************************************************
���������ѿ������� To �������
������xyz �ѿ������� m
	  blh ������꣨b��l��λΪrad��hΪm��
	  R ���򳤰���
	  F �������
�������ܣ����ѿ�������ת��Ϊ�������
****************************************************
*/
void XYZToBLH(const double xyz[3], double blh[3], const double R, const double F)
{
	double X = xyz[0];
	double Y = xyz[1];
	double Z = xyz[2];

	const double e2 = 2.0 * F - F * F;

	double S = sqrt(X * X + Y * Y);
	double L = atan2(Y, X);
	double B = 0;
	double N = 0;
	double tempB = atan2(Z, S);

	int counter = 0;
	while (fabs(B - tempB) > 1e-12 && counter < 25)
	{
		B = tempB;
		N = R / sqrt(1 - e2 * sin(B) * sin(B));
		tempB = atan2(Z + N * e2 * sin(B), S);
		counter++;
	}

	blh[0] = B;
	blh[1] = L;
	blh[2] = S / cos(B) - N;
}

/*
****************************************************
��������������� To �ѿ�������
������BLH ������꣨B��L��λΪrad��HΪm��
	  XYZ �ѿ������� m
	  R ���򳤰���
	  F �������
�������ܣ����������ת��Ϊ�ѿ�������
****************************************************
*/
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F)
{
	double B = BLH[0];
	double L = BLH[1];
	double H = BLH[2];
	double e2 = 2 * F - F * F;
	double N= R / sqrt(1 - e2 * sin(B) * sin(B));
	XYZ[0] = (N + H) * cos(B) * cos(L);
	XYZ[1] = (N + H) * cos(B) * sin(L);
	XYZ[2] = (N * (1 - e2) + H) * sin(B);
}

/*
*****************************************************
����������վ��ƽ����ת��������㺯��
������Blh ��վ�Ĵ�����꣨B��l��λΪrad��hΪm��
	  Mat 3*3��ת������������ʾ
�������ܣ����ݲ�վ�Ĵ������õ���վ��ƽ����ת������
*****************************************************
*/
void BLHToENUMat(const double Blh[], double Mat[])
{
	double B = Blh[0];
	double L = Blh[1];
	double H = Blh[2];
	Mat[0] = -sin(L);
	Mat[1] = cos(L);
	Mat[2] = 0;
	Mat[3] = -sin(B) * cos(L);
	Mat[4] = -sin(B) * sin(L);
	Mat[5] = cos(B);
	Mat[6] = cos(B) * cos(L);
	Mat[7] = cos(B) * sin(L);
	Mat[8] = sin(B);
}

/*
*************************************************************************
����������λ�����㺯��
������type    ��������1-WGS84  2-CGCS2000��
	  X0      ��վ��ȷ�ģ��ѿ���������
	  Xr      ��վ�ģ��ѿ���������
	  dENU    ��վ��ƽϵ�Ķ�λ��� ˳��Ϊ��dE��dN��dH
�������ܣ����ݲ�վ�ľ�ȷ�ѿ�������ʹ��������������������Ķ�λ���
*************************************************************************
*/
void Comp_dEnu(int type, const double X0[], const double Xr[], double dENU[])
{
	// ��վ�ľ�ȷ�ѿ�������ת��Ϊ��������Ժ�������ת������
	double BLH[3];
	if(type == 1) XYZToBLH(X0, BLH, R_WGS84, F_WGS84);
	else if (type == 2) XYZToBLH(X0, BLH, R_CGS2K, F_CGS2K);
	// �����վ��ƽ����ת������
	double Mat[9];
	BLHToENUMat(BLH, Mat);
	double dxyz[3] = { Xr[0] - X0[0], Xr[1] - X0[1], Xr[2] - X0[2] };
	MatrixMultiply(3, 3, 3, 1, Mat, dxyz, dENU);
}

/*
************************************************************
�����������Ǹ߶Ƚǡ���λ�Ǽ��㺯��
������type    ��������1-WGS84  2-CGCS2000��
	  Xr      ��վ�ģ��ѿ��������� m
	  Xs      ���ǵģ��ѿ��������� m
	  Elev    ���Ǹ߶Ƚ� rad
	  Azim    ���Ƿ�λ�� rad
�������ܣ�������������Ͳ�վ����������ǵĸ߶ȽǺͷ�λ��
************************************************************
*/
void CompSatElAz(int type,const double Xr[], const double Xs[], double* Elev, double* Azim)
{
	// ������������ڲ�վ��dNEU ˳��Ϊ��dE��dN��dH
	double dENU[3];
	Comp_dEnu(type,Xr, Xs, dENU);
	*Elev = atan2(dENU[2], sqrt(dENU[0] * dENU[0] + dENU[1] * dENU[1]));
	*Azim = atan2(dENU[0], dENU[1]);
}