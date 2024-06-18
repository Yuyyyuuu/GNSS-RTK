#include"MyHeadFile.h"


/*
****************************************************************************
���������ֲ�̽�⺯��
������	obs �۲�����
�������ܣ���obs�ֲ�̽�Ⲣ��SatObs����Valid,����ÿ�����ǵģ�P��IF��Ϲ۲�ֵ
****************************************************************************
*/
void DetectOutlier(EPOCHOBS* obs)
{
	// Ϊ�˲��ƻ���һ��Ԫ����Ϲ۲�ֵ����������Ļ�����
	MWGF comobs[MAXCHANNUM];// ��ŵ�ǰ��Ԫ�������ֵ
	for (int i = 0; i < obs->SatNum; i++)
	{	
		// ���ÿһ�����ǵ�˫Ƶα�����λ�����Ƿ�����
		if (fabs(obs->SatObs[i].p[0])< 1e-8 || fabs(obs->SatObs[i].p[1]) < 1e-8 || fabs(obs->SatObs[i].l[0]) < 1e-8 || fabs(obs->SatObs[i].l[1]) < 1e-8)
		{
			obs->SatObs[i].Valid = false;
			continue; //�����������ݲ�ȱ����������Ϲ۲�ֵ�ҹ۲�ֵ��Ч������Ϊfalse
		}
		/*
		// ���ý�ֹ�����
		if (fabs(obs->SatObs[i].cn0[0]) < 40 || fabs(obs->SatObs[i].cn0[1]) < 40)
		{
			obs->SatObs[i].Valid = false;
			continue; //������������Ƚϵͣ���������Ϲ۲�ֵ�ҹ۲�ֵ��Ч������Ϊfalse
		}
		*/
		// ���㵱ǰ��Ԫ�ĸ����ǵ�GF��MW���ֵ
		comobs[i].Sys = obs->SatObs[i].System;
		comobs[i].Prn = obs->SatObs[i].Prn;
		comobs[i].GF = obs->SatObs[i].l[0] - obs->SatObs[i].l[1];
		if (comobs[i].Sys == GPS)
		{
			comobs[i].MW = (FG1_GPS * obs->SatObs[i].l[0] - FG2_GPS * obs->SatObs[i].l[1]) / (FG1_GPS - FG2_GPS) -
				(FG1_GPS * obs->SatObs[i].p[0] + FG2_GPS * obs->SatObs[i].p[1]) / (FG1_GPS + FG2_GPS);
			comobs[i].PIF = (FG1_GPS * FG1_GPS * obs->SatObs[i].p[0] - FG2_GPS * FG2_GPS * obs->SatObs[i].p[1]) / (FG1_GPS * FG1_GPS - FG2_GPS * FG2_GPS);
		}
		else if (comobs[i].Sys == BDS)
		{
			comobs[i].MW = (FG1_BDS * obs->SatObs[i].l[0] - FG3_BDS * obs->SatObs[i].l[1]) / (FG1_BDS - FG3_BDS) -
				(FG1_BDS * obs->SatObs[i].p[0] + FG3_BDS * obs->SatObs[i].p[1]) / (FG1_BDS + FG3_BDS);
			comobs[i].PIF = (FG1_BDS * FG1_BDS * obs->SatObs[i].p[0] - FG3_BDS * FG3_BDS * obs->SatObs[i].p[1]) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
		}

		comobs[i].n = 1;

		// ���ϸ���Ԫ��MWGF�����в��Ҹ����ǵ�GF��MW���ֵ
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (obs->ComObs[j].Sys == comobs[i].Sys && obs->ComObs[j].Prn == comobs[i].Prn)
			{
				// �ҵ�����Ƚ϶����Ƿ����޲���
				double dGF = fabs(comobs[i].GF - obs->ComObs[j].GF);
				double dMW = fabs(comobs[i].MW - obs->ComObs[j].MW);

				// û�г��ޣ�����true������MW��ƽ��ֵ
				if (dGF < 0.05 && dMW < 3)
				{
					obs->SatObs[i].Valid = true;
					comobs[i].MW = (obs->ComObs[j].MW * obs->ComObs[j].n + comobs[i].MW) / (obs->ComObs[j].n + 1);
					comobs[i].n = obs->ComObs[j].n + 1;
				}
				break;
				// ���ޣ�����Ϊ�ֲ����ʼ��������false���ʲ��账��
			}
			// ������ϸ���Ԫ��û���ҵ�����֪���Ƿ���ã�Ĭ��Ϊ��ʼ��ʱ��false
			else continue;
		}		
	}
	// ����������Ϲ۲�ֵ�Ŀ�����obs��
	memcpy(obs->ComObs, comobs, sizeof(comobs));
}


/*
****************************************************
���������������������
������H   ��վ�߶�m
	  E   ��������ڲ�վ�ĸ߶Ƚ�rad
����ֵ�����������ֵ
�������ܣ����ݲ�վ�߶Ⱥ����Ǹ߶Ƚ�����Ķ��������
****************************************************
*/
double hopfield(const double H, const double E)
{
	// �����վ�߶Ȳ��ڶ����㷶Χ,�����Ϊ0
	if (fabs(H) > 1e4) return 0;
	// ��׼����Ԫ��(�ο����ϵĸ��¡���ѹ�����ʪ��)
	double H0 = 0;// m
	double T0 = 15 + 273.16;// K
	double p0 = 1013.25;// mbar
	double RH0 = 0.5;
	// ��վ�ϵĸ��¡���ѹ�����ʪ��
	double T = T0 - 0.0065 * (H - H0);
	double p = p0 * pow(1 - 0.0000226 * (H - H0), 5.225);
	double RH = RH0 * exp(-0.0006396 * (H - H0));
	double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	double hw = 11000;
	double hd = 40136 + 148.72 * (T0 - 273.16);
	double Kd = 155.2 * 1e-7 * p * (hd - H) / T;
	double Kw = 155.2 * 1e-7 * 4810 * e * (hw - H) / (T * T);
	double trop = Kd / sin(sqrt(E * E * Deg * Deg + 6.25) * Rad) + Kw / sin(sqrt(E * E * Deg * Deg + 2.25) * Rad);
	return trop;
}