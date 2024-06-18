#include"MyHeadFile.h"


/*
****************************************************************************
函数名：粗差探测函数
参数：	obs 观测数据
函数功能：对obs粗差探测并在SatObs里标记Valid,计算每颗卫星的（P）IF组合观测值
****************************************************************************
*/
void DetectOutlier(EPOCHOBS* obs)
{
	// 为了不破坏上一历元的组合观测值数组而建立的缓冲区
	MWGF comobs[MAXCHANNUM];// 存放当前历元所算组合值
	for (int i = 0; i < obs->SatNum; i++)
	{	
		// 检查每一颗卫星的双频伪距和相位数据是否完整
		if (fabs(obs->SatObs[i].p[0])< 1e-8 || fabs(obs->SatObs[i].p[1]) < 1e-8 || fabs(obs->SatObs[i].l[0]) < 1e-8 || fabs(obs->SatObs[i].l[1]) < 1e-8)
		{
			obs->SatObs[i].Valid = false;
			continue; //本颗卫星数据残缺，不计算组合观测值且观测值有效性设置为false
		}
		/*
		// 设置截止信噪比
		if (fabs(obs->SatObs[i].cn0[0]) < 40 || fabs(obs->SatObs[i].cn0[1]) < 40)
		{
			obs->SatObs[i].Valid = false;
			continue; //本颗卫星信噪比较低，不计算组合观测值且观测值有效性设置为false
		}
		*/
		// 计算当前历元的该卫星的GF和MW组合值
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

		// 从上个历元的MWGF数据中查找该卫星的GF和MW组合值
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (obs->ComObs[j].Sys == comobs[i].Sys && obs->ComObs[j].Prn == comobs[i].Prn)
			{
				// 找到了则比较二者是否在限差内
				double dGF = fabs(comobs[i].GF - obs->ComObs[j].GF);
				double dMW = fabs(comobs[i].MW - obs->ComObs[j].MW);

				// 没有超限，则标记true并计算MW的平滑值
				if (dGF < 0.05 && dMW < 3)
				{
					obs->SatObs[i].Valid = true;
					comobs[i].MW = (obs->ComObs[j].MW * obs->ComObs[j].n + comobs[i].MW) / (obs->ComObs[j].n + 1);
					comobs[i].n = obs->ComObs[j].n + 1;
				}
				break;
				// 超限，则标记为粗差（而初始化本就是false，故不需处理）
			}
			// 如果在上个历元中没有找到，则不知其是否可用，默认为初始化时的false
			else continue;
		}		
	}
	// 将缓冲区组合观测值的拷贝到obs里
	memcpy(obs->ComObs, comobs, sizeof(comobs));
}


/*
****************************************************
函数名：对流层改正函数
参数：H   测站高度m
	  E   卫星相对于测站的高度角rad
返回值：对流层改正值
函数功能：根据测站高度和卫星高度角来算的对流层改正
****************************************************
*/
double hopfield(const double H, const double E)
{
	// 如果测站高度不在对流层范围,则输出为0
	if (fabs(H) > 1e4) return 0;
	// 标准气象元素(参考面上的干温、气压和相对湿度)
	double H0 = 0;// m
	double T0 = 15 + 273.16;// K
	double p0 = 1013.25;// mbar
	double RH0 = 0.5;
	// 测站上的干温、气压和相对湿度
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