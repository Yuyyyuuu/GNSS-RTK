#include"MyHeadFile.h"


extern ROVERCFGINFO cfginfo;


/*
************************************************************
函数名：数据类型转化函数
参数：p 指向所需数据第一个字节的位置
返回值：转化后对应类型的数据
函数功能：将指针p所指向的内存内容解释为对应数据类型的数据
************************************************************
*/
unsigned short U2(unsigned char* p)
{
	unsigned short u;
	memcpy(&u, p, 2);
	return u;
}
unsigned int U4(unsigned char* p)
{
	unsigned int u;
	memcpy(&u, p, 4);
	return u;
}
int I4(unsigned char* p)
{
	int i;
	memcpy(&i, p, 4);
	return i;
}
float R4(unsigned char* p)
{
	float r;
	memcpy(&r, p, 4);
	return r;
}
double R8(unsigned char* p)
{
	double r;
	memcpy(&r, p, 8);
	return r;
}

/*
**************************************************************************************
函数名：校验码生成函数
参数：buff 一条message中的Header和Data构成的字符串的第一个字符的地址
	  len 该字符串的长度
返回值：生成的校验码
函数功能：本地计算机根据数据生成校验码，可将其与解码所得crc作比较来判断数据是否可用
**************************************************************************************
*/
unsigned int crc32(const unsigned char* buff, int len)
{
	int i, j;
	unsigned int crc = 0;
	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
			else crc >>= 1;

		}
	}
	return crc;
}

/*
********************************************************************************************************
函数名：总的解码函数
参数：Buff    存放着从文件中一次所读取的数据的缓冲区
	  Len	  传入时为Buff的长度，传出时为Buff被解码后的剩余字节数
	  obs     存放观测值数据的结构体变量
	  geph    存放GPS的广播星历的结构体数组
	  beph    存放BDS的广播星历的结构体数组
	  pres	  存放接收机解算结果的结构体变量
返回值：1代表函数内读取到观测值类型消息，0代表函数内没有读到观测值类型消息
函数功能：对Buff中的数据解码存储，（事后文件处理）在遇到观测值类型数据后返回1便于在主函数内定位解算
********************************************************************************************************
*/
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[],POSRES* pres) 
{
	int i = 0, j = 0;// 循环控件
	int len=0;// 字符串的长度存储变量
	int flag=0;// 读到观测值的标志(读到观测值类型的message就需要退出循环来定位解算)
	double pos[3];// 用于保留obs中接收机定位结果的辅助量
	int msg_type;// 判断一条message存放着何种类型的数据（obs/eph/pos）
	// 读取数据到obs中的GPSTIME的辅助中间量（因为obs类型的message的data部分没有时间）
	int week;
	double sow;
	// 存储一条message的信息所用的字符串
	unsigned char buff[MAXBUFF];
	
	while (true)
	{	
		// 在Buff字符串内找到头文件的标识
		for (; i < Len - 2; i++)
		{
			if (Buff[i] == OEM7SYNC1 && Buff[i + 1] == OEM7SYNC2 && Buff[i + 2] == OEM7SYNC3)
			{
				break;// 此时已经找到一条message的Header的开头索引，退出循环
			}
		}

		// 进行头文件完整性判断
		if ((i + OEM7HLEN) > (Len - 1)) break;// 若头文件不完整则退出循环
		
		// 将Header部分拷贝给buff
		for (j = 0; j < OEM7HLEN; j++)
		{
			buff[j] = Buff[i + j];
		}

		// 计算Header和Data部分的总长度（字节数）    tip:不计入crc的长度是便于核验crc码
		len = U2(buff + 8) + OEM7HLEN; // U2(buff+8)为Data部分长度
		// 进行message完整性判断
		if ((i + len + 4) > (Len - 1)) break;
		

		// 将Data和CrC的数据也拷入buff，这样buff就是一条完整的message
		for (j = OEM7HLEN; j < len + 4; j++)
		{
			buff[j] = Buff[i + j];
		}

		// 核验crc码
		unsigned int crc = crc32(buff, len);
		if (crc != U4(buff + len))
		{
			i = i + 1;
			continue;// 这个不同于完整性检测，完整性不够的话就需要Buff新读入数据，而crc不对则继续在Buff里遍历到新的message
		}

		// 读取Header中的时间
		week = U2(buff + 14);
		sow = U4(buff + 16) * 0.001;

		// 判断该message存储数据的类型
		msg_type = U2(buff + 4);
		
		// 根据message不同的类型调用相应的读取数据函数
		switch (msg_type)
		{
		case ID_RANGE:
			memset(obs->SatObs, 0, sizeof(obs->SatObs));
			obs->Time.Week = week;
			obs->Time.SecOfWeek = sow;
			flag = decode_rangeb_oem7(buff, obs);//此处将flag赋值为1，表示读取到观测值消息
			break;

		case ID_GPSEPH:
			decode_gpsephem(buff, geph);//传数组名
			break;

		case ID_BDSEPH:
			decode_bdsephem(buff, beph);//传数组名
			break;

		case ID_POS:
			memset(pres, 0, sizeof(pres));
			decode_psrpos(buff,pres);
			break;

		default:
			break;
		}

		// 一条message读完后把索引调整到其结尾后一位以便继续寻找message
		i = i + len + 4;

		// 如果读到了观测值类型消息则退出寻找消息的循环，进行定位解算
		if (flag == 1)
		{
			break;
		}
		memset(buff, 0, sizeof(buff));
	}

	// 调整Buff内容，将已用数据删掉
	for (j = 0; j < Len - i; j++)
	{
		Buff[j] = Buff[i + j];
	}
	
	Len = j; // 将现有数据数量赋给Len，以便Buff决定再读取多少数据
	return flag;// 返回是否需要定位的标识
}

/*
**********************************************************
函数名：观测值数据解码
参数：buff    存有一条完整的观测值messgae的字符串首地址
	  obs	  读取到的观测值数据的存储体
返回值：观测值数据是否解码成功（1-成功，0-失败）
函数功能：对buff中的数据解码，并存储在obs中
**********************************************************
*/
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBS* obs)
{
	short satnum=0;// 卫星个数计数
	short f_index=0;// 频率对应存储索引（GPS：L1-0，L2-1；BDS：B1I-0,B3I-1）
	unsigned int obs_num = U4(buff + OEM7HLEN);// 44个字节的观测数据块的块数(循环所用计数器)
	// 遍历完整个message所含的obs数据块
	for (int i = 0; i < obs_num; i++)
	{
		// 获得观测值的卫星系统
		GNSSSys satsys;
		short sys = (U4(buff + OEM7HLEN +44+44*i) >> 16) & 0x7;
		if (sys == 0) satsys = GPS;
		else if (sys == 4) satsys = BDS;
		else continue;

		// 获得观测值的频率类型
		short f = (U4(buff + OEM7HLEN + 44 + 44 * i) >> 21) & 0x1F;
		if (sys == 0)
		{
			if (f == 0) f_index = 0;
			else if (f == 9) f_index = 1;
			else continue;
		}
		else if (sys == 4)
		{
			if (f == 0||f==4) f_index = 0;
			else if (f == 2||f==6) f_index = 1;
			else continue;
		}
		
		// 解码其他数据
		short half = (U4(buff + OEM7HLEN + 44 + 44 * i) >> 11) & 0x1;
		unsigned short prn = U2(buff + OEM7HLEN + 4+44*i);
		double p = R8(buff + OEM7HLEN + 8 + 44 * i);// m
		double l = -R8(buff + OEM7HLEN + 20 + 44 * i);// cycle
		float d = -R4(buff + OEM7HLEN + 32 + 44 * i);// Hz
		float cn0 = R4(buff + OEM7HLEN + 36 + 44 * i);
		float locktime = R4(buff + OEM7HLEN + 40 + 44 * i);// s

		// 判断该卫星是否已经存入过某个频率的数据
		// 存过-->继续存入另一频率的数据，没有存过-->存入新的一个卫星数据块
		short index=0;//本次数据应存储的数组位置索引
		for (int j = 0; j < satnum; j++)
		{
			if (satsys == obs->SatObs[j].System && prn == obs->SatObs[j].Prn)
			{
				index = j;
			}
			else
			{
				index = satnum;				
			}			
		}

		//进行数据的存储
		obs->SatObs[index].System = satsys;
		obs->SatObs[index].Prn = prn;
		obs->SatObs[index].p[f_index] = p;
		obs->SatObs[index].cn0[f_index] = cn0;                          
		obs->SatObs[index].LockTime[f_index] = locktime;
		obs->SatObs[index].half[f_index] = half;
		if (sys == 0)
		{
			if (f_index == 0)
			{
				obs->SatObs[index].l[f_index] = l* WL1_GPS;// m
				obs->SatObs[index].d[f_index] = d * WL1_GPS;// m/s
			}
			else if(f_index == 1)
			{
				obs->SatObs[index].l[f_index] = l * WL2_GPS;// m
				obs->SatObs[index].d[f_index] = d * WL2_GPS;// m/s
			}
		}
		if (sys == 4)
		{
			if (f_index == 0)
			{
				obs->SatObs[index].l[f_index] = l * WL1_BDS;// m
				obs->SatObs[index].d[f_index] = d * WL1_BDS;// m/s
			}
			else if (f_index == 1)
			{
				obs->SatObs[index].l[f_index] = l * WL3_BDS;// m
				obs->SatObs[index].d[f_index] = d * WL3_BDS;// m/s
			}
		}
		if (index == satnum) satnum++;
	}
	obs->SatNum = satnum;
	return 1;
}

/*
**********************************************************
函数名：GPS广播星历数据解码
参数：buff    存有一条完整的观测值messgae的字符串首地址
	  geph	  读取到的星历数据的存储体
返回值：星历数据是否解码成功（1-成功，0-失败）
函数功能：对buff中的数据解码，并存储在geph中
**********************************************************
*/
int decode_gpsephem(unsigned char* buff, GPSEPHREC* geph)//首地址
{
	// 获得本次数据在GPS星历数组中的存储位置
	int prn=U4(buff + OEM7HLEN);
	GPSEPHREC* eph;
	eph = geph + prn - 1;
	eph->Sys = GPS;
	eph->PRN = U4(buff + OEM7HLEN);
	// 判断数组是否会越界
	if (prn > MAXGPSNUM || prn < 1) return 0;
	eph->SVHealth= U4(buff + OEM7HLEN+12);// 0-正常，1-不正常
	eph->TOC.Week= U4(buff + OEM7HLEN + 24);// GPS周
	eph->TOE.Week= U4(buff + OEM7HLEN + 24);// GPS周
	eph->TOC.SecOfWeek = R8(buff + OEM7HLEN + 164);// GPS周内秒
	eph->TOE.SecOfWeek= R8(buff + OEM7HLEN + 32);// GPS周内秒
	eph->ClkBias= R8(buff + OEM7HLEN + 180);// s
	eph->ClkDrift= R8(buff + OEM7HLEN + 188);// s/s
	eph->ClkDriftRate= R8(buff + OEM7HLEN + 196);// s/s2
	eph->IODE= U4(buff + OEM7HLEN + 20);
	eph->IODC= U4(buff + OEM7HLEN + 16);
	eph->TGD1= R8(buff + OEM7HLEN + 172);//s
	eph->SqrtA= sqrt(R8(buff + OEM7HLEN + 40));// sqrt(m)
	eph->e= R8(buff + OEM7HLEN + 64);
	eph->M0= R8(buff + OEM7HLEN + 56);// rad
	eph->OMEGA= R8(buff + OEM7HLEN + 144);// rad
	eph->i0= R8(buff + OEM7HLEN + 128);// rad
	eph->omega= R8(buff + OEM7HLEN + 72);// rad
	eph->OMEGADot= R8(buff + OEM7HLEN + 152);// rad/s
	eph->iDot= R8(buff + OEM7HLEN + 136);// rad/s
	eph->DeltaN= R8(buff + OEM7HLEN + 48);// rad/s
	eph->Crs= R8(buff + OEM7HLEN + 104);// m
	eph->Cuc= R8(buff + OEM7HLEN + 80);// rad
	eph->Cus= R8(buff + OEM7HLEN + 88);// rad
	eph->Cic= R8(buff + OEM7HLEN + 112);// rad
	eph->Cis= R8(buff + OEM7HLEN + 120);// rad
	eph->Crc= R8(buff + OEM7HLEN + 96);// m
	eph->SVAccuracy= R8(buff + OEM7HLEN + 216);// m2
	return 1;
}

/*
**********************************************************
函数名：BDS广播星历数据解码
参数：buff    存有一条完整的观测值messgae的字符串首地址
	  beph	  读取到的星历数据的存储体
返回值：星历数据是否解码成功（1-成功，0-失败）
函数功能：对buff中的数据解码，并存储在beph中
**********************************************************
*/
int decode_bdsephem(unsigned char* buff, GPSEPHREC* beph)
{
	// 获得本次数据在BDS星历数组中的存储位置
	int prn = U4(buff + OEM7HLEN);
	GPSEPHREC* eph;
	eph = beph + prn - 1;
	eph->Sys = BDS;
	eph->PRN = U4(buff + OEM7HLEN);
	// 判断数组是否会越界
	if (prn > MAXBDSNUM || prn < 1) return 0;

	eph->SVHealth = U4(buff + OEM7HLEN + 16);// 0-正常，1-不正常
	eph->TOC.Week = U4(buff + OEM7HLEN + 4);// BDS周
	eph->TOE.Week = U4(buff + OEM7HLEN + 4);// BDS周
	eph->TOC.SecOfWeek = U4(buff + OEM7HLEN + 40);// BDS周内秒
	eph->TOE.SecOfWeek = U4(buff + OEM7HLEN + 72);// BDS周内秒
	eph->ClkBias = R8(buff + OEM7HLEN + 44);// s
	eph->ClkDrift = R8(buff + OEM7HLEN + 52);// s/s
	eph->ClkDriftRate = R8(buff + OEM7HLEN + 60);// s/s2
	eph->IODE = U4(buff + OEM7HLEN + 68);
	eph->IODC = U4(buff + OEM7HLEN + 36);
	eph->TGD1 = R8(buff + OEM7HLEN + 20);// s
	eph->TGD2 = R8(buff + OEM7HLEN + 28);// s
	eph->SqrtA = R8(buff + OEM7HLEN + 76);// sqrt(m)
	eph->e = R8(buff + OEM7HLEN + 84);
	eph->M0 = R8(buff + OEM7HLEN + 108);// rad
	eph->OMEGA = R8(buff + OEM7HLEN + 116);// rad
	eph->i0 = R8(buff + OEM7HLEN + 132);// rad
	eph->omega = R8(buff + OEM7HLEN + 92);// rad
	eph->OMEGADot = R8(buff + OEM7HLEN + 124);// rad/s
	eph->iDot = R8(buff + OEM7HLEN + 140);// rad/s
	eph->DeltaN = R8(buff + OEM7HLEN + 100);// rad/s
	eph->Crs = R8(buff + OEM7HLEN + 172);// m
	eph->Cuc = R8(buff + OEM7HLEN + 148);// rad
	eph->Cus = R8(buff + OEM7HLEN + 156);// rad
	eph->Cic = R8(buff + OEM7HLEN + 180);// rad
	eph->Cis = R8(buff + OEM7HLEN + 188);// rad
	eph->Crc = R8(buff + OEM7HLEN + 164);// m
	eph->SVAccuracy = R8(buff + OEM7HLEN + 8);// m
	return 1;
}

/*
**********************************************************
函数名：NovAtel定位结果数据解码
参数：buff    存有一条完整的定位结果messgae的字符串首地址
	  pres	  读取到的定位结果数据的存储体
返回值：星历数据是否解码成功（1-成功，0-失败）
函数功能：对buff中的数据解码，并存储在pres中
**********************************************************
*/
int decode_psrpos(unsigned char* buff, POSRES* pres)
{
	pres->Time.Week= U2(buff + 14);
	pres->Time.SecOfWeek= U4(buff + 16) * 0.001;
	pres->Pos[0] = R8(buff + OEM7HLEN + 8)*Rad;//经度(rad)
	pres->Pos[1] = R8(buff + OEM7HLEN + 16) * Rad;//纬度(rad)
	pres->Pos[2] = R8(buff + OEM7HLEN + 24)+ R4(buff + OEM7HLEN + 32);//大地高(m)
	pres->SatNum = U1(buff + OEM7HLEN + 65);//所用卫星数
	return 1;
}