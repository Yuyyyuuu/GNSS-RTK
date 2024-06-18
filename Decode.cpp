#include"MyHeadFile.h"


extern ROVERCFGINFO cfginfo;


/*
************************************************************
����������������ת������
������p ָ���������ݵ�һ���ֽڵ�λ��
����ֵ��ת�����Ӧ���͵�����
�������ܣ���ָ��p��ָ����ڴ����ݽ���Ϊ��Ӧ�������͵�����
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
��������У�������ɺ���
������buff һ��message�е�Header��Data���ɵ��ַ����ĵ�һ���ַ��ĵ�ַ
	  len ���ַ����ĳ���
����ֵ�����ɵ�У����
�������ܣ����ؼ����������������У���룬�ɽ������������crc���Ƚ����ж������Ƿ����
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
���������ܵĽ��뺯��
������Buff    ����Ŵ��ļ���һ������ȡ�����ݵĻ�����
	  Len	  ����ʱΪBuff�ĳ��ȣ�����ʱΪBuff��������ʣ���ֽ���
	  obs     ��Ź۲�ֵ���ݵĽṹ�����
	  geph    ���GPS�Ĺ㲥�����Ľṹ������
	  beph    ���BDS�Ĺ㲥�����Ľṹ������
	  pres	  ��Ž��ջ��������Ľṹ�����
����ֵ��1�������ڶ�ȡ���۲�ֵ������Ϣ��0��������û�ж����۲�ֵ������Ϣ
�������ܣ���Buff�е����ݽ���洢�����º��ļ������������۲�ֵ�������ݺ󷵻�1�������������ڶ�λ����
********************************************************************************************************
*/
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[],POSRES* pres) 
{
	int i = 0, j = 0;// ѭ���ؼ�
	int len=0;// �ַ����ĳ��ȴ洢����
	int flag=0;// �����۲�ֵ�ı�־(�����۲�ֵ���͵�message����Ҫ�˳�ѭ������λ����)
	double pos[3];// ���ڱ���obs�н��ջ���λ����ĸ�����
	int msg_type;// �ж�һ��message����ź������͵����ݣ�obs/eph/pos��
	// ��ȡ���ݵ�obs�е�GPSTIME�ĸ����м�������Ϊobs���͵�message��data����û��ʱ�䣩
	int week;
	double sow;
	// �洢һ��message����Ϣ���õ��ַ���
	unsigned char buff[MAXBUFF];
	
	while (true)
	{	
		// ��Buff�ַ������ҵ�ͷ�ļ��ı�ʶ
		for (; i < Len - 2; i++)
		{
			if (Buff[i] == OEM7SYNC1 && Buff[i + 1] == OEM7SYNC2 && Buff[i + 2] == OEM7SYNC3)
			{
				break;// ��ʱ�Ѿ��ҵ�һ��message��Header�Ŀ�ͷ�������˳�ѭ��
			}
		}

		// ����ͷ�ļ��������ж�
		if ((i + OEM7HLEN) > (Len - 1)) break;// ��ͷ�ļ����������˳�ѭ��
		
		// ��Header���ֿ�����buff
		for (j = 0; j < OEM7HLEN; j++)
		{
			buff[j] = Buff[i + j];
		}

		// ����Header��Data���ֵ��ܳ��ȣ��ֽ�����    tip:������crc�ĳ����Ǳ��ں���crc��
		len = U2(buff + 8) + OEM7HLEN; // U2(buff+8)ΪData���ֳ���
		// ����message�������ж�
		if ((i + len + 4) > (Len - 1)) break;
		

		// ��Data��CrC������Ҳ����buff������buff����һ��������message
		for (j = OEM7HLEN; j < len + 4; j++)
		{
			buff[j] = Buff[i + j];
		}

		// ����crc��
		unsigned int crc = crc32(buff, len);
		if (crc != U4(buff + len))
		{
			i = i + 1;
			continue;// �����ͬ�������Լ�⣬�����Բ����Ļ�����ҪBuff�¶������ݣ���crc�����������Buff��������µ�message
		}

		// ��ȡHeader�е�ʱ��
		week = U2(buff + 14);
		sow = U4(buff + 16) * 0.001;

		// �жϸ�message�洢���ݵ�����
		msg_type = U2(buff + 4);
		
		// ����message��ͬ�����͵�����Ӧ�Ķ�ȡ���ݺ���
		switch (msg_type)
		{
		case ID_RANGE:
			memset(obs->SatObs, 0, sizeof(obs->SatObs));
			obs->Time.Week = week;
			obs->Time.SecOfWeek = sow;
			flag = decode_rangeb_oem7(buff, obs);//�˴���flag��ֵΪ1����ʾ��ȡ���۲�ֵ��Ϣ
			break;

		case ID_GPSEPH:
			decode_gpsephem(buff, geph);//��������
			break;

		case ID_BDSEPH:
			decode_bdsephem(buff, beph);//��������
			break;

		case ID_POS:
			memset(pres, 0, sizeof(pres));
			decode_psrpos(buff,pres);
			break;

		default:
			break;
		}

		// һ��message�������������������β��һλ�Ա����Ѱ��message
		i = i + len + 4;

		// ��������˹۲�ֵ������Ϣ���˳�Ѱ����Ϣ��ѭ�������ж�λ����
		if (flag == 1)
		{
			break;
		}
		memset(buff, 0, sizeof(buff));
	}

	// ����Buff���ݣ�����������ɾ��
	for (j = 0; j < Len - i; j++)
	{
		Buff[j] = Buff[i + j];
	}
	
	Len = j; // ������������������Len���Ա�Buff�����ٶ�ȡ��������
	return flag;// �����Ƿ���Ҫ��λ�ı�ʶ
}

/*
**********************************************************
���������۲�ֵ���ݽ���
������buff    ����һ�������Ĺ۲�ֵmessgae���ַ����׵�ַ
	  obs	  ��ȡ���Ĺ۲�ֵ���ݵĴ洢��
����ֵ���۲�ֵ�����Ƿ����ɹ���1-�ɹ���0-ʧ�ܣ�
�������ܣ���buff�е����ݽ��룬���洢��obs��
**********************************************************
*/
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBS* obs)
{
	short satnum=0;// ���Ǹ�������
	short f_index=0;// Ƶ�ʶ�Ӧ�洢������GPS��L1-0��L2-1��BDS��B1I-0,B3I-1��
	unsigned int obs_num = U4(buff + OEM7HLEN);// 44���ֽڵĹ۲����ݿ�Ŀ���(ѭ�����ü�����)
	// ����������message������obs���ݿ�
	for (int i = 0; i < obs_num; i++)
	{
		// ��ù۲�ֵ������ϵͳ
		GNSSSys satsys;
		short sys = (U4(buff + OEM7HLEN +44+44*i) >> 16) & 0x7;
		if (sys == 0) satsys = GPS;
		else if (sys == 4) satsys = BDS;
		else continue;

		// ��ù۲�ֵ��Ƶ������
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
		
		// ������������
		short half = (U4(buff + OEM7HLEN + 44 + 44 * i) >> 11) & 0x1;
		unsigned short prn = U2(buff + OEM7HLEN + 4+44*i);
		double p = R8(buff + OEM7HLEN + 8 + 44 * i);// m
		double l = -R8(buff + OEM7HLEN + 20 + 44 * i);// cycle
		float d = -R4(buff + OEM7HLEN + 32 + 44 * i);// Hz
		float cn0 = R4(buff + OEM7HLEN + 36 + 44 * i);
		float locktime = R4(buff + OEM7HLEN + 40 + 44 * i);// s

		// �жϸ������Ƿ��Ѿ������ĳ��Ƶ�ʵ�����
		// ���-->����������һƵ�ʵ����ݣ�û�д��-->�����µ�һ���������ݿ�
		short index=0;//��������Ӧ�洢������λ������
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

		//�������ݵĴ洢
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
��������GPS�㲥�������ݽ���
������buff    ����һ�������Ĺ۲�ֵmessgae���ַ����׵�ַ
	  geph	  ��ȡ�����������ݵĴ洢��
����ֵ�����������Ƿ����ɹ���1-�ɹ���0-ʧ�ܣ�
�������ܣ���buff�е����ݽ��룬���洢��geph��
**********************************************************
*/
int decode_gpsephem(unsigned char* buff, GPSEPHREC* geph)//�׵�ַ
{
	// ��ñ���������GPS���������еĴ洢λ��
	int prn=U4(buff + OEM7HLEN);
	GPSEPHREC* eph;
	eph = geph + prn - 1;
	eph->Sys = GPS;
	eph->PRN = U4(buff + OEM7HLEN);
	// �ж������Ƿ��Խ��
	if (prn > MAXGPSNUM || prn < 1) return 0;
	eph->SVHealth= U4(buff + OEM7HLEN+12);// 0-������1-������
	eph->TOC.Week= U4(buff + OEM7HLEN + 24);// GPS��
	eph->TOE.Week= U4(buff + OEM7HLEN + 24);// GPS��
	eph->TOC.SecOfWeek = R8(buff + OEM7HLEN + 164);// GPS������
	eph->TOE.SecOfWeek= R8(buff + OEM7HLEN + 32);// GPS������
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
��������BDS�㲥�������ݽ���
������buff    ����һ�������Ĺ۲�ֵmessgae���ַ����׵�ַ
	  beph	  ��ȡ�����������ݵĴ洢��
����ֵ�����������Ƿ����ɹ���1-�ɹ���0-ʧ�ܣ�
�������ܣ���buff�е����ݽ��룬���洢��beph��
**********************************************************
*/
int decode_bdsephem(unsigned char* buff, GPSEPHREC* beph)
{
	// ��ñ���������BDS���������еĴ洢λ��
	int prn = U4(buff + OEM7HLEN);
	GPSEPHREC* eph;
	eph = beph + prn - 1;
	eph->Sys = BDS;
	eph->PRN = U4(buff + OEM7HLEN);
	// �ж������Ƿ��Խ��
	if (prn > MAXBDSNUM || prn < 1) return 0;

	eph->SVHealth = U4(buff + OEM7HLEN + 16);// 0-������1-������
	eph->TOC.Week = U4(buff + OEM7HLEN + 4);// BDS��
	eph->TOE.Week = U4(buff + OEM7HLEN + 4);// BDS��
	eph->TOC.SecOfWeek = U4(buff + OEM7HLEN + 40);// BDS������
	eph->TOE.SecOfWeek = U4(buff + OEM7HLEN + 72);// BDS������
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
��������NovAtel��λ������ݽ���
������buff    ����һ�������Ķ�λ���messgae���ַ����׵�ַ
	  pres	  ��ȡ���Ķ�λ������ݵĴ洢��
����ֵ�����������Ƿ����ɹ���1-�ɹ���0-ʧ�ܣ�
�������ܣ���buff�е����ݽ��룬���洢��pres��
**********************************************************
*/
int decode_psrpos(unsigned char* buff, POSRES* pres)
{
	pres->Time.Week= U2(buff + 14);
	pres->Time.SecOfWeek= U4(buff + 16) * 0.001;
	pres->Pos[0] = R8(buff + OEM7HLEN + 8)*Rad;//����(rad)
	pres->Pos[1] = R8(buff + OEM7HLEN + 16) * Rad;//γ��(rad)
	pres->Pos[2] = R8(buff + OEM7HLEN + 24)+ R4(buff + OEM7HLEN + 32);//��ظ�(m)
	pres->SatNum = U1(buff + OEM7HLEN + 65);//����������
	return 1;
}