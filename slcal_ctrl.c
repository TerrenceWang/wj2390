/*************************************************************************
  Copyright (c), 1998-2013, �Ϻ�����΢���Ӽ��Źɷ����޹�˾

  �ļ�����: slcal_ctrl.c
  �ļ�����: 

 
  �޶���¼:
         1. ����: ������
            ����: 2013-11-29
            ����: ·�ƿ���Ӧ�ò���ƴ���
		 2. ����: ���ұ�
            ����: 2015-05-27
            ����: 1.���ӵڶ�·PWM, ���������38KHz����, �������ж�, ʱ��Դѡ��TIMER2
                 2. OC0ʱ��Դ�޸�ΪTIMER0
		 3. �޸�: ���ұ�
            ����: 2015-06-19
            ����: �жϺ���ѭ����ʹ�õ�ȫ�ֱ������volatile����
		 4. �޸�: ���ұ�
            ����: 2015-08-04
            ����: ���ӵڶ�·�������, ����Ƶ���޸�Ϊ1KHz
		 5. �޸ģ����ұ�
		    ���ڣ�2015-10-29
			���ݣ� 1. �̵���IO�ڴ�PA7�޸�ΪPD0
			      2. �̵���IO�˴���ʼ��ȡ��
		 6. �޸ģ����ұ�
		    ���ڣ�2015-11-25
			���ݣ� Ϊ���⵱ǰ�в��Զ��������µĲ��Ե��µ�ʱ��մ�, ÿ�θ��µ�ǰ����ʱͬʱ����Ĭ�Ͽ���
		 7. �޸ģ����ұ�
		    ���ڣ�2016-04-26
			���ݣ�Ϊ֧�ֹ�ѹ�ϵ�, ���ƿ���ʱ���͹ص�ʱ������ʵ��IO�ڶ���(Ŀǰ�����˲�)
		 8. ���ߣ����ұ�
		    ���ڣ�2016-04-26
			���ݣ� ���ƴ���������ʱ�����ص�ʱ�����ۼ�ǰ����У��;
			      ���ƴ���������ʱ�����ص�ʱ���ڴ洢eepromǰ����У��;
				  ��鵱ǰ����ʱ, �Ѿ���ѹ��Ϊ�򿪼̵������޶���
				  ��鵱ǰ����ʱ, ��ֱ�ӿ��Ƽ̵��������ж��Ƿ����ϴ�һ��
**************************************************************************/
#include "plc_type.h"
#include "FM312_REG.h"
#include "debug.h"
#include "sysmisc.h"
#include "slcal.h"
#include "slcal_eeprom.h"
#include "slcal_rtc.h"
#include "slcal_ctrl.h"
#include "slcal_check.h"
#include "slcal_tools.h"
#include "slcal_meter.h"

const U32 PWR_NUMBER[15] = {35999,35999,23999,17999,14399,11999,10285,8999,7999,7199,
                            13090,11999,11076,10285,9599};
const U16 BRIGHTNESS_UNIT[15] = {360,360,240,180,144,120,102,90,80,72,130,120,110,102,96};//һ�����ȵ�λ��Ӧ��timer��λֵ

U8 slcal_ctrl_type[MAX_LED_NUM];						//·�ƿ������ͣ�0��ʾȱʡ���ƣ�1��ʾ���ؿ��ƣ�2��ʾǿ�ƿ���
slcal_forcectrl_struct slcal_force_ctrl[MAX_LED_NUM];	//ǿ��״̬�µĿ��Ʋ����ṹ��
slcal_ctrl_struct slcal_default_ctrl[MAX_LED_NUM];		//ȱʡ״̬�µĿ��Ʋ����ṹ��
volatile slcal_ctrl_struct slcal_ctrl[MAX_LED_NUM];	    //��ǰ����״̬�µĿ��Ʋ�����¼����crc16У����

volatile slcal_dim_struct slcal_dim;						//�������ȵ�λֵ��������
volatile U8 slcal_dim_signal[MAX_LED_NUM];					//������ɱ�־
volatile slcal_selfpwm_struct slcal_selfpwm[MAX_LED_NUM];	//����Ӧ������ز���

slcal_ctrl_data_struct slcal_ctrl_data[MAX_LED_NUM];	//·�������ۼƲ���
slcal_ctrl_data_struct eeprom_slcal_ctrl_data[MAX_LED_NUM];	//eeprom�ڵ�·�������ۼƲ���

slcal_init_ctrl_struct slcal_init_ctrl;                  //�ϵ�Ĭ�ϵ�״̬
slcal_rated_power_struct slcal_rated_power[MAX_LED_NUM]; //�����
slcal_ComunicationRetry_struct slcal_ComunicationRetry;//����������ͨ�ſ���



volatile U8 next_duration_addr[MAX_LED_NUM];             

/*----------------------------------------���ؿ�����ز���---------------------------------------------*/

slcal_longi_lati_struct slcal_longi_lati;       //��γ��
slcal_sumtime_struct slcal_sumtime;    			//�ճ�����ʱ��

slcal_localctrl_struct slcal_localctrl;			//���ؿ��Ʋ���
volatile U8 localctrl_init_f;          			//���ؿ��Ƴ�ʼ����ʶ��0-��ʼ�������  1-��Ҫ���³�ʼ����
volatile U8 localctrl_store_f;         			//���ؿ��Ʋ������洢��ʶ(0x00-�洢��� 0x01-����ղ��������洢����Ҫ���� 0x02-�����±��ز��������洢����Ҫ����)
volatile U8 localctrl_sort_f;                   //���ؿ��Ʋ����������ʶ(0-������� 1-��Ҫ����)

U16 last_ctrl_timepoint[MAX_LED_NUM];  		    //���һ�ο���ʱ���

static U8 lastday_date;				   			//ǰһ������ 

U8 Control_command_dispose(U8 command_type, U8 brightness);
/*----------------------------------------------End----------------------------------------------------*/

#ifdef SLC_DEBUG
#pragma optimization_level 0
#endif

/*******************************************Local_Control_Start**********************************************/
static U8 localctrl_time_changeto_timepoint(volatile U16 time_point_array[])
{
    U8 i;
    volatile U16 sunrisetime = 0, sunsettime = 0;
    volatile U16 array[LOCAL_PACKAGE_MAX_NUM];
    volatile U16 tempa, temp16;
    
    if(slcal_localctrl.local_package_num == 0)
    {
        return 0;
    }
    
    sunrisetime = slcal_sumtime.sun_rise_time[0]; //�����ճ�����ʱ�̵�
    sunrisetime *= 60;
    sunrisetime += slcal_sumtime.sun_rise_time[1];

    sunsettime = slcal_sumtime.sun_set_time[0];
    sunsettime *= 60;
    sunsettime += slcal_sumtime.sun_set_time[1];

	for(i = 0; i < LOCAL_PACKAGE_MAX_NUM; i++)//����
	{
		array[i]  = 0;
	}

	for(i = 0; i < slcal_localctrl.local_package_num; i++)
	{
		if((slcal_localctrl.local_package_params[i][0] >> 4) == 1)		//��ʱ����
		{
			array[i]  = slcal_localctrl.local_package_params[i][2];
			array[i] *= 60;
			array[i] += slcal_localctrl.local_package_params[i][3];
		}
		else if((slcal_localctrl.local_package_params[i][0] >> 4) == 2)	//��γ�ȿ���
		{						
			tempa = slcal_localctrl.local_package_params[i][3];//���ֽڹ�ͬ����ƫ�Ƶķ����� ���ֽ���ǰ ���ֽ��ں�
			tempa = tempa << 8;
			tempa += slcal_localctrl.local_package_params[i][2];
			
			temp16 = tempa & 0x7FFF;

			if((slcal_localctrl.local_package_params[i][0] & 0x0F) == 0)//�̵������
			{
				if((((slcal_localctrl.local_package_params[i][4] & 0x0F) == 0x0C) && (((slcal_localctrl.local_package_params[i][4] >> 4) & 0x0F) == 0x0C))
				   || (((slcal_localctrl.local_package_params[i][4] & 0x0F) == 0x0C) && (((slcal_localctrl.local_package_params[i][4] >> 4) & 0x0F) == 0x00))
				   || (((slcal_localctrl.local_package_params[i][4] & 0x0F) == 0x00) && (((slcal_localctrl.local_package_params[i][4] >> 4) & 0x0F) == 0x0C))) //ֻ�йصƲ��������ճ�ʱ��Ϊ��׼��
				{
					if(tempa & 0x8000) //��ƫ��
					{
						if(sunrisetime >= temp16)
							array[i] = sunrisetime - temp16;
						else
							array[i] = 1440 + sunrisetime - temp16;     //����һ��
					}
					else              //��ƫ��
					{
						array[i] = sunrisetime + temp16;
						if(array[i] >= 1440) array[i] = array[i] % 1440;//����һ��							
					}
				}
				else //����������ʱ��Ϊ������
				{
					if(tempa & 0x8000) //��ƫ��
					{
						if(sunsettime >= temp16)
							array[i] = sunsettime - temp16;
						else
							array[i] = 1440 + sunsettime - temp16;       //����һ��
					}
					else              //��ƫ��
					{
						array[i] = sunsettime + temp16;
						if(array[i] >= 1440) array[i]  = array[i] % 1440;//����һ��						
					}
				}
			}
			else if(((slcal_localctrl.local_package_params[i][0] & 0x0F) == 1) || ((slcal_localctrl.local_package_params[i][0] & 0x0F) == 2)) //  PWM or RS485
			{
				if((slcal_localctrl.local_package_params[i][4] & 0xF0) == 0x00)//10%���µ�����Ϊ�صƣ����йصƲ��������ճ�ʱ��Ϊ��׼��
				{
					if(tempa & 0x8000) //��ƫ��
                    {
                        if(sunrisetime >= temp16)
                            array[i] = sunrisetime - temp16;
                        else
                            array[i] = 1440 + sunrisetime - temp16;
                    }
                    else              //��ƫ��
                    {
                        array[i] = sunrisetime + temp16;
                        if(array[i] >= 1440) array[i] = array[i] % 1440;
                            
                    }
				}
				else//����������ʱ��Ϊ������
				{
					if(tempa & 0x8000) //��ƫ��
				    {
						if(sunsettime >= temp16)
						{
							array[i] = sunsettime - temp16;
						}
						else
						{
							array[i] = 1440 + sunsettime - temp16;        //����һ��
						}
					}
					else              //��ƫ��
					{
						array[i] = sunsettime + temp16;

						if(array[i] >= 1440) array[i]  = array[i] % 1440;  //����һ��
					}
				}

			}
		}
		
		time_point_array[i] = array[i];
	}
	
	return 1;
}


/*************************************************************************
 �������ƣ�	localctrl_new_para_save_or_sort
 ����˵����	���ؿ��Ʋ������������
 ��������� save_flag - 1-���� 0-����
 ���ز�����
 *************************************************************************/
static U8 localctrl_new_para_save_or_sort(U8 save_flag)
{
    U8 i, result, j, k, m, n, Min, temp=1;
    U8 buffer[8];
    U16 crc;
    volatile U16 array[LOCAL_PACKAGE_MAX_NUM];
    volatile U16 tempa;
	
	if (!save_flag)
	{
		localctrl_time_changeto_timepoint(array);//��ʱ���ƫ����ת��Ϊͳһ��ʱ���

		for(m = 0; m < (slcal_localctrl.local_package_num-1); m++)//��С��������
		{
			Min = m;

			for(n = (m+1); n < slcal_localctrl.local_package_num; n++)//��ʣ����������ҳ���С��
			{
				if((array[Min] & 0x0FFF) > (array[n] & 0x0FFF))
				{
					Min = n;
				}
			}

			if(Min == m) continue;

			tempa = array[m];
			array[m] = array[Min];
			array[Min] = tempa;

			for(i = 0; i < 6; i++)
			{
				temp = slcal_localctrl.local_package_params[m][i];
				slcal_localctrl.local_package_params[m][i]  = slcal_localctrl.local_package_params[Min][i];
				slcal_localctrl.local_package_params[Min][i]  = temp;
			}
		}
		
		slcal_localctrl.crc = CalcCRC16((U8 *)&slcal_localctrl,194);
	}
	else
	{
			
		/*result = store_data_with_back((U8 *)(&slcal_localctrl),196,SL_LOCAL_CTRL_PARA,SL_LOCAL_CTRL_PARA+SL_BACK_OFFSET);
	   	if(result != SLCAL_EEPROM_FINISH)
	   	{
	   		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
	   		#ifdef SLC_PRINT_ASSERT
	   		ASSERT(0);
	   		#endif
	   		return 0;
	   	}
	   	else
	   	{
	   		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
	   		return 1;
	   	}*/
	   	
	   	for(k=0; k<LOCAL_PACKAGE_MAX_NUM; k++)
		{
			for(j = 0; j < LOCAL_PACKAGE_MAX_LEN; j++)
			{
				buffer[j]=slcal_localctrl.local_package_params[k][j];
			}	
			crc=CalcCRC16(buffer,j);
			buffer[j]=(U8)(crc>>8);
			buffer[j+1]=(U8)(crc&0x00ff);
			result=store_data_with_back(buffer,j+2,SL_LOCAL_CTRL_PARA+k*(LOCAL_PACKAGE_MAX_LEN+2),SL_LOCAL_CTRL_PARA+k*(LOCAL_PACKAGE_MAX_LEN+2)+SL_BACK_OFFSET);
			if(result==SLCAL_EEPROM_FINISH)temp &= 1; 
			else temp &= 0; 
		}
		
		buffer[0]=slcal_localctrl.local_package_num;
		crc=CalcCRC16(buffer,1);
		buffer[1]=(U8)(crc>>8);
		buffer[2]=(U8)(crc&0x00ff);
		result=store_data_with_back(buffer,3,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2),SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+SL_BACK_OFFSET);
		if(result==SLCAL_EEPROM_FINISH)temp &= 1; 
		else temp &= 0; 
		
		buffer[0]=(U8)(slcal_localctrl.crc>>8);
		buffer[1]=(U8)(slcal_localctrl.crc&0x00ff);
		crc=CalcCRC16(buffer,2);
		buffer[2]=(U8)(crc>>8);
		buffer[3]=(U8)(crc&0x00ff);
		result=store_data_with_back(buffer,4,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+4,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+4+SL_BACK_OFFSET);
		if(result==SLCAL_EEPROM_FINISH)temp &= 1; 
		else temp &= 0; 
		
		if(temp)
		{
			slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
			slcal_eeprom_fault_classify &= (~0x20);
			return 1;
		}
		else
		{	
			slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
			slcal_eeprom_fault_classify |= 0x20;
			return 0;
		}
	}
	
	return 1;
}

/*************************************************************************
 �������ƣ�	count_days()
 ����˵����	����Ӹ�������ʱ�乫Ԫ2000��1��1�յ������յ����յ�����
 ��������� now_time[]-��ǰʱ��[0-4:year_low/year_high/month/day/week]   
 ���ز����� days-����
 *************************************************************************/
static U32 count_days(U8 now_time[])
{
    U16  i;
    U32  days=0;
    
    i= (now_time[0] + now_time[1]*256) - 1;               
    while(i>1999)                                     //�������1��1�յ�2000��1��1�չ��ж�����
    {
      if((i%4)==0)  days+=366;
      else          days+=365;
      i--;
    }
    
    for(i=1; i<now_time[2];i++)                       //���㵱��1�վ���1��1�չ��ж����� 
    { 
      if(i<8)
      {
         if(i%2!=0) days+=31;                              // 1��3��5��7Ϊ31��
         else if(i!=2) days+=30;                           // 4��6Ϊ30��
         else
         {
           if(((now_time[0] + now_time[1]*256)%4)==0) 
           {
              days+=29;                                     //����2��29��
           } 
           else
           {
             days+=28;                                      //������2��29��
           }                
         }
      }
      else
      {
        if((i%2)==0) days+=31;                              //8��10��12Ϊ31��
        else         days+=30;                              //9��11��Ϊ30��
      }
    }
    
    days+=now_time[3];                                       //����������2000��1��1�չ��ж�����     
                  
    return (days);
}

/*************************************************************************
 �������ƣ�	rise_set_time_calc()
 ����˵����	�ճ�����ʱ���㷨��ʱ���
 ���������    
 ���ز����� 
 *************************************************************************/
static F32 rise_set_time_algorithm(F32 UT0, U32 days, F32 glatitude, F32 longitude, U8 riseflag)
{  
    U8 sunflag=0;

    F32  temp,temp2;
    F32  t,temp_B,ut;
    F32  l,g,c,lamt,e,gha;
    
    ut=0;
    temp = (ut - UT0)*10;
    while( (temp>1) || (temp<-1) )
    { 
		if(sunflag) UT0=ut;   
			
        //t=(days+UT0/360)/36525;
        t  = UT0;                                //������t	 4887
        t /= 360;
        t += days;
        t /= 36525;

        //L=280.460+36000.770*t; 
        l  = 36000.770;                         //̫����ƽ�ƾ�     5091.897
        l *= t;
        l += 280.460;                           

      
        //G=(357.528+35999.050*t)*3.1415926/180;
        g  = 35999.050;                    	    //̫����ƽ�����    90.211339
        g *= t;
        g += 357.528;
        g *= PI;
        g /= 180;			 

        //D=1.915*sin(G)+0.020*sin(2*G);
        //A=(L+D)*3.1415926/180;     			//lamt == A
        lamt  = f_sin(g);            			//̫���ĻƵ�����
        lamt *= 1.915;                			//1.914986972
        temp  = f_sin(2.0*g);        			//-0.00737705
        temp *= 0.020;
        temp2 = temp+lamt;            			//����̫��ʱ���gha����
        lamt = temp2+l;
        lamt *= PI;
        lamt /= 180;

        //temp_B=(23.4393-0.0130*t)*3.1415926/180;    
        temp_B  = t;                  			//��������
        temp_B *= 0.0130;
        temp_B  = 23.4393-temp_B;
        temp_B *= PI;
        temp_B /= 180;


        // c=asin(sin(A)*sin(temp_B));
        temp  = f_sin(temp_B);       			//̫����ƫ��
        c  = f_sin(lamt);
        temp *= c;
        c = f_asin(temp);                       
      
		// GHA=UTo-180     -1.915?��sinG-0.020?��sin(2G)          +2.466?��sin(2|?)-0.053?��sin(4|?)
		//F=2.466*sin(2*A)-0.053*sin(4*A);
		//GHA=UT0-180-lamt +F;
		temp = f_sin(2*lamt);                   //̫��ʱ��� gha
		temp *= 2.466;                      	 //0.0949997
		temp -= 0.053*f_sin(4*lamt);            //0.0990801961
		gha = UT0-180;
		gha += temp;
		gha -= temp2; 

		//H=sin(Glat)*sin(C);
		//I=cos(Glat)*cos(C);														 
		//H=(sin(-0.01454)-H)/I;
		//E=acos(H)*180/3.1415926;

		temp = f_sin(glatitude);       				 //����ֵe
		temp *= f_sin(c);

		temp2 = -0.887*PI/180;
		temp2 = f_sin(temp2);
		temp = temp2 - temp;

		temp2 = f_cos(glatitude);
		temp2 *= f_cos(c);

		temp /= temp2;
		e = f_acos(temp);
		e *= 180;
		e /= PI;

		//ut=UT0-(GHA+Long+E);
		ut = UT0;
		ut -= gha;
		ut -= longitude;
		if(riseflag== SUNRISEFLAG)
		ut -= e;
		else
		ut += e;

		sunflag=1;

		temp = (ut-UT0)*10;

	} 
	
	return(ut);
}

/*************************************************************************
 �������ƣ�	sunrise_sunset_time_calculate
 ����˵����	���㱾�ܵ��ճ�����ʱ�䣬ֻ����������һ���������(�����ճ�)
 ��������� *long_glat - ��γ��
            now_time[] - ��ǰʱ��[0-4:year_low/year_high/month/day/week]   
            rise_set_UT0[] - ���һ�μ����ճ�����ʱ��
            sunrise_time[] - ����������ճ�ʱ��
            sunset_time[]  - �������������ʱ��          
 ���ز����� ��
 *************************************************************************/
static void sunrise_sunset_time_calculate(slcal_longi_lati_struct longi_glati,U8 now_time[],U8 rise_set_UT0[],U8 sunrise_time[],U8 sunset_time[])                           
{   
    F32    rise_UT0=0x00000000,set_UT0=0x00000000,rise_UT=0x00000000,set_UT=0x00000000;
    F32    rise_t=0x00000000,set_t=0x00000000;
    F32    longitude=0x00000000,glatitude=0x00000000;
    U32    days=0;

    rise_UT = PI;
    set_UT  = PI;
    days = count_days(now_time);                                                            //����ӹ�Ԫ2000��1��1�յ����������
                                    
    longitude  = longi_glati.longitude_fra;                                                 //����ֵ
	longitude /= 100.00;                                                                   
	longitude += longi_glati.longitude_int;
                 
	glatitude  = longi_glati.latitude_fra;                                                  //γ��ֵ
	glatitude /= 100.00;                                                                   
	glatitude += longi_glati.latitude_int;
			 
    glatitude  = glatitude*PI;                                                              //ת��Ϊ����
    glatitude /= 180;             

	rise_UT0  = rise_set_UT0[1];                                                            //���һ�μ����rise_UT0
	rise_UT0 /= 100.00;                                                                   
	rise_UT0 += rise_set_UT0[0];

    if( (rise_UT0>(-2)) && (rise_UT0<2) ) 
	{
		rise_UT0=180;
	}

	set_UT0  = rise_set_UT0[3];                                                            //���һ�μ����set_UT0
	set_UT0 /= 100.00;                                                                   
	set_UT0 += rise_set_UT0[2];
		
    if( (set_UT0>(-2)) && (set_UT0<2) )
	{
	    set_UT0=180;
	}

	rise_UT=rise_set_time_algorithm(rise_UT0,days,glatitude,longitude,SUNRISEFLAG);//�����ճ�ʱ��
	rise_t=rise_UT/15+8; 
	rise_UT0=rise_UT;
	sunrise_time[0]=(U8)(rise_t);                                    //��������ΪСʱ��
	sunrise_time[1]=(U8)((rise_t-(U8)(rise_t))*60);                  //С������Ϊ������

	set_UT=rise_set_time_algorithm(set_UT0,days,glatitude,longitude,SUNSETFLAG);   //��������ʱ��
	set_t=set_UT/15+8;
	set_UT0=set_UT; 
	sunset_time[0]=(U8)(set_t);                                      //��������ΪСʱ��
	sunset_time[1]=(U8)((set_t-(U8)(set_t))*60);                     //С������Ϊ������

    rise_set_UT0[0]=(S8)(rise_UT);
    rise_set_UT0[1]=(S8)((rise_UT-(S8)(rise_UT))*100);
    rise_set_UT0[2]=(S8)(set_UT);
    rise_set_UT0[3]=(S8)((set_UT-(S8)(set_UT))*100);
 
}

/*************************************************************************
 �������ƣ�	update_sun_rise_set_time
 ����˵����	���µ����ճ�����ʱ��
 ��������� 
 ���ز����� 
 *************************************************************************/
void update_sun_rise_set_time(void)
{
	slcal_sumtime.now_day[0] = real_time.year_low;
	slcal_sumtime.now_day[1] = real_time.year_high;
	slcal_sumtime.now_day[2] = real_time.month;
	slcal_sumtime.now_day[3] = real_time.day;
	slcal_sumtime.now_day[4] = real_time.week;

	slcal_sumtime.rise_set_UT0[0] = 180;
	slcal_sumtime.rise_set_UT0[1] = 0;
	slcal_sumtime.rise_set_UT0[2] = 180;
	slcal_sumtime.rise_set_UT0[3] = 0;

	sunrise_sunset_time_calculate(slcal_longi_lati, slcal_sumtime.now_day, slcal_sumtime.rise_set_UT0, slcal_sumtime.sun_rise_time, slcal_sumtime.sun_set_time);
}	

/*************************************************************************
 �������ƣ�	findout_relay_enable_point
 ����˵����	Ѱ�Ҽ̵���������������
 ���������	
 ���ز�����	
 *************************************************************************/
void findout_relay_enable_point(U16 nowtime, U16 remotectrtime[], U8 parameter[][LOCAL_PACKAGE_MAX_LEN],U8 packagenum, U16 *timearray)
{
    U8 i, j, acttingnum, temp = 0, m=0;
    U8 lastacttingflag, lastdaypoint, array[3];
    U32 actting_flag;
    U16 timearrayK1[LOCAL_PACKAGE_MAX_NUM], lastdaytime;
    S8 k;

    for(m = 0; m < MAX_LED_NUM; m++)
    {
    	i = 0; 
		j = 0; 
	    k=0;
	    acttingnum = 0;
	    //lampact = 0;
	    temp = 0;
	    lastacttingflag = 0;
	    lastdaypoint = 0xff;
	    actting_flag = 0;
	    lastdaytime = 0;

		for(i = 0; i < LOCAL_PACKAGE_MAX_NUM; i++)
		{
			timearrayK1[i] = 0;
		}

		//ɸѡ��������ʹ�ܵ�
		for(i = 0; i < packagenum; i++)                             
		{
		   if((parameter[i][1]&BIT(real_time.week))
		      && ((((parameter[i][0]&0x0F) == 0x01)&&((parameter[i][4] & (0x01<<m)) == (0x01<<m)))//PWM���� 
		          || (((parameter[i][0] & 0x0F) == 0x00)&&((((parameter[i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x03) || (((parameter[i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x0C))))) //�̵�������
			{
		       timearrayK1[acttingnum] = timearray[i];
		       actting_flag |= BIT(i);              	//��¼����ʹ�ܵ�
			   acttingnum++;                        
		    }
		}

		if((acttingnum == 0) || (nowtime < timearrayK1[0]))//������ʹ�ܵ� �� ��ǰʱ��С�ڽ��յ�һ��ʹ�ܵ�
		{
		    //ɸѡ��һʹ���յ����һ��
		    if(real_time.week == 0) k = real_time.week - 6;
		    else k=1;
		    
			while (!lastacttingflag)
			{
				for(i = 0; i < packagenum; i++)
				{
					if(parameter[packagenum - 1 - i][1]&BIT((real_time.week - k)))//�ӽ���ǰһ�쿪ʼ��ǰ����
					{
						if(((parameter[packagenum - 1 - i][0] & 0x0F) == 0)
						   && ((((parameter[packagenum - 1 - i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x03)|| (((parameter[packagenum - 1 - i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x0C)))//�̵�������
						{
						    //lampact |= ((parameter[packagenum - 1 - i][4+(m/2)]>>((m%2)*4)) & 0x0F);
						    lastacttingflag = 1;
						    lastdaypoint = packagenum - 1 - i;
						    break;
						}
						else if(((parameter[packagenum - 1 - i][0] & 0x0F) != 0) && ((parameter[packagenum - 1 - i][4] & (0x01<<m)) == (0x01<<m)))//PWM����
						{
							lastacttingflag = 1;
							break;
						}					
					}
				}

				if(lastacttingflag || ((real_time.week - k)== real_time.week))
				{
					break;
				}
				else if ((real_time.week - k - 1)<0)
				{
					k = real_time.week - 6;
				}
				else 
				{
					k++;
				}
			}

			if(lastdaypoint!=0xff)//��һʹ��������Ѳ�����Ϊ�̵�����������Ҫ���ü̵�������
			{
				lastdaytime = timearray[lastdaypoint];
				
				temp = 0;

				if((nowtime < timearrayK1[0]) || (nowtime > lastdaytime) || ((acttingnum == 0)&&(nowtime <1440)))  //��ǰ������һ��
				{
					temp |= 0x01;
				}

				//if((remotectrtime[m] < timearrayK1[0]) || (remotectrtime[m] > lastdaytime) || ((acttingnum == 0)&&(lastday_date == real_time.day)&&(remotectrtime[m] <1440))) //��һ����ʱ�������һ��
				if(((remotectrtime[m] < timearrayK1[0]) || (remotectrtime[m] > lastdaytime) || ((acttingnum == 0) && (remotectrtime[m]<1440))) && (lastday_date == real_time.day))//��һ���Ʋ���������һʱ��
				{
					if(remotectrtime[m] != 0x3CC3)//��ʼֵ 0xff*60+0xff
					{
						temp |= 0x02;
					}
					
				}

				if(((temp & 0x01) == 0x01) && ((temp & 0x02) != 0x02)) //��ǰʱ�����賿�Σ���һ����ʱ����ڷ��賿��
				{
					//LampLoopStatusChanged_Flag[0] = 0;
					
				    array[0] =  slcal_localctrl.local_package_params[lastdaypoint][0];
					array[1] =  slcal_localctrl.local_package_params[lastdaypoint][4];          
					array[2] =  slcal_localctrl.local_package_params[lastdaypoint][5]; 
					instant_ctrl_setting(array,m);
					
		    		/*status[m] = lampact & 0x0F;
				    
					if (status[m] == 0x03)
					{
					  status[m] = LAMP_OPEN;
					}
					else if (status[m] == 0x0c)
					{
					  status[m] = LAMP_CLOSE;
					}

					switch(status[m])
					{
						case LAMP_OPEN:
						case LAMP_CLOSE:
						
							slcal_force_ctrl[m].relay_status  = status[m];    
							slcal_force_ctrl[m].status=ENABLE_FORCECTRL;
							slcal_force_ctrl[m].last_time=0xffff;
							slcal_force_ctrl[m].second_count=0;

							if(status[m] == LAMP_CLOSE)                                                     
							{
								slcal_force_ctrl[m].brightness_status = 0;
							}
							else
							{
								slcal_force_ctrl[m].brightness_status = 100;
							}
							slcal_force_ctrl[m].crc=CalcCRC16((U8 *)&slcal_force_ctrl[m],6);
							
							remotectrtime[m] = (U16)(real_time.hour * 60 + real_time.minute); 
							
							break;
							
						default:
							break;
					}*/

				}
			}
		}
		else //������ʹ�ܵ��ҵ�ǰʱ��λ�ڽ��յ�һ��ʹ�ܵ��
		{
		   for(i = 1; i < acttingnum; i++)
		   {
		       if(nowtime < timearrayK1[i])         //��ǰʱ��С����һִ�в���������������ʱtimearrayK1[i-1]Ϊ��Ҫ��������ʹ�ܵ�
		           break;
		   }

		   for(j = 0; j < acttingnum; j++)
		   {
		       if(remotectrtime[m] < timearrayK1[j])//��һ����ʱ��С����һִ�в���������������ʱtimearrayK1[j-1]ʹ�ܵ㲻���в�����
		           break;
		   }

		   if((i != j) || (remotectrtime[m] == 0x3CC3)|| ((i == j)&&(lastday_date != real_time.day)))//��ǰʱ������һ����ʱ�̲���ͬһʱ��||Զ�̲���ʱ��Ϊ��ʼֵ(��Զ�̲���)||����ͬһʱ�ε����ڲ�ͬ
		   {			
				//LampLoopStatusChanged_Flag[0] = 0;	
				
				temp = 0;

				for(j = 0; j < LOCAL_PACKAGE_MAX_NUM; j++)
				{
					if(actting_flag & BIT(j))
					{
						temp++;

						if(temp == i)          //��temp��1
						{
							if(j <= (packagenum-1))
							{
								if(((parameter[j][0] & 0x0F) == 0)&& ((((parameter[j][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x03)|| (((parameter[j][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x0C)))//�̵������� 																
								{
									array[0] =  slcal_localctrl.local_package_params[j][0];
									array[1] =  slcal_localctrl.local_package_params[j][4];          
									array[2] =  slcal_localctrl.local_package_params[j][5]; 
									instant_ctrl_setting(array,m);
									
								    /*status[m] = parameter[j][4] & 0x0F;
								    
									if (status[m] == 0x03)
									{
									  status[m] = LAMP_OPEN;
									}
									else if (status[m] == 0x0c)
									{
									  status[m] = LAMP_CLOSE;
									}
									
									switch(status[m])
									{
										case LAMP_OPEN:
										case LAMP_CLOSE:
										
											slcal_force_ctrl[m].relay_status  = status[m];    
											slcal_force_ctrl[m].status=ENABLE_FORCECTRL;
											slcal_force_ctrl[m].last_time=0xffff;
											slcal_force_ctrl[m].second_count=0;

											if(status[m] == LAMP_CLOSE)                                                     
											{
												slcal_force_ctrl[m].brightness_status = 0;
											}
											else
											{
												slcal_force_ctrl[m].brightness_status = 100;
											}
											slcal_force_ctrl[m].crc=CalcCRC16((U8 *)&slcal_force_ctrl[m],6);
											
											remotectrtime[m] = (U16)(real_time.hour * 60 + real_time.minute); 
											
											break;

										default:
											break;
									}
									
									//lastacttingflag = 1;*/
									
									
									break;
								}
							}
						}
					}
				}
			}
		}
	}
}
/*************************************************************************
 �������ƣ�	findout_pwm_enable_point
 ����˵����	Ѱ�ҵ��������������
 ���������	
 ���ز�����	
 *************************************************************************/
void findout_pwm_enable_point(U16 nowtime, U16 remotectrtime[], U8 parameter[][LOCAL_PACKAGE_MAX_LEN],U8 packagenum, U16 *timearray)
{
	U8  i, j, k, temp,temp_flag, m=0;
	U8  acttingnum, lastdaypoint,lastacttingflag,array[3];  
	U16 todaytimearray[LOCAL_PACKAGE_MAX_NUM],lastdaytime;
	U32 actting_flag;
	S8	g;
	
	for(m = 0; m < MAX_LED_NUM; m++)//������ԣ�mAX_lED_NUM���·��
	{			
		acttingnum = 0; //�ж�����
		actting_flag = 0;//�ж���־
		  
	    lastdaypoint = 0xff;//
		lastacttingflag = 0;//
		lastdaytime = 0;//
		
		//activepackage = 0xff;
		
		g = 1;
		temp = 0;
		temp_flag = 0;
		i = 0;
		j = 0;
		k = 0;
		
		for(i = 0; i < LOCAL_PACKAGE_MAX_NUM; i++)
		{
			todaytimearray[i] = 0;
		}
		
		//ɸѡ��������ʹ�ܵ�
	    for(i = 0; i < packagenum; i++)
	    {
		   if((parameter[i][1]&BIT(real_time.week))
		      && ((((parameter[i][0]&0x0F) == 0x01)&&((parameter[i][4] & (0x01<<m)) == (0x01<<m)))//PWM
		          || (((parameter[i][0] & 0x0F) == 0x00)&&((((parameter[i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x03) || (((parameter[i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x0C))))) //�̵���
			{
	            todaytimearray[acttingnum] = timearray[i];
	            actting_flag |= BIT(i);
	            acttingnum++;
	        }
	    }

		if((acttingnum == 0) || (nowtime < todaytimearray[0]))    //������ʹ�ܵ� �� ��ǰʱ��С�ڽ��յ�һ��ʹ�ܵ�
		{
		    //��ѯ��һʹ���յ����һ��ʹ�ܵ�
		    if(real_time.week == 0) g = real_time.week - 6;
		    else g=1;
		    
			while (!lastacttingflag)
			{
				for(i = 0; i < packagenum; i++)
				{
					if(parameter[packagenum - 1 - i][1]&BIT((real_time.week - g)))//�ӽ���ǰһ�쿪ʼ��ǰ����
					{
						if(((parameter[packagenum - 1 - i][0] & 0x0F) == 0)
						   && ((((parameter[packagenum - 1 - i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x03)|| (((parameter[packagenum - 1 - i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x0C)))//�̵���
						{
						    lastacttingflag = 1;
						    break;
						}
						else if(((parameter[packagenum - 1 - i][0] & 0x0F) != 0) && ((parameter[packagenum - 1 - i][4] & (0x01<<m)) == (0x01<<m)))//PWM
						{
							lastdaypoint = packagenum - 1 - i;
							lastacttingflag = 1;
							break;
						}					
					}
				}

				if(lastacttingflag || ((real_time.week - g)== real_time.week))
				{
					break;
				}
				else if ((real_time.week - g - 1)<0)
				{
					g = real_time.week - 6;
				}
				else 
				{
					g++;
				}
			}
		
		
			//��һʹ��������Ѳ�����ΪPWM�����������Ҫ���õ������
			if(lastdaypoint != 0xff)                 
			{
				lastdaytime = timearray[lastdaypoint];
	 
				temp_flag = 0;
				
				if((nowtime < todaytimearray[0]) || (nowtime > lastdaytime) || ((acttingnum == 0)&&(nowtime <1440)))//��ǰʱ������һʱ��
				{
					temp_flag |= 0x01;
				}

				if(((remotectrtime[m] < todaytimearray[0]) || (remotectrtime[m] > lastdaytime) || ((acttingnum == 0) && (remotectrtime[m]<1440))) && (lastday_date == real_time.day))//��һ���Ʋ���������һʱ��
				{
					if(remotectrtime[m] != 0x3CC3)//��ʼֵ0xFF*60+0xFF
					{
						temp_flag |= 0x02;
					}
				}

				if(((temp_flag & 0x01) == 0x01) && ((temp_flag & 0x02) != 0x02)) //��ǰʱ�����賿�Σ���ң��ʱ���ڷ��賿��
				{
					//activepackage = lastdaypoint + 1;
					array[0] =  slcal_localctrl.local_package_params[lastdaypoint][0];
					array[1] =  slcal_localctrl.local_package_params[lastdaypoint][4];          
					array[2] =  slcal_localctrl.local_package_params[lastdaypoint][5]; 
					instant_ctrl_setting(array,m);
				} 			
			}
			/*else
			{
				lastdaytime = 0x3CC3;
			}*/					
		}
		else//������ʹ�ܵ��ҵ�ǰʱ��λ�ڽ��յ�һ��ʹ�ܵ��
		{
			for(k = 1; k < acttingnum; k++)
			{
				if(nowtime < todaytimearray[k]) 
					break;
			}

			for(j = 0; j < acttingnum; j++)
			{
				if(remotectrtime[m] < todaytimearray[j])
					break;
			}

			if((k != j) || (remotectrtime[m] == 0x3CC3)|| ((k == j)&&(lastday_date != real_time.day)))
			{
				temp = 0;

				for(j = 0; j < LOCAL_PACKAGE_MAX_NUM; j++)
				{
					if(actting_flag & BIT(j))
					{
						temp++;

						if(temp == k)          //��temp��1
						{
							if(j <= (packagenum-1))
							{
								if(((parameter[j][0]&0x0F) == 0x01)&&((parameter[j][4] & (0x01<<m)) == (0x01<<m)))//PWM���� 
								{
									array[0] =  slcal_localctrl.local_package_params[j][0];
									array[1] =  slcal_localctrl.local_package_params[j][4];          
									array[2] =  slcal_localctrl.local_package_params[j][5]; 
									instant_ctrl_setting(array,m);
								}
							}
							break;
						}
					}
				}
			}

		}

		/*if(activepackage <= packagenum)
		{
			LampLoopStatusChanged_Flag[0] = 1;
			LampLoopStatusChanged_Flag[1] = 1;
			PWMStatusChanged_Flag[pOpParams->LmpVct&0x03] = 0;
			Ctrl_LocalCtrActing(nowtime, remotectrtime, &parameter[activepackage - 1][0], activepackage, targetstatus, LOCAL_DISEN);
		}*/
	}

}
	
/*************************************************************************
 �������ƣ�	correct_ctrl_setting 
 ����˵����	���ؿ��Ʋ�����
 ���������	
 ���ز�����	
 *************************************************************************/
void correct_ctrl_setting(U16 nowtime, U16 last_ctrl_time[], U8 parameter[][LOCAL_PACKAGE_MAX_LEN],
                          U8 package_num, U16 *timearray)
{
	#ifdef SLC_PRINT_TST
	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
		"FUNC correct_ctrl_setting.\r\n");
	#endif
		
    findout_relay_enable_point(nowtime, last_ctrl_time, parameter, package_num, timearray); 
	findout_pwm_enable_point(nowtime, last_ctrl_time, parameter, package_num, timearray);
}

/*************************************************************************
 �������ƣ�	instant_ctrl_setting    
 ����˵����	��ʱ��������
 ���������	array[0]-��4bitѡ��̵�������/PWM�������
         	array[1]-��4bit��һ·��״̬/�ƻ�·ʹ��  ��4bit�ڶ�·��״̬/ռ�ձ�ʮλ
         	array[2]-��4bitƵ��(1~15) ��4bitռ�ձȸ�λ
         	i-�ڼ���· ��0-��һ��· 1-�ڶ���·��
 ���ز�����	set_result - ���ý��
 *************************************************************************/
U8 instant_ctrl_setting(U8 *array, U8 i)
{
	U8 result,set_result=0,status[MAX_LED_NUM];
	U8 tmp_freq,tmp_brightness_status;
	slcal_dim_freq_struct temp;

	#ifdef SLC_PRINT_TST
	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
		"FUNC instant_ctrl_setting.\r\n");
	#endif	

	switch(array[0] & 0x0F)
	{
	    case 0://�̵������
	  
		#ifdef SLC_PRINT_SYS
		dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
			"InstCtrl-relay.\r\n");
		#endif	
		
		status[i] = ((array[1 + (i/2)] >> ((i%2) * 4)) & 0x0F);

		if (status[i] == 0x03)
		{
		  status[i] = LAMP_OPEN;
		}
		else if (status[i] == 0x0c)
		{
		  status[i] = LAMP_CLOSE;
		}
		else if (status[i] == 0x00)
		{
		  status[i] = LAMP_DEFAULT;
		}

		#ifdef SLC_PRINT_SYS
		dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
		"set status[%d] = %x.\r\n",i,status[i]);
		#endif	

		switch(status[i])
		{
		 case LAMP_OPEN://0x33
		 case LAMP_CLOSE: //0xff
		                 /*slcal_force_ctrlң��״̬�½ṹ��*/
			slcal_force_ctrl[i].relay_status  = status[i];    //���õ�״̬
			slcal_force_ctrl[i].status=ENABLE_FORCECTRL;//ֵΪ1 ENABLE_FORCECTRL
			slcal_force_ctrl[i].last_time=0xffff;//
			slcal_force_ctrl[i].second_count=0;//

			#ifdef SLC_PRINT_SYS//���д�ӡ��
			dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
				"task 0x21 0x0020: set led %d force ctrl: %d %d %d.\r\n"
				,i, slcal_force_ctrl[i].relay_status,slcal_force_ctrl[i].brightness_status,slcal_force_ctrl[i].last_time);
			#endif

			if(status[i] == LAMP_CLOSE) //0xcc   LAMP_CLOSE                                                 
			{
				/*PWM����ָ�Ĭ��ֵ*/
				slcal_force_ctrl[i].brightness_status = 0;
			}
			else
			{
				slcal_force_ctrl[i].brightness_status = 100;
			}
			slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);//CalcCRC16��������ֽڳ������ݵ�16λCRC
			
			last_ctrl_timepoint[i] = (U16)(real_time.hour * 60 + real_time.minute); //��ǰʱ��㣨�֣�
			set_result = 1; 
			
			break;  

		  case LAMP_DEFAULT:// LAMP_DEFAULT 0x00
		  
			set_result = 1; 
			break;
				
		  default:
			set_result = 0; 
			break;
			
		}

		if(set_result)return 1;
		else	return 0;
		  				
		break;//��λ

	  case 1://PWM����
	  
	  	  #ifdef SLC_PRINT_SYS
		    dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
			  "InstCtrl-PWM.\r\n");
		  #endif	
	  		      
	      if((slc_model[0] & 0x38) == 0x30||(slc_model[0] & 0x38) == 0x28)//�ͺŷ��ϲŶ�������
		  {
		  	  tmp_freq = array[2]&0x0F;
		      tmp_brightness_status = (array[1]>>4)*10 + (array[2]>>4) ;
		      
		      //�ж�Ƶ��
		      if(!tmp_freq) return 0;
		      
			  //�ж�����
		      if(tmp_brightness_status >100) tmp_brightness_status = 100;
		      
		      #ifdef SLC_PRINT_SYS
			  dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
				  "brightness_status is %d.\r\n",tmp_brightness_status);
			  #endif
		  
		  	  //set_pwm_freq(tmp_freq);//���õ���Ƶ��
		  	  
		  	  //�洢��ǰ����Ƶ��
		  	  temp.dim_freq = tmp_freq;
		  	  temp.crc=CalcCRC16((U8 *)&temp,2);
			  result=store_data_with_back((U8 *)(&temp),4,SL_CURRENT_DIM_FREQ,SL_CURRENT_DIM_FREQ+SL_BACK_OFFSET);
			  if(result!=SLCAL_EEPROM_FINISH)
			  {
				  slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
				  #ifdef SLC_PRINT_ASSERT
				  ASSERT(0);
				  #endif
			  }
			  else
			  {
				  slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
			  }
		  	  
			  if((array[1] & (0x01<<i)) == (0x01<<i))
			  {
			  	  //�жϵ��ⷽʽ
			      if((tmp_freq%2)&&(tmp_brightness_status>=10))//Ƶ�ʵ����ҵ�����ڵ���10%������Ӧ����ʹ��
			      {
			          slcal_selfpwm[i].selfpwm_enable = 1;
			          #ifdef SLC_PRINT_SYS
					    dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						  "slcal_selfpwm[i].selfpwm_enable = 1.\r\n");
					  #endif	
			      }
			      else										   //Ƶ��˫�������С��10%(��Ϊ�صƲ���)������Ӧ�����ֹ
			      {
			      	  slcal_selfpwm[i].selfpwm_enable = 0;
					  if(tmp_brightness_status>=10)
					  {
						  if(Control_command_dispose(COMMAND_TYPE_ON,0) == 1)//�����������������ȿ���
					  	  {
					  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
						   	  slcal_ComunicationRetry.Control_retry_times = 0;
					  	  }
						  slcal_delay(15000); //���ƺ͵���֮����Ҫ�ȴ�100ms
						  if(Control_command_dispose(COMMAND_TYPE_DIMMING,tmp_brightness_status)== 1)
						  {
					  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
						   	  slcal_ComunicationRetry.Control_retry_times = 0;
					  	  }
						  slcal_ctrl[i].current_brightness_status = tmp_brightness_status;
						}
			      }
			      
		  		  if (tmp_brightness_status<10)//0-9%���� �ص�
		  		  {
					  slcal_force_ctrl[i].relay_status  = LAMP_CLOSE;    //���õ�״̬
					  slcal_force_ctrl[i].brightness_status = 0;
					  slcal_force_ctrl[i].status=ENABLE_FORCECTRL;
					  slcal_force_ctrl[i].last_time=0xffff;
					  slcal_force_ctrl[i].second_count=0;
					  slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);
				  }
				  else
				  {
					  slcal_force_ctrl[i].relay_status  = LAMP_OPEN;    //���õ�״̬
					  slcal_force_ctrl[i].brightness_status = tmp_brightness_status;
					  slcal_force_ctrl[i].status=ENABLE_FORCECTRL;
					  slcal_force_ctrl[i].last_time=0xffff;
					  slcal_force_ctrl[i].second_count=0;
					  slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);
					
					  #ifdef SLC_PRINT_SYS
					    dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						    "slcal_force_ctrl[%d].brightness_status is %d.\r\n",i,slcal_force_ctrl[i].brightness_status);
					  #endif	
				  }
				  
				  last_ctrl_timepoint[i] = (U16)(real_time.hour * 60 + real_time.minute); //��ǰʱ��㣨�֣�
				  
			  }
			  return 1; 
		  }
		  else
		  {
		      tmp_brightness_status = (array[1]>>4)*10 + (array[2]>>4) ;
		      if(tmp_brightness_status >100) tmp_brightness_status = 100;
		      
			  if((array[1] & (0x01<<i)) == (0x01<<i))
			  {
		  		  if (tmp_brightness_status<10)//0-9%���� �ص�
		  		  {
					  slcal_force_ctrl[i].relay_status  = LAMP_CLOSE;    //���õ�״̬
					  slcal_force_ctrl[i].brightness_status = 0;
					  slcal_force_ctrl[i].status=ENABLE_FORCECTRL;
					  slcal_force_ctrl[i].last_time=0xffff;
					  slcal_force_ctrl[i].second_count=0;
					  slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);
				  }
				  else
				  {
					  slcal_force_ctrl[i].relay_status  = LAMP_OPEN;    //���õ�״̬
					  slcal_force_ctrl[i].brightness_status = 100;
					  slcal_force_ctrl[i].status=ENABLE_FORCECTRL;
					  slcal_force_ctrl[i].last_time=0xffff;
					  slcal_force_ctrl[i].second_count=0;
					  slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);	
				  }
				  
				  last_ctrl_timepoint[i] = (U16)(real_time.hour * 60 + real_time.minute); //��ǰʱ��㣨�֣�
			  }

		  	  return 1;
		  }
    
	  	break;

      default: 
      	return 0;    
	  	break;
	}  
        
}

/*************************************************************************
 �������ƣ�	slcal_localctrl_process_auto
 ����˵����	·�Ʊ����Զ����Ʋ���
 ��������� 
 ���ز����� 
 *************************************************************************/
void slcal_localctrl_process_auto()
{
    U8  i, j;
    U8 array[3];
    U16 timearray[LOCAL_PACKAGE_MAX_NUM];
    U16 now_time;

    now_time = (U16)(real_time.hour * 60 + real_time.minute); //��ǰʱ��㣨�֣�
    
    if(localctrl_time_changeto_timepoint(timearray))//��������Ĳ������е�ʱ���ƫ����ת��Ϊʱ���
	{		
        if(localctrl_init_f)
		{		    
            correct_ctrl_setting(now_time,last_ctrl_timepoint,slcal_localctrl.local_package_params, slcal_localctrl.local_package_num, timearray);//������
		}
				
        for(i = 0; i < slcal_localctrl.local_package_num; i++)
        {
            if(timearray[i] >= 1440)  continue;  //����24Сʱ������

            if(now_time == timearray[i])//��ǰʱ�䴦��һ�����ؿ���ʱ��         
            {
                if(slcal_localctrl.local_package_params[i][1]&(0x0001<<real_time.week))//�ñ���ʱ�̽���ʹ��
                {
					for(j=0; j<MAX_LED_NUM; j++)
                	{
	                    if(now_time != last_ctrl_timepoint[j])//��ǰʱ�̲�����ǰһ����ʱ�̣������ز���
	                    {
	                        array[0] =  slcal_localctrl.local_package_params[i][0];
							array[1] =  slcal_localctrl.local_package_params[i][4];          
							array[2] =  slcal_localctrl.local_package_params[i][5]; 
							instant_ctrl_setting(array,j);
	                    }
	                }
                }
            }
        }
    }
    
	localctrl_init_f = 0;
}

/*************************************************************************
 �������ƣ�	change_localctrl
 ����˵����	��ѯ·�Ʊ��ؿ��Ʋ��Բ��ж��Ƿ����·�ƿ��Ƹı�
 ��������� 
 ���ز����� 
 *************************************************************************/
void change_localctrl()       
{
	static U8 err_cnt = 0;
	U8 result,buffer[3];
	U16 mode,year,crc;
	
	if(localctrl_store_f)//����Լ����ñ��ز���������д洢
    {
    	result = localctrl_new_para_save_or_sort(1);//���ؿ��Ʋ������洢(������)
    	if(result)
    	{
	    	if((localctrl_store_f&0x03)!=0x01)//���±��ز���������ʱ
            {
	    		localctrl_sort_f = 1;//��ʱ����Ҫ��������
	    	}
	    	localctrl_store_f = 0; //�洢�ɹ������ٽ��д洢
    	}
    }
	
    mode = ((U16)(((slc_model[0]) & 0x00ff) + (((slc_model[1]) & 0x00ff) << 8)));
    if(((mode & 0x0100) == 0x0100) &&(real_time.status==SLCAL_RTC_TIMING)&&((slcal_rtc_fault&0x02)!=0x02))//��ʱ�ӹ������Ѷ�ʱ�ҹ�ȥ��ʱ�ӹ��Ͼ�->���б��ؿ���			             	 
    {
        #ifdef SLC_PRINT_TST
		dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
			"FUNC change_localctrl.\r\n");
		#endif	
		
        //��ѯ��ǰ�Ƿ����ʱ�䳬��
        year = (real_time.year_high * 256 + real_time.year_low);
        if( (year < 2015) || (year > 2065) )
        {
            if(err_cnt < 5)
            {
                err_cnt++;

                if(err_cnt > 2)
                {
					slcal_rtc_fault |= 0x02; //��ʱ�Ӿ�
                }
            }
        }
        else
        {
            err_cnt = 0;
            slcal_rtc_fault &= (~0x02);      //����
        }

        if(((real_time.hour == 0) && (real_time.minute == 0) && (real_time.second == 0)) ||(localctrl_init_f == 1))  //ÿ��0����ʼ�����ؿ��ƹ���ʱ�����ճ�����ʱ�����
        {    
			update_sun_rise_set_time();
        }
        
        if(localctrl_sort_f)
	    {
	    	localctrl_new_para_save_or_sort(0);//���ؿ��Ʋ��������� (��Ҫ�õ���ȷ���ճ�����ʱ�䣬�����Ҫ�����ճ�����ʱ���ſ�������)
	    	localctrl_sort_f = 0;
	    }
	    
	    if((real_time.hour == 23) && (real_time.minute == 59) && (real_time.second == 59))//����ǰ���㸴λ���� //if(lastday_date == real_time.day)
        {
            buffer[0] = 0;
			crc=CalcCRC16(buffer,1);
			buffer[1]=(U8)(crc>>8);
			buffer[2]=(U8)(crc&0x00ff);
			result=store_data_with_back(buffer,3,SL_RESET_NUM,SL_RESET_NUM+SL_BACK_OFFSET);
			if(result==SLCAL_EEPROM_FINISH)	
			{
				//д��ram�еĵ�ֵַ
				copy_U8(buffer,slc_reset_times,3);
				slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
			}
			else
			{
				slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
			}
        }
        
        slcal_localctrl_process_auto();//���ؿ��ƾ��崦��

        lastday_date = real_time.day;

    }
    else //if((mode & 0x0100) != 0x0100)//��ʱ�ӹ���
    {
        lastday_date = 0xff;     	
    }
	
}

/*******************************************Local_Control_End**********************************************/


/*************************************************************************
 �������ƣ�	Get_Lamp_dat
 ����˵����	·�����ݻ�ȡ����
 ��������� 
           
 ���ز�����
 *************************************************************************/

U8 Get_Lamp_data(U8* lamp_data)
{
	U8 data[8];
	U8 count = 0,resv_flag = 0;
	
	data[0] = 0x02;
	data[1] = 0x00;
	data[2] = 0x43;
	data[3] = 0x00;
	data[4] = 0x01;
	data[5] = 0x00;
	data[6] = 0x03;
	data[7] = XORCheckSum(data,7);
	do
	{
		count++;
		control_uart_send(data,8);
		//sys_activate(SYS_MOD_TASK_SYS);// ιϵͳ������   
		//slcal_delay(15000); //��ʱԼ50ms�ȴ��������ظ�
		resv_flag = control_uart_recv(lamp_data,12);   
		if((resv_flag == 12) &&(lamp_data[2] == (data[2]|0x80)))
		{
			return 0;
		}
		
	}while(count < 3);
	
	return 1;
}
/*************************************************************************
 �������ƣ�	sampling_para_collect
 ����˵����	�ۼƲ����ռ�����
 ��������� 
           
 ���ز�����
 *************************************************************************/

void sampling_para_collect()
{
	U8 data[12];
	U16 temp_voltage;
	U32 temp_current,temp_activepower;
		
	
	if(Get_Lamp_data(data) == 0)
	{
		temp_voltage = data[6]&0x03;
		temp_voltage *= 256;
		temp_voltage += data[7];
		electric_para[0].voltage = temp_voltage * 100;
		temp_current = (U32)data[8] *100;
		temp_current /=60;
		electric_para[0].current = temp_current;
		temp_activepower = electric_para[0].current * electric_para[0].voltage;
		temp_activepower /=  1000;
		electric_para[0].active_power = temp_activepower;
		Temperature = data[9];
	}

}
/*************************************************************************
 �������ƣ�	Control_command_dispose
 ����˵���� ���������
 ��������� 
           
 ���ز�����
 *************************************************************************/

U8 Control_command_dispose(U8 command_type, U8 brightness)
{
	U8 data[8],resv_data[8];
	U8 count = 0,resv_flag = 0;
	
	data[0] = 0x02;
	data[1] = 0x00;
	switch(command_type)
	{
		case COMMAND_TYPE_ON:
			data[2] = 0x40;
			data[5] = 0x00;
		break;
		case COMMAND_TYPE_OFF:
			data[2] = 0x41;
			data[5] = 0x00;
		break;
		case COMMAND_TYPE_DIMMING:
			data[2] = 0x42;
			if(brightness < 40)
			{
				data[5] = 40;
			}
			else
			{
				data[5] = brightness;
			}
		break;
		default:
		break;
	}
	data[3] = 0x00;
	data[4] = 0x01;
	
	data[6] = 0x03;
	
	data[7] = XORCheckSum(data,7);

	do
	{
		count++;
		control_uart_send(data,8);
		//sys_activate(SYS_MOD_TASK_SYS);// ιϵͳ������   
		//slcal_delay(15000); //��ʱԼ50ms�ȴ��������ظ�
		resv_flag = control_uart_recv(resv_data,8);   
		if((resv_flag == 8) &&(resv_data[2] == (data[2]|0x80)))
		{
			return 0;
		}
		
	}while(count < 3);
	
	return 1;
	
}
/*************************************************************************
 �������ƣ�	set_relay
 ����˵����	���ü̵���ͨ��
 ��������� U8 led_num: 0 ��һ· 1 �ڶ�·
            U8 status: 0 �̵����ж�  ��0 �̵�������
 ���ز�����
 *************************************************************************/
/*void set_relay(U8 led_num, U8 status)
{
	U32 psr;
    
    switch (led_num)
    {
        case 0:
            psr=DisableInt();
        	if(status==0)
        	{
        		GPIO.PDGPIO_OUT.all|=0x80;
        	}
        	else
        	{
        		GPIO.PDGPIO_OUT.all&=0x7f;
        	}
        	RestoreInt(psr);
            break;
        case 1:
            psr=DisableInt();
        	if(status==0)
        	{
        		GPIO.PDGPIO_OUT.all|=0x01;
        	}
        	else
        	{
        		GPIO.PDGPIO_OUT.all&=0xfe;
        	}
        	RestoreInt(psr);
            break;
        default:
            break;
    }
	
}*/

/*************************************************************************
 �������ƣ�	set_brightness
 ����˵����	·���������ú������Ϸ�ֵΪ0-100����ֵԽ������Խ�ߣ�����100��100��
 ���������	U8 led_num:0-��һ· 1-�ڶ�·
            U8 brightness:���õ�����
 ���ز�����
 *************************************************************************/
/*void set_brightness(U8 led_num, U8 brightness)
{
	if(brightness>100)
	{
		brightness=100;
	}
	switch (led_num)
	{
        case 0:
            if(brightness<100)
            {
            	OC0.OCxR1=BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*brightness;
            }
            else
            {
            	OC0.OCxR1=BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*brightness -1;
            }
            break;
        case 1:
            if(brightness<100)
            {
            	OC1.OCxR1=BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*brightness;
            }
            else
            {
            	OC1.OCxR1=BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*brightness -1;
            }
            break;
        default:
            break;
    }	
}*/

/*************************************************************************
 �������ƣ�	set_pwm_freq
 ����˵����	����Ƶ������
 ���������	freq-Ƶ��(��100HzΪ��λ����Χ��1~15)
 ���ز�����
 *************************************************************************/
/*void set_pwm_freq(U8 freq)
{
	U16 oc0_current_value,oc1_current_value;
	U16 current_brightness_unit;

	oc0_current_value = OC0.OCxR1;				//������ǰOC0ֵ
	oc1_current_value = OC1.OCxR1;   			//������ǰOC1ֵ
	current_brightness_unit = slcal_dim.brightness_unit;   //��ǰ���ȵ�λֵ

	//PIT3.PCSR.all &=0xfcfe;		    //��ֹtimer2������Ԥ��Ƶ
	
	if(freq == 1)
		PIT3.PCSR.all = 0x0412;	//16��Ƶ
	else if (freq <11)				
		PIT3.PCSR.all = 0x0312;	//8��Ƶ
	else
		PIT3.PCSR.all = 0x0212;	//4��Ƶ
	
    PIT3.PMR = PWR_NUMBER[freq-1];	//���Ԥ����ֵ
    
	PIT3.PCSR.all |= 0x0001;        //ʹ��timer2
	
	slcal_dim.brightness_unit = BRIGHTNESS_UNIT[freq-1]; //�����ȵ�λֵ
    slcal_dim.dim_interval = freq*2;					 //�ۼƵ�20ms���н�����������Ҫ�����¼�����(Timer2����)��
	
	#ifdef SLC_PRINT_SYS
	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
		"slcal_dim.brightness_unit = %d,\r\n",slcal_dim.brightness_unit);
	#endif
	
	if(oc0_current_value == (BRIGHTNESS_ORIGIN + current_brightness_unit*100 -1))
	{
		OC0.OCxR1 = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*100 -1;
	}
	else
	{
		OC0.OCxR1 = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*(oc0_current_value/current_brightness_unit); //���Ȳ���ǰ���£�����OC0.OCxR1ֵ
	}
	
	if(oc1_current_value == (BRIGHTNESS_ORIGIN + current_brightness_unit*100 -1))
	{
		OC1.OCxR1 = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*100 -1;
	}
	else
	{
		OC1.OCxR1 = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*(oc1_current_value/current_brightness_unit); //���Ȳ���ǰ���£�����OC1.OCxR1ֵ
	}
	
	#ifdef SLC_PRINT_SYS
	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
		"set target OC0 value: %d\r\n",OC0.OCxR1);
	#endif
	
	#ifdef SLC_PRINT_SYS
	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
		"set target OC1 value: %d\r\n",OC1.OCxR1);
	#endif
}*/


/*************************************************************************
 �������ƣ�	ISR_OC0
 ����˵����	PWM�����жϺ�����ʵ���Խ���ʱ��Ϊ�����ڽ��е��⽥�䣬ÿ�ε���ʱ
 			������Ҫ�������������ȣ����뵱ǰ���ȱȽϣ���һ�µ���һ�����ȵ�λ
 			�ƽ�����ֵ
 ���������
 ���ز�����
 *************************************************************************/
#pragma interrupt on
void ISR_OC0()
{
	//static U32 OCint_count=0;
	//U16 current_value,terminal_value;

	OC0.OCxCON.all&=0xffef;	//���жϱ�־
	
	/*OCint_count++;

	if((OCint_count>=slcal_dim.dim_interval)&&((!slcal_selfpwm[0].selfpwm_enable)||(slcal_selfpwm[0].selfpwm_execution)))//������Ӧ����Ŀǰ�̶�20ms��һ����Ԫ���䡢����Ӧ����ÿ2sΪһ���ڵ����ɵ�Ԫ����
	{
		OCint_count=0;	//�����ж��ۼƴ���

		current_value=OC0.OCxR1;	//������ǰOCֵ

		if(slcal_ctrl[0].relay_status==LAMP_CLOSE)	//��������ѹرգ�������ֱ�ӵ���0
		{	
			if(current_value!=BRIGHTNESS_ORIGIN)
			{
				OC0.OCxR1=BRIGHTNESS_ORIGIN;
			}
			slcal_dim_signal[0]=DIM_FINISH;
			return;
		}
		else if(!slcal_selfpwm[0].selfpwm_enable) //���㵱ǰOCֵ���趨ֵ�Ĳ�࣬�в�����һ�����ȵ�λ�ӽ�
		{
		    if(slcal_ctrl[0].brightness_status<100)
		    {
		    	terminal_value = BRIGHTNESS_ORIGIN+slcal_dim.brightness_unit*slcal_ctrl[0].brightness_status;
		    }
		    else
		    {
		    	terminal_value = BRIGHTNESS_ORIGIN+slcal_dim.brightness_unit*100 -1;
		    }
			
			if(current_value==terminal_value)
			{
				slcal_dim_signal[0]=DIM_FINISH;	//��ɵ��⣬�����־�����Խ����Լ�
				return;
			}
			else if(current_value>terminal_value)
			{
			    if(current_value == (BRIGHTNESS_ORIGIN+slcal_dim.brightness_unit*100 -1))
			    {
					OC0.OCxR1=current_value-slcal_dim.brightness_unit+1;
				}
				else
				{
					OC0.OCxR1=current_value-slcal_dim.brightness_unit;
				}
			}
			else
			{
				if(current_value == (BRIGHTNESS_ORIGIN+slcal_dim.brightness_unit*99))
			    {
					OC0.OCxR1=current_value+slcal_dim.brightness_unit-1;
				}
				else
				{
					OC0.OCxR1=current_value+slcal_dim.brightness_unit;
				}
			}
		}
		else
		{
			if(slcal_selfpwm[0].change_unit_cnt>0)
			{
			    if(current_value>(BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit))
			    {
					OC0.OCxR1=current_value-slcal_dim.brightness_unit;
					slcal_selfpwm[0].change_unit_cnt--;
					if(slcal_selfpwm[0].change_unit_cnt==0)
					{
						slcal_selfpwm[0].selfpwm_execution = 0;
					}
					#ifdef SLC_PRINT_SYS
					dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						"(>0)OC0.OCxR1=%d,change_unit_cnt[0]=%d,selfpwm_execution[0]=%d\r\n",OC0.OCxR1,slcal_selfpwm[0].change_unit_cnt,slcal_selfpwm[0].selfpwm_execution);
					#endif
				}
				else
				{
					OC0.OCxR1 = BRIGHTNESS_ORIGIN;
					slcal_selfpwm[0].change_unit_cnt = 0;
					slcal_selfpwm[0].selfpwm_execution = 0;
					#ifdef SLC_PRINT_SYS
					dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						"(>0)over-limit,OC0.OCxR1 is min_value.\r\n");
					#endif
				}
			}
			else if(slcal_selfpwm[0].change_unit_cnt<0) 
			{    
			    if(current_value<(BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*99))
			    {
					OC0.OCxR1=current_value+slcal_dim.brightness_unit;
					slcal_selfpwm[0].change_unit_cnt++;
					if(slcal_selfpwm[0].change_unit_cnt==0)
					{
						slcal_selfpwm[0].selfpwm_execution = 0;
					}
					#ifdef SLC_PRINT_SYS
					dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						"(<0)OC0.OCxR1=%d,change_unit_cnt[0]=%d,selfpwm_execution[0]=%d\r\n",OC0.OCxR1,slcal_selfpwm[0].change_unit_cnt,slcal_selfpwm[0].selfpwm_execution);
					#endif
				}
				else
				{
					OC0.OCxR1 = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*100 -1;
					slcal_selfpwm[0].change_unit_cnt = 0;
					slcal_selfpwm[0].selfpwm_execution = 0;
					#ifdef SLC_PRINT_SYS
					dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						"(<0)over-limit,OC0.OCxR1 is max_value.\r\n");
					#endif
				}
			}
			else
			{
				slcal_selfpwm[0].selfpwm_execution = 0;
			}
		}

		#ifdef SLC_PRINT_SYS
		dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
			"set OC0 value: %d\r\n",OC0.OCxR1);
		#endif
	}*/
}
#pragma interrupt off


/*************************************************************************
 �������ƣ�	ISR_OC1
 ����˵����	PWM1�����жϺ�����ʵ���Խ���ʱ��Ϊ�����ڽ��е��⽥�䣬ÿ�ε���ʱ
 			������Ҫ�������������ȣ����뵱ǰ���ȱȽϣ���һ�µ���һ�����ȵ�λ
 			�ƽ�����ֵ
 ���������
 ���ز�����
 *************************************************************************/
#pragma interrupt on
void ISR_OC1()
{
	//static U32 OCint_count=0;
	//U16 current_value,terminal_value;

	OC1.OCxCON.all&=0xffef;	//���жϱ�־
	
	/*OCint_count++;

	if((OCint_count>=slcal_dim.dim_interval)&&((!slcal_selfpwm[1].selfpwm_enable)||(slcal_selfpwm[1].selfpwm_execution)))//������Ӧ����Ŀǰ�̶�20ms��һ����Ԫ���䡢����Ӧ����ÿ2sΪһ���ڵ����ɵ�Ԫ����
	{
		OCint_count=0;	//�����ж��ۼƴ���

		current_value=OC1.OCxR1;	//������ǰOCֵ

		if(slcal_ctrl[1].relay_status==LAMP_CLOSE)	//��������ѹرգ�������ֱ�ӵ���0
		{	
			if(current_value!=BRIGHTNESS_ORIGIN)
			{
				OC1.OCxR1=BRIGHTNESS_ORIGIN;
			}
			slcal_dim_signal[1]=DIM_FINISH;
			return;
		}
		else if(!slcal_selfpwm[1].selfpwm_enable) //���㵱ǰOCֵ���趨ֵ�Ĳ�࣬�в�����һ�����ȵ�λ�ӽ�
		{
	        if(slcal_ctrl[1].brightness_status<100)
	        {
				terminal_value = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*slcal_ctrl[1].brightness_status;
			}
			else
			{
				terminal_value = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*100 -1;
			}
			
			if(current_value==terminal_value)
			{
				slcal_dim_signal[1]=DIM_FINISH;	//��ɵ��⣬�����־�����Խ����Լ�
				return;
			}
			else if(current_value>terminal_value)
			{
				if(current_value == (BRIGHTNESS_ORIGIN+slcal_dim.brightness_unit*100 -1))
				{
					OC1.OCxR1=current_value-slcal_dim.brightness_unit+1;
				}
				else
				{
					OC1.OCxR1=current_value-slcal_dim.brightness_unit;
				}
			}
			else
			{
				if(current_value == (BRIGHTNESS_ORIGIN+slcal_dim.brightness_unit*99))
				{
					OC1.OCxR1=current_value+slcal_dim.brightness_unit-1;
				}
				else
				{
					OC1.OCxR1=current_value+slcal_dim.brightness_unit;
				}
			}
		}
		else
		{
			if(slcal_selfpwm[1].change_unit_cnt>0)
			{
			    if(current_value>(BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit))
			    {
					OC1.OCxR1=current_value-slcal_dim.brightness_unit;
					slcal_selfpwm[1].change_unit_cnt--;
					if(slcal_selfpwm[1].change_unit_cnt==0)
					{
						slcal_selfpwm[1].selfpwm_execution = 0;
					}
					#ifdef SLC_PRINT_SYS
					dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						"(>0)OC1.OCxR1=%d,change_unit_cnt[1]=%d,selfpwm_execution[1]=%d\r\n",OC1.OCxR1,slcal_selfpwm[1].change_unit_cnt,slcal_selfpwm[1].selfpwm_execution);
					#endif
				}
				else
				{
					OC1.OCxR1 = BRIGHTNESS_ORIGIN;
					slcal_selfpwm[1].change_unit_cnt = 0;
					slcal_selfpwm[1].selfpwm_execution = 0;
					#ifdef SLC_PRINT_SYS
					dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						"(>0)over-limit,OC1.OCxR1 is min_value.\r\n");
					#endif
				}
			}
			else if(slcal_selfpwm[1].change_unit_cnt<0) 
			{    		
			    if(current_value<(BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*99))
			    {
					OC1.OCxR1=current_value+slcal_dim.brightness_unit;
					slcal_selfpwm[1].change_unit_cnt++;
					if(slcal_selfpwm[1].change_unit_cnt==0)
					{
						slcal_selfpwm[1].selfpwm_execution = 0;
					}
					#ifdef SLC_PRINT_SYS
					dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						"(<0)OC1.OCxR1=%d,change_unit_cnt[1]=%d,selfpwm_execution[1]=%d\r\n",OC1.OCxR1,slcal_selfpwm[1].change_unit_cnt,slcal_selfpwm[1].selfpwm_execution);
					#endif
				}
				else
				{
					OC1.OCxR1 = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*100 -1;
					slcal_selfpwm[1].change_unit_cnt = 0;
					slcal_selfpwm[1].selfpwm_execution = 0;
					#ifdef SLC_PRINT_SYS
					dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						"(<0)over-limit,OC1.OCxR1 is max_value.\r\n");
					#endif
				}
			}
			else
			{
				slcal_selfpwm[1].selfpwm_execution = 0;
			}
		}

		#ifdef SLC_PRINT_SYS
		dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
			"set OC1 value: %d\r\n",OC1.OCxR1);
		#endif
	}*/
}
#pragma interrupt off

/*************************************************************************
 �������ƣ�	force_ctrl_timing
 ����˵����	ǿ�ƿ��Ƽ�ʱ��������ǿ�ƿ���״̬ʱ�����㻹���ڱ�״̬��ʱ�䣬
            ʱ�䵽�����ǿ�ƿ���
 ���������
 ���ز�����
 *************************************************************************/
/*void force_ctrl_timing()
{
    U8 i;
    for (i = 0; i < MAX_LED_NUM; i++)
    {
        if(slcal_force_ctrl[i].status==ENABLE_FORCECTRL)	//����ǿ�ƿ���̬ʱ����
    	{
    		if(slcal_force_ctrl[i].last_time==0)	//����ʱ��Ϊ0��ֱ�ӽ���
    		{
    			#ifdef SLC_PRINT_ASSERT
    			ASSERT(0);
    			#endif
    			slcal_force_ctrl[i].status=DISABLE_FORCECTRL;
    		}
    		else if(slcal_force_ctrl[i].last_time==0xffff)
    		{
    			return;
    		}
    		else
    		{
    			slcal_force_ctrl[i].second_count++;
    			if(slcal_force_ctrl[i].second_count>59)	//���ʱ��һ���ӣ�������ʱ���һ����
    			{
    				slcal_force_ctrl[i].second_count=0;
    				slcal_force_ctrl[i].last_time--;
    			}
    			if(slcal_force_ctrl[i].last_time==0)	//��ѯ����ʱ���Ƿ��Ѿ�����
    			{
    				slcal_force_ctrl[i].status=DISABLE_FORCECTRL;
    			}
    		}
    		slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);	//���¼���crc
    	}
    }
}*/

/*************************************************************************
 �������ƣ�	change_ctrl
 ����˵����	��ѯ·�ƿ��Ʋ��Բ��ж��Ƿ����·�ƿ��Ƹı�
 ���������
 ���ز�����
 *************************************************************************/
void change_ctrl()
{
	U8 j,result;
	U8 relay_status,brightness_status,led_ctrl_type;
	U16 crc1, crc2;
	U32 change_signal=0;//i,strategy_index=0xff,
	static U8 last_selfpwm_enable_status[MAX_LED_NUM]={0};
	U8 flag = 0;
	U8 second;
	//U8 rtc_buffer[7];
	//U32 minute1,minute2;

    for (j = 0; j < MAX_LED_NUM; j++)
    {
        //strategy_index=0xff;//��ʼ������
       // change_signal=0;
        if(slcal_force_ctrl[j].status==ENABLE_FORCECTRL && slcal_force_ctrl[j].last_time!=0)	//��ǿ�ƿ�����Ч�һ��ڳ�������ǿ�ƿ��ƽ���
    	{
    		relay_status=slcal_force_ctrl[j].relay_status;
    		brightness_status=slcal_force_ctrl[j].brightness_status;
    		led_ctrl_type=SLCAL_CTRL_FORCE;
    	}
    	else																				    //���Ȳ���ǿ�ƿ���״̬��Ҳδ�Թ�ʱ����ȱʡ���ƽ���
    	{
    		relay_status=slcal_default_ctrl[j].relay_status;
    		brightness_status=slcal_default_ctrl[j].brightness_status;
    		led_ctrl_type=SLCAL_CTRL_DEFAULT;
    	}
    	
    	#if 0
    	//��rtc�Ѷ�ʱ���������Ϊ�գ����������ƽ���
    	else if(real_time.status==SLCAL_RTC_TIMING && slcal_strategy_collection[j].strategy_num!=0)	
    	{
    		led_ctrl_type=SLCAL_CTRL_STRATEGY;
    		get_rtc_value(rtc_buffer);	//��ȡ��ǰʱ��
    		minute1=rtc_buffer[2]*60+rtc_buffer[1];            
    		for(i=0;i<slcal_strategy_collection[j].strategy_num;i++)	//��ѯ��ǰʱ�������ĸ�������
    		{
    			minute2=slcal_strategy_collection[j].strategy[i].hour*60+slcal_strategy_collection[j].strategy[i].minute;
    			if(minute1<minute2)	//���㵱ǰʱ��С���������Ե��л�ʱ��
    			{
    				strategy_index=i;
    				break;
    			}
    		}
    		if(strategy_index!=0)	//��strategy_index��Ϊ0ʱ����ʾ��ǰʱ���ж�Ӧ����
    		{
    			if(strategy_index==0xff)	//��Ϊ��ֵ�����ʾ��ǰʱ�̴������һ�����ԵĿ�ʼʱ�䣬���������һ������
    			{
    				strategy_index=slcal_strategy_collection[j].strategy_num-1;
    			}
    			else
    			{
    				strategy_index--;
    			}
    			relay_status=slcal_strategy_collection[j].strategy[strategy_index].relay_status;
    			brightness_status=slcal_strategy_collection[j].strategy[strategy_index].brightness_status;
                //Ϊ���⵱ǰ�в��Զ��������µĲ��Ե��µ�ʱ��մ�, ÿ�θ��µ�ǰ����ʱͬʱ����Ĭ�Ͽ��� 20151125 BEGIN
                slcal_default_ctrl[j].relay_status=relay_status;
                slcal_default_ctrl[j].brightness_status=brightness_status;
                slcal_default_ctrl[j].crc=CalcCRC16((U8 *)&slcal_default_ctrl[j],2);
                //Ϊ���⵱ǰ�в��Զ��������µĲ��Ե��µ�ʱ��մ�, ÿ�θ��µ�ǰ����ʱͬʱ����Ĭ�Ͽ��� 20151125 END
    		}
    		else	//��ǰʱ���ڵ�һ�����Կ�ʼ֮ǰ�����޲��Զ�Ӧ��ʹ��ȱʡ����
    		{
    			relay_status=slcal_default_ctrl[j].relay_status;
    			brightness_status=slcal_default_ctrl[j].brightness_status;
    			led_ctrl_type=SLCAL_CTRL_DEFAULT;
    		}
    	}
    	#endif
    	
    	//if(relay_status==LAMP_CLOSE)	//������ƹرգ�������ǿ����Ϊ0
    	//{
    	//	brightness_status=0;
    	//}
    	
    	/*if (relay_status != 0 &&threshold[j].u_error_signal_u)
        {//��ǰΪ�򿪼̵����ҹ�ѹ �޶���
            ;
        }
        else
        {                
            	      		
        }*/
        
   	  // second=real_time.second;	//ȡ����ǰ��ֵ
    	
    	//if(relay_status!=slcal_ctrl[j].relay_status)										//��ѯ·�ƿ����Ƿ�Ҫ�����仯
    	//{	

			 if (relay_status == LAMP_CLOSE)
	    	{
	    		if(Control_command_dispose(COMMAND_TYPE_OFF,0)== 1)//set_relay(j, 0); //�ص�
	    		{
			  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
				   	  slcal_ComunicationRetry.Control_retry_times = 0;
					//  flag ++;
				}
	    	}  
	    	else
	    	{
	    		if(Control_command_dispose(COMMAND_TYPE_ON,0) == 1)//set_relay(j, 1); //���� 
	    		{
			  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
				   	  slcal_ComunicationRetry.Control_retry_times = 0;
					 // flag++;
				}
				slcal_delay(15000); //���ƺ͵���֮����Ҫ�ȴ�100ms
				if(Control_command_dispose(COMMAND_TYPE_DIMMING,brightness_status)== 1)
				{
			  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
				   	  slcal_ComunicationRetry.Control_retry_times = 0;
					 // flag++;
				}
				slcal_ctrl[j].current_brightness_status = brightness_status;
	    	} 
			
           /* if(relay_status==LAMP_OPEN)	//���ݴ˴ο��صƵ����ԣ��ۼӿ��ƴ���
    		{
                crc1=CalcCRC16((U8 *)&slcal_ctrl_data[j].open_times,2);
    	        crc2=slcal_ctrl_data[j].open_crc;
                if (crc1 != crc2)
                {
                    slcal_check_signal[j]|=SLCAL_CHECK_DATA;//���߲�һ��, ����һ���Լ�
                }
                else
                {
                    slcal_ctrl_data[j].open_times++;
    			    slcal_ctrl_data[j].open_crc=CalcCRC16((U8 *)&slcal_ctrl_data[j],2);
                }
    		}*/
    		//else//״̬��Ϊ�ص�ʱ������Ӧ���ⷽʽ��ֹ
    		//{
    		//	slcal_selfpwm[j].selfpwm_enable = 0; 
    		//}
    		 
          //  change_signal++;
			//if (flag !=0){
            slcal_ctrl[j].relay_status=relay_status;  //}  		
    	//}
    	
    	if(brightness_status!=slcal_ctrl[j].brightness_status)								//��ѯ·�������Ƿ�Ҫ�����仯
    	{
    		change_signal++;//�������
			if(brightness_status > 9)//�������״̬����9
			{
	    		slcal_ctrl[j].brightness_status=brightness_status;//������״̬��ֵ���õ�
	    		slcal_dim_signal[j]=DIM_UNFINISH;//������ɱ�־
				//Control_command_dispose(COMMAND_TYPE_ON,0);//�����������������ȿ���
				//slcal_delay(15000); //���ƺ͵���֮����Ҫ�ȴ�100ms
				Control_command_dispose(COMMAND_TYPE_DIMMING,brightness_status);
			}
			else
			{
				slcal_ctrl[j].brightness_status=0;
				slcal_dim_signal[j]=DIM_UNFINISH;
				Control_command_dispose(COMMAND_TYPE_OFF,0);//С��10%�ص�
			}
    	}
    	else
    	{
    		if(last_selfpwm_enable_status[j]!=slcal_selfpwm[j].selfpwm_enable)					//��ͬ����ٷֱȵ����ⷽʽ�ı�Ҳִ�е���
	    	{
	    		slcal_dim_signal[j]=DIM_UNFINISH;
	    		last_selfpwm_enable_status[j]=slcal_selfpwm[j].selfpwm_enable;
	    	}
    	}
    	
    	if(led_ctrl_type!=slcal_ctrl_type[j])												//��ѯ����ģʽ�Ƿ����˱仯
    	{
    		//change_signal++;
    		slcal_ctrl_type[j]=led_ctrl_type;
    	}

    	//if(change_signal!=0)																//��·�ƿ��Ʋ��������仯ʱ�����¼���У����
    	//{
    		#ifdef SLC_PRINT_SYS
    		dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
    			"led %d ctrl mode change to %d, status:%d, brightness:%d.\r\n",
    			j,slcal_ctrl_type[j],relay_status,brightness_status);
    		#endif
    		
    		slcal_ctrl[j].crc=CalcCRC16((U8 *)&slcal_ctrl[j],2);
    		
    		//�洢��ǰ·�ƿ��Ʋ���
    		result=store_data_with_back((U8 *)(&slcal_ctrl[j]),4,SL_CURRENT_CTRL+4*j,SL_CURRENT_CTRL+4*j+SL_BACK_OFFSET);
    		if(result!=SLCAL_EEPROM_FINISH)
        	{
        		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
        		#ifdef SLC_PRINT_ASSERT
        		ASSERT(0);
        		#endif
        	}
        	else
        	{
        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
        	}
    	//} 
    }
}

/*************************************************************************
 �������ƣ�	self_adaption_pwm
 ����˵����	����Ӧ���⴦����
 ���������
 ���ز�����
 *************************************************************************/
void pwm_self_adaption()
{
	U8 i;
	S32 power_distance[MAX_LED_NUM] = {0}; 
	static S32 record_power_distance[MAX_LED_NUM][2] = {0, 0};
	static U8 dim_speed[MAX_LED_NUM]= {FAST_MODE};
	static U8 end_adaption_flag[MAX_LED_NUM] = {0};
	
	#ifdef SLC_PRINT_SYS
	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
	  "FUNC pwm_self_adaption(selfpwm_per_2s).\r\n");
	#endif	
	
	for(i=0;i<MAX_LED_NUM;i++)
	{
		if((slcal_ctrl[i].relay_status==LAMP_OPEN)&&(slcal_selfpwm[i].selfpwm_enable))
		{
            if (slcal_dim_signal[i]== DIM_UNFINISH) 
	        {                    
				power_distance[i] = (S32)((electric_para[i].active_power * 10)/slcal_rated_power[i].rated_power - slcal_ctrl[i].brightness_status);    
				
				#ifdef SLC_PRINT_SYS
				dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
					"power_distance[%d]= %d,active_power[%d]=%d,rated_power[%d]=%d,brightness_status[%d]=%d\r\n",i,power_distance[i],i,electric_para[i].active_power,i,slcal_rated_power[i].rated_power,i,slcal_ctrl[i].brightness_status);
				#endif
																			
			    if((power_distance[i]<=-2)||(power_distance[i]>=2))//������ƫ����Χ������2%,��������Ӧ����
			    {
		    		record_power_distance[i][1] = record_power_distance[i][0];                                   
					record_power_distance[i][0] = power_distance[i];
			    
			    	if(((record_power_distance[i][1] >= 2)&&(record_power_distance[i][0] <= -2))
					|| ((record_power_distance[i][1] <= -2)&&(record_power_distance[i][0] >= 2)))
					{
						if(dim_speed[i] == FAST_MODE)			//�������ε�������������λ��Ŀ��ֵ���࣬���ÿ�ε������Ϊ1%
						{
							dim_speed[i] = SLOW_MODE;                                                                
						}
						else                          			//�����޷�����Ӧ��������Χ�ڣ�ѡȡ���Ž�������´ε��������´�ǰ���ٽ��е���
						{				
						    dim_speed[i] = STOP_MODE;										    
							slcal_dim_signal[i]= DIM_FINISH;
								
							if(record_power_distance[i][0] < 0)
							{
								record_power_distance[i][0] = -record_power_distance[i][0];
								if(record_power_distance[i][0] > record_power_distance[i][1])
								{
									slcal_selfpwm[i].change_unit_cnt = -1;
								}
							}
							else
							{
								record_power_distance[i][1] = -record_power_distance[i][1];
								if(record_power_distance[i][0] > record_power_distance[i][1])
								{
									slcal_selfpwm[i].change_unit_cnt = 1;
								}
							}
						}	
					}

		        	if(dim_speed[i] == FAST_MODE)                                                       
					{
						if(power_distance[i] <= -20) 
						{
							slcal_selfpwm[i].change_unit_cnt = -4;
						}
						else if(power_distance[i] <= -8) 
						{
							slcal_selfpwm[i].change_unit_cnt = -3;
						}
						else if(power_distance[i] <= -3) 
						{
							slcal_selfpwm[i].change_unit_cnt = -2;
						}
						else if(power_distance[i] <= -2) 
						{
							slcal_selfpwm[i].change_unit_cnt = -1;
						}
						else if(power_distance[i] < 3) 
						{
							slcal_selfpwm[i].change_unit_cnt = 1;
						}
						else if(power_distance[i] < 8) 
						{
							slcal_selfpwm[i].change_unit_cnt = 2;
						}
						else if(power_distance[i] < 20) 
						{
							slcal_selfpwm[i].change_unit_cnt = 3;
						}
						else 
						{
							slcal_selfpwm[i].change_unit_cnt = 4;
						}	
					}
					else if (dim_speed[i] == SLOW_MODE)                   
					{
						if(power_distance[i] <= -2) 
						{
							slcal_selfpwm[i].change_unit_cnt = -1;
						}
						else
						{
							slcal_selfpwm[i].change_unit_cnt = 1;
						}
													
						#ifdef SLC_PRINT_SYS
						dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						  "SLOW_MODE.\r\n");
						#endif	
					}
					else//STOP_MODE
					{
						#ifdef SLC_PRINT_SYS
						dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						  "self_pwm stops.\r\n");
						#endif	
					
						record_power_distance[i][1] = 0;//����ʼֵ��׼����һ�ε������
						record_power_distance[i][0] = 0;
						dim_speed[i] = FAST_MODE;	
					}
					
					if(slcal_selfpwm[i].change_unit_cnt != 0)//��Ҫ���⣬ִ�б�־��λ
					{
						slcal_selfpwm[i].selfpwm_execution = 1;
						
					}
					if(Control_command_dispose(COMMAND_TYPE_ON,0) == 1)//�����������������ȿ���
					{
			  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
				   	  slcal_ComunicationRetry.Control_retry_times = 0;
			  	    }
					slcal_delay(15000); //���ƺ͵���֮����Ҫ�ȴ�100ms
					slcal_ctrl[i].current_brightness_status -= slcal_selfpwm[i].change_unit_cnt;
					if(slcal_ctrl[i].current_brightness_status <40)
					{
						slcal_ctrl[i].current_brightness_status = 40;
						end_adaption_flag[i]++;
					}
					else if(slcal_ctrl[i].current_brightness_status > 100)
					{
						slcal_ctrl[i].current_brightness_status = 100;
						end_adaption_flag[i]++;
					}

					if(end_adaption_flag[i] > 4)
					{
						end_adaption_flag[i] = 0;
						slcal_dim_signal[i]= DIM_FINISH;
						record_power_distance[i][1] = 0;//����ʼֵ��׼����һ�ε������
						record_power_distance[i][0] = 0;				
						dim_speed[i] = FAST_MODE;	
					}
					if(Control_command_dispose(COMMAND_TYPE_DIMMING,slcal_ctrl[i].current_brightness_status)== 1)
					{
				  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
					   	  slcal_ComunicationRetry.Control_retry_times = 0;
					}
					
					#ifdef SLC_PRINT_SYS
					dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
					  "selfpwm_executionn[%d] is %d,change_unit_cnt[%d] is %d.\r\n",i,slcal_selfpwm[i].selfpwm_execution,i,slcal_selfpwm[i].change_unit_cnt);
					#endif	
					
		    	}
		    	else
		    	{
		    		record_power_distance[i][1] = 0;//����ʼֵ��׼����һ�ε������
					record_power_distance[i][0] = 0;				
					dim_speed[i] = FAST_MODE;
					slcal_dim_signal[i]= DIM_FINISH;
		    	}	
			}
		}
		else
		{
			record_power_distance[i][1] = 0;//����ʼֵ��׼����һ�ε������
			record_power_distance[i][0] = 0;
			dim_speed[i] = FAST_MODE;
		}
	}
}

/*************************************************************************
 �������ƣ�	add_duration
 ����˵����	����ʱ���ʹ���ص�ʱ����ʱ������1s����һ�Σ����ݵ�ǰ�̵�����״̬
 			�ۼӲ�ͬ�ļ�ʱ
 ���������
 ���ز�����
 *************************************************************************/
void add_duration()
{
    U8 i;
    U16 crc1, crc2;
    
    for (i = 0; i < MAX_LED_NUM; i++)
    {
        switch (i)
        {
            case 0:
                if(((meter_status == METER_WELL)&&(electric_para[i].current == 0))				//����оƬ������������ǰΪ0���صƻ�����ص�״̬
                  ||((meter_status != METER_WELL)&&((GPIO.PDGPIO_OUT.all&0b10000000)!=0))) 	    //����оƬ�쳣���Լ̵���״̬Ϊ׼���ص�״̬��2017/11/9�� 
                {
                    crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
    	            crc2=slcal_ctrl_data[i].close_crc;
                    if(crc1!=crc2)
                    {
                        slcal_check_signal[i]|=SLCAL_CHECK_DATA;//���߲�һ��, ����һ���Լ�
                    }
                    else
                    {
                        slcal_ctrl_data[i].close_duration++;
    		            slcal_ctrl_data[i].close_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
                    }
                }
                else if (((meter_status == METER_WELL)&&(electric_para[i].current > 0))			//����оƬ������������ǰ>0�����ƻ���������״̬
                        ||((meter_status!=METER_WELL)&&((GPIO.PDGPIO_OUT.all&0b10000000)==0)))	//����оƬ�쳣���Լ̵���״̬Ϊ׼������״̬ 
                {
                    crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_duration,4);
    	            crc2=slcal_ctrl_data[i].duration_crc;
                    if(crc1!=crc2)
                    {
                        slcal_check_signal[i]|=SLCAL_CHECK_DATA;//���߲�һ��, ����һ���Լ�
                    }
                    else
                    {
                        slcal_ctrl_data[i].open_duration++;
    		            slcal_ctrl_data[i].duration_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_duration,4);
                    }
                }
                break;
            case 1:
                if(((meter_status == METER_WELL)&&(electric_para[i].current == 0))					//����оƬ������������ǰΪ0���صƻ�����ص�״̬
                  ||((meter_status!=METER_WELL)&&((GPIO.PDGPIO_OUT.all&0b00000001)!=0)))			//����оƬ�쳣���Լ̵���״̬Ϊ׼���ص�״̬  
                {
                    crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
    	            crc2=slcal_ctrl_data[i].close_crc;
                    if(crc1!=crc2)
                    {
                        slcal_check_signal[i]|=SLCAL_CHECK_DATA;//���߲�һ��, ����һ���Լ�
                    }
                    else
                    {
                        slcal_ctrl_data[i].close_duration++;
    		            slcal_ctrl_data[i].close_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
                    }
                }
                else if (((meter_status == METER_WELL)&&(electric_para[i].current > 0))				//������ǰ>0 ���ƻ���������״̬
                        ||((meter_status!=METER_WELL)&&((GPIO.PDGPIO_OUT.all&0b00000001)==0)))		//����оƬ�쳣���Լ̵���״̬Ϊ׼������״̬
                {
                    crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_duration,4);
    	            crc2=slcal_ctrl_data[i].duration_crc;
                    if(crc1!=crc2)
                    {
                        slcal_check_signal[i]|=SLCAL_CHECK_DATA;//���߲�һ��, ����һ���Լ�
                    }
                    else
                    {
                        slcal_ctrl_data[i].open_duration++;
    		            slcal_ctrl_data[i].duration_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_duration,4);
                    }
                }
                break;
            default:
                break;
        }
    }
}

/*************************************************************************
 �������ƣ�	store_open_times
 ����˵����	���ƴ����洢������ƽʱ15min����һ�Σ���ram�еĿ��ƴ�����eerpom
 			�еĲ�ͬʱ����ram�еĿ��ƴ���д��eeprom��
 ���������
 ���ز�����	0:�洢�ɹ�	1:ʧ��
 *************************************************************************/
#if 0
U8 store_open_times()
{
	U8 result,i;
    U16 crc1, crc2;

    for (i = 0; i < MAX_LED_NUM; i++)
    {
        crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_times,2);
    	crc2=slcal_ctrl_data[i].open_crc;
        if (crc1 != crc2)
        {
            slcal_check_signal[i]|=SLCAL_CHECK_DATA;//���߲�һ��, ����һ���Լ�
            continue;
        }
        else
        {
            //�жϿ��ƴ������eeprom�ڵ������Ƿ����˱仯
        	if(slcal_ctrl_data[i].open_times==eeprom_slcal_ctrl_data[i].open_times)
        	{
        		continue;
        	}

        	//�������仯ʱ��Ҫ����eeprom����
        	result=store_data_with_back((U8 *)&slcal_ctrl_data[i],4,SL_OPEN_TIMES+6*i,SL_OPEN_TIMES+6*i+SL_BACK_OFFSET);
        	if(result!=SLCAL_EEPROM_FINISH)
        	{
        		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
        		#ifdef SLC_PRINT_ASSERT
        		ASSERT(0);
        		#endif
        		return 1;
        	}
        	else
        	{
        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
        	}

        	//debug��ָʾeepromд�����ۼ�����
        	#ifdef SLC_PRINT_SYS
        	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
        		"store led %d open times:%d.\r\n", i, slcal_ctrl_data[i].open_times);
        	#endif
        	
        	//����ram�е�eeprom���ݱ���
        	eeprom_slcal_ctrl_data[i].open_times=slcal_ctrl_data[i].open_times;
        	eeprom_slcal_ctrl_data[i].open_crc=slcal_ctrl_data[i].open_crc;
        }
    }
	return 0;
}
#endif
/*************************************************************************
 �������ƣ�	store_close_duration
 ����˵����	����ص�ʱ���洢������ƽʱ15min����һ�Σ���ram�еĿ��ƴ�����eerpom
 			�еĲ�ͬʱ����ram�еĿ��ƴ���д��eeprom��
 ���������
 ���ز�����	0:�洢�ɹ�	1:ʧ��
 *************************************************************************/
#if 0
U8 store_close_duration()
{
	U8 result, i;
    U16 crc1, crc2;
    
    for (i = 0; i < MAX_LED_NUM; i++)
    {
        crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
    	crc2=slcal_ctrl_data[i].close_crc;
        if(crc1!=crc2)
        {
            slcal_check_signal[i]|=SLCAL_CHECK_DATA;//���߲�һ��, ����һ���Լ�
            continue;
        }
        else
        {
            //�жϿ��ƴ������eeprom�ڵ������Ƿ����˱仯
        	if((slcal_ctrl_data[i].close_duration/60)==(eeprom_slcal_ctrl_data[i].close_duration/60))
        	{
        		continue;
        	}

        	//�������仯ʱ��Ҫ����eeprom����
        	result=store_data_with_back((U8 *)&slcal_ctrl_data[i].close_duration,6,SL_CLOSE_DURATION+6*i,SL_CLOSE_DURATION+6*i+SL_BACK_OFFSET);
        	if(result!=SLCAL_EEPROM_FINISH)
        	{
        		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
        		#ifdef SLC_PRINT_ASSERT
        		ASSERT(0);
        		#endif
        		return 1;
        	}
        	else
        	{
        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
        	}

        	//debug��ָʾeepromд�����ۼ�����
        	#ifdef SLC_PRINT_SYS
        	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
        		"store led %d close duration:%d.\r\n", i, slcal_ctrl_data[i].close_duration);
        	#endif
        	
        	//����eeprom�е�����
        	eeprom_slcal_ctrl_data[i].close_duration=slcal_ctrl_data[i].close_duration;
        	eeprom_slcal_ctrl_data[i].close_crc=slcal_ctrl_data[i].close_crc;
        }
    }
    
	return 0;
}
#endif
/*************************************************************************
 �������ƣ�	store_open_duration
 ����˵����	���翪��ʱ���洢������ƽʱ15min����һ�Σ���ram�еĿ��ƴ�����eeprom
 			�еĲ�ͬʱ����ram�еĿ��ƴ���д��eeprom��
 ���������
 ���ز�����	0:�洢�ɹ�	1:ʧ��
 *************************************************************************/
U8 store_open_duration()
{
	U8 result,i;
    U16 crc1, crc2;
    
    for (i = 0; i < MAX_LED_NUM; i++)
    {
        crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_duration,4);
    	crc2=slcal_ctrl_data[i].duration_crc;
        if(crc1!=crc2)
        {
            slcal_check_signal[i]|=SLCAL_CHECK_DATA;//���߲�һ��, ����һ���Լ�
            //continue;
        }
        else
        {
            //�жϿ��ƴ������eeprom�ڵ������Ƿ����˱仯
        	if((slcal_ctrl_data[i].open_duration)!=(eeprom_slcal_ctrl_data[i].open_duration))
        	{
        		//�������仯ʱ��Ҫ����eeprom����
	        	result=store_data_with_back((U8 *)&slcal_ctrl_data[i].open_duration,6,SL_OPEN_DURATION+12*next_duration_addr[i]+6*i,SL_OPEN_DURATION+12*next_duration_addr[i]+6*i+SL_BACK_OFFSET);
	        	if(result!=SLCAL_EEPROM_FINISH)
	        	{
					slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
					slcal_eeprom_fault_classify |= 0x80;
	        		#ifdef SLC_PRINT_ASSERT
	        		ASSERT(0);
	        		#endif
	        		return 1;
	        	}
	        	else
	        	{
	        		next_duration_addr[i]++;
	        		if(next_duration_addr[i]==BLOCK_NUM) next_duration_addr[i]=0;
	        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
	        		slcal_eeprom_fault_classify &= (~0x80);
	        	}
	        	
	        	//debug��ָʾeepromд�����ۼ�����
	        	#ifdef SLC_PRINT_SYS
	        	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
	        		"store led %d open duration:%d.\r\n", i, slcal_ctrl_data[i].open_duration);
	        	#endif
	        	
	        	//����eeprom�е�����
	        	eeprom_slcal_ctrl_data[i].open_duration=slcal_ctrl_data[i].open_duration;
	        	eeprom_slcal_ctrl_data[i].duration_crc=slcal_ctrl_data[i].duration_crc;
        	}
        }        
    }
    
	return 0;
}

/*************************************************************************
 �������ƣ�	clear_ctrl_data
 ����˵����	�����ƴ�����ʱ��������ص�ʱ�����Ϊ0������ֵд�뵽eeprom��
 ���������
 ���ز�����	0:�洢�ɹ�	1:ʧ��
 *************************************************************************/
U8 clear_ctrl_data()
{
	U8 result,i,j;
	slcal_ctrl_data_struct temp;

    for (i = 0; i < MAX_LED_NUM; i++)
    {
        //��������ۼ�ֵд��eeprom
    	temp.open_times=0;//���ƴ�������
    	temp.open_crc=CalcCRC16((U8 *)&temp,2);//
    	/*result=store_data_with_back((U8 *)&temp,4,SL_OPEN_TIMES+6*i,SL_OPEN_TIMES+6*i+SL_BACK_OFFSET);
    	if(result!=SLCAL_EEPROM_FINISH)
    	{
    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
    		#ifdef SLC_PRINT_ASSERT
    		ASSERT(0);
    		#endif
    		return 1;
    	}
	    else
	   	{
	   		slcal_eeprom_fault &= ~SLCAL_ERR_EEPROM;
	   	}*/
	   	
    	temp.close_duration=0;//�ص�ʱ����Ϊ��
    	temp.close_crc=CalcCRC16((U8 *)&temp.close_duration,4);
    	/*result=store_data_with_back((U8 *)&temp.close_duration,6,SL_CLOSE_DURATION+6*i,SL_CLOSE_DURATION+6*i+SL_BACK_OFFSET);
    	if(result!=SLCAL_EEPROM_FINISH)
    	{
    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
    		#ifdef SLC_PRINT_ASSERT
    		ASSERT(0);
    		#endif
    		return 1;
    	}
	    else
	   	{
	   		slcal_eeprom_fault &= ~SLCAL_ERR_EEPROM;
	   	}*/
	   	
	   	
    	temp.open_duration=0;//����ʱ����Ϊ��
    	temp.duration_crc=CalcCRC16((U8 *)&temp.open_duration,4);
    	
    	for(j = 0; j < BLOCK_NUM; j++)//BLOCK_NUM//�ۻ����洢block
    	{
	    	result=store_data_with_back((U8 *)&temp.open_duration,6,SL_OPEN_DURATION+12*j+6*i,SL_OPEN_DURATION+12*j+6*i+SL_BACK_OFFSET);//U8 store_data_with_back(U8 *buffer,U8 len,U32 address,U32 back_address)��eepromд����
	    	if(result!=SLCAL_EEPROM_FINISH)//SLCAL_EEPROM_FINISHΪeeprom������ѯ״̬���е�һ����������ѯ��д�ز��ɹ�
	    	{
	    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
	    		slcal_eeprom_fault_classify |= 0x80;
	    		#ifdef SLC_PRINT_ASSERT
	    		ASSERT(0);
	    		#endif
	    		
	    		return 1;
	    	}
	    	else
		   	{
		   		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
		   		slcal_eeprom_fault_classify &= (~0x80);
		   	}
		}
    	
    	//������������
    	copy_U8((U8 *)&temp,(U8 *)&slcal_ctrl_data[i],sizeof(slcal_ctrl_data_struct));
    	copy_U8((U8 *)&temp,(U8 *)&eeprom_slcal_ctrl_data[i],sizeof(slcal_ctrl_data_struct));
    	next_duration_addr[i]=0;
    	
    	//debug��ָʾ���ۼƿ�������
    	#ifdef SLC_PRINT_SYS
    	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
    		"led %d clear ctrl data.\r\n", i);
    	#endif
    }
	
	return 0;
}

/*************************************************************************
 �������ƣ�	ini_ctrl_data
 ����˵����	��ʼ���ƿ��ۼ����ݣ���eeprom�ж����ƿ�����
 ���������
 ���ز�����
 *************************************************************************/
void ini_ctrl_data()
{
	U8 result;
	U32 *ram_data=(U32 *)0x00803400;
    U8 i,j,err_duration=0;
    U16 crc1, crc2;
    slcal_ctrl_struct temp[MAX_LED_NUM];
    slcal_dim_freq_struct temp1;
    slcal_ctrl_data_struct temp2;


	slcal_ctrl[0].current_brightness_status = 100;
	slcal_ComunicationRetry.Control_retry_times = 0;
	slcal_ComunicationRetry.Control_succeed_flag = 0;
	
    for (i = 0; i < MAX_LED_NUM; i++)
    {
		
        //�������ƴ���
    	/*result=get_data_with_crc((U8 *)(&slcal_ctrl_data[i]),4,SL_OPEN_TIMES+6*i,SL_OPEN_TIMES+6*i+SL_BACK_OFFSET);
    	if(result!=SLCAL_EEPROM_FINISH)	//�����ۼ����ݶ���ʧ�ܣ�ʹ��Ĭ�ϲ���
    	{
    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
    		#ifdef SLC_PRINT_ASSERT
    		ASSERT(0);
    		#endif
    		
    		if(slcal_ctrl_data[i].open_times==0xffff)//�����ϵ�
    		{
	    		slcal_ctrl_data[i].open_times=0;
	    		slcal_ctrl_data[i].open_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_times,2);
	    		result=store_data_with_back((U8 *)(&slcal_ctrl_data[i].open_times),4,SL_OPEN_TIMES+6*i,SL_OPEN_TIMES+6*i+SL_BACK_OFFSET);
		    	if(result!=SLCAL_EEPROM_FINISH)	//�����ۼ����ݶ���ʧ�ܣ�ʹ��Ĭ�ϲ���
		    	{
		    		//slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
		    	    #ifdef SLC_PRINT_ASSERT
		    		ASSERT(0);
		    		#endif
		    	}
		    	else
	        	{
	        		slcal_eeprom_fault &= ~SLCAL_ERR_EEPROM;
	        	}
    		}
    		else
    		{
    			slcal_ctrl_data[i].open_times=0;
	    		slcal_ctrl_data[i].open_crc=0;
    		}
    		slcal_check_signal[i]|=SLCAL_CHECK_DATA;	//����һ���Լ�
    	}
    	else
    	{
    		slcal_eeprom_fault &= ~SLCAL_ERR_EEPROM;
    	}

    	//��������ص�ʱ��
    	result=get_data_with_crc((U8 *)(&slcal_ctrl_data[i].close_duration),6,SL_CLOSE_DURATION+6*i,SL_CLOSE_DURATION+6*i+SL_BACK_OFFSET);
    	if(result!=SLCAL_EEPROM_FINISH)	//�����ۼ����ݶ���ʧ�ܣ�ʹ��Ĭ�ϲ���
    	{
    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
    		#ifdef SLC_PRINT_ASSERT
    		ASSERT(0);
    		#endif
    		
    		if(slcal_ctrl_data[i].close_duration==0xffffffff)//�����ϵ�
    		{
	    		slcal_ctrl_data[i].close_duration=0;
	    		slcal_ctrl_data[i].close_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
	    		result=store_data_with_back((U8 *)(&slcal_ctrl_data[i].close_duration),6,SL_CLOSE_DURATION+6*i,SL_CLOSE_DURATION+6*i+SL_BACK_OFFSET);
		    	if(result!=SLCAL_EEPROM_FINISH)	//�����ۼ����ݶ���ʧ�ܣ�ʹ��Ĭ�ϲ���
		    	{
		    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
				    #ifdef SLC_PRINT_ASSERT
		    		ASSERT(0);
		    		#endif
		    	}
		    	else
	        	{
	        		slcal_eeprom_fault &= ~SLCAL_ERR_EEPROM;
	        	}
    		}
    		else
    		{
	    		slcal_ctrl_data[i].close_duration=0;
	    		slcal_ctrl_data[i].close_crc=0;
    		}
    		
    		slcal_check_signal[i]|=SLCAL_CHECK_DATA;	//����һ���Լ�
    	}
    	else
    	{
    		slcal_eeprom_fault &= ~SLCAL_ERR_EEPROM;
    	}*/

    	//��������ʱ��
	    slcal_ctrl_data[i].open_duration=0;//·�������ۼƲ���
		slcal_ctrl_data[i].duration_crc=0;//
    	next_duration_addr[i] = 0;
    	
    	for(j = 0; j < BLOCK_NUM; j++)//�ۻ����洢block//ÿ��block�����ۼ����ݶ�ȡ����ȡʧ��ϵͳ��λ������һ���Լ죻��ȡ�ɹ�
    	{
	    	result=get_data_with_crc((U8 *)(&temp2.open_duration),6,SL_OPEN_DURATION+12*j+6*i,SL_OPEN_DURATION+12*j+6*i+SL_BACK_OFFSET);//
	    	if(result!=SLCAL_EEPROM_FINISH)	//�����ۼ����ݶ���ʧ�ܣ�ʹ��Ĭ�ϲ���
	    	{
	    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
	    		slcal_eeprom_fault_classify |= 0x80;
	    		#ifdef SLC_PRINT_ASSERT
	    		ASSERT(0);
	    		#endif

				
	    		//ϵͳ��λ
	    		if(temp2.open_duration==0xffffffff)//���ʱ������
	    		{
	    		    err_duration++;//�����Ǹ�λ����
	    		    if(err_duration==BLOCK_NUM)//�����ϵ�ϵͳ��λ(����ÿ���ƶ���)
	    		    {
			    		slcal_ctrl_data[i].open_duration=0;
			    		slcal_ctrl_data[i].duration_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_duration,4);
			    		result=store_data_with_back((U8 *)(&slcal_ctrl_data[i].open_duration),6,SL_OPEN_DURATION+6*i,SL_OPEN_DURATION+6*i+SL_BACK_OFFSET);
				    	if(result!=SLCAL_EEPROM_FINISH)	//�������ۼ����ݶ���ʧ�ܣ�ʹ��Ĭ�ϲ���
				    	{
				    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
				    		slcal_eeprom_fault_classify |= 0x80;
				    		#ifdef SLC_PRINT_ASSERT
				    		ASSERT(0);
				    		#endif
				    	}
				    	else
			        	{
			        	    next_duration_addr[i] = 1;
			        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
			        		slcal_eeprom_fault_classify &= (~0x80);
			        	}
			        }
	    		}

	    		slcal_check_signal[i]|=SLCAL_CHECK_DATA;	//����һ���Լ�
	    	}
			
	    	else//�����ۼ����ݶ�ȡ�ɹ�
	    	{
	    	    if (temp2.open_duration >= slcal_ctrl_data[i].open_duration)
	    	    {
    	    		slcal_ctrl_data[i].open_duration = temp2.open_duration;//���ϴ���ۼ�·�����в������
		    		slcal_ctrl_data[i].duration_crc = temp2.duration_crc;
	    	    	if(j<(BLOCK_NUM-1))
	        		{
	        			next_duration_addr[i] = j+1;//
	        		}
	        		else
	        		{
	        			next_duration_addr[i] = 0;
	        		}	
	    	    }
	    		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
	    		slcal_eeprom_fault_classify &= (~0x80);
	    	}
	    }
    	
    	//ͬ����ʷ���ۼ�����
    	copy_U8((U8 *)&slcal_ctrl_data[i],(U8 *)&eeprom_slcal_ctrl_data[i],sizeof(slcal_ctrl_data_struct));
    	
    	//�жϴ˴γ�ʼ���Ƿ�Ϊ���µ縴λ
    	if(*ram_data!=0x20140408)
    	{
    		/*�ϵ縴λ*/
    		//ini_rtc();//��ʼ��rtcʱ�䣬Ϊ���ϵ��rtcһ��δ��ʱ��Ĭ��ʱ��
			//ini_timer0();//rtc�����׼(5ms)rtc���붨ʱ��׼����ʱ����Ϊ5ms
    		
            crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_times,2);//slcal_ctrl_data_struct slcal_ctrl_data[MAX_LED_NUM];	//·�������ۼƲ���//����crc
	        crc2=slcal_ctrl_data[i].open_crc;//
            if (crc1 != crc2)//crcУ��
            {
                slcal_check_signal[i]|=SLCAL_CHECK_DATA;//���߲�һ��, ����һ���Լ�
            }
            else
            {
                slcal_ctrl_data[i].open_times++;	//���ϱ����ϵ翪�ƴ���
    		    slcal_ctrl_data[i].open_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_times,2);//���¼��㵱ǰcrc
            }

            //�洢�ϵ�·�ƿ��Ʋ���
    		result=store_data_with_back((U8 *)(&slcal_ctrl[i]),4,SL_CURRENT_CTRL+4*i,SL_CURRENT_CTRL+4*i+SL_BACK_OFFSET);
    		if(result!=SLCAL_EEPROM_FINISH)//����ѯeeprom����״̬�ֲ�Ϊ2//���洢���ɹ�/*��Ȼ�������ǿ��Ե�*/
        	{
        		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;//0x02//eeprom״̬����Ϊ����
        		#ifdef SLC_PRINT_ASSERT//���Դ�ӡ��
        		ASSERT(0);
        		#endif
        	}
        	else
        	{
        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);//����ͳɹ�
        	}
        	
	   	  	//�洢�ϵ����Ƶ��
		  	temp1.dim_freq = DEFAULT_FREQ;
		  	temp1.crc=CalcCRC16((U8 *)&temp1,2);
			result=store_data_with_back((U8 *)(&temp1),4,SL_CURRENT_DIM_FREQ,SL_CURRENT_DIM_FREQ+SL_BACK_OFFSET);
		    if(result!=SLCAL_EEPROM_FINISH)//
			{
				slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
				#ifdef SLC_PRINT_ASSERT
				ASSERT(0);
				#endif
			}
			else
			{
				slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
			}   
			
			last_ctrl_timepoint[i] = 0xff * 60 +0xff; //��ʼֵ
    	}
		/*ϵͳ��λ*/
    	else
    	{
    		/*#ifdef SLC_PRINT_ASSERT
    		ASSERT(0);
    		#endif*/
    		
    		//ϵͳ��λ
    		//ini_timer0();//rtc�����׼(5ms)
    		/*дϵͳ��λǰ�ĵ�״̬*/
			
    		
    		//����ϵͳ��λǰ��״̬(���õ�ǰ��״̬)
	    	//result=get_data_with_crc((U8 *)(&temp[i]),4,SL_CURRENT_CTRL+4*i,SL_CURRENT_CTRL+4*i+SL_BACK_OFFSET);//����ϵͳ��λǰ�ĵ�״̬
	    result=get_data_with_crc((U8 *)(&temp[i]),6,SL_CURRENT_CTRL+6*i,SL_CURRENT_CTRL+6*i+SL_BACK_OFFSET);//����ϵͳ��λǰ�ĵ�״̬
	    //	result=get_data_with_crc((U8 *)(&temp[i]),8,SL_CURRENT_CTRL+8*i,SL_CURRENT_CTRL+8*i+SL_BACK_OFFSET);//����ϵͳ��λǰ�ĵ�״̬
	    	if(result!=SLCAL_EEPROM_FINISH)	//�����ۼ����ݶ���ʧ�ܣ�ʹ��Ĭ�ϲ���(�ϵ翪�ص�״̬)
	    	{
	    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
	    		#ifdef SLC_PRINT_ASSERT
	    		ASSERT(0);
	    		#endif
	    	}
	    	else
	    	{
	    	    //��ʼ���ܿ�״̬����
		    	slcal_force_ctrl[i].last_time=0xffff;
		    	slcal_force_ctrl[i].relay_status=temp[i].relay_status;
		    	slcal_force_ctrl[i].brightness_status=temp[i].brightness_status;
		    	slcal_force_ctrl[i].status=ENABLE_FORCECTRL;
		    	slcal_force_ctrl[i].second_count=0;
		    	slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);
		    	
		    	slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
	    	}
	    	
	    	//����ϵͳ��λǰ����Ƶ��
	    	result=get_data_with_crc((U8 *)(&temp1),4,SL_CURRENT_DIM_FREQ,SL_CURRENT_DIM_FREQ+SL_BACK_OFFSET);
	    	if(result!=SLCAL_EEPROM_FINISH)	//�����ۼ����ݶ���ʧ�ܣ�ʹ��Ĭ�ϲ���
	    	{
	    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
	    		#ifdef SLC_PRINT_ASSERT
	    		ASSERT(0);
	    		#endif
	    	}
	    	else
	    	{
	    		if((slc_model[0] & 0x38) == 0x30||(slc_model[0] & 0x38) == 0x28)//�ͺŷ��ϲŶ�������
			    {
			        //set_pwm_freq((U8)temp1.dim_freq);//���õ���Ƶ��
			        
    				#ifdef SLC_PRINT_SYS//���д�ӡ��
				      dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
					    "dim_freq after reset is %d.\r\n",temp1.dim_freq);
				    #endif	
			    	
			    	//�жϵ��ⷽʽ
			      /*	if((temp1.dim_freq%2)&&(slcal_force_ctrl[i].relay_status>=10))//Ƶ�ʵ����ҵ�����ڵ���10%������Ӧ����ʹ��
			      	{
			            slcal_selfpwm[i].selfpwm_enable = 1;
			      	}
			     	else										                  //Ƶ��˫�������С��10%(��Ϊ�صƲ���)������Ӧ�����ֹ
			      	{
			      	    slcal_selfpwm[i].selfpwm_enable = 0;
			      	}*/
			    }
			    
		    	slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
	    	}
	    	
	    	last_ctrl_timepoint[i] = 0xff * 60 +0xff; //��ʼֵ
    	}
    }
    
    *ram_data=0x20140408;	//��ramָ��λ�ô��ϱ�ǣ�����֮��λ�ж��Ƿ�Ϊ���µ縴λ
}

/*************************************************************************
 �������ƣ�	ini_ctrl_para
 ����˵����	��ʼ���ƿز�������eeprom�ж����ƿز���
 ���������
 ���ز�����
 *************************************************************************/
void ini_ctrl_para()
{
	U8 result,i,j,k,tmp = 1;
	U8 buffer[8];
	U16 crc;

	for (i = 0; i < MAX_LED_NUM; i++)
	{   
	    #ifdef SLC_PRINT_SYS//���д�ӡ��
	   	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
	   		"FUNC ini_ctrl_para(led %d init ctrl state).\r\n",i);
		#endif
		
		//�������ϵ�Ĭ��״̬��ʧ�ܸ�Ĭ��ֵ
		result=get_data_with_crc((U8 *)(&slcal_init_ctrl),4,SL_DEFAULT_CTRL,SL_DEFAULT_CTRL+SL_BACK_OFFSET);
		
	    if(result!=SLCAL_EEPROM_FINISH)//Ĭ�Ͽ��Ʋ�������ʧ�ܣ�ʹ��Ĭ�ϲ���
	    {		
	    	slcal_eeprom_fault |= SLCAL_ERR_EEPROM;//E2����״̬��  = eeprom�Ĵ���״̬��0x02 
	    	slcal_eeprom_fault_classify |= 0x01;//e2���Ϸ����� = 0x01
	   		#ifdef SLC_PRINT_ASSERT//���Դ�ӡ��
			ASSERT(0);
			#endif
			
	    	if (slcal_init_ctrl.init_ctrl_state == 0xffff)//�����ϵ�,U16 init_ctrl_state����λ��(����ʧ�ܳ����ϵ�������)
	    	{
				slcal_default_ctrl[i].relay_status=LAMP_OPEN;//Ĭ�Ͽ���
				slcal_default_ctrl[i].brightness_status=100;//����
				slcal_default_ctrl[i].crc=CalcCRC16((U8 *)&slcal_default_ctrl[i],2);//У��
				
	    		slcal_init_ctrl.init_ctrl_state = 0;//�Ƴ�ʼ��״̬��ֵ��ֵ0
	    		slcal_init_ctrl.crc=CalcCRC16((U8 *)&slcal_init_ctrl,2);//�Ƴ�ʼ����У��
	    		result=store_data_with_back((U8 *)(&slcal_init_ctrl),4,SL_DEFAULT_CTRL,SL_DEFAULT_CTRL+SL_BACK_OFFSET);//дȱʡ����״̬��
	        	if(result!=SLCAL_EEPROM_FINISH)
	        	{
	        		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
	        		slcal_eeprom_fault_classify |= 0x01;
	        		#ifdef SLC_PRINT_ASSERT
	        		ASSERT(0);
	        		#endif
	        	}
	        	else
	        	{
	        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
	        		slcal_eeprom_fault_classify &= (~0x01);
	        	}
	    	}
	    	else//�ǳ����ϵ�(����ʧ�ܷ��״��ϵ�������)
	    	{
	    		slcal_default_ctrl[i].relay_status=LAMP_OPEN;//Ĭ�Ͽ���
				slcal_default_ctrl[i].brightness_status=100;
				slcal_default_ctrl[i].crc=0;
	    		slcal_init_ctrl.init_ctrl_state = 0;
	    		slcal_init_ctrl.crc=0;
	    	}

			slcal_check_signal[i]|=SLCAL_CHECK_CTRL;	//����һ�ο����Լ�(����ʧ�ܵ����һ���ᴥ��һ���Լ�)
		}
		else//�����ɹ�
		{
			if(slcal_init_ctrl.init_ctrl_state & (0x0001<<i))//�����ʼ�����Ƶĵ�iλΪ����������
			{
				slcal_default_ctrl[i].relay_status=LAMP_CLOSE;//Ĭ�Ϲص�
				slcal_default_ctrl[i].brightness_status=0;
				slcal_default_ctrl[i].crc=CalcCRC16((U8 *)&slcal_default_ctrl[i],2);
			}
			else//������������
			{
				slcal_default_ctrl[i].relay_status=LAMP_OPEN;//Ĭ�Ͽ���
				slcal_default_ctrl[i].brightness_status=100;
				slcal_default_ctrl[i].crc=CalcCRC16((U8 *)&slcal_default_ctrl[i],2);
			}
			
			slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
			slcal_eeprom_fault_classify &= (~0x01);//�����ɹ�������õı�־
		}//end�����ɹ�
    	
    	//��ʼ���ܿ�״̬����
    	slcal_force_ctrl[i].last_time=0;
    	slcal_force_ctrl[i].relay_status=LAMP_OPEN;
    	slcal_force_ctrl[i].brightness_status=100;
    	slcal_force_ctrl[i].status=DISABLE_FORCECTRL;
    	slcal_force_ctrl[i].second_count=0;
    	slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);

    	//��ʼ����ǰ����״̬
    	copy_U8((U8 *)&slcal_default_ctrl[i],(U8 *)&slcal_ctrl[i],4);
    	slcal_ctrl_type[i]=SLCAL_CTRL_DEFAULT;
    	
    	//��ʼ�������(��ʼ�������ΪʲôҪ����,���϶���Ϊ�����ж���д)
    	result=get_data_with_crc((U8 *)(&slcal_rated_power[i]),4,SL_RATED_POWER+4*i,SL_RATED_POWER+4*i+SL_BACK_OFFSET);//��ʼ�������
    	if(result!=SLCAL_EEPROM_FINISH)//����ʧ�ܣ�ʹ��Ĭ�ϲ���
    	{
    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
    		slcal_eeprom_fault_classify |= 0x02;
    		#ifdef SLC_PRINT_ASSERT
    		ASSERT(0);
    		#endif
    		
    		if (slcal_rated_power[i].rated_power == 0xffff)//�����ϵ�
	    	{
    			slcal_rated_power[i].rated_power = 250;
				slcal_rated_power[i].crc = CalcCRC16((U8 *)&slcal_rated_power[i],2);
	    		result=store_data_with_back((U8 *)(&slcal_rated_power[i]),4,SL_RATED_POWER+4*i,SL_RATED_POWER+4*i+SL_BACK_OFFSET);
	        	if(result!=SLCAL_EEPROM_FINISH)
	        	{
	        		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
	        		slcal_eeprom_fault_classify |= 0x02;
	        		#ifdef SLC_PRINT_ASSERT
	        		ASSERT(0);
	        		#endif
	        	}
	        	else
	        	{
	        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
	        		slcal_eeprom_fault_classify &= (~0x02);
	        	}
	    	}
	    	else//����ʧ�ܣ��ǳ����ϵ�
	    	{
				slcal_rated_power[i].rated_power = 250;
				slcal_rated_power[i].crc = 0;
	    	}

    		slcal_check_signal[i]|=SLCAL_CHECK_CTRL;	//����һ�ο����Լ�
    	}
    	else//Ī����ֵ���ζ��ɹ�
    	{
    		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
    		slcal_eeprom_fault_classify &= (~0x02);
    	}
    	
    	//��ʼ���������
    	slcal_selfpwm[i].selfpwm_enable = 0;
    	slcal_selfpwm[i].selfpwm_execution = 0;
    	slcal_selfpwm[i].change_unit_cnt = 0;
    }
    
    
    //��ʼ����γ��
    result=get_data_with_crc((U8 *)(&slcal_longi_lati),6,SL_LONGI_LATI,SL_LONGI_LATI+SL_BACK_OFFSET);
	if(result!=SLCAL_EEPROM_FINISH)//����ʧ�ܣ�ʹ��Ĭ�ϲ���
	{
		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
		slcal_eeprom_fault_classify |= 0x10;
		#ifdef SLC_PRINT_ASSERT
		ASSERT(0);
		#endif
		
		slcal_longi_lati.longitude_int = 108;
		slcal_longi_lati.longitude_fra = 55;
		slcal_longi_lati.latitude_int = 34;
		slcal_longi_lati.latitude_fra = 15;
		
		if (slcal_longi_lati.crc == 0xffff)//�����ϵ�
    	{
			slcal_longi_lati.crc = CalcCRC16((U8 *)&slcal_longi_lati,4);
			
    		result=store_data_with_back((U8 *)(&slcal_longi_lati),6,SL_LONGI_LATI,SL_LONGI_LATI+SL_BACK_OFFSET);
        	if(result!=SLCAL_EEPROM_FINISH)
        	{
        		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
        		slcal_eeprom_fault_classify |= 0x10;
        		#ifdef SLC_PRINT_ASSERT
        		ASSERT(0);
        		#endif
        	}
        	else
        	{
        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
        		slcal_eeprom_fault_classify &= (~0x10);
        	}
    	}
    	else
    	{
			slcal_longi_lati.crc = 0;
    	}

       	for (i = 0; i < MAX_LED_NUM; i++)
       	{
			slcal_check_signal[i]|=SLCAL_CHECK_LOCAL_CTRL;	//����һ�α��ؿ����Լ�
		}
	}
	else
	{
		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
		slcal_eeprom_fault_classify &= (~0x10);
	}
    
    
    //��ʼ�����ز��� 
    for(k=0; k<LOCAL_PACKAGE_MAX_NUM; k++)
    {
		result=get_data_with_crc(buffer,LOCAL_PACKAGE_MAX_LEN+2,SL_LOCAL_CTRL_PARA+k*(LOCAL_PACKAGE_MAX_LEN+2),SL_LOCAL_CTRL_PARA+k*(LOCAL_PACKAGE_MAX_LEN+2)+SL_BACK_OFFSET);
		if(result!=SLCAL_EEPROM_FINISH)//����ʧ�ܣ�ʹ��Ĭ�ϲ���
		{
			tmp &= 0; 
		}
		else
		{
			for(j = 0; j < LOCAL_PACKAGE_MAX_LEN; j++)
			{
				slcal_localctrl.local_package_params[k][j] = buffer[j];
			}
			tmp &= 1; 
		}
		
	}
	result=get_data_with_crc(buffer,3,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2),SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+SL_BACK_OFFSET);
	if(result!=SLCAL_EEPROM_FINISH)//����ʧ�ܣ�ʹ��Ĭ�ϲ���
	{
		tmp &= 0; 
	}
	else
	{
		slcal_localctrl.local_package_num = buffer[0];
		tmp &= 1; 
	}
	result=get_data_with_crc(buffer,4,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+4,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+4+SL_BACK_OFFSET);
	if(result!=SLCAL_EEPROM_FINISH)//����ʧ�ܣ�ʹ��Ĭ�ϲ���
	{
		tmp &= 0; 
	}
	else
	{
		slcal_localctrl.crc = (((U16)buffer[0])<<8)+((U16)buffer[1]);
		tmp &= 1; 
	}
	
	if (tmp)
	{
		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
		slcal_eeprom_fault_classify &= (~0x20);
	}
	else
	{
		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
		slcal_eeprom_fault_classify |= 0x20;
		#ifdef SLC_PRINT_ASSERT
		ASSERT(0);
		#endif
		
		tmp = 1;
		
		for(k = 0; k < LOCAL_PACKAGE_MAX_NUM; k++)
		{
			for(j = 0; j < LOCAL_PACKAGE_MAX_LEN; j++)
			{
				slcal_localctrl.local_package_params[k][j]=0;
			}
		}
		slcal_localctrl.local_package_num = 0;
		
		slcal_localctrl.crc = (((U16)buffer[0])<<8)+((U16)buffer[1]);
		if (slcal_localctrl.crc == 0xffff)//�����ϵ�
	    {
			slcal_localctrl.crc = CalcCRC16((U8 *)&slcal_localctrl,194);
			
    		/*result=store_data_with_back((U8 *)(&slcal_localctrl),196,SL_LOCAL_CTRL_PARA,SL_LOCAL_CTRL_PARA+SL_BACK_OFFSET);
        	if(result!=SLCAL_EEPROM_FINISH)
        	{
        		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
        		#ifdef SLC_PRINT_ASSERT
        		ASSERT(0);
        		#endif
        	}
        	else
        	{
        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
        	}*/
        	
        	for(k=0; k<LOCAL_PACKAGE_MAX_NUM; k++)
			{
				for(j = 0; j < LOCAL_PACKAGE_MAX_LEN; j++)
				{
					buffer[j]=slcal_localctrl.local_package_params[k][j];
				}	
				crc=CalcCRC16(buffer,j);
				buffer[j]=(U8)(crc>>8);
				buffer[j+1]=(U8)(crc&0x00ff);
				result=store_data_with_back(buffer,j+2,SL_LOCAL_CTRL_PARA+k*(LOCAL_PACKAGE_MAX_LEN+2),SL_LOCAL_CTRL_PARA+k*(LOCAL_PACKAGE_MAX_LEN+2)+SL_BACK_OFFSET);
				if(result==SLCAL_EEPROM_FINISH)tmp &= 1; 
				else tmp &= 0; 
			}
			
			buffer[0]=slcal_localctrl.local_package_num;
			crc=CalcCRC16(buffer,1);
			buffer[1]=(U8)(crc>>8);
			buffer[2]=(U8)(crc&0x00ff);
			result=store_data_with_back(buffer,3,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2),SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+SL_BACK_OFFSET);
			if(result==SLCAL_EEPROM_FINISH)tmp &= 1; 
			else tmp &= 0; 
			
			buffer[0]=(U8)(slcal_localctrl.crc>>8);
			buffer[1]=(U8)(slcal_localctrl.crc&0x00ff);
			crc=CalcCRC16(buffer,2);
			buffer[2]=(U8)(crc>>8);
			buffer[3]=(U8)(crc&0x00ff);
			result=store_data_with_back(buffer,4,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+4,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+4+SL_BACK_OFFSET);
			if(result==SLCAL_EEPROM_FINISH)tmp &= 1; 
			else tmp &= 0; 
			
			if(tmp)
			{
				slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
				slcal_eeprom_fault_classify &= (~0x20);
			}
			else
			{	
				slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
				slcal_eeprom_fault_classify |= 0x20;
			}
    	}
    	else
    	{
			slcal_localctrl.crc = 0;
    	}
    	
		for (i = 0; i < MAX_LED_NUM; i++)
		{
			slcal_check_signal[i]|=SLCAL_CHECK_LOCAL_CTRL;	//����һ�α��ؿ����Լ�
		}
	}
	localctrl_store_f = 0;//���ز������洢���
	if(!slcal_localctrl.local_package_num) localctrl_sort_f = 0;
	else localctrl_sort_f = 1;//������������
	
}

/*************************************************************************
 �������ƣ�	ini_relay
 ����˵����	��ʼ���̵�������
 ���������
 ���ز�����
 *************************************************************************/
void ini_relay()
{
    PDCTRL[7].all=0x08;//��һ·�̵�������
    PDCTRL[0].all=0x08;//�ڶ�·�̵������� �޸�ΪPDO 20151022
}

/*************************************************************************
 �������ƣ�	ini_timer2
 ����˵����	��ʼ��timer2����ΪPWM�����ʱ���׼����ʼ����ʱ����2.5ms
 ���������
 ���ز�����
 *************************************************************************/
void ini_timer2()
{
	CLOCK.PITUARTCLKEN.bit.Pit3ClkEn=1;

	PIT3.PCSR.all=0x0312;	//8��Ƶ
    PIT3.PMR=17999;			//��ʱ�����޸�Ϊ2.5ms(Ĭ��400Hz)
    
    slcal_dim.brightness_unit = BRIGHTNESS_UNIT[3];
    slcal_dim.dim_interval = 8;
    
	PIT3.PCSR.all=0x0313;
}

/*************************************************************************
 �������ƣ�	ini_oc0
 ����˵����	��ʼ��ini_oc0����������PWM���β����ڲ���ռ�ձ�, ʱ��Timer�޸�ΪTimer2
 ���������
 ���ز�����
 *************************************************************************/
void ini_oc0()
{
	CLOCK.OCCLKEN.all|=0x01;

	PDCTRL[2].all=0x10;

	PLSR[15]=18;
	INT.NIER.all|=0x00040000;
	
	OC0.OCxR1=BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*100 -1;
	OC0.OCxR2=0;
	OC0.OCxCON.all=0x08cd;
}

/*************************************************************************
 �������ƣ�	ini_oc1
 ����˵����	��ʼ��ini_oc1�����������ڶ�·PWM���β����ڲ���ռ�ձ�, ʱ��TimerΪTimer2
 ���������
 ���ز�����
 *************************************************************************/
void ini_oc1()
{
	CLOCK.OCCLKEN.all|=0x02;

	PDCTRL[3].all=0x10;

	PLSR[16]=4;
	INT.NIER.all|=0x00000010;
	
	OC1.OCxR1=BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*100 -1;
	OC1.OCxR2=0;
	OC1.OCxCON.all=0x08cd;
}


/*************************************************************************
 �������ƣ�	ini_oc3
 ����˵����	��ʼ��ini_oc3�������������ص�Դ��38KHz
 ���������
 ���ز�����
 *************************************************************************/
/*void ini_oc3()
{
	CLOCK.OCCLKEN.all|=0x08;

	PCCTRL[6].all=0x18;//IO����ΪOC3���

	PLSR[18]=24;
	INT.NIER.all|=0x01000000;//��24��
	
	OC3.OCxR1=95;
	OC3.OCxR2=0;
	OC3.OCxCON.all=0x08c5;//�ص����벶���ж� 20150528
}*/

/*************************************************************************
 �������ƣ�	ini_slcal_ctrl
 ����˵����	��ʼ��·�ƿ��ƺ����������̵�����PWM����
 ���������
 ���ز�����
 *************************************************************************/
void ini_slcal_ctrl()
{
    U8 i;
    
    ini_timer2();	//������ʹ��,��ʼ����ʱ����2.5ms(400Hz)
    
	ini_oc0();		

    ini_oc1();	
    
	ini_ctrl_para();//��ʼ���ƿز���(Ĭ�Ͽ���״̬��������������ؿ�����ز���)

	ini_ctrl_data();//��ʼ���ƿ��ۻ�����(���ƴ���������ʱ�䡢����ص�ʱ�䡢�жϸ�λ����)

	//ini_relay();//ȡ���ڴ˴��ĳ�ʼ������ 20151022 -> ����:main->bsp_init->Ini_sys->Ini_Pad
	
    //ini_oc3();//���ӵڶ�·PWM����38KHz��ʼ��
    
    for(i = 0; i < MAX_LED_NUM; i++)
    {
    	if(slcal_ctrl[i].relay_status == LAMP_CLOSE)
    	{
        	if(Control_command_dispose(COMMAND_TYPE_OFF,0) == 1)//set_relay(i, 0);//�ص�
        	{
	  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
		   	  slcal_ComunicationRetry.Control_retry_times = 0;
	  	  	}
        }
        else
        {
        	if(Control_command_dispose(COMMAND_TYPE_ON,0) == 1)//set_relay(i, 1);//���� 
        	{
		  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
			   	  slcal_ComunicationRetry.Control_retry_times = 0;
		  	}
        }
    }
	
	#ifdef SLC_PRINT_SYS
	for (i = 0; i < MAX_LED_NUM; i++)
	{
	    dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
		"set led %d origin ctrl as: %d, status:%x, brightness:%d.\r\n",
		i, slcal_ctrl_type[i],slcal_ctrl[i].relay_status,slcal_ctrl[i].brightness_status);
	}
	#endif
}
