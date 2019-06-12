/*************************************************************************
  Copyright (c), 1998-2013, 上海复旦微电子集团股份有限公司

  文件名称: slcal_ctrl.c
  文件描述: 

 
  修订记录:
         1. 作者: 张熠阳
            日期: 2013-11-29
            内容: 路灯控制应用层控制代码
		 2. 作者: 梁家宝
            日期: 2015-05-27
            内容: 1.增加第二路PWM, 仅用于输出38KHz方波, 不产生中断, 时钟源选择TIMER2
                 2. OC0时钟源修改为TIMER0
		 3. 修改: 梁家宝
            日期: 2015-06-19
            内容: 中断和主循环中使用的全局变量添加volatile属性
		 4. 修改: 梁家宝
            日期: 2015-08-04
            内容: 增加第二路调光控制, 调光频率修改为1KHz
		 5. 修改：梁家宝
		    日期：2015-10-29
			内容： 1. 继电器IO口从PA7修改为PD0
			      2. 继电器IO此处初始化取消
		 6. 修改：梁家宝
		    日期：2015-11-25
			内容： 为避免当前有策略而后续有新的策略导致的时间空窗, 每次更新当前策略时同时更新默认控制
		 7. 修改：梁家宝
		    日期：2016-04-26
			内容：为支持过压断电, 单灯开灯时长和关灯时长按照实际IO口而定(目前暂无滤波)
		 8. 作者：梁家宝
		    日期：2016-04-26
			内容： 开灯次数、开灯时长、关灯时长在累加前进行校验;
			      开灯次数、开灯时长、关灯时长在存储eeprom前进行校验;
				  检查当前控制时, 已经过压且为打开继电器则无动作
				  检查当前控制时, 先直接控制继电器不再判断是否与上次一致
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
const U16 BRIGHTNESS_UNIT[15] = {360,360,240,180,144,120,102,90,80,72,130,120,110,102,96};//一个亮度单位对应的timer单位值

U8 slcal_ctrl_type[MAX_LED_NUM];						//路灯控制类型，0表示缺省控制，1表示本地控制，2表示强制控制
slcal_forcectrl_struct slcal_force_ctrl[MAX_LED_NUM];	//强控状态下的控制参数结构体
slcal_ctrl_struct slcal_default_ctrl[MAX_LED_NUM];		//缺省状态下的控制参数结构体
volatile slcal_ctrl_struct slcal_ctrl[MAX_LED_NUM];	    //当前控制状态下的控制参数记录，带crc16校验码

volatile slcal_dim_struct slcal_dim;						//调光亮度单位值，调光间隔
volatile U8 slcal_dim_signal[MAX_LED_NUM];					//调光完成标志
volatile slcal_selfpwm_struct slcal_selfpwm[MAX_LED_NUM];	//自适应调光相关参数

slcal_ctrl_data_struct slcal_ctrl_data[MAX_LED_NUM];	//路灯运行累计参数
slcal_ctrl_data_struct eeprom_slcal_ctrl_data[MAX_LED_NUM];	//eeprom内的路灯运行累计参数

slcal_init_ctrl_struct slcal_init_ctrl;                  //上电默认灯状态
slcal_rated_power_struct slcal_rated_power[MAX_LED_NUM]; //额定功率
slcal_ComunicationRetry_struct slcal_ComunicationRetry;//电子镇流器通信控制



volatile U8 next_duration_addr[MAX_LED_NUM];             

/*----------------------------------------本地控制相关参数---------------------------------------------*/

slcal_longi_lati_struct slcal_longi_lati;       //经纬度
slcal_sumtime_struct slcal_sumtime;    			//日出日落时间

slcal_localctrl_struct slcal_localctrl;			//本地控制参数
volatile U8 localctrl_init_f;          			//本地控制初始化标识（0-初始化已完成  1-需要重新初始化）
volatile U8 localctrl_store_f;         			//本地控制参数包存储标识(0x00-存储完毕 0x01-仅清空参数包，存储不需要排序 0x02-设置新本地参数包，存储且需要排序)
volatile U8 localctrl_sort_f;                   //本地控制参数包排序标识(0-排序完毕 1-需要排序)

U16 last_ctrl_timepoint[MAX_LED_NUM];  		    //最近一次控制时间点

static U8 lastday_date;				   			//前一天日期 

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
    
    sunrisetime = slcal_sumtime.sun_rise_time[0]; //计算日出日落时刻点
    sunrisetime *= 60;
    sunrisetime += slcal_sumtime.sun_rise_time[1];

    sunsettime = slcal_sumtime.sun_set_time[0];
    sunsettime *= 60;
    sunsettime += slcal_sumtime.sun_set_time[1];

	for(i = 0; i < LOCAL_PACKAGE_MAX_NUM; i++)//清零
	{
		array[i]  = 0;
	}

	for(i = 0; i < slcal_localctrl.local_package_num; i++)
	{
		if((slcal_localctrl.local_package_params[i][0] >> 4) == 1)		//定时控制
		{
			array[i]  = slcal_localctrl.local_package_params[i][2];
			array[i] *= 60;
			array[i] += slcal_localctrl.local_package_params[i][3];
		}
		else if((slcal_localctrl.local_package_params[i][0] >> 4) == 2)	//经纬度控制
		{						
			tempa = slcal_localctrl.local_package_params[i][3];//两字节共同代表偏移的分钟数 低字节在前 高字节在后
			tempa = tempa << 8;
			tempa += slcal_localctrl.local_package_params[i][2];
			
			temp16 = tempa & 0x7FFF;

			if((slcal_localctrl.local_package_params[i][0] & 0x0F) == 0)//继电器输出
			{
				if((((slcal_localctrl.local_package_params[i][4] & 0x0F) == 0x0C) && (((slcal_localctrl.local_package_params[i][4] >> 4) & 0x0F) == 0x0C))
				   || (((slcal_localctrl.local_package_params[i][4] & 0x0F) == 0x0C) && (((slcal_localctrl.local_package_params[i][4] >> 4) & 0x0F) == 0x00))
				   || (((slcal_localctrl.local_package_params[i][4] & 0x0F) == 0x00) && (((slcal_localctrl.local_package_params[i][4] >> 4) & 0x0F) == 0x0C))) //只有关灯操作，以日出时间为基准点
				{
					if(tempa & 0x8000) //负偏移
					{
						if(sunrisetime >= temp16)
							array[i] = sunrisetime - temp16;
						else
							array[i] = 1440 + sunrisetime - temp16;     //至上一天
					}
					else              //正偏移
					{
						array[i] = sunrisetime + temp16;
						if(array[i] >= 1440) array[i] = array[i] % 1440;//至下一天							
					}
				}
				else //否则以日落时间为基础点
				{
					if(tempa & 0x8000) //负偏移
					{
						if(sunsettime >= temp16)
							array[i] = sunsettime - temp16;
						else
							array[i] = 1440 + sunsettime - temp16;       //至上一天
					}
					else              //正偏移
					{
						array[i] = sunsettime + temp16;
						if(array[i] >= 1440) array[i]  = array[i] % 1440;//至下一天						
					}
				}
			}
			else if(((slcal_localctrl.local_package_params[i][0] & 0x0F) == 1) || ((slcal_localctrl.local_package_params[i][0] & 0x0F) == 2)) //  PWM or RS485
			{
				if((slcal_localctrl.local_package_params[i][4] & 0xF0) == 0x00)//10%以下调光视为关灯，进行关灯操作，以日出时间为基准点
				{
					if(tempa & 0x8000) //负偏移
                    {
                        if(sunrisetime >= temp16)
                            array[i] = sunrisetime - temp16;
                        else
                            array[i] = 1440 + sunrisetime - temp16;
                    }
                    else              //正偏移
                    {
                        array[i] = sunrisetime + temp16;
                        if(array[i] >= 1440) array[i] = array[i] % 1440;
                            
                    }
				}
				else//否则以日落时间为基础点
				{
					if(tempa & 0x8000) //正偏移
				    {
						if(sunsettime >= temp16)
						{
							array[i] = sunsettime - temp16;
						}
						else
						{
							array[i] = 1440 + sunsettime - temp16;        //至上一天
						}
					}
					else              //负偏移
					{
						array[i] = sunsettime + temp16;

						if(array[i] >= 1440) array[i]  = array[i] % 1440;  //至下一天
					}
				}

			}
		}
		
		time_point_array[i] = array[i];
	}
	
	return 1;
}


/*************************************************************************
 函数名称：	localctrl_new_para_save_or_sort
 功能说明：	本地控制参数保存或排序
 输入参数： save_flag - 1-保存 0-排序
 返回参数：
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
		localctrl_time_changeto_timepoint(array);//将时间或偏移量转化为统一的时间点

		for(m = 0; m < (slcal_localctrl.local_package_num-1); m++)//从小到大排序
		{
			Min = m;

			for(n = (m+1); n < slcal_localctrl.local_package_num; n++)//从剩余的数组里找出最小的
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
 函数名称：	count_days()
 功能说明：	计算从格林威治时间公元2000年1月1日到计算日的周日的天数
 输入参数： now_time[]-当前时间[0-4:year_low/year_high/month/day/week]   
 返回参数： days-天数
 *************************************************************************/
static U32 count_days(U8 now_time[])
{
    U16  i;
    U32  days=0;
    
    i= (now_time[0] + now_time[1]*256) - 1;               
    while(i>1999)                                     //计算今年1月1日到2000年1月1日共有多少天
    {
      if((i%4)==0)  days+=366;
      else          days+=365;
      i--;
    }
    
    for(i=1; i<now_time[2];i++)                       //计算当月1日距离1月1日共有多少天 
    { 
      if(i<8)
      {
         if(i%2!=0) days+=31;                              // 1、3、5、7为31天
         else if(i!=2) days+=30;                           // 4、6为30天
         else
         {
           if(((now_time[0] + now_time[1]*256)%4)==0) 
           {
              days+=29;                                     //闰年2月29天
           } 
           else
           {
             days+=28;                                      //非闰年2月29天
           }                
         }
      }
      else
      {
        if((i%2)==0) days+=31;                              //8、10、12为31天
        else         days+=30;                              //9、11月为30天
      }
    }
    
    days+=now_time[3];                                       //计算今天距离2000年1月1日共有多少天     
                  
    return (days);
}

/*************************************************************************
 函数名称：	rise_set_time_calc()
 功能说明：	日出日落时间算法算时间点
 输入参数：    
 返回参数： 
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
        t  = UT0;                                //世纪数t	 4887
        t /= 360;
        t += days;
        t /= 36525;

        //L=280.460+36000.770*t; 
        l  = 36000.770;                         //太阳的平黄径     5091.897
        l *= t;
        l += 280.460;                           

      
        //G=(357.528+35999.050*t)*3.1415926/180;
        g  = 35999.050;                    	    //太阳的平近点角    90.211339
        g *= t;
        g += 357.528;
        g *= PI;
        g /= 180;			 

        //D=1.915*sin(G)+0.020*sin(2*G);
        //A=(L+D)*3.1415926/180;     			//lamt == A
        lamt  = f_sin(g);            			//太阳的黄道经度
        lamt *= 1.915;                			//1.914986972
        temp  = f_sin(2.0*g);        			//-0.00737705
        temp *= 0.020;
        temp2 = temp+lamt;            			//用于太阳时间角gha计算
        lamt = temp2+l;
        lamt *= PI;
        lamt /= 180;

        //temp_B=(23.4393-0.0130*t)*3.1415926/180;    
        temp_B  = t;                  			//地球的倾角
        temp_B *= 0.0130;
        temp_B  = 23.4393-temp_B;
        temp_B *= PI;
        temp_B /= 180;


        // c=asin(sin(A)*sin(temp_B));
        temp  = f_sin(temp_B);       			//太阳的偏差
        c  = f_sin(lamt);
        temp *= c;
        c = f_asin(temp);                       
      
		// GHA=UTo-180     -1.915?ásinG-0.020?ásin(2G)          +2.466?ásin(2|?)-0.053?ásin(4|?)
		//F=2.466*sin(2*A)-0.053*sin(4*A);
		//GHA=UT0-180-lamt +F;
		temp = f_sin(2*lamt);                   //太阳时间角 gha
		temp *= 2.466;                      	 //0.0949997
		temp -= 0.053*f_sin(4*lamt);            //0.0990801961
		gha = UT0-180;
		gha += temp;
		gha -= temp2; 

		//H=sin(Glat)*sin(C);
		//I=cos(Glat)*cos(C);														 
		//H=(sin(-0.01454)-H)/I;
		//E=acos(H)*180/3.1415926;

		temp = f_sin(glatitude);       				 //修正值e
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
 函数名称：	sunrise_sunset_time_calculate
 功能说明：	计算本周的日出日落时间，只将计算的最后一个结果保存(当天日出)
 输入参数： *long_glat - 经纬度
            now_time[] - 当前时间[0-4:year_low/year_high/month/day/week]   
            rise_set_UT0[] - 最后一次计算日出日落时间
            sunrise_time[] - 保存计算后的日出时间
            sunset_time[]  - 保存计算后的日落时间          
 返回参数： 无
 *************************************************************************/
static void sunrise_sunset_time_calculate(slcal_longi_lati_struct longi_glati,U8 now_time[],U8 rise_set_UT0[],U8 sunrise_time[],U8 sunset_time[])                           
{   
    F32    rise_UT0=0x00000000,set_UT0=0x00000000,rise_UT=0x00000000,set_UT=0x00000000;
    F32    rise_t=0x00000000,set_t=0x00000000;
    F32    longitude=0x00000000,glatitude=0x00000000;
    U32    days=0;

    rise_UT = PI;
    set_UT  = PI;
    days = count_days(now_time);                                                            //计算从公元2000年1月1日到今天的天数
                                    
    longitude  = longi_glati.longitude_fra;                                                 //经度值
	longitude /= 100.00;                                                                   
	longitude += longi_glati.longitude_int;
                 
	glatitude  = longi_glati.latitude_fra;                                                  //纬度值
	glatitude /= 100.00;                                                                   
	glatitude += longi_glati.latitude_int;
			 
    glatitude  = glatitude*PI;                                                              //转化为弧度
    glatitude /= 180;             

	rise_UT0  = rise_set_UT0[1];                                                            //最后一次计算的rise_UT0
	rise_UT0 /= 100.00;                                                                   
	rise_UT0 += rise_set_UT0[0];

    if( (rise_UT0>(-2)) && (rise_UT0<2) ) 
	{
		rise_UT0=180;
	}

	set_UT0  = rise_set_UT0[3];                                                            //最后一次计算的set_UT0
	set_UT0 /= 100.00;                                                                   
	set_UT0 += rise_set_UT0[2];
		
    if( (set_UT0>(-2)) && (set_UT0<2) )
	{
	    set_UT0=180;
	}

	rise_UT=rise_set_time_algorithm(rise_UT0,days,glatitude,longitude,SUNRISEFLAG);//计算日出时间
	rise_t=rise_UT/15+8; 
	rise_UT0=rise_UT;
	sunrise_time[0]=(U8)(rise_t);                                    //整数部分为小时数
	sunrise_time[1]=(U8)((rise_t-(U8)(rise_t))*60);                  //小数部分为分钟数

	set_UT=rise_set_time_algorithm(set_UT0,days,glatitude,longitude,SUNSETFLAG);   //计算日落时间
	set_t=set_UT/15+8;
	set_UT0=set_UT; 
	sunset_time[0]=(U8)(set_t);                                      //整数部分为小时数
	sunset_time[1]=(U8)((set_t-(U8)(set_t))*60);                     //小数部分为分钟数

    rise_set_UT0[0]=(S8)(rise_UT);
    rise_set_UT0[1]=(S8)((rise_UT-(S8)(rise_UT))*100);
    rise_set_UT0[2]=(S8)(set_UT);
    rise_set_UT0[3]=(S8)((set_UT-(S8)(set_UT))*100);
 
}

/*************************************************************************
 函数名称：	update_sun_rise_set_time
 功能说明：	更新当日日出日落时间
 输入参数： 
 返回参数： 
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
 函数名称：	findout_relay_enable_point
 功能说明：	寻找继电器操作补操作点
 输入参数：	
 返回参数：	
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

		//筛选今日所有使能点
		for(i = 0; i < packagenum; i++)                             
		{
		   if((parameter[i][1]&BIT(real_time.week))
		      && ((((parameter[i][0]&0x0F) == 0x01)&&((parameter[i][4] & (0x01<<m)) == (0x01<<m)))//PWM调光 
		          || (((parameter[i][0] & 0x0F) == 0x00)&&((((parameter[i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x03) || (((parameter[i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x0C))))) //继电器操作
			{
		       timearrayK1[acttingnum] = timearray[i];
		       actting_flag |= BIT(i);              	//记录今日使能点
			   acttingnum++;                        
		    }
		}

		if((acttingnum == 0) || (nowtime < timearrayK1[0]))//今日无使能点 或 当前时间小于今日第一个使能点
		{
		    //筛选上一使能日的最后一点
		    if(real_time.week == 0) k = real_time.week - 6;
		    else k=1;
		    
			while (!lastacttingflag)
			{
				for(i = 0; i < packagenum; i++)
				{
					if(parameter[packagenum - 1 - i][1]&BIT((real_time.week - k)))//从今日前一天开始向前查找
					{
						if(((parameter[packagenum - 1 - i][0] & 0x0F) == 0)
						   && ((((parameter[packagenum - 1 - i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x03)|| (((parameter[packagenum - 1 - i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x0C)))//继电器操作
						{
						    //lampact |= ((parameter[packagenum - 1 - i][4+(m/2)]>>((m%2)*4)) & 0x0F);
						    lastacttingflag = 1;
						    lastdaypoint = packagenum - 1 - i;
						    break;
						}
						else if(((parameter[packagenum - 1 - i][0] & 0x0F) != 0) && ((parameter[packagenum - 1 - i][4] & (0x01<<m)) == (0x01<<m)))//PWM调光
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

			if(lastdaypoint!=0xff)//上一使能日最后已操作点为继电器操作，需要补该继电器操作
			{
				lastdaytime = timearray[lastdaypoint];
				
				temp = 0;

				if((nowtime < timearrayK1[0]) || (nowtime > lastdaytime) || ((acttingnum == 0)&&(nowtime <1440)))  //当前点在哪一段
				{
					temp |= 0x01;
				}

				//if((remotectrtime[m] < timearrayK1[0]) || (remotectrtime[m] > lastdaytime) || ((acttingnum == 0)&&(lastday_date == real_time.day)&&(remotectrtime[m] <1440))) //上一控制时间点在哪一段
				if(((remotectrtime[m] < timearrayK1[0]) || (remotectrtime[m] > lastdaytime) || ((acttingnum == 0) && (remotectrtime[m]<1440))) && (lastday_date == real_time.day))//上一控制操作点在哪一时段
				{
					if(remotectrtime[m] != 0x3CC3)//初始值 0xff*60+0xff
					{
						temp |= 0x02;
					}
					
				}

				if(((temp & 0x01) == 0x01) && ((temp & 0x02) != 0x02)) //当前时段在凌晨段，上一控制时间点在非凌晨段
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
		else //今日有使能点且当前时间位于今日第一个使能点后
		{
		   for(i = 1; i < acttingnum; i++)
		   {
		       if(nowtime < timearrayK1[i])         //当前时间小于下一执行操作点则跳出，此时timearrayK1[i-1]为需要补操作的使能点
		           break;
		   }

		   for(j = 0; j < acttingnum; j++)
		   {
		       if(remotectrtime[m] < timearrayK1[j])//上一控制时刻小于下一执行操作点则跳出，此时timearrayK1[j-1]使能点不进行补操作
		           break;
		   }

		   if((i != j) || (remotectrtime[m] == 0x3CC3)|| ((i == j)&&(lastday_date != real_time.day)))//当前时间与上一操作时刻不在同一时段||远程操作时刻为初始值(无远程操作)||处于同一时段但日期不同
		   {			
				//LampLoopStatusChanged_Flag[0] = 0;	
				
				temp = 0;

				for(j = 0; j < LOCAL_PACKAGE_MAX_NUM; j++)
				{
					if(actting_flag & BIT(j))
					{
						temp++;

						if(temp == i)          //第temp个1
						{
							if(j <= (packagenum-1))
							{
								if(((parameter[j][0] & 0x0F) == 0)&& ((((parameter[j][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x03)|| (((parameter[j][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x0C)))//继电器操作 																
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
 函数名称：	findout_pwm_enable_point
 功能说明：	寻找调光操作补操作点
 输入参数：	
 返回参数：	
 *************************************************************************/
void findout_pwm_enable_point(U16 nowtime, U16 remotectrtime[], U8 parameter[][LOCAL_PACKAGE_MAX_LEN],U8 packagenum, U16 *timearray)
{
	U8  i, j, k, temp,temp_flag, m=0;
	U8  acttingnum, lastdaypoint,lastacttingflag,array[3];  
	U16 todaytimearray[LOCAL_PACKAGE_MAX_NUM],lastdaytime;
	U32 actting_flag;
	S8	g;
	
	for(m = 0; m < MAX_LED_NUM; m++)//补光策略，mAX_lED_NUM最大路数
	{			
		acttingnum = 0; //行动次数
		actting_flag = 0;//行动标志
		  
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
		
		//筛选今日所有使能点
	    for(i = 0; i < packagenum; i++)
	    {
		   if((parameter[i][1]&BIT(real_time.week))
		      && ((((parameter[i][0]&0x0F) == 0x01)&&((parameter[i][4] & (0x01<<m)) == (0x01<<m)))//PWM
		          || (((parameter[i][0] & 0x0F) == 0x00)&&((((parameter[i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x03) || (((parameter[i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x0C))))) //继电器
			{
	            todaytimearray[acttingnum] = timearray[i];
	            actting_flag |= BIT(i);
	            acttingnum++;
	        }
	    }

		if((acttingnum == 0) || (nowtime < todaytimearray[0]))    //今日无使能点 或 当前时间小于今日第一个使能点
		{
		    //查询上一使能日的最后一个使能点
		    if(real_time.week == 0) g = real_time.week - 6;
		    else g=1;
		    
			while (!lastacttingflag)
			{
				for(i = 0; i < packagenum; i++)
				{
					if(parameter[packagenum - 1 - i][1]&BIT((real_time.week - g)))//从今日前一天开始向前查找
					{
						if(((parameter[packagenum - 1 - i][0] & 0x0F) == 0)
						   && ((((parameter[packagenum - 1 - i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x03)|| (((parameter[packagenum - 1 - i][4+(m/2)]>>((m%2)*4)) & 0x0F) == 0x0C)))//继电器
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
		
		
			//上一使能日最后已操作点为PWM调光操作，需要补该调光操作
			if(lastdaypoint != 0xff)                 
			{
				lastdaytime = timearray[lastdaypoint];
	 
				temp_flag = 0;
				
				if((nowtime < todaytimearray[0]) || (nowtime > lastdaytime) || ((acttingnum == 0)&&(nowtime <1440)))//当前时间在哪一时段
				{
					temp_flag |= 0x01;
				}

				if(((remotectrtime[m] < todaytimearray[0]) || (remotectrtime[m] > lastdaytime) || ((acttingnum == 0) && (remotectrtime[m]<1440))) && (lastday_date == real_time.day))//上一控制操作点在哪一时段
				{
					if(remotectrtime[m] != 0x3CC3)//初始值0xFF*60+0xFF
					{
						temp_flag |= 0x02;
					}
				}

				if(((temp_flag & 0x01) == 0x01) && ((temp_flag & 0x02) != 0x02)) //当前时间在凌晨段，而遥控时间在非凌晨段
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
		else//今日有使能点且当前时间位于今日第一个使能点后
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

						if(temp == k)          //第temp个1
						{
							if(j <= (packagenum-1))
							{
								if(((parameter[j][0]&0x0F) == 0x01)&&((parameter[j][4] & (0x01<<m)) == (0x01<<m)))//PWM调光 
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
 函数名称：	correct_ctrl_setting 
 功能说明：	本地控制补操作
 输入参数：	
 返回参数：	
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
 函数名称：	instant_ctrl_setting    
 功能说明：	即时控制设置
 输入参数：	array[0]-低4bit选择继电器操作/PWM调光操作
         	array[1]-低4bit第一路灯状态/灯回路使能  高4bit第二路灯状态/占空比十位
         	array[2]-低4bit频率(1~15) 高4bit占空比个位
         	i-第几回路 （0-第一回路 1-第二回路）
 返回参数：	set_result - 设置结果
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
	    case 0://继电器输出
	  
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
		                 /*slcal_force_ctrl遥控状态下结构体*/
			slcal_force_ctrl[i].relay_status  = status[i];    //设置灯状态
			slcal_force_ctrl[i].status=ENABLE_FORCECTRL;//值为1 ENABLE_FORCECTRL
			slcal_force_ctrl[i].last_time=0xffff;//
			slcal_force_ctrl[i].second_count=0;//

			#ifdef SLC_PRINT_SYS//运行打印宏
			dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
				"task 0x21 0x0020: set led %d force ctrl: %d %d %d.\r\n"
				,i, slcal_force_ctrl[i].relay_status,slcal_force_ctrl[i].brightness_status,slcal_force_ctrl[i].last_time);
			#endif

			if(status[i] == LAMP_CLOSE) //0xcc   LAMP_CLOSE                                                 
			{
				/*PWM输出恢复默认值*/
				slcal_force_ctrl[i].brightness_status = 0;
			}
			else
			{
				slcal_force_ctrl[i].brightness_status = 100;
			}
			slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);//CalcCRC16计算给定字节长度数据的16位CRC
			
			last_ctrl_timepoint[i] = (U16)(real_time.hour * 60 + real_time.minute); //当前时间点（分）
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
		  				
		break;//复位

	  case 1://PWM调光
	  
	  	  #ifdef SLC_PRINT_SYS
		    dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
			  "InstCtrl-PWM.\r\n");
		  #endif	
	  		      
	      if((slc_model[0] & 0x38) == 0x30||(slc_model[0] & 0x38) == 0x28)//型号符合才动作调光
		  {
		  	  tmp_freq = array[2]&0x0F;
		      tmp_brightness_status = (array[1]>>4)*10 + (array[2]>>4) ;
		      
		      //判断频率
		      if(!tmp_freq) return 0;
		      
			  //判断亮度
		      if(tmp_brightness_status >100) tmp_brightness_status = 100;
		      
		      #ifdef SLC_PRINT_SYS
			  dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
				  "brightness_status is %d.\r\n",tmp_brightness_status);
			  #endif
		  
		  	  //set_pwm_freq(tmp_freq);//设置调光频率
		  	  
		  	  //存储当前调光频率
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
			  	  //判断调光方式
			      if((tmp_freq%2)&&(tmp_brightness_status>=10))//频率单数且调光大于等于10%，自适应调光使能
			      {
			          slcal_selfpwm[i].selfpwm_enable = 1;
			          #ifdef SLC_PRINT_SYS
					    dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
						  "slcal_selfpwm[i].selfpwm_enable = 1.\r\n");
					  #endif	
			      }
			      else										   //频率双数或调光小于10%(认为关灯操作)，自适应调光禁止
			      {
			      	  slcal_selfpwm[i].selfpwm_enable = 0;
					  if(tmp_brightness_status>=10)
					  {
						  if(Control_command_dispose(COMMAND_TYPE_ON,0) == 1)//电子镇流器单灯需先开灯
					  	  {
					  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
						   	  slcal_ComunicationRetry.Control_retry_times = 0;
					  	  }
						  slcal_delay(15000); //开灯和调光之间需要等待100ms
						  if(Control_command_dispose(COMMAND_TYPE_DIMMING,tmp_brightness_status)== 1)
						  {
					  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
						   	  slcal_ComunicationRetry.Control_retry_times = 0;
					  	  }
						  slcal_ctrl[i].current_brightness_status = tmp_brightness_status;
						}
			      }
			      
		  		  if (tmp_brightness_status<10)//0-9%调光 关灯
		  		  {
					  slcal_force_ctrl[i].relay_status  = LAMP_CLOSE;    //设置灯状态
					  slcal_force_ctrl[i].brightness_status = 0;
					  slcal_force_ctrl[i].status=ENABLE_FORCECTRL;
					  slcal_force_ctrl[i].last_time=0xffff;
					  slcal_force_ctrl[i].second_count=0;
					  slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);
				  }
				  else
				  {
					  slcal_force_ctrl[i].relay_status  = LAMP_OPEN;    //设置灯状态
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
				  
				  last_ctrl_timepoint[i] = (U16)(real_time.hour * 60 + real_time.minute); //当前时间点（分）
				  
			  }
			  return 1; 
		  }
		  else
		  {
		      tmp_brightness_status = (array[1]>>4)*10 + (array[2]>>4) ;
		      if(tmp_brightness_status >100) tmp_brightness_status = 100;
		      
			  if((array[1] & (0x01<<i)) == (0x01<<i))
			  {
		  		  if (tmp_brightness_status<10)//0-9%调光 关灯
		  		  {
					  slcal_force_ctrl[i].relay_status  = LAMP_CLOSE;    //设置灯状态
					  slcal_force_ctrl[i].brightness_status = 0;
					  slcal_force_ctrl[i].status=ENABLE_FORCECTRL;
					  slcal_force_ctrl[i].last_time=0xffff;
					  slcal_force_ctrl[i].second_count=0;
					  slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);
				  }
				  else
				  {
					  slcal_force_ctrl[i].relay_status  = LAMP_OPEN;    //设置灯状态
					  slcal_force_ctrl[i].brightness_status = 100;
					  slcal_force_ctrl[i].status=ENABLE_FORCECTRL;
					  slcal_force_ctrl[i].last_time=0xffff;
					  slcal_force_ctrl[i].second_count=0;
					  slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);	
				  }
				  
				  last_ctrl_timepoint[i] = (U16)(real_time.hour * 60 + real_time.minute); //当前时间点（分）
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
 函数名称：	slcal_localctrl_process_auto
 功能说明：	路灯本地自动控制策略
 输入参数： 
 返回参数： 
 *************************************************************************/
void slcal_localctrl_process_auto()
{
    U8  i, j;
    U8 array[3];
    U16 timearray[LOCAL_PACKAGE_MAX_NUM];
    U16 now_time;

    now_time = (U16)(real_time.hour * 60 + real_time.minute); //当前时间点（分）
    
    if(localctrl_time_changeto_timepoint(timearray))//将以排序的参数包中的时间或偏移量转化为时间点
	{		
        if(localctrl_init_f)
		{		    
            correct_ctrl_setting(now_time,last_ctrl_timepoint,slcal_localctrl.local_package_params, slcal_localctrl.local_package_num, timearray);//补操作
		}
				
        for(i = 0; i < slcal_localctrl.local_package_num; i++)
        {
            if(timearray[i] >= 1440)  continue;  //超过24小时不动作

            if(now_time == timearray[i])//当前时间处于一个本地控制时刻         
            {
                if(slcal_localctrl.local_package_params[i][1]&(0x0001<<real_time.week))//该本地时刻今日使能
                {
					for(j=0; j<MAX_LED_NUM; j++)
                	{
	                    if(now_time != last_ctrl_timepoint[j])//当前时刻不等于前一控制时刻，允许本地参数
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
 函数名称：	change_localctrl
 功能说明：	查询路灯本地控制策略并判断是否进行路灯控制改变
 输入参数： 
 返回参数： 
 *************************************************************************/
void change_localctrl()       
{
	static U8 err_cnt = 0;
	U8 result,buffer[3];
	U16 mode,year,crc;
	
	if(localctrl_store_f)//清空以及设置本地参数包需进行存储
    {
    	result = localctrl_new_para_save_or_sort(1);//本地控制参数包存储(不排序)
    	if(result)
    	{
	    	if((localctrl_store_f&0x03)!=0x01)//有新本地参数包设置时
            {
	    		localctrl_sort_f = 1;//对时后需要进行排序
	    	}
	    	localctrl_store_f = 0; //存储成功，不再进行存储
    	}
    }
	
    mode = ((U16)(((slc_model[0]) & 0x00ff) + (((slc_model[1]) & 0x00ff) << 8)));
    if(((mode & 0x0100) == 0x0100) &&(real_time.status==SLCAL_RTC_TIMING)&&((slcal_rtc_fault&0x02)!=0x02))//有时钟功能且已对时且过去无时钟故障警->进行本地控制			             	 
    {
        #ifdef SLC_PRINT_TST
		dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
			"FUNC change_localctrl.\r\n");
		#endif	
		
        //查询当前是否存在时间超差
        year = (real_time.year_high * 256 + real_time.year_low);
        if( (year < 2015) || (year > 2065) )
        {
            if(err_cnt < 5)
            {
                err_cnt++;

                if(err_cnt > 2)
                {
					slcal_rtc_fault |= 0x02; //置时钟警
                }
            }
        }
        else
        {
            err_cnt = 0;
            slcal_rtc_fault &= (~0x02);      //消警
        }

        if(((real_time.hour == 0) && (real_time.minute == 0) && (real_time.second == 0)) ||(localctrl_init_f == 1))  //每日0点或初始化本地控制功能时进行日出日落时间计算
        {    
			update_sun_rise_set_time();
        }
        
        if(localctrl_sort_f)
	    {
	    	localctrl_new_para_save_or_sort(0);//本地控制参数包排序 (需要用到正确的日出日落时间，因此需要计算日出日落时间后才可以排序)
	    	localctrl_sort_f = 0;
	    }
	    
	    if((real_time.hour == 23) && (real_time.minute == 59) && (real_time.second == 59))//跨日前清零复位次数 //if(lastday_date == real_time.day)
        {
            buffer[0] = 0;
			crc=CalcCRC16(buffer,1);
			buffer[1]=(U8)(crc>>8);
			buffer[2]=(U8)(crc&0x00ff);
			result=store_data_with_back(buffer,3,SL_RESET_NUM,SL_RESET_NUM+SL_BACK_OFFSET);
			if(result==SLCAL_EEPROM_FINISH)	
			{
				//写入ram中的地址值
				copy_U8(buffer,slc_reset_times,3);
				slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
			}
			else
			{
				slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
			}
        }
        
        slcal_localctrl_process_auto();//本地控制具体处理

        lastday_date = real_time.day;

    }
    else //if((mode & 0x0100) != 0x0100)//无时钟功能
    {
        lastday_date = 0xff;     	
    }
	
}

/*******************************************Local_Control_End**********************************************/


/*************************************************************************
 函数名称：	Get_Lamp_dat
 功能说明：	路灯数据获取函数
 输入参数： 
           
 返回参数：
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
		//sys_activate(SYS_MOD_TASK_SYS);// 喂系统任务软狗   
		//slcal_delay(15000); //延时约50ms等待镇流器回复
		resv_flag = control_uart_recv(lamp_data,12);   
		if((resv_flag == 12) &&(lamp_data[2] == (data[2]|0x80)))
		{
			return 0;
		}
		
	}while(count < 3);
	
	return 1;
}
/*************************************************************************
 函数名称：	sampling_para_collect
 功能说明：	累计参数收集函数
 输入参数： 
           
 返回参数：
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
 函数名称：	Control_command_dispose
 功能说明： 控制命令处理
 输入参数： 
           
 返回参数：
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
		//sys_activate(SYS_MOD_TASK_SYS);// 喂系统任务软狗   
		//slcal_delay(15000); //延时约50ms等待镇流器回复
		resv_flag = control_uart_recv(resv_data,8);   
		if((resv_flag == 8) &&(resv_data[2] == (data[2]|0x80)))
		{
			return 0;
		}
		
	}while(count < 3);
	
	return 1;
	
}
/*************************************************************************
 函数名称：	set_relay
 功能说明：	设置继电器通断
 输入参数： U8 led_num: 0 第一路 1 第二路
            U8 status: 0 继电器切断  非0 继电器连接
 返回参数：
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
 函数名称：	set_brightness
 功能说明：	路灯亮度设置函数，合法值为0-100，数值越大亮度越高，超过100以100计
 输入参数：	U8 led_num:0-第一路 1-第二路
            U8 brightness:设置的亮度
 返回参数：
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
 函数名称：	set_pwm_freq
 功能说明：	调光频率设置
 输入参数：	freq-频率(以100Hz为单位，范围是1~15)
 返回参数：
 *************************************************************************/
/*void set_pwm_freq(U8 freq)
{
	U16 oc0_current_value,oc1_current_value;
	U16 current_brightness_unit;

	oc0_current_value = OC0.OCxR1;				//读出当前OC0值
	oc1_current_value = OC1.OCxR1;   			//读出当前OC1值
	current_brightness_unit = slcal_dim.brightness_unit;   //当前亮度单位值

	//PIT3.PCSR.all &=0xfcfe;		    //禁止timer2，撤销预分频
	
	if(freq == 1)
		PIT3.PCSR.all = 0x0412;	//16分频
	else if (freq <11)				
		PIT3.PCSR.all = 0x0312;	//8分频
	else
		PIT3.PCSR.all = 0x0212;	//4分频
	
    PIT3.PMR = PWR_NUMBER[freq-1];	//添加预加载值
    
	PIT3.PCSR.all |= 0x0001;        //使能timer2
	
	slcal_dim.brightness_unit = BRIGHTNESS_UNIT[freq-1]; //新亮度单位值
    slcal_dim.dim_interval = freq*2;					 //累计到20ms进行渐进调光所需要的新事件周期(Timer2周期)数
	
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
		OC0.OCxR1 = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*(oc0_current_value/current_brightness_unit); //亮度不变前提下，更新OC0.OCxR1值
	}
	
	if(oc1_current_value == (BRIGHTNESS_ORIGIN + current_brightness_unit*100 -1))
	{
		OC1.OCxR1 = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*100 -1;
	}
	else
	{
		OC1.OCxR1 = BRIGHTNESS_ORIGIN + slcal_dim.brightness_unit*(oc1_current_value/current_brightness_unit); //亮度不变前提下，更新OC1.OCxR1值
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
 函数名称：	ISR_OC0
 功能说明：	PWM调光中断函数，实现以渐变时间为的周期进行调光渐变，每次调光时
 			读出需要调到的最终亮度，并与当前亮度比较，不一致调整一个亮度单位
 			逼近最终值
 输入参数：
 返回参数：
 *************************************************************************/
#pragma interrupt on
void ISR_OC0()
{
	//static U32 OCint_count=0;
	//U16 current_value,terminal_value;

	OC0.OCxCON.all&=0xffef;	//清中断标志
	
	/*OCint_count++;

	if((OCint_count>=slcal_dim.dim_interval)&&((!slcal_selfpwm[0].selfpwm_enable)||(slcal_selfpwm[0].selfpwm_execution)))//非自适应调光目前固定20ms调一个单元渐变、自适应调光每2s为一周期调若干单元渐变
	{
		OCint_count=0;	//清零中断累计次数

		current_value=OC0.OCxR1;	//读出当前OC值

		if(slcal_ctrl[0].relay_status==LAMP_CLOSE)	//如果单灯已关闭，则亮度直接调至0
		{	
			if(current_value!=BRIGHTNESS_ORIGIN)
			{
				OC0.OCxR1=BRIGHTNESS_ORIGIN;
			}
			slcal_dim_signal[0]=DIM_FINISH;
			return;
		}
		else if(!slcal_selfpwm[0].selfpwm_enable) //计算当前OC值与设定值的差距，有差距则调一个亮度单位接近
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
				slcal_dim_signal[0]=DIM_FINISH;	//完成调光，置起标志，可以进行自检
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
 函数名称：	ISR_OC1
 功能说明：	PWM1调光中断函数，实现以渐变时间为的周期进行调光渐变，每次调光时
 			读出需要调到的最终亮度，并与当前亮度比较，不一致调整一个亮度单位
 			逼近最终值
 输入参数：
 返回参数：
 *************************************************************************/
#pragma interrupt on
void ISR_OC1()
{
	//static U32 OCint_count=0;
	//U16 current_value,terminal_value;

	OC1.OCxCON.all&=0xffef;	//清中断标志
	
	/*OCint_count++;

	if((OCint_count>=slcal_dim.dim_interval)&&((!slcal_selfpwm[1].selfpwm_enable)||(slcal_selfpwm[1].selfpwm_execution)))//非自适应调光目前固定20ms调一个单元渐变、自适应调光每2s为一周期调若干单元渐变
	{
		OCint_count=0;	//清零中断累计次数

		current_value=OC1.OCxR1;	//读出当前OC值

		if(slcal_ctrl[1].relay_status==LAMP_CLOSE)	//如果单灯已关闭，则亮度直接调至0
		{	
			if(current_value!=BRIGHTNESS_ORIGIN)
			{
				OC1.OCxR1=BRIGHTNESS_ORIGIN;
			}
			slcal_dim_signal[1]=DIM_FINISH;
			return;
		}
		else if(!slcal_selfpwm[1].selfpwm_enable) //计算当前OC值与设定值的差距，有差距则调一个亮度单位接近
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
				slcal_dim_signal[1]=DIM_FINISH;	//完成调光，置起标志，可以进行自检
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
 函数名称：	force_ctrl_timing
 功能说明：	强制控制计时，当处于强制控制状态时，计算还需在本状态的时间，
            时间到则结束强制控制
 输入参数：
 返回参数：
 *************************************************************************/
/*void force_ctrl_timing()
{
    U8 i;
    for (i = 0; i < MAX_LED_NUM; i++)
    {
        if(slcal_force_ctrl[i].status==ENABLE_FORCECTRL)	//当在强制控制态时进入
    	{
    		if(slcal_force_ctrl[i].last_time==0)	//持续时间为0，直接结束
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
    			if(slcal_force_ctrl[i].second_count>59)	//秒计时到一分钟，将持续时间减一分钟
    			{
    				slcal_force_ctrl[i].second_count=0;
    				slcal_force_ctrl[i].last_time--;
    			}
    			if(slcal_force_ctrl[i].last_time==0)	//查询持续时间是否已经用完
    			{
    				slcal_force_ctrl[i].status=DISABLE_FORCECTRL;
    			}
    		}
    		slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);	//重新计算crc
    	}
    }
}*/

/*************************************************************************
 函数名称：	change_ctrl
 功能说明：	查询路灯控制策略并判断是否进行路灯控制改变
 输入参数：
 返回参数：
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
        //strategy_index=0xff;//初始化参数
       // change_signal=0;
        if(slcal_force_ctrl[j].status==ENABLE_FORCECTRL && slcal_force_ctrl[j].last_time!=0)	//若强制控制有效且还在持续，按强制控制进行
    	{
    		relay_status=slcal_force_ctrl[j].relay_status;
    		brightness_status=slcal_force_ctrl[j].brightness_status;
    		led_ctrl_type=SLCAL_CTRL_FORCE;
    	}
    	else																				    //若既不在强制控制状态，也未对过时，则按缺省控制进行
    	{
    		relay_status=slcal_default_ctrl[j].relay_status;
    		brightness_status=slcal_default_ctrl[j].brightness_status;
    		led_ctrl_type=SLCAL_CTRL_DEFAULT;
    	}
    	
    	#if 0
    	//若rtc已对时，且任务表不为空，按任务表控制进行
    	else if(real_time.status==SLCAL_RTC_TIMING && slcal_strategy_collection[j].strategy_num!=0)	
    	{
    		led_ctrl_type=SLCAL_CTRL_STRATEGY;
    		get_rtc_value(rtc_buffer);	//获取当前时刻
    		minute1=rtc_buffer[2]*60+rtc_buffer[1];            
    		for(i=0;i<slcal_strategy_collection[j].strategy_num;i++)	//查询当前时刻属于哪个策略中
    		{
    			minute2=slcal_strategy_collection[j].strategy[i].hour*60+slcal_strategy_collection[j].strategy[i].minute;
    			if(minute1<minute2)	//计算当前时间小于哪条策略的切换时刻
    			{
    				strategy_index=i;
    				break;
    			}
    		}
    		if(strategy_index!=0)	//当strategy_index不为0时，表示当前时刻有对应策略
    		{
    			if(strategy_index==0xff)	//若为初值，则表示当前时刻大于最后一条策略的开始时间，故适用最后一条策略
    			{
    				strategy_index=slcal_strategy_collection[j].strategy_num-1;
    			}
    			else
    			{
    				strategy_index--;
    			}
    			relay_status=slcal_strategy_collection[j].strategy[strategy_index].relay_status;
    			brightness_status=slcal_strategy_collection[j].strategy[strategy_index].brightness_status;
                //为避免当前有策略而后续有新的策略导致的时间空窗, 每次更新当前策略时同时更新默认控制 20151125 BEGIN
                slcal_default_ctrl[j].relay_status=relay_status;
                slcal_default_ctrl[j].brightness_status=brightness_status;
                slcal_default_ctrl[j].crc=CalcCRC16((U8 *)&slcal_default_ctrl[j],2);
                //为避免当前有策略而后续有新的策略导致的时间空窗, 每次更新当前策略时同时更新默认控制 20151125 END
    		}
    		else	//当前时刻在第一条策略开始之前，故无策略对应，使用缺省策略
    		{
    			relay_status=slcal_default_ctrl[j].relay_status;
    			brightness_status=slcal_default_ctrl[j].brightness_status;
    			led_ctrl_type=SLCAL_CTRL_DEFAULT;
    		}
    	}
    	#endif
    	
    	//if(relay_status==LAMP_CLOSE)	//如果单灯关闭，则亮度强制设为0
    	//{
    	//	brightness_status=0;
    	//}
    	
    	/*if (relay_status != 0 &&threshold[j].u_error_signal_u)
        {//当前为打开继电器且过压 无动作
            ;
        }
        else
        {                
            	      		
        }*/
        
   	  // second=real_time.second;	//取出当前秒值
    	
    	//if(relay_status!=slcal_ctrl[j].relay_status)										//查询路灯开关是否要发生变化
    	//{	

			 if (relay_status == LAMP_CLOSE)
	    	{
	    		if(Control_command_dispose(COMMAND_TYPE_OFF,0)== 1)//set_relay(j, 0); //关灯
	    		{
			  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
				   	  slcal_ComunicationRetry.Control_retry_times = 0;
					//  flag ++;
				}
	    	}  
	    	else
	    	{
	    		if(Control_command_dispose(COMMAND_TYPE_ON,0) == 1)//set_relay(j, 1); //开灯 
	    		{
			  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
				   	  slcal_ComunicationRetry.Control_retry_times = 0;
					 // flag++;
				}
				slcal_delay(15000); //开灯和调光之间需要等待100ms
				if(Control_command_dispose(COMMAND_TYPE_DIMMING,brightness_status)== 1)
				{
			  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
				   	  slcal_ComunicationRetry.Control_retry_times = 0;
					 // flag++;
				}
				slcal_ctrl[j].current_brightness_status = brightness_status;
	    	} 
			
           /* if(relay_status==LAMP_OPEN)	//根据此次开关灯的属性，累加开灯次数
    		{
                crc1=CalcCRC16((U8 *)&slcal_ctrl_data[j].open_times,2);
    	        crc2=slcal_ctrl_data[j].open_crc;
                if (crc1 != crc2)
                {
                    slcal_check_signal[j]|=SLCAL_CHECK_DATA;//两者不一致, 触发一次自检
                }
                else
                {
                    slcal_ctrl_data[j].open_times++;
    			    slcal_ctrl_data[j].open_crc=CalcCRC16((U8 *)&slcal_ctrl_data[j],2);
                }
    		}*/
    		//else//状态改为关灯时，自适应调光方式禁止
    		//{
    		//	slcal_selfpwm[j].selfpwm_enable = 0; 
    		//}
    		 
          //  change_signal++;
			//if (flag !=0){
            slcal_ctrl[j].relay_status=relay_status;  //}  		
    	//}
    	
    	if(brightness_status!=slcal_ctrl[j].brightness_status)								//查询路灯亮度是否要发生变化
    	{
    		change_signal++;//调光次数
			if(brightness_status > 9)//如果亮度状态大于9
			{
	    		slcal_ctrl[j].brightness_status=brightness_status;//把亮度状态赋值给该灯
	    		slcal_dim_signal[j]=DIM_UNFINISH;//调光完成标志
				//Control_command_dispose(COMMAND_TYPE_ON,0);//电子镇流器单灯需先开灯
				//slcal_delay(15000); //开灯和调光之间需要等待100ms
				Control_command_dispose(COMMAND_TYPE_DIMMING,brightness_status);
			}
			else
			{
				slcal_ctrl[j].brightness_status=0;
				slcal_dim_signal[j]=DIM_UNFINISH;
				Control_command_dispose(COMMAND_TYPE_OFF,0);//小于10%关灯
			}
    	}
    	else
    	{
    		if(last_selfpwm_enable_status[j]!=slcal_selfpwm[j].selfpwm_enable)					//相同调光百分比但调光方式改变也执行调光
	    	{
	    		slcal_dim_signal[j]=DIM_UNFINISH;
	    		last_selfpwm_enable_status[j]=slcal_selfpwm[j].selfpwm_enable;
	    	}
    	}
    	
    	if(led_ctrl_type!=slcal_ctrl_type[j])												//查询控制模式是否发生了变化
    	{
    		//change_signal++;
    		slcal_ctrl_type[j]=led_ctrl_type;
    	}

    	//if(change_signal!=0)																//当路灯控制参数发生变化时，重新计算校验码
    	//{
    		#ifdef SLC_PRINT_SYS
    		dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
    			"led %d ctrl mode change to %d, status:%d, brightness:%d.\r\n",
    			j,slcal_ctrl_type[j],relay_status,brightness_status);
    		#endif
    		
    		slcal_ctrl[j].crc=CalcCRC16((U8 *)&slcal_ctrl[j],2);
    		
    		//存储当前路灯控制参数
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
 函数名称：	self_adaption_pwm
 功能说明：	自适应调光处理函数
 输入参数：
 返回参数：
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
																			
			    if((power_distance[i]<=-2)||(power_distance[i]>=2))//若功率偏移误差范围超过±2%,进行自适应调光
			    {
		    		record_power_distance[i][1] = record_power_distance[i][0];                                   
					record_power_distance[i][0] = power_distance[i];
			    
			    	if(((record_power_distance[i][1] >= 2)&&(record_power_distance[i][0] <= -2))
					|| ((record_power_distance[i][1] <= -2)&&(record_power_distance[i][0] >= 2)))
					{
						if(dim_speed[i] == FAST_MODE)			//相邻两次调光均超出误差且位于目标值两侧，其后每次调光幅度为1%
						{
							dim_speed[i] = SLOW_MODE;                                                                
						}
						else                          			//功率无法自适应到允许误差范围内，选取最优结果且在下次调光命令下达前不再进行调光
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
					
						record_power_distance[i][1] = 0;//赋初始值，准备下一次调光操作
						record_power_distance[i][0] = 0;
						dim_speed[i] = FAST_MODE;	
					}
					
					if(slcal_selfpwm[i].change_unit_cnt != 0)//需要调光，执行标志置位
					{
						slcal_selfpwm[i].selfpwm_execution = 1;
						
					}
					if(Control_command_dispose(COMMAND_TYPE_ON,0) == 1)//电子镇流器单灯需先开灯
					{
			  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
				   	  slcal_ComunicationRetry.Control_retry_times = 0;
			  	    }
					slcal_delay(15000); //开灯和调光之间需要等待100ms
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
						record_power_distance[i][1] = 0;//赋初始值，准备下一次调光操作
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
		    		record_power_distance[i][1] = 0;//赋初始值，准备下一次调光操作
					record_power_distance[i][0] = 0;				
					dim_speed[i] = FAST_MODE;
					slcal_dim_signal[i]= DIM_FINISH;
		    	}	
			}
		}
		else
		{
			record_power_distance[i][1] = 0;//赋初始值，准备下一次调光操作
			record_power_distance[i][0] = 0;
			dim_speed[i] = FAST_MODE;
		}
	}
}

/*************************************************************************
 函数名称：	add_duration
 功能说明：	开灯时长和带电关灯时长计时函数，1s调用一次，根据当前继电器的状态
 			累加不同的计时
 输入参数：
 返回参数：
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
                if(((meter_status == METER_WELL)&&(electric_para[i].current == 0))				//采样芯片正常，电流当前为0，关灯或意外关灯状态
                  ||((meter_status != METER_WELL)&&((GPIO.PDGPIO_OUT.all&0b10000000)!=0))) 	    //采样芯片异常，以继电器状态为准，关灯状态（2017/11/9） 
                {
                    crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
    	            crc2=slcal_ctrl_data[i].close_crc;
                    if(crc1!=crc2)
                    {
                        slcal_check_signal[i]|=SLCAL_CHECK_DATA;//两者不一致, 触发一次自检
                    }
                    else
                    {
                        slcal_ctrl_data[i].close_duration++;
    		            slcal_ctrl_data[i].close_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
                    }
                }
                else if (((meter_status == METER_WELL)&&(electric_para[i].current > 0))			//采样芯片正常，电流当前>0，开灯或意外亮灯状态
                        ||((meter_status!=METER_WELL)&&((GPIO.PDGPIO_OUT.all&0b10000000)==0)))	//采样芯片异常，以继电器状态为准，开灯状态 
                {
                    crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_duration,4);
    	            crc2=slcal_ctrl_data[i].duration_crc;
                    if(crc1!=crc2)
                    {
                        slcal_check_signal[i]|=SLCAL_CHECK_DATA;//两者不一致, 触发一次自检
                    }
                    else
                    {
                        slcal_ctrl_data[i].open_duration++;
    		            slcal_ctrl_data[i].duration_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_duration,4);
                    }
                }
                break;
            case 1:
                if(((meter_status == METER_WELL)&&(electric_para[i].current == 0))					//采样芯片正常，电流当前为0，关灯或意外关灯状态
                  ||((meter_status!=METER_WELL)&&((GPIO.PDGPIO_OUT.all&0b00000001)!=0)))			//采样芯片异常，以继电器状态为准，关灯状态  
                {
                    crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
    	            crc2=slcal_ctrl_data[i].close_crc;
                    if(crc1!=crc2)
                    {
                        slcal_check_signal[i]|=SLCAL_CHECK_DATA;//两者不一致, 触发一次自检
                    }
                    else
                    {
                        slcal_ctrl_data[i].close_duration++;
    		            slcal_ctrl_data[i].close_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
                    }
                }
                else if (((meter_status == METER_WELL)&&(electric_para[i].current > 0))				//电流当前>0 开灯或意外亮灯状态
                        ||((meter_status!=METER_WELL)&&((GPIO.PDGPIO_OUT.all&0b00000001)==0)))		//采样芯片异常，以继电器状态为准，开灯状态
                {
                    crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_duration,4);
    	            crc2=slcal_ctrl_data[i].duration_crc;
                    if(crc1!=crc2)
                    {
                        slcal_check_signal[i]|=SLCAL_CHECK_DATA;//两者不一致, 触发一次自检
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
 函数名称：	store_open_times
 功能说明：	开灯次数存储函数，平时15min调用一次，当ram中的开灯次数与eerpom
 			中的不同时，将ram中的开灯次数写入eeprom中
 输入参数：
 返回参数：	0:存储成功	1:失败
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
            slcal_check_signal[i]|=SLCAL_CHECK_DATA;//两者不一致, 触发一次自检
            continue;
        }
        else
        {
            //判断开灯次数相比eeprom内的数据是否发生了变化
        	if(slcal_ctrl_data[i].open_times==eeprom_slcal_ctrl_data[i].open_times)
        	{
        		continue;
        	}

        	//当发生变化时需要进行eeprom更新
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

        	//debug口指示eeprom写控制累加数据
        	#ifdef SLC_PRINT_SYS
        	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
        		"store led %d open times:%d.\r\n", i, slcal_ctrl_data[i].open_times);
        	#endif
        	
        	//更新ram中的eeprom数据备份
        	eeprom_slcal_ctrl_data[i].open_times=slcal_ctrl_data[i].open_times;
        	eeprom_slcal_ctrl_data[i].open_crc=slcal_ctrl_data[i].open_crc;
        }
    }
	return 0;
}
#endif
/*************************************************************************
 函数名称：	store_close_duration
 功能说明：	带电关灯时长存储函数，平时15min调用一次，当ram中的开灯次数与eerpom
 			中的不同时，将ram中的开灯次数写入eeprom中
 输入参数：
 返回参数：	0:存储成功	1:失败
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
            slcal_check_signal[i]|=SLCAL_CHECK_DATA;//两者不一致, 触发一次自检
            continue;
        }
        else
        {
            //判断开灯次数相比eeprom内的数据是否发生了变化
        	if((slcal_ctrl_data[i].close_duration/60)==(eeprom_slcal_ctrl_data[i].close_duration/60))
        	{
        		continue;
        	}

        	//当发生变化时需要进行eeprom更新
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

        	//debug口指示eeprom写控制累加数据
        	#ifdef SLC_PRINT_SYS
        	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
        		"store led %d close duration:%d.\r\n", i, slcal_ctrl_data[i].close_duration);
        	#endif
        	
        	//更新eeprom中的数据
        	eeprom_slcal_ctrl_data[i].close_duration=slcal_ctrl_data[i].close_duration;
        	eeprom_slcal_ctrl_data[i].close_crc=slcal_ctrl_data[i].close_crc;
        }
    }
    
	return 0;
}
#endif
/*************************************************************************
 函数名称：	store_open_duration
 功能说明：	带电开灯时长存储函数，平时15min调用一次，当ram中的开灯次数与eeprom
 			中的不同时，将ram中的开灯次数写入eeprom中
 输入参数：
 返回参数：	0:存储成功	1:失败
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
            slcal_check_signal[i]|=SLCAL_CHECK_DATA;//两者不一致, 触发一次自检
            //continue;
        }
        else
        {
            //判断开灯次数相比eeprom内的数据是否发生了变化
        	if((slcal_ctrl_data[i].open_duration)!=(eeprom_slcal_ctrl_data[i].open_duration))
        	{
        		//当发生变化时需要进行eeprom更新
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
	        	
	        	//debug口指示eeprom写控制累加数据
	        	#ifdef SLC_PRINT_SYS
	        	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
	        		"store led %d open duration:%d.\r\n", i, slcal_ctrl_data[i].open_duration);
	        	#endif
	        	
	        	//更新eeprom中的数据
	        	eeprom_slcal_ctrl_data[i].open_duration=slcal_ctrl_data[i].open_duration;
	        	eeprom_slcal_ctrl_data[i].duration_crc=slcal_ctrl_data[i].duration_crc;
        	}
        }        
    }
    
	return 0;
}

/*************************************************************************
 函数名称：	clear_ctrl_data
 功能说明：	将开灯次数、时长、带电关灯时长清除为0，并将值写入到eeprom中
 输入参数：
 返回参数：	0:存储成功	1:失败
 *************************************************************************/
U8 clear_ctrl_data()
{
	U8 result,i,j;
	slcal_ctrl_data_struct temp;

    for (i = 0; i < MAX_LED_NUM; i++)
    {
        //把清零的累计值写入eeprom
    	temp.open_times=0;//开灯次数置零
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
	   	
    	temp.close_duration=0;//关灯时长置为零
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
	   	
	   	
    	temp.open_duration=0;//开灯时长置为零
    	temp.duration_crc=CalcCRC16((U8 *)&temp.open_duration,4);
    	
    	for(j = 0; j < BLOCK_NUM; j++)//BLOCK_NUM//累积量存储block
    	{
	    	result=store_data_with_back((U8 *)&temp.open_duration,6,SL_OPEN_DURATION+12*j+6*i,SL_OPEN_DURATION+12*j+6*i+SL_BACK_OFFSET);//U8 store_data_with_back(U8 *buffer,U8 len,U32 address,U32 back_address)往eeprom写函数
	    	if(result!=SLCAL_EEPROM_FINISH)//SLCAL_EEPROM_FINISH为eeprom驱动查询状态字中的一个，即若查询到写回不成功
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
    	
    	//控制数据清零
    	copy_U8((U8 *)&temp,(U8 *)&slcal_ctrl_data[i],sizeof(slcal_ctrl_data_struct));
    	copy_U8((U8 *)&temp,(U8 *)&eeprom_slcal_ctrl_data[i],sizeof(slcal_ctrl_data_struct));
    	next_duration_addr[i]=0;
    	
    	//debug口指示清累计控制数据
    	#ifdef SLC_PRINT_SYS
    	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
    		"led %d clear ctrl data.\r\n", i);
    	#endif
    }
	
	return 0;
}

/*************************************************************************
 函数名称：	ini_ctrl_data
 功能说明：	初始化灯控累计数据，从eeprom中读出灯控数据
 输入参数：
 返回参数：
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
		
        //读出开灯次数
    	/*result=get_data_with_crc((U8 *)(&slcal_ctrl_data[i]),4,SL_OPEN_TIMES+6*i,SL_OPEN_TIMES+6*i+SL_BACK_OFFSET);
    	if(result!=SLCAL_EEPROM_FINISH)	//控制累计数据读出失败，使用默认参数
    	{
    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
    		#ifdef SLC_PRINT_ASSERT
    		ASSERT(0);
    		#endif
    		
    		if(slcal_ctrl_data[i].open_times==0xffff)//初次上电
    		{
	    		slcal_ctrl_data[i].open_times=0;
	    		slcal_ctrl_data[i].open_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_times,2);
	    		result=store_data_with_back((U8 *)(&slcal_ctrl_data[i].open_times),4,SL_OPEN_TIMES+6*i,SL_OPEN_TIMES+6*i+SL_BACK_OFFSET);
		    	if(result!=SLCAL_EEPROM_FINISH)	//控制累计数据读出失败，使用默认参数
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
    		slcal_check_signal[i]|=SLCAL_CHECK_DATA;	//触发一次自检
    	}
    	else
    	{
    		slcal_eeprom_fault &= ~SLCAL_ERR_EEPROM;
    	}

    	//读出带电关灯时长
    	result=get_data_with_crc((U8 *)(&slcal_ctrl_data[i].close_duration),6,SL_CLOSE_DURATION+6*i,SL_CLOSE_DURATION+6*i+SL_BACK_OFFSET);
    	if(result!=SLCAL_EEPROM_FINISH)	//控制累计数据读出失败，使用默认参数
    	{
    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
    		#ifdef SLC_PRINT_ASSERT
    		ASSERT(0);
    		#endif
    		
    		if(slcal_ctrl_data[i].close_duration==0xffffffff)//初次上电
    		{
	    		slcal_ctrl_data[i].close_duration=0;
	    		slcal_ctrl_data[i].close_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].close_duration,4);
	    		result=store_data_with_back((U8 *)(&slcal_ctrl_data[i].close_duration),6,SL_CLOSE_DURATION+6*i,SL_CLOSE_DURATION+6*i+SL_BACK_OFFSET);
		    	if(result!=SLCAL_EEPROM_FINISH)	//控制累计数据读出失败，使用默认参数
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
    		
    		slcal_check_signal[i]|=SLCAL_CHECK_DATA;	//触发一次自检
    	}
    	else
    	{
    		slcal_eeprom_fault &= ~SLCAL_ERR_EEPROM;
    	}*/

    	//读出开灯时长
	    slcal_ctrl_data[i].open_duration=0;//路灯运行累计参数
		slcal_ctrl_data[i].duration_crc=0;//
    	next_duration_addr[i] = 0;
    	
    	for(j = 0; j < BLOCK_NUM; j++)//累积量存储block//每个block控制累计数据读取，读取失败系统复位，触发一次自检；读取成功
    	{
	    	result=get_data_with_crc((U8 *)(&temp2.open_duration),6,SL_OPEN_DURATION+12*j+6*i,SL_OPEN_DURATION+12*j+6*i+SL_BACK_OFFSET);//
	    	if(result!=SLCAL_EEPROM_FINISH)	//控制累计数据读出失败，使用默认参数
	    	{
	    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
	    		slcal_eeprom_fault_classify |= 0x80;
	    		#ifdef SLC_PRINT_ASSERT
	    		ASSERT(0);
	    		#endif

				
	    		//系统复位
	    		if(temp2.open_duration==0xffffffff)//如果时长饱和
	    		{
	    		    err_duration++;//可能是复位次数
	    		    if(err_duration==BLOCK_NUM)//初次上电系统复位(对于每个灯而言)
	    		    {
			    		slcal_ctrl_data[i].open_duration=0;
			    		slcal_ctrl_data[i].duration_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_duration,4);
			    		result=store_data_with_back((U8 *)(&slcal_ctrl_data[i].open_duration),6,SL_OPEN_DURATION+6*i,SL_OPEN_DURATION+6*i+SL_BACK_OFFSET);
				    	if(result!=SLCAL_EEPROM_FINISH)	//若控制累计数据读出失败，使用默认参数
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

	    		slcal_check_signal[i]|=SLCAL_CHECK_DATA;	//触发一次自检
	    	}
			
	    	else//控制累计数据读取成功
	    	{
	    	    if (temp2.open_duration >= slcal_ctrl_data[i].open_duration)
	    	    {
    	    		slcal_ctrl_data[i].open_duration = temp2.open_duration;//将较大的累计路灯运行参数存回
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
    	
    	//同步历史的累计数据
    	copy_U8((U8 *)&slcal_ctrl_data[i],(U8 *)&eeprom_slcal_ctrl_data[i],sizeof(slcal_ctrl_data_struct));
    	
    	//判断此次初始化是否为上下电复位
    	if(*ram_data!=0x20140408)
    	{
    		/*上电复位*/
    		//ini_rtc();//初始化rtc时间，为刚上电的rtc一个未对时的默认时间
			//ini_timer0();//rtc走秒基准(5ms)rtc走秒定时基准，定时周期为5ms
    		
            crc1=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_times,2);//slcal_ctrl_data_struct slcal_ctrl_data[MAX_LED_NUM];	//路灯运行累计参数//计算crc
	        crc2=slcal_ctrl_data[i].open_crc;//
            if (crc1 != crc2)//crc校验
            {
                slcal_check_signal[i]|=SLCAL_CHECK_DATA;//两者不一致, 触发一次自检
            }
            else
            {
                slcal_ctrl_data[i].open_times++;	//加上本次上电开灯次数
    		    slcal_ctrl_data[i].open_crc=CalcCRC16((U8 *)&slcal_ctrl_data[i].open_times,2);//重新计算当前crc
            }

            //存储上电路灯控制参数
    		result=store_data_with_back((U8 *)(&slcal_ctrl[i]),4,SL_CURRENT_CTRL+4*i,SL_CURRENT_CTRL+4*i+SL_BACK_OFFSET);
    		if(result!=SLCAL_EEPROM_FINISH)//若查询eeprom驱动状态字不为2//若存储不成功/*当然咱这里是可以的*/
        	{
        		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;//0x02//eeprom状态字置为错误
        		#ifdef SLC_PRINT_ASSERT//断言打印宏
        		ASSERT(0);
        		#endif
        	}
        	else
        	{
        		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);//否则就成功
        	}
        	
	   	  	//存储上电调光频率
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
			
			last_ctrl_timepoint[i] = 0xff * 60 +0xff; //初始值
    	}
		/*系统复位*/
    	else
    	{
    		/*#ifdef SLC_PRINT_ASSERT
    		ASSERT(0);
    		#endif*/
    		
    		//系统复位
    		//ini_timer0();//rtc走秒基准(5ms)
    		/*写系统复位前的灯状态*/
			
    		
    		//读出系统复位前灯状态(想获得当前灯状态)
	    	//result=get_data_with_crc((U8 *)(&temp[i]),4,SL_CURRENT_CTRL+4*i,SL_CURRENT_CTRL+4*i+SL_BACK_OFFSET);//读出系统复位前的灯状态
	    result=get_data_with_crc((U8 *)(&temp[i]),6,SL_CURRENT_CTRL+6*i,SL_CURRENT_CTRL+6*i+SL_BACK_OFFSET);//读出系统复位前的灯状态
	    //	result=get_data_with_crc((U8 *)(&temp[i]),8,SL_CURRENT_CTRL+8*i,SL_CURRENT_CTRL+8*i+SL_BACK_OFFSET);//读出系统复位前的灯状态
	    	if(result!=SLCAL_EEPROM_FINISH)	//控制累计数据读出失败，使用默认参数(上电开关灯状态)
	    	{
	    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
	    		#ifdef SLC_PRINT_ASSERT
	    		ASSERT(0);
	    		#endif
	    	}
	    	else
	    	{
	    	    //初始化受控状态参数
		    	slcal_force_ctrl[i].last_time=0xffff;
		    	slcal_force_ctrl[i].relay_status=temp[i].relay_status;
		    	slcal_force_ctrl[i].brightness_status=temp[i].brightness_status;
		    	slcal_force_ctrl[i].status=ENABLE_FORCECTRL;
		    	slcal_force_ctrl[i].second_count=0;
		    	slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);
		    	
		    	slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
	    	}
	    	
	    	//读出系统复位前调光频率
	    	result=get_data_with_crc((U8 *)(&temp1),4,SL_CURRENT_DIM_FREQ,SL_CURRENT_DIM_FREQ+SL_BACK_OFFSET);
	    	if(result!=SLCAL_EEPROM_FINISH)	//控制累计数据读出失败，使用默认参数
	    	{
	    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
	    		#ifdef SLC_PRINT_ASSERT
	    		ASSERT(0);
	    		#endif
	    	}
	    	else
	    	{
	    		if((slc_model[0] & 0x38) == 0x30||(slc_model[0] & 0x38) == 0x28)//型号符合才动作调光
			    {
			        //set_pwm_freq((U8)temp1.dim_freq);//设置调光频率
			        
    				#ifdef SLC_PRINT_SYS//运行打印宏
				      dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
					    "dim_freq after reset is %d.\r\n",temp1.dim_freq);
				    #endif	
			    	
			    	//判断调光方式
			      /*	if((temp1.dim_freq%2)&&(slcal_force_ctrl[i].relay_status>=10))//频率单数且调光大于等于10%，自适应调光使能
			      	{
			            slcal_selfpwm[i].selfpwm_enable = 1;
			      	}
			     	else										                  //频率双数或调光小于10%(认为关灯操作)，自适应调光禁止
			      	{
			      	    slcal_selfpwm[i].selfpwm_enable = 0;
			      	}*/
			    }
			    
		    	slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
	    	}
	    	
	    	last_ctrl_timepoint[i] = 0xff * 60 +0xff; //初始值
    	}
    }
    
    *ram_data=0x20140408;	//在ram指定位置打上标记，用以之后复位判断是否为上下电复位
}

/*************************************************************************
 函数名称：	ini_ctrl_para
 功能说明：	初始化灯控参数，从eeprom中读出灯控参数
 输入参数：
 返回参数：
 *************************************************************************/
void ini_ctrl_para()
{
	U8 result,i,j,k,tmp = 1;
	U8 buffer[8];
	U16 crc;

	for (i = 0; i < MAX_LED_NUM; i++)
	{   
	    #ifdef SLC_PRINT_SYS//运行打印宏
	   	dbg_printf(DBG_MODULE_AL_A, 0, DBG_INFO_0, DBG_PRM_MASTER,
	   		"FUNC ini_ctrl_para(led %d init ctrl state).\r\n",i);
		#endif
		
		//读出灯上电默认状态，失败赋默认值
		result=get_data_with_crc((U8 *)(&slcal_init_ctrl),4,SL_DEFAULT_CTRL,SL_DEFAULT_CTRL+SL_BACK_OFFSET);
		
	    if(result!=SLCAL_EEPROM_FINISH)//默认控制参数读出失败，使用默认参数
	    {		
	    	slcal_eeprom_fault |= SLCAL_ERR_EEPROM;//E2故障状态字  = eeprom的错误状态字0x02 
	    	slcal_eeprom_fault_classify |= 0x01;//e2故障分类字 = 0x01
	   		#ifdef SLC_PRINT_ASSERT//断言打印宏
			ASSERT(0);
			#endif
			
	    	if (slcal_init_ctrl.init_ctrl_state == 0xffff)//初次上电,U16 init_ctrl_state所有位置(读出失败初次上电的情况下)
	    	{
				slcal_default_ctrl[i].relay_status=LAMP_OPEN;//默认开灯
				slcal_default_ctrl[i].brightness_status=100;//亮度
				slcal_default_ctrl[i].crc=CalcCRC16((U8 *)&slcal_default_ctrl[i],2);//校验
				
	    		slcal_init_ctrl.init_ctrl_state = 0;//灯初始化状态的值赋值0
	    		slcal_init_ctrl.crc=CalcCRC16((U8 *)&slcal_init_ctrl,2);//灯初始化的校验
	    		result=store_data_with_back((U8 *)(&slcal_init_ctrl),4,SL_DEFAULT_CTRL,SL_DEFAULT_CTRL+SL_BACK_OFFSET);//写缺省工作状态入
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
	    	else//非初次上电(读出失败非首次上电的情况下)
	    	{
	    		slcal_default_ctrl[i].relay_status=LAMP_OPEN;//默认开灯
				slcal_default_ctrl[i].brightness_status=100;
				slcal_default_ctrl[i].crc=0;
	    		slcal_init_ctrl.init_ctrl_state = 0;
	    		slcal_init_ctrl.crc=0;
	    	}

			slcal_check_signal[i]|=SLCAL_CHECK_CTRL;	//触发一次控制自检(读出失败的最后一定会触发一次自检)
		}
		else//读出成功
		{
			if(slcal_init_ctrl.init_ctrl_state & (0x0001<<i))//如果初始化控制的第i位为真这样设置
			{
				slcal_default_ctrl[i].relay_status=LAMP_CLOSE;//默认关灯
				slcal_default_ctrl[i].brightness_status=0;
				slcal_default_ctrl[i].crc=CalcCRC16((U8 *)&slcal_default_ctrl[i],2);
			}
			else//否则这样设置
			{
				slcal_default_ctrl[i].relay_status=LAMP_OPEN;//默认开灯
				slcal_default_ctrl[i].brightness_status=100;
				slcal_default_ctrl[i].crc=CalcCRC16((U8 *)&slcal_default_ctrl[i],2);
			}
			
			slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
			slcal_eeprom_fault_classify &= (~0x01);//读出成功的情况置的标志
		}//end读出成功
    	
    	//初始化受控状态参数
    	slcal_force_ctrl[i].last_time=0;
    	slcal_force_ctrl[i].relay_status=LAMP_OPEN;
    	slcal_force_ctrl[i].brightness_status=100;
    	slcal_force_ctrl[i].status=DISABLE_FORCECTRL;
    	slcal_force_ctrl[i].second_count=0;
    	slcal_force_ctrl[i].crc=CalcCRC16((U8 *)&slcal_force_ctrl[i],6);

    	//初始化当前控制状态
    	copy_U8((U8 *)&slcal_default_ctrl[i],(U8 *)&slcal_ctrl[i],4);
    	slcal_ctrl_type[i]=SLCAL_CTRL_DEFAULT;
    	
    	//初始化额定功率(初始化额定功率为什么要读呢,读肯定是为了先判断再写)
    	result=get_data_with_crc((U8 *)(&slcal_rated_power[i]),4,SL_RATED_POWER+4*i,SL_RATED_POWER+4*i+SL_BACK_OFFSET);//初始化额定功率
    	if(result!=SLCAL_EEPROM_FINISH)//读出失败，使用默认参数
    	{
    		slcal_eeprom_fault |= SLCAL_ERR_EEPROM;
    		slcal_eeprom_fault_classify |= 0x02;
    		#ifdef SLC_PRINT_ASSERT
    		ASSERT(0);
    		#endif
    		
    		if (slcal_rated_power[i].rated_power == 0xffff)//初次上电
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
	    	else//读出失败，非初次上电
	    	{
				slcal_rated_power[i].rated_power = 250;
				slcal_rated_power[i].crc = 0;
	    	}

    		slcal_check_signal[i]|=SLCAL_CHECK_CTRL;	//触发一次控制自检
    	}
    	else//莫名奇怪的这次读成功
    	{
    		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
    		slcal_eeprom_fault_classify &= (~0x02);
    	}
    	
    	//初始化调光参数
    	slcal_selfpwm[i].selfpwm_enable = 0;
    	slcal_selfpwm[i].selfpwm_execution = 0;
    	slcal_selfpwm[i].change_unit_cnt = 0;
    }
    
    
    //初始化经纬度
    result=get_data_with_crc((U8 *)(&slcal_longi_lati),6,SL_LONGI_LATI,SL_LONGI_LATI+SL_BACK_OFFSET);
	if(result!=SLCAL_EEPROM_FINISH)//读出失败，使用默认参数
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
		
		if (slcal_longi_lati.crc == 0xffff)//初次上电
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
			slcal_check_signal[i]|=SLCAL_CHECK_LOCAL_CTRL;	//触发一次本地控制自检
		}
	}
	else
	{
		slcal_eeprom_fault &= (~SLCAL_ERR_EEPROM);
		slcal_eeprom_fault_classify &= (~0x10);
	}
    
    
    //初始化本地参数 
    for(k=0; k<LOCAL_PACKAGE_MAX_NUM; k++)
    {
		result=get_data_with_crc(buffer,LOCAL_PACKAGE_MAX_LEN+2,SL_LOCAL_CTRL_PARA+k*(LOCAL_PACKAGE_MAX_LEN+2),SL_LOCAL_CTRL_PARA+k*(LOCAL_PACKAGE_MAX_LEN+2)+SL_BACK_OFFSET);
		if(result!=SLCAL_EEPROM_FINISH)//读出失败，使用默认参数
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
	if(result!=SLCAL_EEPROM_FINISH)//读出失败，使用默认参数
	{
		tmp &= 0; 
	}
	else
	{
		slcal_localctrl.local_package_num = buffer[0];
		tmp &= 1; 
	}
	result=get_data_with_crc(buffer,4,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+4,SL_LOCAL_CTRL_PARA+LOCAL_PACKAGE_MAX_NUM*(LOCAL_PACKAGE_MAX_LEN+2)+4+SL_BACK_OFFSET);
	if(result!=SLCAL_EEPROM_FINISH)//读出失败，使用默认参数
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
		if (slcal_localctrl.crc == 0xffff)//初次上电
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
			slcal_check_signal[i]|=SLCAL_CHECK_LOCAL_CTRL;	//触发一次本地控制自检
		}
	}
	localctrl_store_f = 0;//本地参数包存储完毕
	if(!slcal_localctrl.local_package_num) localctrl_sort_f = 0;
	else localctrl_sort_f = 1;//参数包需排序
	
}

/*************************************************************************
 函数名称：	ini_relay
 功能说明：	初始化继电器控制
 输入参数：
 返回参数：
 *************************************************************************/
void ini_relay()
{
    PDCTRL[7].all=0x08;//第一路继电器开关
    PDCTRL[0].all=0x08;//第二路继电器开关 修改为PDO 20151022
}

/*************************************************************************
 函数名称：	ini_timer2
 功能说明：	初始化timer2，作为PWM输出的时间基准，初始化定时周期2.5ms
 输入参数：
 返回参数：
 *************************************************************************/
void ini_timer2()
{
	CLOCK.PITUARTCLKEN.bit.Pit3ClkEn=1;

	PIT3.PCSR.all=0x0312;	//8分频
    PIT3.PMR=17999;			//定时周期修改为2.5ms(默认400Hz)
    
    slcal_dim.brightness_unit = BRIGHTNESS_UNIT[3];
    slcal_dim.dim_interval = 8;
    
	PIT3.PCSR.all=0x0313;
}

/*************************************************************************
 函数名称：	ini_oc0
 功能说明：	初始化ini_oc0，用来产生PWM波形并调节波形占空比, 时钟Timer修改为Timer2
 输入参数：
 返回参数：
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
 函数名称：	ini_oc1
 功能说明：	初始化ini_oc1，用来产生第二路PWM波形并调节波形占空比, 时钟Timer为Timer2
 输入参数：
 返回参数：
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
 函数名称：	ini_oc3
 功能说明：	初始化ini_oc3，用来产生开关电源用38KHz
 输入参数：
 返回参数：
 *************************************************************************/
/*void ini_oc3()
{
	CLOCK.OCCLKEN.all|=0x08;

	PCCTRL[6].all=0x18;//IO配置为OC3输出

	PLSR[18]=24;
	INT.NIER.all|=0x01000000;//第24级
	
	OC3.OCxR1=95;
	OC3.OCxR2=0;
	OC3.OCxCON.all=0x08c5;//关掉输入捕获中断 20150528
}*/

/*************************************************************************
 函数名称：	ini_slcal_ctrl
 功能说明：	初始化路灯控制函数，启动继电器和PWM功能
 输入参数：
 返回参数：
 *************************************************************************/
void ini_slcal_ctrl()
{
    U8 i;
    
    ini_timer2();	//供调光使用,初始化定时周期2.5ms(400Hz)
    
	ini_oc0();		

    ini_oc1();	
    
	ini_ctrl_para();//初始化灯控参数(默认开灯状态、调光参数、本地控制相关参数)

	ini_ctrl_data();//初始化灯控累积数据(开灯次数、开灯时间、带电关灯时间、判断复位类型)

	//ini_relay();//取消在此处的初始化动作 20151022 -> 移至:main->bsp_init->Ini_sys->Ini_Pad
	
    //ini_oc3();//增加第二路PWM产生38KHz初始化
    
    for(i = 0; i < MAX_LED_NUM; i++)
    {
    	if(slcal_ctrl[i].relay_status == LAMP_CLOSE)
    	{
        	if(Control_command_dispose(COMMAND_TYPE_OFF,0) == 1)//set_relay(i, 0);//关灯
        	{
	  	      slcal_ComunicationRetry.Control_succeed_flag = 1;
		   	  slcal_ComunicationRetry.Control_retry_times = 0;
	  	  	}
        }
        else
        {
        	if(Control_command_dispose(COMMAND_TYPE_ON,0) == 1)//set_relay(i, 1);//开灯 
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
