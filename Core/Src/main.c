/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "touch.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
#include "stm32746g_discovery_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HIST  550
#define WAVE_Y 40
#define WAVE_W 480
#define WAVE_H 140
#define DIG_Y 226
#define DIG_H 15
#define USE_SELF_LOOP 0
#define BTN_Y 186
#define BTN_H 32
#define VIEW_PTS 500
#define X_OF(i) ((uint16_t)(((uint32_t)(i) * (WAVE_W - 1)) / (VIEW_PTS - 1)))  //"第 i 个采样点"换成"第几个像素"


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//chuan kou 串口
uint8_t rx_byte;
volatile uint8_t rb[8192];    //huan xing huan chong   环形缓冲
volatile uint16_t rb_head = 0 , rb_tail = 0;     //rb_head——xie zhi 写指针(中断)     rb_tail——读指针（主循环）
volatile uint8_t  last_byte = 0;    //上一字节   判断帧头
volatile uint32_t rx_count = 0;      //累计收到字节数


char line_buf[64];
volatile uint16_t line_len = 0;
volatile uint32_t line_ok =0, line_err = 0;
volatile uint32_t  last_raw = 0;
volatile uint8_t last_kind =0 ;

volatile  uint8_t lcd_ret = 0xff;
//uint32_t t_text = 0;

//最后状态 用于显示
volatile uint32_t last_ch1 = 0;      //CH1 原始值
volatile uint32_t last_ch2 = 0;
volatile uint8_t last_dig = 0;
volatile uint32_t last_seq = 0;
volatile uint32_t last_frame_tick = 0;
volatile uint16_t crc_test = 0;
//button
uint8_t ch_mask = 0x1f;
uint8_t gear_sel = 0;
uint8_t run_flag = 1;



//zhen jie xi帧解析
uint8_t fr[260];         //接收帧缓冲
uint8_t tf[260];
uint16_t fr_len = 0;         //mu qian shou dao zi jie shu
uint8_t fr_st = 0;           //帧解析状态机                                     
volatile uint32_t frm_ok = 0;     //CRC通过帧数量
volatile uint32_t frm_crc_err = 0;        //CRC错误帧数量
volatile uint32_t frm_type_err = 0;
volatile uint32_t seq_err = 0;   //序列号跳变数

volatile uint16_t first_raw1 = 0;
volatile uint16_t first_raw2 = 0;
volatile uint8_t first_dig = 0;


//历史数据 绘图唯一数据
uint16_t hist1[HIST];    
uint16_t hist2[HIST];
uint8_t histd[HIST];

volatile uint8_t last_gear = 0;

//测量 变量  
uint32_t freq1_hz;
uint32_t freq2_hz;
uint32_t period1_us;
uint32_t period2_us;
uint32_t duty1_pct;
uint32_t duty2_pct;


uint32_t freqd1_hz;
uint32_t freqd2_hz;
uint32_t freqd3_hz;
uint32_t periodd1_us;
uint32_t periodd2_us;
uint32_t periodd3_us;
uint32_t dutyd1_pct;
uint32_t dutyd2_pct;
uint32_t dutyd3_pct;
char line3[40];
char line4[40];


uint8_t touch_ok =0;


uint16_t out_freq_hz = 100;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t crc16_ccitt_false(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  uint32_t i;
  uint8_t  j;
  for (i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (j = 0; j < 8; j++)
    {
      if (crc & 0x8000)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}
//————————————————————————————————————————————————CRC16校验———————————上方——————


static void  draw_minmax_line(const uint16_t *h , uint32_t color)
{                                                                        //在draw_wave里面用
  uint16_t mn = 4095,mx =0 ;
  int imx = 0;
  int i;
  int ymax ,ymin,xm;
  
  for(i =HIST - VIEW_PTS ; i<HIST ; i++)
  {
    if(h[i] < mn){mn = h[i];}
    if(h[i] > mx){mx = h[i]; imx = i;}
  }
  ymax = WAVE_Y + WAVE_H - 1 - (int)((uint32_t)mx * (WAVE_H - 1)/4095);
  ymin = WAVE_Y + WAVE_H - 1 - (int)((uint32_t)mn * (WAVE_H - 1)/4095);
  xm = (int)((uint32_t)(imx - (HIST - VIEW_PTS)) * (WAVE_W - 1)/(VIEW_PTS - 1));
  
  BSP_LCD_SetTextColor(color);
  
  BSP_LCD_DrawLine(0,(uint16_t)ymax,480,(uint16_t)ymax);
  BSP_LCD_DrawLine(0,(uint16_t)ymin,480,(uint16_t)ymin);
  
}
void draw_wave(void)
{
  int i;
  int y;
  int y_prev;
  //——————————————————————————————画网格——————————————————————————————————————————————————
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(0, WAVE_Y,WAVE_W,WAVE_H);
  
  BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
  for(i = 0;i<WAVE_W;i+=48)
  {
    BSP_LCD_DrawLine((uint16_t)i,WAVE_Y,(uint16_t)i,(uint16_t)(WAVE_Y + WAVE_H - 1));
  }
  
  for(i = 0;i<WAVE_H;i+=45)
  {
    BSP_LCD_DrawLine(0,(uint16_t)(WAVE_Y +i),(uint16_t)(WAVE_W - 1),(uint16_t)(WAVE_Y +i));
  }
  //——————————————————————————电压值————————————————————————————————————————————————————————————
//  static const char *lab[4] = {"3.3V","2.2V","1.1V","0V"};
//  int k;
//  
//  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
//  BSP_LCD_SetFont(&Font12);
//  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//  
//  for(k = 0;k<4 ; k ++)
//  {
//    BSP_LCD_DisplayStringAt(446,(uint16_t)(WAVE_Y + k * 45 + 1),(uint8_t *)lab[k],LEFT_MODE);
//  }
//  BSP_LCD_SetFont(&Font16);
  
//————————————————————————————————绘制CH1——————————————————————————————————————
  if(ch_mask &0x01)
  {
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  y_prev = WAVE_Y +WAVE_H - 1 - (int)(((uint32_t)hist1[HIST - VIEW_PTS] * (WAVE_H - 1))/4095);
  for(i = 1 ; i<VIEW_PTS ; i++)
  {
    y = WAVE_Y + WAVE_H - 1 - (int)(((uint32_t)hist1[HIST - VIEW_PTS+i] * (WAVE_H - 1))/4095);
    BSP_LCD_DrawLine(X_OF(i-1),(uint16_t)y_prev,X_OF(i),(uint16_t)y);
    y_prev = y;
  }
  }
  //————————————————————————————————绘制CH2——————————————————————————————————————
  if(ch_mask &0x02)
  {
  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
  y_prev = WAVE_Y + WAVE_H -1 -(int)(((uint32_t)hist2[HIST - VIEW_PTS] * (WAVE_H - 1)) / 4095);
  for(i = 1 ; i<VIEW_PTS ; i++)
  {
    y = WAVE_Y + WAVE_H - 1 - (int)(((uint32_t)hist2[HIST - VIEW_PTS+i] * (WAVE_H - 1))/4095);
    BSP_LCD_DrawLine(X_OF(i-1),(uint16_t)y_prev,X_OF(i),(uint16_t)y);
    y_prev = y;
  }
}
//if(ch_mask & 0x01){draw_minmax_line(hist1,LCD_COLOR_WHITE);}
//if(ch_mask & 0x02){draw_minmax_line(hist2,LCD_COLOR_ORANGE);}
}
static const uint16_t samples_us_tab[5] = {200 , 100, 20, 10, 2};

void meas_dig(const uint8_t mask , uint32_t *freq_hz, uint32_t *period_us, uint32_t *duty_pct)
{
  
  uint16_t first_edge = 0;
  uint16_t last_edge = 0;
  uint16_t edge_cnt = 0;
  uint16_t high_cnt= 0;
  uint32_t sample_us;
  uint8_t gear;
  int i;
  int prev  = 0;
  
  *freq_hz =0;  
  *period_us = 0;
  *duty_pct = 0;
  gear = last_gear;
  if(gear > 4 ){gear = 0;}
  sample_us = samples_us_tab[gear];
  
  
  for(i = HIST - VIEW_PTS;i<HIST;i++)
  {
    uint8_t cur = (histd[i] & mask )?1 :0;
    
    if(prev == 0  && cur == 1)
    {
      if(edge_cnt == 0){first_edge = i;}
      last_edge = i;
      edge_cnt ++;
    }
    prev = cur;
    }
  
  if(edge_cnt < 2){return;}
  if(last_edge < first_edge){return;}
  
  for(i = first_edge ; i<= last_edge;i++)
  {
    if(histd[i] & mask){high_cnt ++;}
  }
   *freq_hz =(uint32_t)(edge_cnt - 1)*1000000/((uint32_t)(last_edge - first_edge)*sample_us);  
  *period_us = (uint32_t)(last_edge - first_edge)*sample_us/(edge_cnt - 1);
  *duty_pct = (uint32_t)high_cnt * 100 /(uint32_t)(last_edge - first_edge + 1);

  }

void draw_dig(void)
{
  static const uint32_t col[3] = {LCD_COLOR_BLUE,LCD_COLOR_CYAN,LCD_COLOR_MAGENTA};
  static const uint8_t  bit[3] = {0x01,0x02,0x04};
  uint32_t d1_f,d2_f,d3_f,d1_p,d2_p,d3_p,d1_d,d2_d,d3_d;
  char d1text[24],d2text[24],d3text[24];
    
  uint32_t hi_us;                                 
  char tag[3];
  int ch;
  int i;
  int lane;
  int y_hi;
  int y_lo;
  int y;
  int y_prev;
  //——————————————————————画数字通道框线————————————————————————————————————————————
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(0,DIG_Y,WAVE_W,DIG_H * 3);   //DIG_Y 226   WAVE_W  480 DIG_H 15
  
  BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
  for(i = 0; i<WAVE_W;i+=48)
  {
    BSP_LCD_DrawLine((uint16_t)i,DIG_Y,(uint16_t)i,(uint16_t)(DIG_Y + DIG_H * 3 - 1));
  }
  //————————————————————————————依次for循环画数字通道线————————————————————————————————————————————————————
    for(ch = 0;ch<3;ch++)
    {
      if((ch_mask & (uint8_t)(0x04 <<ch)) == 0)       //为0说明被关掉了，就continue跳过不画   ch 0 1 2
      {continue;}
      lane = DIG_Y + ch * DIG_H;
      y_hi = lane + 3;
      y_lo = lane +DIG_H - 3;                         //两边内缩3，防止和其他通道混叠
     
      BSP_LCD_SetTextColor(col[ch]);
      
      y_prev = (histd[HIST - VIEW_PTS] & bit[ch])? y_hi :y_lo;
      
      for(i = 1;i< VIEW_PTS;i++)
      {
        y = (histd[HIST- VIEW_PTS +i] & bit[ch])? y_hi :y_lo;
        BSP_LCD_DrawLine(X_OF(i -1),(uint16_t)y_prev,X_OF(i),(uint16_t)y);
        y_prev = y;   
      }    
  }
  //————————分析数字通道数据——————————————————
  meas_dig(0x01,&d1_f,&d1_p,&d1_d);
  meas_dig(0x02,&d2_f,&d2_p,&d2_d);
  meas_dig(0x04,&d3_f,&d3_p,&d3_d);
  
  
  sprintf(d1text,"%luHz %lu%%",(unsigned long)d1_f,(unsigned long)d1_d);    
  sprintf(d2text,"%luHz %lu%%",(unsigned long)d2_f,(unsigned long)d2_d);   
  sprintf(d3text,"%luHz %lu%%",(unsigned long)d3_f,(unsigned long)d3_d);

//hi_us = (uint32_t)d2_p * d2_d / 100; 
//sprintf(d2text, "%luHz %lu.%02lu ms",
//        (unsigned long)d2_f,
//        (unsigned long)(hi_us / 1000),
//        (unsigned long)((hi_us % 1000) / 10));
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  
  //————————————————————————————画数字通道标志D1 D2 D3——————————————————————————————————
  for(ch = 0;ch < 3;ch++)
  {
    if((ch_mask & (uint8_t)(0x04 << ch)) == 0)
    { continue;
    }
    
    tag[0] = 'D';
    tag[1] = (char)('1' + ch);
    tag[2] = 0;
    
    BSP_LCD_SetTextColor(col[ch]);
    
    
    BSP_LCD_DisplayStringAt(2,(uint16_t)(DIG_Y + ch *DIG_H + 2),(uint8_t *)tag,LEFT_MODE);
    
    if(ch == 0)
    {
      BSP_LCD_DisplayStringAt(300,(uint16_t)(DIG_Y + ch * DIG_H + 2),(uint8_t *)d1text,LEFT_MODE);
    }
    if(ch ==1)
    {
      BSP_LCD_DisplayStringAt(300,(uint16_t)(DIG_Y + ch * DIG_H + 2),(uint8_t *)d2text,LEFT_MODE);
    }
    if(ch == 2)
    {
      BSP_LCD_DisplayStringAt(300,(uint16_t)(DIG_Y + ch * DIG_H + 2),(uint8_t *)d3text,LEFT_MODE);
    }
  }
  BSP_LCD_SetFont(&Font16);
}



void meas_analog(const uint16_t *samples , uint16_t vmin, uint16_t vmax, uint32_t *freq_hz, uint32_t *period_us, uint32_t *duty_pct)
{
  int mid_level;
  int hyst;
  int armed = 0;
  uint16_t first_edge = 0;
  uint16_t last_edge = 0;
  uint16_t edge_cnt = 0;
  uint16_t high_cnt= 0;
  uint32_t sample_us;
  uint8_t gear;
  int i;
  
  mid_level = ((int)vmin + (int)vmax)/2;
  hyst = ((int)vmax - (int)vmin)/8;
  if(hyst< 2){hyst =2;}
  
  for(i = HIST - VIEW_PTS;i<HIST;i++)
  {
    if((int)samples[i] < (mid_level - hyst)){armed = 1;}
    if((armed == 1) && (int)samples[i] > (mid_level +hyst))
    {
      armed =0;
      if(edge_cnt == 0){first_edge = (uint16_t)i;}
      last_edge = (uint16_t)i;
      edge_cnt++;
    }
  }
  *freq_hz =0;  
  *period_us = 0;
  *duty_pct = 0;
  
  
  if(edge_cnt < 2)return;
  if(last_edge <= first_edge )return;
  
  for(i = first_edge;i<= last_edge ;i++)
  {
    if((int) samples[i] > mid_level){high_cnt++;}
    
  }
  
  gear = last_gear;
  if(gear > 4 ){gear = 0;}
  sample_us = samples_us_tab[gear];
  
  *freq_hz =(uint32_t)(edge_cnt - 1)* 1000000UL / ((uint32_t)(last_edge - first_edge)* sample_us);  
  *period_us = (uint32_t)(last_edge - first_edge) *sample_us /(uint32_t)(edge_cnt - 1);
  *duty_pct = (uint32_t) high_cnt *100 / (uint32_t)(last_edge - first_edge);

}

void draw_meas(void)
{
  uint16_t mn1 = 4095;
  uint16_t mx1 = 0;
    uint16_t mn2 = 4095;
  uint16_t mx2 = 0;
  uint32_t pp1;
  uint32_t pp2;
  uint32_t vx1;
  uint32_t vn1;
  uint32_t vx2;
  uint32_t vn2;
  uint32_t efv1;
   char t1[40];
   char t2[40];
   int i;
   
   
   for(i = HIST - VIEW_PTS ;i<HIST ;i++)
   {
     uint16_t a = hist1[i];
     uint16_t b = hist2[i];
     
     if(mn1 > a) {mn1 = a;}
     if(mx1 < a) {mx1 =a;}
     if(mn2 > b) {mn2 = b;}
     if(mx2 < b) {mx2 =b;}

   }
  pp1 = (uint32_t)(mx1 - mn1) *3300/4095;
  pp2 = (uint32_t)(mx2 - mn2) *3300/4095;
  vx1 = (uint32_t)mx1 *3300/4095;
  vx2 = (uint32_t)mx2 *3300/4095;
  vn1 = (uint32_t)mn1 *3300/4095;
  vn2 = (uint32_t)mn2 *3300/4095;
//  efv1 = pp1/sqrt(2);     //开根号
  meas_analog(hist1, mn1 , mx1, &freq1_hz , &period1_us, &duty1_pct);
  meas_analog(hist2, mn2 , mx2, &freq2_hz , &period2_us, &duty2_pct);
  
  //CH1VPP 最大最小值
  sprintf(t1, "CH1 PP %lu.%02lu V",               //efv %lu.%02lu  MX %lu.%02lu MN %lu.%02lu
          (unsigned long)(pp1/1000),(unsigned long)((pp1 %1000)/10)
//          (unsigned long)(vx1/1000),(unsigned long)((vx1 %1000)/10),
//          (unsigned long)(vn1/1000),(unsigned long)((vn1 %1000)/10),
//           (unsigned long)(efv1/1000),(unsigned long)((efv1 %1000)/10)
          );
  //CH2VPP 最大最小值
  sprintf(t2, "CH2 PP %lu.%02lu V",              //  MX %lu.%02lu MN %lu.%02lu
          (unsigned long)(pp2/1000),(unsigned long)((pp2 %1000)/10)
//          (unsigned long)(vx2/1000),(unsigned long)((vx2 %1000)/10),
//          (unsigned long)(vn2/1000),(unsigned long)((vn2 %1000)/10)
          );
  //CH1频率 周期 占空比
  if(freq1_hz > 0)
  {
    sprintf(line3,"F %luHZ",          // D%lu%%   T %lu.%lums
            (unsigned long)freq1_hz
//             (unsigned long)(period1_us/1000)
//             (unsigned long)((period1_us%1000)/10),
//              (unsigned long)duty1_pct
                );
  }
  else
  {
    sprintf(line3,"F --- T--- D---");
  }
  
    //CH2频率 周期 占空比
   if(freq2_hz > 0)
  {
    sprintf(line4,"F %luHZ",          // D%lu%%     T %lu.%lums
            (unsigned long)freq2_hz
//             (unsigned long)(period2_us/1000)
//             (unsigned long)((period2_us%1000)/10),
//              (unsigned long)duty2_pct
                );
  }
  else
  {
    sprintf(line4,"F --- T--- D---");
  }
  
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(0,WAVE_Y +2,208,28);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(4,WAVE_Y + 4, (uint8_t*)t1, LEFT_MODE);
  BSP_LCD_DisplayStringAt(4,WAVE_Y + 17, (uint8_t*)line3, LEFT_MODE);
   BSP_LCD_DisplayStringAt(4,WAVE_Y + 30, (uint8_t*)t2, LEFT_MODE);
   BSP_LCD_DisplayStringAt(4,WAVE_Y + 43, (uint8_t*)line4, LEFT_MODE);
  BSP_LCD_SetFont(&Font16);
  
  
}

//UART控制
void send_ctrl(UART_HandleTypeDef *huart, uint8_t cmd, uint16_t param)
{
  uint8_t cf[9];
  uint16_t cc;
  
  cf[0] = 0xAA;
  cf[1] = 0x55;
  cf[2] = 0x02;
  cf[3] = cmd;
  cf[4] = (uint8_t)(param &0xff);
  cf[5] = (uint8_t)(param >> 8);
  cf[6] = 0x00;
  
  
  cc = crc16_ccitt_false(cf,7);
  
  cf[7] = (uint8_t)(cc & 0xff);
  cf[8] = (uint8_t)(cc>>8);
  
  HAL_UART_Transmit(huart,cf, 9,20);
  
  
}
//调节方波频率
void set_out_freq(uint16_t hz)
{
  if(hz < 100){hz = 100;}
  if(hz > 1000){hz = 1000;}
  
  out_freq_hz = hz;
  
#if !USE_SELF_LOOP
  send_ctrl(&huart6,0x07,out_freq_hz);
#endif
}

//画按钮函数
void draw_button(uint16_t x, uint16_t y,uint16_t w,uint16_t h,const char *s ,uint8_t on)
{
  uint32_t bg;
  bg = on ? LCD_COLOR_DARKGREEN : LCD_COLOR_DARKGRAY;
  
  BSP_LCD_SetTextColor(bg);
  BSP_LCD_FillRect(x,y,w,h);
  
  BSP_LCD_SetBackColor(bg);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt((uint16_t)(x + 6),(uint16_t)(y +10),(uint8_t *)s,LEFT_MODE);
}

//设计按钮数量类型
void draw_buttons(void)
{
  char txt[8];
  
  BSP_LCD_SetFont(&Font12);
  
  if(gear_sel == 0){sprintf(txt,"10ms");}
  else{sprintf(txt,"5ms");}
  
  draw_button(4,BTN_Y,54,BTN_H,txt,1);
  
  draw_button(63,BTN_Y,44,BTN_H,"CH1",(ch_mask & 0x01) ?1:0);
  draw_button(110,BTN_Y,44,BTN_H,"CH2",(ch_mask & 0x02) ?1:0);
  draw_button(157,BTN_Y,34,BTN_H,"D1",(ch_mask & 0x04) ?1:0);
  draw_button(194,BTN_Y,34,BTN_H,"D2",(ch_mask & 0x08) ?1:0);
  draw_button(231,BTN_Y,34,BTN_H,"D3",(ch_mask & 0x10) ?1:0);
  
  draw_button(268,BTN_Y,34,BTN_H,"Hz-",1);
  draw_button(305,BTN_Y,64,BTN_H,"Hz+",1);
  draw_button(342,BTN_Y,58,BTN_H,(run_flag ?"RUN":"HOLD"),1);
  draw_button(403,BTN_Y,68,BTN_H,"SAVE",1);
  
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font16);
}
//导出CSV
void export_csv(void)
{
  const char *hdr = "idx,ch1_mv,ch2_mv,d1,d2,d3\r\n";
  char line[40];
  int i;
  
  HAL_UART_Transmit(&huart1,(uint8_t *)hdr,(uint16_t)strlen(hdr),50);
  
  for(i = HIST -VIEW_PTS;i< HIST ;i++)
  {
    uint32_t mv1 = (uint32_t)hist1[i] *3300 /4095;
    uint32_t mv2 = (uint32_t)hist2[i] *3300 /4095;
    uint8_t  dg = histd[i];
    
    sprintf(line,"%d,%lu,%lu,%u,%u,%u\r\n",
            i - (HIST - VIEW_PTS),
            (unsigned long)mv1,  (unsigned long)mv2,
            (unsigned)((dg &0x01) ?1 :0),
            (unsigned)((dg &0x02) ?1 :0),
            (unsigned)((dg &0x04) ?1 :0)
            );
    HAL_UART_Transmit(&huart1,(uint8_t *)line,(uint16_t)strlen(line),20);
  }
  
}

void touch_action(uint16_t x, uint16_t y)
{
  if(y<BTN_Y || y>(BTN_Y + BTN_H)){return;}
  
  if(x>= 4&& x<= 59)
  {
    gear_sel = (gear_sel == 0)?1:0;

#if !USE_SELF_LOOP
    send_ctrl(&huart6,0x01,gear_sel);
#endif
  }
  else if(x>= 63 && x<=106)
  {
    ch_mask ^= 0x01;  
#if !USE_SELF_LOOP 
    send_ctrl(&huart6,0x02,ch_mask);
#endif
  }
  else if(x>= 110 && x <=153)
  {
    ch_mask ^= 0x02;
#if !USE_SELF_LOOP 
    send_ctrl(&huart6,0x02,ch_mask);
#endif    
  }
  else if(x>=157 && x<= 190)
  {
    ch_mask ^= 0x04;
#if !USE_SELF_LOOP 
    send_ctrl(&huart6,0x02,ch_mask);
#endif
  }
  else if(x>=194 && x<= 227)
  {
    ch_mask ^= 0x08;
#if !USE_SELF_LOOP 
    send_ctrl(&huart6,0x02,ch_mask);
#endif
  }
  
  else if(x>=231 && x<= 264)
  {
    ch_mask ^= 0x10;
#if !USE_SELF_LOOP 
    send_ctrl(&huart6,0x02,ch_mask);
#endif
  }
  
  else if(x>= 268 && x <= 301)
  {
    set_out_freq((uint16_t)(out_freq_hz - 100));
  }
  else if(x>= 305 && x <= 338)
  {
    set_out_freq((uint16_t)(out_freq_hz + 100));
  }
  else if(x>=342 && x<= 399)
  {
    run_flag = (run_flag == 0)?1:0;
  }
  else if(x>=403 && x<= 470)
  {
    export_csv();
  }
  else{return;}
  
  draw_buttons();
}

void touch_service(void)
{
  static uint8_t touched = 0;
  static uint32_t t_touch = 0;
  uint16_t x =0;
  uint16_t y = 0;
  
  if(HAL_GetTick() - t_touch < 20){return;}
  
  t_touch = HAL_GetTick();
  
  if(touch_read(&x,&y))
  {
    if(touched == 0)
    {
      touched =1;
      touch_action(x,y);
    }
  }
  else
  {
    touched = 0;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
   MPU_Region_InitTypeDef mpu = {0};

  HAL_MPU_Disable();

  mpu.Enable           = MPU_REGION_ENABLE;
  mpu.Number           = MPU_REGION_NUMBER1;
  mpu.BaseAddress      = 0xC0000000;
  mpu.Size             = MPU_REGION_SIZE_32MB;
  mpu.SubRegionDisable = 0x00;
  mpu.TypeExtField     = MPU_TEX_LEVEL0;
  mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  mpu.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  mpu.IsCacheable      = MPU_ACCESS_CACHEABLE;
  mpu.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&mpu);

  mpu.Number           = MPU_REGION_NUMBER2;
  mpu.BaseAddress      = 0xA0000000;
  mpu.Size             = MPU_REGION_SIZE_8KB;
  mpu.SubRegionDisable = 0x00;
  mpu.TypeExtField     = MPU_TEX_LEVEL0;
  mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  mpu.IsShareable      = MPU_ACCESS_SHAREABLE;
  mpu.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  mpu.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&mpu);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

  lcd_ret = BSP_LCD_Init();

  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_DisplayOn();
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(8, 8, (uint8_t *)"LCD OK", LEFT_MODE);

//  BSP_LCD_SetTextColor(LCD_COLOR_RED);
//  BSP_LCD_DrawLine(0, 136, 479, 136);

  HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
  
  MX_I2C3_Init();
  touch_ok = touch_init();
  draw_buttons();
  {
    char msg[20];
    if(touch_ok) {sprintf(msg,"TOUCH OK\r\n");}
    else{sprintf(msg,"TOUCH ERR %u\r\n",(unsigned)touch_error());}
    HAL_UART_Transmit(&huart1,(uint8_t *)msg,(uint16_t)strlen(msg),20);
  }
  
  
  
  
  uint8_t crc_vec[] = "123456789";
  crc_test = crc16_ccitt_false(crc_vec,9);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
#if USE_SELF_LOOP
    static uint32_t t_tx = 0;
    static uint16_t tx_seq  = 0x1234;
    
    if(HAL_GetTick() - t_tx >= 20)
    {
      uint16_t k;
      uint16_t cc;
      
      t_tx = HAL_GetTick();
      tf[0] = 0xAA;
      tf[1] = 0x55;
      tf[2] = 0x01;
      tf[3] = (uint8_t)(tx_seq & 0xff);
      tf[4] = (uint8_t)(tx_seq >> 8);
      tf[5] = 0x00;
      tf[6] = 0x1F;
      tf[7] = 50;
      
      for(k = 0; k<50 ;k++)
      {
       uint16_t n =(uint16_t)((tx_seq * 50 + k)%50);
       uint16_t raw1 = (n < 25)?3500 : 500;
       uint16_t raw2 = (uint16_t)((tx_seq * 5 + k )&0x0fff);
//       uint16_t raw1 = (uint16_t)((tx_seq * 7 + k )&0x0fff);
//       uint16_t raw2 = (uint16_t)((tx_seq * 5 + k )&0x0fff);

       //三角波测试
//       uint16_t raw1 = (n < 25) ? (uint16_t)(n*170):(uint16_t)((49 - n) * 170);
       
       uint16_t w1 = (uint16_t)(raw1 << 4);
       uint16_t w2 = (uint16_t)(raw2 << 4);
       
       tf[8 + k * 5 + 0] = (uint8_t)(w1 & 0xff);
       tf[8 + k * 5 + 1] = (uint8_t)(w1>> 8);
       tf[8 + k * 5 + 2] = (uint8_t)(w2 & 0xff);
       tf[8 + k * 5 + 3] = (uint8_t)(w2>> 8);
       tf[8 + k * 5 + 4] = (uint8_t)(tx_seq & 0x07);
      }
      
      cc = crc16_ccitt_false(tf, 258);
      tf[258] = (uint8_t)(cc & 0xff);
      tf[259] = (uint8_t)(cc>>8);
      
      HAL_UART_Transmit(&huart6,tf,260,100);
      tx_seq++;
    }
#endif  
    
        touch_service();
    
//    ——————————————————————自测控制发送————————————————————————————————————————————————————————————————
//   static uint32_t t_cmd = 0;
//   if(HAL_GetTick() - t_cmd >= 1000)
//   {
//     t_cmd = HAL_GetTick();
//     send_ctrl(&huart1, 0x06,0);
//   }
    
//———————————————————————————————————临时轮询验证坐标—————————————————————————————————————————————————
//    static uint32_t t_touch =0 ;
//    if(HAL_GetTick() - t_touch >= 100)
//    {
//      uint16_t tx = 0;
//      uint16_t ty = 0;
//        
//        t_touch = HAL_GetTick();
//      
//      if(touch_read(&tx,&ty))
//      {
//        char msg[24];
//        sprintf(msg,"T %u %u\r\n",(unsigned)tx,(unsigned)ty);
//        BSP_LCD_SetFont(&Font12);
//        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//        BSP_LCD_DisplayStringAt(2,18,(uint8_t *)msg,LEFT_MODE);
////        HAL_UART_Transmit(&huart1,(uint8_t *)msg,(uint16_t)strlen(msg),20);
//      }
//    }
//  ——————————————————————————————下边是xy交叉线——————————————————————————————————————————————  
//    static uint32_t t_touch =0 ;
//    if(HAL_GetTick() - t_touch >= 100)
//    {
//      uint16_t tx = 0;
//      uint16_t ty = 0;
//        
//        t_touch = HAL_GetTick();
//        if(touch_read(&tx,&ty))
//            {
//                    if(tx <WAVE_W && ty <272)
//                    {
//                      int x = (int)tx;
//                      int y = (int)ty;
//                      
//                      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//                      
//                        BSP_LCD_DrawLine(0,(uint16_t)y,480,(uint16_t)y);
//              
//                        BSP_LCD_DrawLine((uint16_t)x,41,(uint16_t)x,219);
//                      
//                    }
//            }
//    }

    
//    __________________________________________________________________
////    //单次发帧dancifazhen
//    static uint8_t tf_sent = 0;
//    if(tf_sent == 0)
//    {
//      uint16_t k;
//      uint16_t cc;
//      
//      tf_sent = 1;
//      tf[0] = 0xAA;
//      tf[1] = 0x55;
//      tf[2] = 0x01;
//      tf[3] = 0x34;
//      tf[4] = 0x12;
//      tf[5] = 0x00;
//      tf[6] = 0x1F;
//      tf[7] = 50;
//      
//      for(k = 0; k<50 ;k++)
//      {
//        uint16_t w1 = (uint16_t)((100+k)<<4);
//        uint16_t w2 = (uint16_t)((200+k)<<4);
//        tf[8 + k *5 + 0] = (uint16_t)(w1 & 0xFF);
//        tf[8 + k *5 + 1] = (uint16_t)(w1 >> 8);
//        tf[8 + k *5 + 2] = (uint16_t)(w2 & 0xFF);
//        tf[8 + k *5 + 3] = (uint16_t)(w2>> 8);
//        tf[8 + k * 5 + 4] = 0x05;
//      }
//      
//      cc = crc16_ccitt_false(tf, 258);
//      tf[258] =(uint8_t)(cc &0xFF);
//      tf[259] =(uint8_t)(cc >>8);
//     HAL_UART_Transmit(&huart6,tf,260,100);
//    }
 
//——————————————————————————————————————————————————————————————————————————————————————————————
//    验证UART发字符
//    if(HAL_GetTick() - t_text >= 1000)
//    {
//      t_text = HAL_GetTick();
//      uint8_t msg[] = "A1,2048\r\n";
//      HAL_UART_Transmit(&huart6, msg,sizeof (msg) - 1,20);
//    }
//    if(HAL_GetTick() - t_text >= 3000)
//    {
//      t_text = HAL_GetTick();
//      static uint8_t test_step = 0;
//      uint8_t m0[] = "A2,100,200,1\r\n";
//      uint8_t m1[] = "A2,300,400,2\r\n";
//      uint8_t m2[] = "A2,500,600,4\r\n";
//      
//      if(test_step == 0)
//      {
//        HAL_UART_Transmit(&huart6, m0,sizeof (m0) - 1,20);
//      }
//      else if(test_step == 1)
//      {
//        HAL_UART_Transmit(&huart6, m1,sizeof (m1) - 1,20);
//      }
//      else
//      {
//         HAL_UART_Transmit(&huart6,m2,sizeof (m2) - 1,20);
//      }
//      
//      test_step++;
//      if(test_step >= 3)
//      {
//        test_step =0;
//      }
//    }
    
//————————————————————————————————————————————————————————————————————————————————————————————————————————
//    while (rb_tail != rb_head)
//    {
//      uint8_t b  = rb[rb_tail];
//      rb_tail = (uint16_t)((rb_tail + 1) & 8191);
//      rx_count++;
//      
//      if(b == '\n')
//      {
//        unsigned int a = 0,b =0, c =0 ;
//        line_buf[line_len] = '\0';
//        
//        if(line_len > 0 && line_buf[line_len-1]=='\r')
//        {
//          line_buf [line_len -1] = '\0';
//        }
//        if(sscanf(line_buf, "A2,%u,%u,%u",&a,&b,&c)== 3)
//        {
//          last_ch1 =a;
//          last_ch2 =b;
//          last_dig = (uint8_t)c;
//          last_kind = '2';
//          line_ok++;
//        }
//        else if(sscanf(line_buf,"A1,%u",&a)== 1)
//          {
//            last_raw =a;
//            last_kind = '1';
//            line_ok++;
//          }
//        
//        else
//        {
//          line_err++;
//        }
//        line_len = 0;
//        
//      }
//      else
//      {
//        if(line_len < 63)
//        {
//          line_buf[line_len] = (char)b;
//          line_len++;
//        }
//        else
//        {
//          line_len = 0;
//        }
//      }
//    }
// ——————————————————————————————————————————————————————————————————————   
        while (rb_tail != rb_head)
    {
      uint8_t b = rb[rb_tail];
      rb_tail= (uint16_t)((rb_tail + 1) & 8191);
      rx_count++;
      
      if(fr_st == 0)
      {
        if(b == 0xAA)
        {
          fr[0] = b;
          fr_len = 1;
          fr_st = 1;

        }
      }
        else if(fr_st == 1)
        {
          if(b == 0x55)
          {
            fr[1] = b;
            fr_len = 2;
            fr_st = 2;
          }
          else if(b != 0xAA)
          {
            fr_len = 0;
            fr_st = 0;
          }
        }
        else
        {
          fr[fr_len] = b;
          fr_len++;
          
          if(fr_len >= 260)
          {
            if(fr[2] != 0x01)
            {
              frm_type_err++; 
            }
            else
            {
              uint16_t c_calc = crc16_ccitt_false(fr , 258);
              uint16_t c_recv = (uint16_t)(fr[258]|((uint16_t)fr[259]<<8));
              
              if(c_calc != c_recv)
              {
                frm_crc_err++;
              }
              else
              {
                uint16_t seq = (uint16_t)(fr[3]|((uint16_t)fr[4]<< 8 ));
                  
                memmove(hist1,hist1 + 50,(HIST - 50)*sizeof(uint16_t));
                memmove(hist2,hist2 + 50,(HIST - 50)*sizeof(uint16_t));
                memmove(histd,histd + 50,(HIST - 50)*sizeof(uint8_t));
                if(frm_ok > 0 && seq != (uint16_t)(last_seq + 1))
                {
                  seq_err+= (uint16_t)(seq - last_seq + 1 );
                }
                last_seq = seq;
                last_gear = fr[5];                
                {
                  uint16_t k;
                  for (k = 0;k<50;k++)
                  {
                    uint16_t w1 = (uint16_t)(fr[8 + k * 5]|((uint16_t)fr[8 + k*5+1] << 8));
                    uint16_t w2 = (uint16_t)(fr[8 + k * 5+2]|((uint16_t)fr[8 + k*5+3] << 8));
                    uint16_t r1 = (uint16_t) (w1 >> 4);
                    uint16_t r2 = (uint16_t) (w2 >> 4);
                    uint8_t dg = fr[8 + k * 5 +4];
                    
                    if(k ==0)
                    {
                      first_raw1 = r1;
                      first_raw2 = r2;
                      first_dig = dg;
                    }
                    last_ch1 = r1;
                    last_ch2 = r2;
                    last_dig = dg;
                    
                    
                   hist1[HIST - 50 +k] = r1;
                   hist2[HIST - 50 +k] = r2;
                   histd[HIST - 50 +k] = dg;
                  }
                }
                frm_ok++;
                last_frame_tick  = HAL_GetTick();
              }
            }
            fr_len = 0;
            fr_st = 0;
          }
        }
      }
    static uint32_t t_lcd = 0;
    
    static uint32_t t_fps = 0  , last_ok = 0, fps = 0;
    
    if(HAL_GetTick() - t_fps >= 1000)
    {
      fps = frm_ok - last_ok;
      last_ok = frm_ok;
      t_fps = HAL_GetTick();
    }
//    char txt1[32];
//    sprintf(txt1, "FPS %lu", (unsigned long)fps);
//    BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
//    BSP_LCD_DisplayStringAt(400, 2, (uint8_t *)txt1, LEFT_MODE);
    if(HAL_GetTick() - t_lcd >= 200)
    {
      char txt[32];
//      uint32_t tot = frm_crc_err + frm_ok;
//      uint32_t pl;
//      if (tot != 0)
//      {
//        pl = seq_err * 100/ tot;
//      }
//      else
//      {
//        pl = 0; //总帧数为 0，避免除零 
//      }
      
      
      
      t_lcd = HAL_GetTick();
      
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                                                                    //      BSP_LCD_FillRect(0,0,300,160);     
      BSP_LCD_FillRect(0,0,480,36);
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
      BSP_LCD_SetFont(&Font16);
      
      sprintf(txt,"FRM %lu SEQ %u  CE %lu  SE  %lu",
              (unsigned long)frm_ok,
              (unsigned long)last_seq,
              (unsigned long)frm_crc_err,
              (unsigned long)seq_err    //求丢包数时候加上
              );
      BSP_LCD_DisplayStringAt(4,2,(uint8_t *)txt,LEFT_MODE);
      
//      sprintf(txt,"CH1 %u  CH2 %u  DIG %u",(unsigned)(last_ch1),(unsigned)(last_ch2),(unsigned)last_dig);
//      BSP_LCD_DisplayStringAt(4,18,(uint8_t *)txt,LEFT_MODE);
      
//      可以换成具体数值
//      sprintf(txt," CH1 %lumV CH2 %lumV D%u",
//        (unsigned long)(last_ch1*3300UL/4095),
//        (unsigned long)(last_ch2*3300UL/4095),
//        (unsigned)last_dig);
//      BSP_LCD_DisplayStringAt(4,18,(uint8_t *)txt,LEFT_MODE);
//      ————————————————————————————G值————————————————————————————————
//      sprintf(txt,"G%u",(unsigned)last_gear);
//      BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
//      BSP_LCD_DisplayStringAt(4,18,(uint8_t *)txt,LEFT_MODE);
      

      
      if((HAL_GetTick() - last_frame_tick) > 500)
      {
         BSP_LCD_SetTextColor(LCD_COLOR_RED);
         BSP_LCD_DisplayStringAt(100,18,(uint8_t *)"LINK LOST",LEFT_MODE);  //前面加上的时候需要右移
      }
      else
      {
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
         BSP_LCD_DisplayStringAt(100,18,(uint8_t *)"LINK OK",LEFT_MODE);
      }
      //控制方波频率
//      sprintf(txt,"OUT %uHz",(unsigned)out_freq_hz);
//      BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
//      BSP_LCD_DisplayStringAt(2,18,(uint8_t *)txt,LEFT_MODE);
//      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
      
      
//      sprintf(txt,"FRM %lu",(unsigned long)frm_ok);
//      BSP_LCD_DisplayStringAt(8,4,(uint8_t *)txt,LEFT_MODE);
//      
//      sprintf(txt,"SEQ %u",(unsigned)last_seq);
//      BSP_LCD_DisplayStringAt(8,30,(uint8_t *)txt,LEFT_MODE);
//      
//      sprintf(txt,"CH1 %u",(unsigned)last_ch1);
//      BSP_LCD_DisplayStringAt(8,56,(uint8_t *)txt,LEFT_MODE);
//      
//      sprintf(txt,"CH2 %u",(unsigned)last_ch2);
//      BSP_LCD_DisplayStringAt(8,82,(uint8_t *)txt,LEFT_MODE);
//      
//      sprintf(txt,"DIG %u",(unsigned)last_dig);
//      BSP_LCD_DisplayStringAt(8,108,(uint8_t *)txt,LEFT_MODE);
//      
//      sprintf(txt,"CE %lu  SE  %lu",(unsigned long)frm_crc_err,(unsigned long)seq_err);
//      BSP_LCD_DisplayStringAt(8,134,(uint8_t *)txt,LEFT_MODE);
    }
    static uint32_t t_wave = 0;
      if(HAL_GetTick() - t_wave >= 100)
      {
        t_wave = HAL_GetTick();
        if(run_flag)
        {       
        draw_wave();
        draw_dig();
        draw_meas();
        }
      }
      BSP_LCD_SetFont(&Font12); 
      BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      BSP_LCD_DisplayStringAt(300, 20, (uint8_t *)"Li Kaixu 23211239", LEFT_MODE);
      BSP_LCD_DisplayStringAt(300, 40, (uint8_t *)"Zhan Lin 23271081", LEFT_MODE);
      BSP_LCD_SetFont(&Font16);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
  uint16_t next = (uint16_t)((rb_head + 1) & 8191);
  if(next != rb_tail)
  {
    rb[rb_head] = rx_byte;
    rb_head = next;
  }
  HAL_UART_Receive_IT(&huart6, &rx_byte,1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if( huart->Instance == USART6)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart6, &rx_byte,1);
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
