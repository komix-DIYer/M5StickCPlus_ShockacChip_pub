/* 
 * M5StickCPlus_ShockacChip
 * ショッカクチップ6DoF-P018のデータ取得プログラム
 * - 取得データ：Fx, Fy, Fz, Mx, My, Mz, T;
 * - UART送信 (有線/Bluetooth)
 * - 単純移動平均による平滑化
 * - M5StickCPlusのBボタン押下時に零点補正
 */

#include <M5StickCPlus.h>
#include "BluetoothSerial.h"
//#include <Wire.h>

BluetoothSerial SerialBT;

int16_t LCD_HEIGHT, LCD_WIDTH;

#define I2C_ADDR 0x00 // ショッカクチップのI2Cアドレス (製品に付属する仕様書を参照)
#define DATA_NUM 7 // ショッカクチップの出力データ数
#define BUFF_NUM 10 // 移動平均のサンプリング数

float D[DATA_NUM], D_buff[DATA_NUM][BUFF_NUM], D_sma[DATA_NUM], D_sum[DATA_NUM], D_off[DATA_NUM];
float V_calib[DATA_NUM] = {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00}; // ショッカクチップの出力値から物理量への変換係数 (製品に付属する仕様書を参照)
int off_cnt = -1, off_num = -1;

// データ ( Fx, Fy, Fz, Mx, My, Mz, T ) の取得
void getFT()
{
  int temp;
  
  Wire.beginTransmission(I2C_ADDR); // Slave Address 7bit
  Wire.endTransmission(); // Recieve ACK
  
  // Recieve Data1 (Binary Counter, 14)
  Wire.requestFrom(I2C_ADDR, 1);
  if ( Wire.available() ) temp = Wire.read();
  
  // Recieve Data2~15
  if ( temp==14 )
  {
    Wire.requestFrom(I2C_ADDR, 14);
    
    for ( int i=0; i<14; i++ )
    {
      // Bind H&L bit of the dada
      if (Wire.available()) temp = Wire.read();
      temp = temp << 8;
      if (Wire.available()) temp = temp | Wire.read();
      
      switch (i)
      {
        case 0:
          D[0] = temp * V_calib[0] - D_off[0]; // [N]
        case 2:
          D[1] = temp * V_calib[1] - D_off[1];
        case 4:
          D[2] = temp * V_calib[2] - D_off[2];
        case 6:
          D[3] = temp * V_calib[3] - D_off[3]; // [Nm]
        case 8:
          D[4] = temp * V_calib[4] - D_off[4];
        case 10:
          D[5] = temp * V_calib[5] - D_off[5];
        case 12:
          D[6] = temp * V_calib[6]; // - D_off[6]; // [C]
      }
      
      i++;
    }
  }
}

// データの移動平均 (サンプリング数はBUFF_NUM)
void calcSMA(void)
{
  for (int i = 0; i < DATA_NUM; i++)
  {
    for (int j = 0; j < BUFF_NUM - 1; j++)
      D_buff[i][j] = D_buff[i][j+1];

    D_buff[i][BUFF_NUM-1] = D[i];
  }
  
  for (int i = 0; i < DATA_NUM; i++)
  {
    float sma = 0;
    for (int j = 0; j < BUFF_NUM; j++)
      sma = sma + D_buff[i][j];
    
    D_sma[i] = sma/BUFF_NUM;
  }
}

// データのオフセット量の算定 (サンプリング数はoff_num)
// off_num に正の値が与えられたとき実行される (初期値はoff_num = -1)
void getOffset()
{
  // 与えられたサンプリング数off_numをカウンタoff_cntにセット
  if ( off_num > 0 && off_cnt < 0 )
    off_cnt = off_num - 1;

  // カウンタが正である限りデータの移動平均値を積算 (カウンタはディクリメント)
  // カウンタが0のときオフセット量を確定 (積算値はリセット)
  // カウンタが負になった瞬間に off_num = -1 とする (この処理を一巡したら抜ける)
  if (off_cnt >= 0)
  { 
    for ( int i = 0; i < DATA_NUM; i++ )
    {
      D_sum[i] = D_sum[i] + D_sma[i];

      if (off_cnt == 0)
      {
        D_off[i] = D_off[i] + D_sum[i] / off_num;
        D_sum[i] = 0;
      }
    }

    off_cnt--;

    if ( off_cnt < 0)
      off_num = -1;
  }
  else // off_cnt < 0
  {
    off_num = -1;
  }
}

void setup()
{
  M5.begin(); // LCD:true, AXP:true, UART:true, -, https://qiita.com/penguinprogrammer/items/5abf3e4f9583e41e9cfa
  
  //Wire.begin();
  //Wire.begin(26, 32); // for M5ATOM
  Wire.begin(32, 33); // for M5StickCPlus
  
  //Serial.begin(115200);
  //while (!Serial);
  //Serial.println();
  
  SerialBT.begin("S-chip");
  
  //M5.Lcd.clear(BLACK);
  M5.Lcd.setRotation(0); // LDC表示の回転 (3:Aボタンが左側)
  LCD_HEIGHT = M5.Lcd.height();
  LCD_WIDTH = M5.Lcd.width();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.println(("S-chip"));
  M5.Lcd.drawLine(0, 30, LCD_WIDTH, 30, WHITE);

  for (int i = 0; i < DATA_NUM; i++)
  {
    D_sma[i] = 0.0;
    D_off[i] = 0.0;
    
    for (int j = 0; j < BUFF_NUM; j++)
      D_buff[i][j] = 0.0;
  }
}

void loop()
{
  M5.update();
  
  // オフセット量算定サンプリング数の入力 (有線シリアル通信時)(50~100)
  if ( Serial.available() > 0 )
  {
    String myString = Serial.readString();
    Serial.println(myString);
    off_num = myString.toInt();
    Serial.println(off_num);
  }
  
  getFT();
  calcSMA();
  getOffset();
  
  Serial.print(D_sma[0]); Serial.print(", ");
  Serial.print(D_sma[1]); Serial.print(", ");
  Serial.print(D_sma[2]); Serial.print(", ");
  Serial.print(D_sma[3]); Serial.print(", ");
  Serial.print(D_sma[4]); Serial.print(", ");
  Serial.print(D_sma[5]); Serial.print(", ");
  Serial.print(D_sma[6]);
  
  Serial.println();
  
  SerialBT.print(D_sma[0]); SerialBT.print(", ");
  SerialBT.print(D_sma[1]); SerialBT.print(", ");
  SerialBT.print(D_sma[2]); SerialBT.print(", ");
  SerialBT.print(D_sma[3]); SerialBT.print(", ");
  SerialBT.print(D_sma[4]); SerialBT.print(", ");
  SerialBT.print(D_sma[5]); SerialBT.print(", ");
  SerialBT.print(D_sma[6]);
  
  SerialBT.println();
  
  // 電源ボタンが押されたらリセット
  if (M5.Axp.GetBtnPress()) ESP.restart();

  // Aボタンが押されたら... (ビープ音)
  if (M5.BtnA.isPressed())
  {
    M5.Beep.tone(4000);
    delay(100);
    M5.Beep.mute();
    
    while(M5.BtnA.isPressed())
    {
      M5.update();
      delay(10);
    }
  }
  
  // Bボタンが押されたら零点補正 (100サンプリング)
  if (M5.BtnB.isPressed())
  { 
    off_num = 100;
    
    while(M5.BtnB.isPressed())
    {
      M5.update();
      delay(10);
    }
  }
  
  delay(50);
}
