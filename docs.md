## 第1章: システム概要

**目的**: STM32F103VET6 マイコン上で，MPU6000（9軸IMU），MAG3110（磁力計），BMP280（気圧計）などのセンサーを読み取り，機体姿勢を推定し，PID 制御でサーボを駆動する飛行制御システム。

**機能構成**:

1. ハードウェア定義 (ピン、I2C アドレス、マクロ)
2. 初期化処理 (クロック、GPIO、SysTick, NVIC)
3. 周辺機能設定 (タイマー PWM, I2C, USART, ADC/DMA)
4. センサー読み取り (I2C 通信, データ変換)
5. 姿勢推定 (Madgwick AHRS)
6. 制御ループ (TIM6 割り込み + PID ロジック)
7. 割り込みハンドラ (受信、例外、1kHz タイマー)
8. ユーティリティ関数 (\_delay, sendData など)

## 第2章: ハードウェア定義とマクロ

* `#define ACCEL_2G_LSB 16384.0` など: センサー固有の分解能変換係数.
* `MPU6000_ADDRESS 0b11010010` 等: I2C アドレスおよびレジスタ定義.
* `TIM1_CH1 = GPIO_Pin_9` など: PWM 出力チャネルのピン定義.
* 割り込み優先度マクロ: `EXTI0_PRIORITY` など.

これらマクロでピンやレジスタ番号を一元管理し，後の設定関数で利用。

## 第3章: 初期化処理

この章では，システム起動時に必須となるマイコン周辺機能の初期化処理を解説します。各関数の中で行っている具体的な操作と，関連する専門用語の意味・役割も合わせて説明します。

---

### 3.1 BoardInit()

```c
void BoardInit(void) {
  // ※STM32F1 用のシステムクロック設定
  // 内部 PLL, 外部クリスタル等の設定を行い，CPU クロック（72MHz）を生成
  SystemInit();      // CMSIS 標準関数
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  // 以降，RCC_Clocks.HCLK=72MHz を前提にペリフェラル設定を行う
}
```

* **SystemInit()**: ベンダ提供のスタートアップコードで，リセット後にクロックソースを設定する CMSIS 関数。
* **RCC**: Reset and Clock Control。バスクロックやペリフェラルクロックのゲート管理。

---

### 3.2 GPIO\_Configuration()

```c
void GPIO_Configuration(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // 1) JTAG 無効化してデバッグポートのピンを GPIO として開放
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

  // 2) LED 用 GPIOD PD4 をプッシュプル出力モードに設定
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  // 3) LED 初期状態を HIGH (点灯)
  GPIO_SetBits(GPIOD, GPIO_Pin_4);
}
```

* **GPIO (General Purpose Input/Output)**: 汎用入出力ピン。CPU から制御できるデジタル I/O。
* **クロック有効化** (`RCC_APB2PeriphClockCmd`): GPIOD のレジスタを書き込む前に，RCC 側でクロックをオンにする必要がある。
* **GPIO\_Mode\_Out\_PP**: プッシュプル出力 (Push-Pull)。Pin 高と Pin 低の両方を能動的に駆動。
* **JTAG 無効化**: JTAG/SWD デバッグ用ピンをリリースし，一般 GPIO として使えるようにする設定。

---

### 3.3 SysTick\_Configuration()

```c
void SysTick_Configuration(void) {
  // SysTick.LOAD: カウンタのリロード値 (24bit)
  SysTick->LOAD = 0xFFFFFF;
  // CTRL: ENABLE + TICKINT:disable 割り込み, CLKSOURCE=AHB
  SysTick->CTRL = 0x000005;
}
```

* **SysTick**: Cortex-M コア内蔵タイマ。24ビットカウンタで簡易タイムベースを生成。
* **LOAD**: カウンタのリセット値。ここでは最大値 0xFFFFFF を設定して常時カウントダウン。
* **CTRL フラグ**:

  * ビット0 ENABLE: カウンタ有効
  * ビット1 TICKINT: 割り込み生成（0=割り込みなし）
  * ビット2 CLKSOURCE: クロックソース (1=AHB 系)
* **用途**: 高分解能タイミング測定 (パルス幅計測など) に使用。

---

### 3.4 EXTI\_Configuration() & NVIC 設定

```c
void EXTI_Configuration(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // 1) AFIO と GPIOE のクロック有効化
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOE, ENABLE);

  // 2) PE0〜PE5 をプルダウン入力モードに設定
  GPIO_InitStructure.GPIO_Pin  = 0x003F;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  // 3) EXTI ラインと GPIO ポート/ピンを結び付け
  for(int i=0;i<6;i++) {
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, i);
  }

  // 4) ライン0 は立ち上がり+立下り，ライン1〜5 は立下り割り込み
  EXTI_InitStructure.EXTI_Line    = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  EXTI_InitStructure.EXTI_Line    = 0x003E; // ライン1〜5ビットマスク
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_Init(&EXTI_InitStructure);

  EXTI_ClearITPendingBit(0x003F);

  // 5) NVIC 優先度グループ設定
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  // 6) 各 EXTI 割り込みハンドラを優先度順に設定
  int prio[6] = {EXTI0_PRIORITY, EXTI1_PRIORITY, EXTI2_PRIORITY,
                 EXTI3_PRIORITY, EXTI4_PRIORITY, EXTI9_5_PRIORITY};
  for(int i=0;i<6;i++){
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn + i;
    NVIC_InitStructure.NVIC_IRQChannelCmd           = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = prio[i];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority       = 0;
    NVIC_Init(&NVIC_InitStructure);
  }
}
```

* **EXTI (External Interrupt)**: GPIO ピンの状態変化を割り込みとして扱う。
* **NVIC (Nested Vectored Interrupt Controller)**: Cortex-M の割り込みコントローラ。優先度と有効化を管理。
* **GPIO\_Mode\_IPD**: プルダウン入力 (Input Pull-Down)。ピンがフロートしないよう，下方向にプル。

---

### 3.5 ADC\_DMA\_Configuration()

```c
void ADC_DMA_Configuration(void) {
  // 1) ADC1, GPIOC クロック
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);            // ADCCLK=12MHz
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 |
                         RCC_APB2Periph_GPIOC, ENABLE);

  // 2) PC0 をアナログ入力に設定
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // 3) DMA1_Channel1 を Circular モードで設定
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&ADCConvertedValue;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize         = 1;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);

  // 4) ADC1 を Scan + Continuous + DMA 有効化
  ADC_InitTypeDef ADC_InitStructure;
  ADC_DeInit(ADC1);
  ADC_InitStructure.ADC_Mode              = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode      = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode= ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv  = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign         = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel      = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_13Cycles5);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);

  // 5) キャリブレーション + 変換開始
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
```

* **ADC (Analog-to-Digital Converter)**: アナログ電圧をデジタル値に変換。
* **DMA (Direct Memory Access)**: CPU を介さずにペリフェラルとメモリ間でデータ転送。
* **Circular モード**: バッファ終端に達したら先頭に戻り継続転送。

---

以上で Chapter 3 初期化処理の詳細解説を終わります。各専門用語の役割と実装方法も含めましたので，次はこれらを動作させるコードの全体フローをイメージしてみてください。: 周辺機能設定

### 4.1 タイマー設定

* TIM1: サーボ PWM 出力 (15kHz, 72MHz→1MHz→15kHz)
* TIM2: 例外ハンドリング (0.1s 周期)
* TIM4: ドロップ制御用 PWM
* TIM5: 1kHz センサ取得タイマー (50Hz?)
* TIM6: 20Hz 制御ループ
* TIM7: 任意遅延関数 `_delay_us` 用
* TIM8: 受信パルス幅計測用

### 4.2 I2C/NMEA/USART/DMA

* I2C1: Arduino からパラメータ取得用
* I2C2: MPU6000/MAG3110/BMP280 用
* USART1 + DMA1\_Channel4: ログ送出，DMA1\_Channel5: 受信
* ADC\_DMA: ADC1 によるアナログ入力 (Battery 電圧等)

## 第5章: センサー読み取りと変換

### 5.1 MPU6000 (IMU)

* `MPU6000_Configuration()`: パワー管理，FSR 設定.
* `MPU6000_getParameter()`: I2C で 14 バイト連続読み出し→生データ→加速度 \[m/s^2]，温度 \[°C]，角速度 \[deg/s] に変換.

### 5.2 MAG3110 (磁力計)

* `MAG3110_Configuration()`: リセット→アクティブモード
* `MAG3110_getParameter()`: I2C で 6 バイト読み出し→\[milliGauss] 変換
* `MAG3110_Calibration()`: 500 サンプルでオフセット算出

### 5.3 BMP280 (気圧計)

* `BMP280_Configuration()`: リセット→補正係数取得→動作モード設定→初期標高計算
* `BMP280_getTrimParameter()`, `BMP280_temperature()`, `BMP280_pressure()`: データ取得→補正式→絶対圧力算出
* `BMP280_altitude()`: 気圧から高度計算 (国際標準大気モデル)

## 第6章: 姿勢推定 (Madgwick AHRS)

* `MadgwickAHRS()`, `MadgwickAHRS_IMU()`: 加速度，磁力，ジャイロを使ったクォータニオン更新
* `invSqrt`, `computeAngles`, `getPhi/Theta/Psi`: クォータニオン → オイラー角変換 (rad→deg)
* 比較的軽量なアルゴリズムでリアルタイム姿勢推定を実現

## 第7章: 制御ループ (TIM6\_IRQHandler)

1. 割り込み周期: 0.05秒 (20Hz)
2. モード選択 (`ControlMode`) に応じた目標姿勢設定 (phi\_t, theta\_t, psi\_t)
3. PID ゲインの更新 (Aileron/Elevator/Rudder)
4. `PID_Controller()` または `PID_Roll_Controller()` 実行
5. サーボデューティに変換 (`aileronDeg2Duty` 等)
6. ログ送出 (`sendDataPID` / `sendDataOption`)

各モード (水平飛行, 回頭, サークル, Figure Eight, 自動離着陸 など) を switch-case で切り替え。

## 第8章: 受信割り込みとパルス幅計測

* EXTI0〜EXTI4, EXTI9\_5: RC 送信機のチャネル信号を SysTick を使ってパルス幅測定→`Receiver_Data[]`
* `Convert_ReceiverData_to_PWM()`: 受信データ→デューティ (72で割る)
* TIM8\_UP\_IRQHandler: チャネル 6 のトグルで制御開始/モード変更

## 第9章: メインループ

```c
int main() {
  BoardInit(); GPIO_Configuration(); SysTick_Configuration();
  EXTI_Configuration(); NVIC_Configuration(); ADC_DMA_Configuration();
  I2C1_Configuration(); I2C2_Configuration(); USART1_Configuration();
  Sensor_ConectionCheck();
  MPU6000_Configuration(); MAG3110_Configuration(); BMP280_Configuration();
  TIM1_Configuration(); TIM2_Configuration(); TIM4_Configuration();
  TIM5_Configuration(); TIM6_Configuration(); TIM8_Configuration();
  while(1);
}
```

* 全割り込み + DMA で動作し，メインループでは何もしない。

## 第10章: ユーティリティ関数

* `_delay_us`, `_delay_ms`: TIM7 を使ったビジーウェイト遅延
* `sendData`, `sendDataOption/Offset/Calibration/PID`: tsprintf でフォーマットし DMA 送出
* `normalize`: \[入力範囲]→\[出力範囲] の線形変換

---

以上がコード全体の構成と，各機能の詳細な役割説明です。初心者の方は，まずは各章ごとにコードと当ドキュメントを突き合わせながら動作イメージを掴んでください。各周辺機能 (タイマー，I2C，DMA) が割り込み駆動で連携することで，リアルタイム制御を実現しています。
