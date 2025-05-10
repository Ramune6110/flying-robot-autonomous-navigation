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

コード冒頭で多くの `#define` マクロが並んでいますが、これらはハードウェア仕様や定数を中央集権的に管理し、以下のメリットを提供します：

* **可読性**: ソース中に "0x3B" と直接書くより `MPU6000_ACCEL_XOUT_H` と書いたほうが何を意味するか瞬時に理解できる
* **保守性**: センサーやピン配置を変更する際、コード全体を探す必要なく定義部だけ更新すればよい
* **ドキュメント**: 定義名がそのまま仕様書代わりになる
* **型安全**: マジックナンバーではなく意味のある名前で扱う

以下、主要な定義群と設計思想をコード例交えて解説します。

### 2.1 センサー分解能・スケール定数

```c
#define ACCEL_2G_LSB      16384.0   // 2g フルスケールで 1g=16384 LSB (MPU6000)
#define GYRO_500DEG_LSB    65.5      // 500°/s フルスケールで 1 LSB=65.5°/s
#define MAG_LSB           32768.0    // 磁力計 ±1000mGauss で 1 mGauss ≒ 32768 LSB
#define GRAVITY_ACCLE     9.80665    // 地球重力加速度 [m/s²]
```

* **設計思想**: センサー生データを物理単位に変換する際、これらの「分母」や換算係数が不可欠。マクロでまとめることで、変換ロジック（`raw/ACCEL_2G_LSB*GRAVITY_ACCLE`）がシンプルに。
* **ポイント**: テスタやデータシートに記載された分解能値を、そのままマクロに置く。

### 2.2 サーボ PWM デューティ定義

```c
#define AILERON_DUTY_VAL    1    // Dummy: 後で使うかも？
#define ELEVATOR_DUTY_VAL   1
#define RUDDER_DUTY_VAL     1

#define AILERON_MAX        40   // サーボ角度上限 [deg]
#define AILERON_MIN       -40   // 下限
#define ELEVATOR_MAX       40
#define ELEVATOR_MIN      -40
#define RUDDER_MAX         40
#define RUDDER_MIN        -40
```

* **設計思想**: サーボ角度レンジと制御限界を明示。PID 出力をこれら `*_MAX/MIN` でクリップし、サーボ破損を防止。
* `*_DUTY_VAL` は実装上不要ですが、後の拡張用プレースホルダとして残す場合あり。

### 2.3 I²C アドレス＆レジスタ定義

```c
// MPU6000 (0x68) 用 I2C アドレス + レジスタ
#define MPU6000_ADDRESS         0b11010010
#define MPU6000_ACCEL_XOUT_H    0x3B
#define MPU6000_PWR_MGMT_1      0x6B
#define MPU6000_WHO_AM_I        0x75
#define MPU6000_CHIPID          0x68

// MAG3110 用
#define MAG3110_ADDRESS         0b00011100
#define MAG3110_OUT_X_MSB       0x01
#define MAG3110_WHO_AM_I        0x07
#define MAG3110_CHIPID          0xC4

// BMP280 用
#define BMP280_ADDRESS          0b11101100
#define BMP280_DIG_T1           0x88
#define BMP280_CTRL_MEAS        0xF4
#define BMP280_WHO_AM_I         0xD0
#define BMP280_CHIPID           0x58
```

* **設計思想**: 各センサーの I2C アドレスは 7-bit + R/W ビット形式で定義。

  * 例: `0b1101001` (7bit) <<1 ＋ R/W ビット 0 → `0b11010010`
* レジスタオフセットも同様にマクロ化し、I²C コマンド部分 (`i2c2Read(..., MPU6000_WHO_AM_I)`) が何を意味するか一目瞭然。

### 2.4 Arduino I2C アドレス

```c
#define ARDUINO_ADDRESS         1
```

* **設計思想**: センサーとは別系統の I2C バス（I2C1）で，Arduino から電圧値取得。マイコン間通信用途の識別子。

### 2.5 ピンマッピング（タイマー・EXTI）

```c
// E タイマー入力ピンマスク
#define PE0_5           0x003f   // PE0~PE5 ビットマスク

// TIM1 PWM 出力チャネル
#define TIM1_CH1        GPIO_Pin_9
#define TIM1_CH2        GPIO_Pin_11
#define TIM1_CH3        GPIO_Pin_13
#define TIM1_CH4        GPIO_Pin_14

// TIM4 PWM 出力チャネル
#define TIM4_CH1        GPIO_Pin_12
// ... TIM4_CH4
```

* **設計思想**: 物理ピンと機能（PWM, EXTI）の紐付けを一元管理。
* `GPIO_Pin_X` は STM32 標準ヘッダでビットマスク定義済。

### 2.6 割り込み優先度定義

```c
#define EXTI0_PRIORITY      1
#define EXTI1_PRIORITY      2
// ...
#define TIM2_PRIORITY       20
```

* **設計思想**: 割り込み優先度はプロジェクト／システム要件に応じて調整が必要。

  * 例: RC 信号キャプチャ(EXTI) は最優先、制御ループ(TIM6) は中程度、ログ出力は低優先。
* 優先度だけマクロ化し、後からチューニングしやすく。

---

これら定数定義は、後続の設定関数（GPIO\_Init, I2C\_Init, TIM\_TimeBaseInit など）で読み込まれ、1行1行がハードウェア仕様書（データシート）の写像になっています。組み込み開発ではマジックナンバーを避け、仕様書通りの定義を見出しつきで管理することが設計の基本です。

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

## 第4章周辺機能設定

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

以下、「第4章：割り込み設計とペリフェラル設定」という構成で解説します。中盤以降にコード抜粋を交えながら、

* 割り込み処理の設計思想
* I2C1／I2C2／USART1 と DMA 設定の具体的中身
* MCU の「ペリフェラル」とは何か、このプロジェクトで何を使っているか

を順にカバーします。組み込み初心者の方にもわかるよう、用語ごとに役割と実装方法も補足しています。

---

## 4-1. MCU ペリフェラル（周辺機能）とは？

マイコン（MCU）の「ペリフェラル」とは、CPU コアの外側にある入出力や制御を司るハードウェア機能全般を指します。代表的なものは：

* **GPIO**（General-Purpose I/O）… 汎用的なデジタル入出力ピン。LED やスイッチとの接続に使う。
* **タイマ／カウンタ**… 時間計測や PWM 信号生成、割り込み周期のトリガに利用。
* **UART／USART**… シリアル通信（PC や XBee など）に使う。
* **I²C, SPI**… センサーや外部 EEPROM などとのバス通信に使う。
* **ADC／DAC**… アナログ信号の A/D 変換や D/A 変換に使う。
* **DMA**… メモリ⇔ペリフェラル間のデータ転送を CPU 不使用で自動化。
* **外部割り込み（EXTI）**… ピン変化をトリガに高速割り込みを発生。
* **NVIC**… Cortex-M 系の割り込み制御ブロック（割り込み優先度・マスクを管理）。

このプロジェクトでは、上記ほぼすべてを活用し、センサー読み出し→姿勢推定→PID→サーボ駆動の一連を割り込みで高速かつ効率的に実現しています。

---

## 4-2. 割り込み処理の設計思想

1. **周期割り込み（タイマ）による処理分担**

   * TIM5（50 Hz）：センサー取得＋Madgwick姿勢推定
   * TIM6（20 Hz）：PID 制御ループ
   * TIM2（10 Hz）：I²C エラー監視・例外ログ
   * TIM8：RC 信号取得→PWM出力更新
   * その他、TIM1/TIM4 で PWM → サーボ直接制御

2. **ピン変化割り込み（EXTI）による RC 受信**

   * RC 受信機のパルス幅計測は PE0–PE5 を EXTI0–EXTI5 でキャプチャし、SysTick カウンタと組み合わせて高分解能計測。

3. **NVIC による優先度制御**

   * 各割り込みにプリエンプション優先度を設定し、センサ取得や PID よりも RC パルス計測（EXTI）を高優先度に。遅くてもいいログ出力は低優先度に。

> **設計ポイント**：
>
> * 「時間決め打ちの制御」はタイマ割り込みで確実に周期実行
> * 「外部イベント」は EXTI 割り込みでキャプチャ
> * 「複数割り込みの共存」は NVIC 優先度で調整

---

## 4-3. EXTI／NVIC 設定（RC 信号計測）

```c
void EXTI_Configuration(void) {
  // (1) GPIOE ピン 0–5 をプルダウン入力に
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOE, ENABLE);
  GPIO_InitTypeDef gpio = {
    .GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1|...|GPIO_Pin_5,
    .GPIO_Mode  = GPIO_Mode_IPD,   // プルダウン
    .GPIO_Speed = GPIO_Speed_50MHz
  };
  GPIO_Init(GPIOE, &gpio);

  // (2) PE0–PE5 を EXTI ライン 0–5 にマッピング
  for(int pin=0; pin<=5; pin++){
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, pin);
  }

  // (3) EXTI0 (PE0) は立ち上がり/立ち下がり両トリガ
  EXTI_InitTypeDef exti = {
    .EXTI_Line    = EXTI_Line0,
    .EXTI_Mode    = EXTI_Mode_Interrupt,
    .EXTI_Trigger = EXTI_Trigger_Rising_Falling,
    .EXTI_LineCmd = ENABLE
  };
  EXTI_Init(&exti);

  // (4) PE1–PE5 は立ち下がりのみキャプチャ
  exti.EXTI_Line    = EXTI_Line1 | ... | EXTI_Line5;
  exti.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_Init(&exti);

  // (5) NVIC に割り込み優先度を登録
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  NVIC_InitTypeDef nvic = {
    .NVIC_IRQChannelPreemptionPriority = EXTI0_PRIORITY,
    .NVIC_IRQChannelSubPriority        = 0,
    .NVIC_IRQChannelCmd                = ENABLE
  };
  for(int irq=EXTI0_IRQn; irq<=EXTI9_5_IRQn; irq++){
    nvic.NVIC_IRQChannel = irq;
    NVIC_Init(&nvic);
  }

  EXTI_ClearITPendingBit(0x003F);
}
```

* **GPIO\_Mode\_IPD**：引き脚（プルダウン）付き入力。アイドル時に 0 V に保つ。
* **EXTI\_Trigger\_Rising\_Falling**：立ち上がり・立ち下がり両方検知。

PE0→SysTick→PE0→SysTick 差分からパルス幅計測し、RCサーボ信号を得ています。

---

## 4-4. DMA 設定（USART1 TX/RX）

```c
void DMA1_Configuration(uint32_t memAddr, uint16_t bufSize) {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  DMA_DeInit(DMA1_Channel4);

  DMA_InitTypeDef dma = {
    .DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR,
    .DMA_MemoryBaseAddr     = memAddr,
    .DMA_DIR                = DMA_DIR_PeripheralDST,  // メモリ→USART1
    .DMA_BufferSize         = bufSize,
    .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc          = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte,
    .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
    .DMA_Mode               = DMA_Mode_Normal,
    .DMA_Priority           = DMA_Priority_VeryHigh
  };
  DMA_Init(DMA1_Channel4, &dma);
}
```

* **DMA（Direct Memory Access）**：CPU を介さず、ペリフェラルとメモリ間で自動転送。
* チャネル4 を USART1\_TX に紐づけ、`sendData()` 呼び出しで DMA 転送→TX 送信完了待ち。

---

## 4-5. I²C1／I²C2 設定（センサー・Arduino 通信）

```c
void I2C1_Configuration(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

  // PB6=SCL, PB7=SDA をオープンドレイン出力に
  GPIO_InitTypeDef gpio = {
    .GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7,
    .GPIO_Mode  = GPIO_Mode_AF_OD,
    .GPIO_Speed = GPIO_Speed_50MHz
  };
  GPIO_Init(GPIOB, &gpio);

  // I2C1 本体の設定
  I2C_InitTypeDef i2c = {
    .I2C_ClockSpeed      = 400000,                  // 400 kHz
    .I2C_Mode            = I2C_Mode_I2C,
    .I2C_DutyCycle       = I2C_DutyCycle_2,
    .I2C_Ack             = I2C_Ack_Enable,
    .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit
  };
  I2C_Init(I2C1, &i2c);
  I2C_Cmd(I2C1, ENABLE);
}

// I2C2 は PB10/SCL, PB11/SDA、センサー（MPU6000, MAG3110, BMP280）用
void I2C2_Configuration(void) {
  // （略：I2C1 とほぼ同じで、ペリフェラルが I2C2）
}
```

* **GPIO\_Mode\_AF\_OD**：I²C バス用の開放型ドレイン。
* 400 kHz の高速モードでセンサーをポーリング読み出し。

---

## 4-6. USART1 設定（XBee／シリアルコンソール）

```c
void USART1_Configuration(void) {
  // クロック供給
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // PA9=TX: AF Push-Pull, PA10=RX: Floating Input
  GPIO_InitTypeDef gpio = {
    .GPIO_Pin   = GPIO_Pin_9,
    .GPIO_Mode  = GPIO_Mode_AF_PP,
    .GPIO_Speed = GPIO_Speed_50MHz
  };
  GPIO_Init(GPIOA, &gpio);
  gpio.GPIO_Pin = GPIO_Pin_10;
  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio);

  // DMA Channel5 を USART1_RX に設定
  DMA_InitTypeDef dma = { /* RX 用設定 */ };
  DMA_Init(DMA1_Channel5, &dma);
  DMA_Cmd(DMA1_Channel5, ENABLE);
  DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

  // USART1 本体設定
  USART_InitTypeDef usart = {
    .USART_BaudRate            = 115200,
    .USART_WordLength          = USART_WordLength_8b,
    .USART_StopBits            = USART_StopBits_1,
    .USART_Parity              = USART_Parity_No,
    .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
    .USART_Mode                = USART_Mode_Rx | USART_Mode_Tx
  };
  USART_Init(USART1, &usart);
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  USART_Cmd(USART1, ENABLE);
}
```

* **USART\_Mode\_RX|TX**：双方向シリアル通信を有効化。
* **DMAReq\_Tx**：送信も DMA 連携で効率化。

---

## 4-7. 割り込み設計のまとめとポイント

1. **周期制御**…タイマ割り込みで必ず一定周期に処理を呼び出し
2. **イベント制御**…EXTI＋SysTick で外部パルス幅を高分解能でキャプチャ
3. **DMA 連携**…大量データ転送（シリアルや A/D）を CPU 負荷ゼロで実行
4. **NVIC 優先度**…安全に複数割り込みを共存させ、最も重要な信号取得を最優先

各ペリフェラルの役割を理解し、用途に応じて「割り込み or ポーリング」「DMA or CPU転送」を選ぶのが組み込み設計のキモです。今回のコードでは、

* **I²C**：センサー制御（ポーリング＋エラー監視）
* **USART + DMA**：XBee／デバッグ出力（大容量文字列）
* **EXTI + SysTick**：RC 信号計測
* **TIMx**：周期処理／PWM

をうまく使い分けています。

---

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
