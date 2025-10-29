# STM32F103C8T6 (Blue Pill) - Tabella GPIO CNC 3 Assi
## Pinout SENZA CONFLITTI - PC13 Libero per LED

## 📋 TABELLA CONNESSIONI COMPLETE

| # | Lato | Pos | Pin STM32 | Funzione CNC | Periferica | Dir | Tipo | Note |
|---|------|-----|-----------|--------------|------------|-----|------|------|
| **MOTORI PASSO-PASSO** |
| 1 | **SX** | **5** | **PA0** | **STEP_X** | TIM2_CH1 | OUT | PWM | Timer HW 20kHz - A0/Analog |
| 2 | **SX** | **6** | **PA1** | **STEP_Y** | TIM2_CH2 | OUT | PWM | Timer HW 20kHz - A1/PWM2/2 |
| 3 | **SX** | **13** | **PB0** | **STEP_Z** | TIM3_CH3 | OUT | PWM | Timer HW 20kHz - A3/PWM3/3 |
| 4 | **DX** | **20** | **PB12** | **DIR_X** | GPIO | OUT | GPIO | Direzione X - SPI2 NSS, 5V tolerant |
| 5 | **DX** | **19** | **PB13** | **DIR_Y** | GPIO | OUT | GPIO | Direzione Y - SPI2 SCK, 5V tolerant |
| 6 | **DX** | **18** | **PB14** | **DIR_Z** | GPIO | OUT | GPIO | Direzione Z - SPI2 MISO, 5V tolerant |
| 7 | **DX** | **10** | **PB3** | **ENABLE_X** | GPIO | OUT | GPIO | Enable X - D3/SPI1 NSS |
| 8 | **DX** | **9** | **PB4** | **ENABLE_Y** | GPIO | OUT | GPIO | Enable Y - D10/SPI1 MISO |
| 9 | **DX** | **8** | **PB5** | **ENABLE_Z** | GPIO | OUT | GPIO | Enable Z - D15 |
| **ENCODER (Lettura Posizione)** |
| 10 | **SX** | **11** | **PA6** | **ENC_X_A** | TIM3_CH1 | IN | Encoder | Encoder X canale A - D12/PWM3/1 |
| 11 | **SX** | **12** | **PA7** | **ENC_X_B** | TIM3_CH2 | IN | Encoder | Encoder X canale B - D11/PWM3/2 |
| 12 | **DX** | **7** | **PB6** | **ENC_Y_A** | TIM4_CH1 | IN | Encoder | Encoder Y canale A - D14, 5V tolerant |
| 13 | **DX** | **6** | **PB7** | **ENC_Y_B** | TIM4_CH2 | IN | Encoder | Encoder Y canale B - I2C1 SCL, 5V tolerant |
| 14 | **DX** | **16** | **PA8** | **ENC_Z_A** | TIM1_CH1 | IN | Encoder | Encoder Z canale A - PWM1/1, 5V tolerant |
| 15 | **DX** | **15** | **PA9** | **ENC_Z_B** | TIM1_CH2 | IN | Encoder | Encoder Z canale B - TX1, 5V tolerant |
| **FINECORSA (Limit Switches)** |
| 16 | **DX** | **5** | **PB8** | **X_MIN** | EXTI8 | IN | INT | Finecorsa X min - 5V tolerant |
| 17 | **DX** | **4** | **PB9** | **X_MAX** | EXTI9 | IN | INT | Finecorsa X max - 5V tolerant |
| 18 | **SX** | **7** | **PA2** | **Y_MIN** | EXTI2 | IN | INT | Finecorsa Y min - D1/Serial2 TX |
| 19 | **SX** | **8** | **PA3** | **Y_MAX** | EXTI3 | IN | INT | Finecorsa Y max - D0/PWM2/4 |
| 20 | **SX** | **9** | **PA4** | **Z_MIN** | EXTI4 | IN | INT | Finecorsa Z min - A2/SPI1 NSS |
| 21 | **SX** | **10** | **PA5** | **Z_MAX** | EXTI5 | IN | INT | Finecorsa Z max - A13/SPI1 SCK |
| **COMUNICAZIONE USB** |
| 22 | **DX** | **13** | **PA11** | **USB_DM** | USB | BIDIR | USB | USB D- - 5V tolerant |
| 23 | **DX** | **12** | **PA12** | **USB_DP** | USB | BIDIR | USB | USB D+ - 5V tolerant |
| **COMUNICAZIONE UART3** |
| 24 | **SX** | **15** | **PB10** | **UART3_TX** | USART3 | OUT | UART | TX ESP32 - D6/Serial3 TX, 5V tolerant |
| 25 | **SX** | **16** | **PB11** | **UART3_RX** | USART3 | IN | UART | RX ESP32 - Serial3 RX, 5V tolerant |
| **DISPLAY LCD 4x20 (4-bit) - CONFIGURAZIONE CORRETTA** |
| 26 | **DX** | **14** | **PA10** | **LCD_RS** | GPIO | OUT | GPIO | Register Select - 5V tolerant |
| 27 | **SX** | **14** | **PB1** | **LCD_E** | GPIO | OUT | GPIO | Enable/Clock - PWM3/4 |
| 28 | **SX** | **3** | **PC14** | **LCD_D4** | GPIO | OUT | GPIO | Data bit 4 |
| 29 | **SX** | **4** | **PC15** | **LCD_D5** | GPIO | OUT | GPIO | Data bit 5 |
| 30 | **DX** | **17** | **PB15** | **LCD_D6** | GPIO | OUT | GPIO | ✅ Data bit 6 - SPI2 MOSI, 5V tolerant |
| 31 | **DX** | **11** | **PA15** | **LCD_D7** | GPIO | OUT | GPIO | Data bit 7 - 5V tolerant |
| **PROGRAMMAZIONE SWD** |
| 32 | **PAD** | **-** | **PA13** | **SWDIO** | SWD | BIDIR | DEBUG | ✅ Pad sulla board - sempre disponibile! |
| 33 | **PAD** | **-** | **PA14** | **SWCLK** | SWD | IN | DEBUG | ✅ Pad sulla board - sempre disponibile! |
| **SICUREZZA & DIAGNOSTICA** |
| 34 | **SX** | **2** | **PC13** | **LED_STATUS** | GPIO | OUT | GPIO | ✅ LED integrato - LIBERO PER USO! |
| **ALIMENTAZIONE** |
| 35 | **SX** | **1** | **VBAT** | VBAT | Power | - | PWR | Backup RTC (opzionale) |
| 36 | **DX** | **1** | **3V3** | VCC_3V3 | Power | - | PWR | Alimentazione logica 3.3V |
| 37 | **DX** | **2** | **GND** | GND | Power | - | PWR | Massa comune |
| 38 | **DX** | **3** | **5V** | VCC_5V | Power | - | PWR | 5V da USB/regolatore |
| 39 | **SX** | **18** | **+3.3V** | VCC_3V3 | Power | - | PWR | Altro pin 3.3V |
| 40 | **SX** | **19,20** | **GND** | GND | Power | - | PWR | Altri pin GND |
| 41 | **SX** | **17** | **RESET** | NRST | Reset | IN | RST | Reset button |

---


## 🗺️ MAPPA VISUALE AGGIORNATA

### **LATO SINISTRO:**
```
Pos  GPIO     Funzione CNC              Note
─────────────────────────────────────────────────
 1   VBAT     -                         RTC backup
 2   PC13  →  LED_STATUS ✅            Solo LED onboard
 3   PC14  →  LCD_D4
 4   PC15  →  LCD_D5
 5   PA0   →  STEP_X ⚙️
 6   PA1   →  STEP_Y ⚙️
 7   PA2   →  Y_MIN 🛑
 8   PA3   →  Y_MAX 🛑
 9   PA4   →  Z_MIN 🛑
10   PA5   →  Z_MAX 🛑
11   PA6   →  ENC_X_A 🔄
12   PA7   →  ENC_X_B 🔄
13   PB0   →  STEP_Z ⚙️
14   PB1   →  LCD_E
15   PB10  →  UART3_TX 📡 (5V✓)
16   PB11  →  UART3_RX 📡 (5V✓)
17   RESET
18   +3.3V
19   GND
20   GND
```

### **LATO DESTRO:**
```
Pos  GPIO     Funzione CNC              Note
─────────────────────────────────────────────────
 1   3V3      Alimentazione
 2   GND      Massa
 3   5V       Alimentazione 5V
 4   PB9   →  X_MAX 🛑              (5V✓)
 5   PB8   →  X_MIN 🛑              (5V✓)
 6   PB7   →  ENC_Y_B 🔄            (5V✓)
 7   PB6   →  ENC_Y_A 🔄            (5V✓)
 8   PB5   →  ENABLE_Z ⚙️
 9   PB4   →  ENABLE_Y ⚙️
10   PB3   →  ENABLE_X ⚙️
11   PA15  →  LCD_D7                (5V✓)
12   PA12  →  USB_DP 🔌            (5V✓)
13   PA11  →  USB_DM 🔌            (5V✓)
14   PA10  →  LCD_RS                (5V✓)
15   PA9   →  ENC_Z_B 🔄            (5V✓)
16   PA8   →  ENC_Z_A 🔄            (5V✓)
17   PB15  →  LCD_D6 ✅            (5V✓) NUOVO!
18   PB14  →  DIR_Z ⚙️              (5V✓)
19   PB13  →  DIR_Y ⚙️              (5V✓)
20   PB12  →  DIR_X ⚙️              (5V✓)
```

---


## 📊 RIEPILOGO FINALE

```
GPIO Totali:           37
GPIO Usati:            31
GPIO Liberi:           6

Pin utilizzati:
- Motori:              9
- Encoder:             6
- Finecorsa:           6
- USB:                 2 (hardware)
- UART:                2
- Display:             6 (SENZA conflitti!)
- LED Status:          1 (dedicato!)
- SWD:                 2 (su pad)
```


