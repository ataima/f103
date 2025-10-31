# Analisi Overhead CPU - Lettura Encoder in SysTick ISR

## Configurazione Sistema

- **MCU**: STM32F103C8T6 @ 72 MHz
- **SysTick**: 1 kHz (interrupt ogni 1ms)
- **Lettura Encoder**: 50 Hz (ogni 20ms tramite divisore)

## Operazioni nel Callback Encoder (50Hz)

### Funzione: `cnc_encoder_isr_callback()`

```c
static void cnc_encoder_isr_callback(void)
{
    /* 1. Lettura TIM3_CNT (asse X) */
    i16 delta_x = encoder_read_x();  // Legge registro + azzera

    /* 2. Lettura TIM4_CNT (asse Y) */
    i16 delta_y = encoder_read_y();

    /* 3. Lettura TIM1_CNT (asse Z) */
    i16 delta_z = encoder_read_z();

    /* 4. Aggiornamento posizioni assolute (3 add 32-bit) */
    encoder_positions.x += (i32)delta_x;
    encoder_positions.y += (i32)delta_y;
    encoder_positions.z += (i32)delta_z;
}
```

## Stima Cicli CPU per Operazione

### Lettura encoder (encoder_read_x/y/z):
```c
i16 encoder_read_x(void)
{
    i16 delta = (i16)(TIM3_CNT & 0xFFFF);  // Lettura registro: ~3 cicli
    TIM3_CNT = 0;                          // Scrittura registro: ~2 cicli
    return delta;                           // Return: ~1 ciclo
}
```

**Totale per un asse**: ~6 cicli CPU
**Totale 3 assi**: ~18 cicli CPU

### Aggiornamento posizioni (3 add 32-bit):
```c
encoder_positions.x += (i32)delta_x;  // Sign extend + add: ~4 cicli
encoder_positions.y += (i32)delta_y;  // ~4 cicli
encoder_positions.z += (i32)delta_z;  // ~4 cicli
```

**Totale**: ~12 cicli CPU

### Overhead callback stesso:
- Jump to callback: ~3 cicli
- Return from callback: ~3 cicli

**Totale overhead**: ~6 cicli

## Calcolo Overhead Totale

### Per singola chiamata callback (50Hz):
```
Letture encoder:        18 cicli
Aggiornamenti posizioni: 12 cicli
Overhead chiamata:        6 cicli
----------------------------------
TOTALE:                 ~36 cicli CPU
```

**Tempo esecuzione @ 72 MHz**: 36 / 72000000 = **0.5 µs**

### Overhead SysTick ISR (1kHz):

**Senza callback encoder (ogni 1ms)**:
- Incremento counter: ~2 cicli
- Incremento stats: ~2 cicli
- Check callback: ~3 cicli (if + compare)
- Incremento divider: ~2 cicli
- Totale: **~9 cicli** = **0.125 µs**

**Con callback encoder (ogni 20ms, 1 volta su 20)**:
- Base ISR: ~9 cicli
- Callback completo: ~36 cicli
- Totale: **~45 cicli** = **0.625 µs**

## Percentuale CPU Utilizzata

### Per SysTick senza encoder:
- Frequenza interrupt: 1000 Hz
- Cicli per interrupt: 9
- Cicli/sec: 9 × 1000 = 9000
- **Overhead**: 9000 / 72000000 = **0.0125%**

### Per callback encoder (50Hz):
- Frequenza callback: 50 Hz
- Cicli per callback: 36
- Cicli/sec: 36 × 50 = 1800
- **Overhead encoder**: 1800 / 72000000 = **0.0025%**

### Overhead SysTick con divisore (check ogni ms):
- Cicli extra per divisore: 5 cicli × 1000 Hz = 5000 cicli/sec
- **Overhead divisore**: 5000 / 72000000 = **0.007%**

### **TOTALE OVERHEAD**:
```
SysTick base:      0.0125%
Encoder callback:  0.0025%
Divisore:          0.007%
------------------------
TOTALE:           ~0.022% CPU @ 72 MHz
```

## Confronto con Polling (alternativa)

Se leggessimo encoder nel main loop a 50Hz con polling attivo:

```c
while(1) {
    if (systick_timeout(last_read, 20)) {  // 4 cicli
        cnc_update_encoder_positions();     // 36 cicli
        last_read = systick_get_tick();     // 3 cicli
    }
    // ... altro codice main loop
}
```

**Overhead polling**: 4 cicli × frequenza loop principale

Se il main loop gira a 10 kHz (ogni 100µs):
- Check timeout: 4 × 10000 = 40000 cicli/sec
- Callback: 36 × 50 = 1800 cicli/sec
- **Totale polling**: 41800 / 72000000 = **0.058%**

### **Vantaggio interrupt vs polling**: ~2.6x meno overhead!

## Impatto su Altri Interrupt

### Latenza interrupt:
- SysTick ISR durata tipica: **0.125 µs**
- SysTick ISR con encoder (1/20 volte): **0.625 µs**

**Impatto su altri interrupt**:
- EXTI (finecorsa): può essere ritardato max 0.625 µs ogni 20ms
- TIM2 (step emergenza): può essere ritardato max 0.625 µs ogni 20ms

**Conclusione**: Impatto TRASCURABILE (< 1µs delay)

## Raccomandazioni

✅ **Implementazione OTTIMA per questo sistema**:
- Overhead CPU: **0.022%** (trascurabile)
- Latenza max ISR: **0.625 µs** (eccellente)
- Risoluzione temporale: **20ms** (50Hz sufficiente per encoder meccanici)
- Precisione: **Deterministica** (no jitter da polling)

### Alternativa se serve maggiore frequenza:
Se servisse lettura a 100Hz invece che 50Hz:
- Cambiare divisore da 20 a 10
- Overhead: 0.022% → 0.025% (ancora trascurabile)

### Nota sulla priorità interrupt:
Con l'architettura attuale:
- **EXTI (finecorsa)**: priorità 0 (massima)
- **SysTick**: priorità default (normale)
- **TIM2**: priorità 2

EXTI può interrompere SysTick → sicurezza garantita!

## Conclusione

**OVERHEAD COMPLESSIVO: 0.022% CPU @ 72 MHz**

Questo è un valore **ECCELLENTE** e lascia il **99.978% della CPU libera** per:
- Elaborazione G-code
- Generazione traiettorie
- Controllo movimento
- Comunicazione seriale
- Altre funzionalità

**Raccomandazione finale**: ✅ Implementazione APPROVATA per produzione
