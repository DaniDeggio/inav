# Piano di Integrazione ADRC in INAV

## 1. Analisi Comparativa delle Architetture

### 1.1 Differenze Fondamentali tra Betaflight e INAV

| Aspetto | ADRC-Betaflight | INAV |
|---------|----------------|------|
| **Struttura PID** | `pidProfile->pid[axis]` con P/I/D/F (uint16_t) | `pidBank_t` con `pid8_t` (P/I/D/FF uint16_t) |
| **Controller Types** | PID semplice | PID, PIFF, con Level/Heading/Nav |
| **Multirotor/Fixed-Wing** | Unico profilo | Due bank separati (`bank_mc`, `bank_fw`) |
| **State Management** | `pidRuntime` con ADRC state | `pidState_t` con TPA, filters, Smith predictor |
| **Parameter Group** | `PG_REGISTER_PROFILE` | `PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE` |
| **Feature Avanzate** | Nessuna | Anti-gravity, D-Boost, ITerm Relax, Smith Predictor, Adaptive Filter, RPM Filter |
| **TPA** | Semplice | Complesso (MC, FW, airspeed-based) |
| **Control Derivative** | No | Sì (`kCD` term) |
| **Tracking Anti-windup** | No | Sì (`kT` term) |

### 1.2 Architettura INAV Attuale

INAV ha un'architettura PID più sofisticata di Betaflight:

```
pidState_t (per ogni asse):
├── kP, kI, kD, kFF, kCD, kT    (guadini calcolati)
├── errorGyroIf                   (integral term)
├── angleFilterState              (per ANGLE mode)
├── axisAccelFilter               (setpoint limiting)
├── ptermLpfState                 (low-pass P-term)
├── dtermLpfState                 (low-pass D-term)
├── rateTargetFilter              (control derivative)
├── previousRateTarget/gyro       (per D-term)
├── smithPredictor_t              (per fixed-wing)
├── fwPidAttenuation_t            (attenuazione FW)
├── dBoostLpf/dBoostGyroLpf       (D-Boost, se enabled)
└── stickPosition                 (stick input)
```

**Flusso di Controllo Attuale:**
1. `pidController(dT)` → seleziona controller (MC/FW)
2. `pidApplySetpointRateLimiting()` → limita accelerazione setpoint
3. `pidLevel()` → ANGLE/HORIZON mode (calcola rateTarget)
4. Controller rate:
   - **MC**: `pidApplyMulticopterRateController()` → P + I + D + CD + anti-windup
   - **FW**: `pidApplyFixedWingRateController()` → P + I + D + FF + iTermLock
5. `updatePIDCoefficients()` → aggiorna kP/kI/kD/FF/CD/T con TPA

---

## 2. Design dell'Integrazione ADRC

### 2.1 Strategia: Modalità Ibrida (ADRC Mode Flag)

**Approccio consigliato:** ADRC come alternativa al controller PID, non come sostituzione parziale.

```c
typedef enum {
    ADRC_DISABLED = 0,
    ADRC_ENABLED_MC,    // ADRC per Multirotor
    ADRC_ENABLED_FW,    // ADRC per Fixed-Wing
} adrcMode_e;
```

L'utente seleziona la modalità tramite `pidProfile->pidControllerType` esistente o un nuovo campo:
- `PID_TYPE_PID` → controller PID tradizionale (comportamento attuale)
- `PID_TYPE_ADRC` → controller ADRC (nuovo)

### 2.2 Mappatura dei Parametri

**Opzione A (consigliata): Nuovi campi in pidProfile_t**

```c
// In pidProfile_t, aggiungere:
typedef struct {
    float adrcWc;         // Control bandwidth [rad/s] - analogo a P
    float adrcWo;         // Observer bandwidth [rad/s] - analogo a I
    float adrcB0;         // System gain - analogo a D
    uint8_t adrcMode;     // 0=disabled, 1=enabled
} adrcProfile_t;
```

**Opzione B: Riutilizzare P/I/D (come Betaflight)**

```c
// P field → Control Bandwidth (wc)
// I field → Observer Bandwidth (wo)
// D field → System Gain (b0)
// FF field → Non usato in ADRC
```

**Raccomandazione:** Opzione A è migliore perché:
1. Non rompe la compatibilità con i profili esistenti
2. Permette ADRC e PID tradizionali coesistenti
3. I nomi sono più descrittivi (`adrcWc` vs "P field")

### 2.3 Aggiunta dello Stato ADRC

Aggiungere a `pidState_t`:

```c
typedef struct pidState_s {
    // ...existing fields...

#ifdef USE_ADRC
    // ADRC Extended State Observer (ESO) states
    float adrc_z1;              // ESO state 1: estimated system state
    float adrc_z2;              // ESO state 2: estimated disturbance derivative
    float adrc_z3;              // ESO state 3: total disturbance estimate
    float adrc_lastOutput;      // Last control output for observer feedback
    bool adrcActive;            // Whether ADRC is enabled for this axis
#endif
} pidState_t;
```

---

## 3. Modifiche File per File

### 3.1 `src/main/flight/pid.h`

#### Aggiungere:

```c
// === ADRC Configuration ===
#ifdef USE_ADRC
#define ADRC_WC_SCALE       1.0f
#define ADRC_WO_SCALE       1.0f
#define ADRC_B0_SCALE       10.0f
#define ADRC_B0_FALLBACK    50.0f

// ADRC controller type
#define PID_TYPE_ADRC       4      // After PID_TYPE_AUTO

// ADRC mode enum
typedef enum {
    ADRC_DISABLED = 0,
    ADRC_ENABLED_MC,
    ADRC_ENABLED_FW
} adrcMode_e;
#endif

// In pidProfile_t, aggiungere:
#ifdef USE_ADRC
    uint8_t         adrcMode;             // ADRC mode: 0=disabled, 1=MC, 2=FW
    float           adrcWc;               // Control bandwidth [rad/s]
    float           adrcWo;               // Observer bandwidth [rad/s]
    float           adrcB0;               // System gain
#endif

// In pidState_t (già modificato sopra)
#ifdef USE_ADRC
    float adrc_z1;
    float adrc_z2;
    float adrc_z3;
    float adrc_lastOutput;
#endif

// Nuove funzioni API
#ifdef USE_ADRC
void adrcResetState(void);
void adrcUpdateState(float *z1, float *z2, float *z3, float lastOutput,
                     float gyroRate, float wc, float wo, float b0, float dt);
float adrcComputeControl(float setpoint, float z1, float z2, float z3, float b0,
                        float wc, float wo);
#endif
```

### 3.2 `src/main/flight/pid.c`

#### Aggiungere Costanti e Stato Globale

```c
#ifdef USE_ADRC
static EXTENDED_FASTRAM bool adrcGainsUpdateRequired = true;
#endif
```

#### Aggiungere alla Reset Template

```c
#ifdef USE_ADRC
    .adrcMode = ADRC_DISABLED,
    .adrcWc = 30.0f,        // Default control bandwidth
    .adrcWo = 90.0f,        // Default observer bandwidth (3*wc)
    .adrcB0 = 500.0f,       // Default system gain (50 * 10)
#endif
```

#### Funzione Core ADRC (da inserire in pid.c)

```c
#ifdef USE_ADRC
/**
 * ADRC Extended State Observer (ESO) Update + Control Law
 *
 * This implements a second-order linear ADRC:
 * 1. ESO estimates system state and total disturbance
 * 2. Control law computes virtual PD with bandwidth-based gains
 * 3. Output is scaled by system gain b0
 *
 * Args:
 *   pidState: Pointer to PID state for this axis
 *   setpoint: Desired rate setpoint (deg/s)
 *   gyroRate: Current gyro rate measurement (deg/s)
 *   wc: Control bandwidth (rad/s)
 *   wo: Observer bandwidth (rad/s)
 *   b0: System gain
 *   dt: Control loop timestep (s)
 *
 * Returns:
 *   ADRC control output (to be added to axisPID)
 */
static float FAST_CODE adrcController(pidState_t *pidState, float setpoint,
                                       float gyroRate, float wc, float wo,
                                       float b0, float dt)
{
    // --- ESO Update ---
    // Standard linear ADRC observer gains
    const float beta1 = 3.0f * wo;
    const float beta2 = 3.0f * (wo * wo);
    const float beta3 = wo * wo * wo;

    // ESO state error
    float error_eso = pidState->adrc_z1 - gyroRate;

    // Euler forward integration of ESO states
    // z1: estimated system state (tracks gyroRate)
    // z2: disturbance derivative
    // z3: total disturbance estimate
    pidState->adrc_z1 += dt * (pidState->adrc_z2 - beta1 * error_eso);
    pidState->adrc_z2 += dt * (pidState->adrc_z3 + (b0 * pidState->adrc_lastOutput) - beta2 * error_eso);
    pidState->adrc_z3 += dt * (-beta3 * error_eso);

    // --- Control Law (Virtual PD) ---
    // kp = wc^2  (proportional to bandwidth squared)
    // kd = 2*wc  (derivative on estimated disturbance)
    const float kp = wc * wc;
    const float kd = 2.0f * wc;

    // P term: error in estimated state
    const float pTerm = kp * (setpoint - pidState->adrc_z1) / b0;

    // D term: estimated disturbance derivative
    const float dTerm = -kd * pidState->adrc_z2 / b0;

    // I term: total disturbance estimate
    const float iTerm = -pidState->adrc_z3 / b0;

    // Save output for next ESO iteration
    pidState->adrc_lastOutput = pTerm + dTerm + iTerm;

    return pTerm + dTerm + iTerm;
}

/**
 * Reset ADRC observer states to current gyro readings
 * Called on arm, mode change, or stabilization disabled
 */
void adrcResetState(void)
{
    for (int axis = 0; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++) {
        pidState[axis].adrc_z1 = pidState[axis].gyroRate;
        pidState[axis].adrc_z2 = 0.0f;
        pidState[axis].adrc_z3 = 0.0f;
        pidState[axis].adrc_lastOutput = 0.0f;
    }
}

/**
 * Check if ADRC should be active for given axis and mode
 */
static bool adrcShouldBeActive(pidState_t *pidState, pidIndex_e pidIndex)
{
    if (pidProfile()->adrcMode == ADRC_DISABLED)
        return false;

    // Only apply ADRC to rate-controlled axes
    if (pidIndex >= PID_ITEM_COUNT)
        return false;

    // Check if this PID slot has ADRC gains configured
    const pid8_t *p = &pidBank()->pid[pidIndex];
    return (p->P > 0 && p->I > 0 && p->D > 0);
}
#endif
```

#### Modificare `pidApplyMulticopterRateController()`

```c
static void FAST_CODE NOINLINE pidApplyMulticopterRateController(pidState_t *pidState, float dT, float dT_inv)
{
    const float rateTarget = getFlightAxisRateOverride(pidState->axis, pidState->rateTarget);
    const float rateError = rateTarget - pidState->gyroRate;

#ifdef USE_ADRC
    // ADRC Mode
    if (pidProfile()->adrcMode == ADRC_ENABLED_MC && adrcShouldBeActive(pidState, pidState->axis)) {
        float wc = pidProfile()->adrcWc * ADRC_WC_SCALE;
        float wo = pidProfile()->adrcWo * ADRC_WO_SCALE;
        float b0 = pidProfile()->adrcB0;

        // Sane fallbacks
        if (wc < 1.0f) wc = 10.0f;
        if (wo < 1.0f) wo = 30.0f;
        if (b0 < 1.0f) b0 = ADRC_B0_FALLBACK * ADRC_B0_SCALE;

        float adrcOutput = adrcController(pidState, rateTarget, pidState->gyroRate,
                                           wc, wo, b0, dT);

        axisPID[pidState->axis] = constrainf(adrcOutput, -pidState->pidSumLimit, +pidState->pidSumLimit);

#ifdef USE_BLACKBOX
        axisPID_P[pidState->axis] = (int32_t)(adrcOutput * 1000);  // Debug: split into P/D/I for blackbox
        axisPID_I[pidState->axis] = (int32_t)(-pidState->adrc_z3 / pidProfile()->adrcB0 * 1000);
        axisPID_D[pidState->axis] = (int32_t)(-pidState->adrc_z2 * pidProfile()->adrcWc * 2 / pidProfile()->adrcB0 * 1000);
        axisPID_F[pidState->axis] = 0;
        axisPID_Setpoint[pidState->axis] = rateTarget;
#endif
        return;  // Skip traditional PID
    }
#endif

    // --- Traditional PID Mode (existing code) ---
    const float rateError = rateTarget - pidState->gyroRate;
    const float newPTerm = pTermProcess(pidState, rateError, dT);
    const float newDTerm = dTermProcess(pidState, rateTarget, dT, dT_inv);
    // ...existing code...
}
```

#### Modificare `pidApplyFixedWingRateController()`

```c
static void NOINLINE pidApplyFixedWingRateController(pidState_t *pidState, float dT, float dT_inv)
{
    const float rateTarget = getFlightAxisRateOverride(pidState->axis, pidState->rateTarget);

#ifdef USE_ADRC
    if (pidProfile()->adrcMode == ADRC_ENABLED_FW && adrcShouldBeActive(pidState, pidState->axis)) {
        float wc = pidProfile()->adrcWc * ADRC_WC_SCALE;
        float wo = pidProfile()->adrcWo * ADRC_WO_SCALE;
        float b0 = pidProfile()->adrcB0;

        if (wc < 1.0f) wc = 10.0f;
        if (wo < 1.0f) wo = 30.0f;
        if (b0 < 1.0f) b0 = ADRC_B0_FALLBACK * ADRC_B0_SCALE;

        float adrcOutput = adrcController(pidState, rateTarget, pidState->gyroRate,
                                           wc, wo, b0, dT);

        // For fixed-wing, add feedforward term
        const float newFFTerm = rateTarget * pidState->kFF;
        const float totalOutput = adrcOutput + newFFTerm;
        const uint16_t limit = pidState->pidSumLimit;

        axisPID[pidState->axis] = constrainf(totalOutput, -limit, +limit);

#ifdef USE_BLACKBOX
        axisPID_P[pidState->axis] = (int32_t)(adrcOutput * 1000);
        axisPID_F[pidState->axis] = (int32_t)(newFFTerm * 1000);
        axisPID_Setpoint[pidState->axis] = rateTarget;
#endif
        pidState->previousRateGyro = pidState->gyroRate;
        return;
    }
#endif

    // --- Traditional PID Mode (existing code) ---
    const float rateError = rateTarget - pidState->gyroRate;
    // ...existing code...
}
```

### 3.3 `src/main/flight/pid.c` — Modifiche a `updatePIDCoefficients()`

Aggiungere aggiornamento guadagni ADRC:

```c
void updatePIDCoefficients(void)
{
    // ...existing TPA/anti-gravity code...

#ifdef USE_ADRC
    if (pidProfile()->adrcMode != ADRC_DISABLED) {
        adrcGainsUpdateRequired = true;
    }
#endif

    // ...existing PID coefficient update code...
}
```

### 3.4 `src/main/flight/pid.c` — Modifiche a `pidController()`

Aggiungere reset ADRC quando il controller viene disabilitato:

```c
void pidController(float dT)
{
    // ...existing code...

#ifdef USE_ADRC
    // Reset ADRC states when disarming or switching modes
    if (!ARMING_FLAG(ARMED) || FLIGHT_MODE(FAILSAFE) || !STATE(NAV_CONTROLLING_RATE)) {
        // Check if any axis needs reset
        bool needReset = false;
        for (int i = 0; i < FLIGHT_DYNAMICS_INDEX_COUNT; i++) {
            if (pidState[i].adrcActive) {
                needReset = true;
                break;
            }
        }
        if (needReset) {
            adrcResetState();
        }
    }
#endif

    // ...existing controller selection code...
}
```

### 3.5 `src/main/flight/pid.c` — Modifiche a `pidInitFilters()`

```c
bool pidInitFilters(void)
{
    // ...existing filter initialization code...

#ifdef USE_ADRC
    // Initialize ADRC states
    for (int axis = 0; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++) {
        pidState[axis].adrc_z1 = pidState[axis].gyroRate;
        pidState[axis].adrc_z2 = 0.0f;
        pidState[axis].adrc_z3 = 0.0f;
        pidState[axis].adrc_lastOutput = 0.0f;
    }
#endif

    pidFiltersConfigured = true;
    return true;
}
```

---

## 4. File CMakeLists.txt

Aggiungere `USE_ADRC` alla build:

```cmake
# In src/main/CMakeLists.txt o equivalente:
option(USE_ADRC "Enable ADRC controller" OFF)
```

---

## 5. Integrazione con Feature Esistenti di INAV

### 5.1 Anti-Gravity
- **Conflitto:** Anti-gravity modifica il guadagno ITerm in base alla throttle
- **Soluzione:** Disabilitare anti-gravity quando ADRC è attivo (l'ESO già stima le distanze)

```c
#ifdef USE_ADRC
    if (pidProfile()->adrcMode != ADRC_DISABLED) {
        // Anti-gravity is redundant with ADRC ESO
        // Skip anti-gravity gain application
    } else {
        // Existing anti-gravity code
    }
#endif
```

### 5.2 ITerm Relax
- **Conflitto:** ITerm Relax applica un filtro al setpoint per ridurre l'accumulo ITerm durante manovre aggressive
- **Soluzione:** Disabilitare ITerm Relax quando ADRC è attivo (l'ESO gestisce già le distorsioni)

### 5.3 Smith Predictor
- **Compatibilità:** Il Smith Predictor è specifico per fixed-wing con ritardi di sistema
- **Soluzione:** ADRC può funzionare con Smith Predictor in FW — il Smith Predictor compensa il ritardo, ADRC gestisce le distorsioni

### 5.4 Adaptive Filter / RPM Filter
- **Compatibilità:** I filtri gyro sono applicati PRIMA del controller
- **Soluzione:** Nessuna modifica necessaria — ADRC riceve gyro già filtrato

### 5.5 Control Derivative (kCD)
- **Conflitto:** kCD usa la derivata del setpoint, ADRC usa la derivata stimata dall'ESO
- **Soluzione:** Disabilitare kCD quando ADRC è attivo

### 5.6 Tracking Anti-windup (kT)
- **Conflitto:** Anti-windup usa kT per prevenire l'accumulo quando i motori sono saturati
- **Soluzione:** ADRC ha già built-in anti-windup attraverso l'ESO (z3 stima la distorsione totale)

---

## 6. Tabella Riassuntiva delle Modifiche

| File | Modifica | Righe Stimate |
|------|----------|---------------|
| `pid.h` | Aggiungere `USE_ADRC` struct fields, enums, costanti | +30 |
| `pid.h` | Aggiungere funzioni API ADRC | +10 |
| `pid.c` | Aggiungere costanti ADRC e stato globale | +10 |
| `pid.c` | Aggiungere `adrcController()` core function | +40 |
| `pid.c` | Aggiungere `adrcResetState()` | +15 |
| `pid.c` | Modificare `pidApplyMulticopterRateController()` | +30 |
| `pid.c` | Modificare `pidApplyFixedWingRateController()` | +30 |
| `pid.c` | Modificare `updatePIDCoefficients()` | +5 |
| `pid.c` | Modificare `pidController()` per reset ADRC | +15 |
| `pid.c` | Modificare `pidInitFilters()` per init ADRC | +10 |
| `pid.c` | Disabilitare anti-gravity/iterm-relax con ADRC | +20 |
| **Totale** | | **~230 righe** |

---

## 7. Valori di Default e Tuning

### 7.1 Valori di Default Raccomandati

```c
// Multirotor (velocità di risposta media)
#define ADRC_WC_DEFAULT       30.0f    // Control bandwidth: 30 rad/s (~5 Hz)
#define ADRC_WO_DEFAULT       90.0f    // Observer bandwidth: 3*wc = 90 rad/s
#define ADRC_B0_DEFAULT       500.0f   // System gain: 50 * 10

// Multirotor (velocità di risposta alta - racing)
// wc = 50-80, wo = 3*wc, b0 = 500-800

// Fixed-Wing (più lento, più inerzia)
#define ADRC_WC_FW_DEFAULT    15.0f    // Control bandwidth: 15 rad/s (~2.4 Hz)
#define ADRC_WO_FW_DEFAULT    45.0f    // Observer bandwidth: 3*wc
#define ADRC_B0_FW_DEFAULT    300.0f   // System gain: 30 * 10
```

### 7.2 Guida al Tuning

| Parametro | Significato | Range Tipico | Effetto se Aumentato |
|-----------|-------------|--------------|---------------------|
| `wc` (P) | Velocità di risposta | 10-80 rad/s | Più aggressivo, più rumore |
| `wo` (I) | Velocità osservatore | 30-300 rad/s | Più reattivo alle distorsioni, più rumore |
| `b0` (D) | Guadagno di sistema | 100-1000 | Scala l'output di controllo |

**Regole di tuning:**
1. Iniziare con `wc = 30`, `wo = 3*wc`, `b0 = 500`
2. Aumentare `wc` fino a quando la risposta non diventa troppo aggressiva
3. Aumentare `wo` fino a quando le distorsioni non vengono più compensate
4. Regolare `b0` per ottenere l'output corretto sui motori

---

## 8. Confronto con Betaflight ADRC

| Aspetto | Betaflight ADRC | INAV ADRC (proposto) |
|---------|-----------------|---------------------|
| **Integrazione** | Sostituisce PID completamente | Modalità ibrida (PID o ADRC) |
| **Parametri** | P/I/D ripurpati | Campi dedicati (adrcWc/Wo/B0) |
| **MC/FW** | Unico implementazione | Separato per MC e FW |
| **Anti-gravity** | Non presente | Disabilitato automaticamente |
| **ITerm Relax** | Non presente | Disabilitato automaticamente |
| **Smith Predictor** | Non presente | Compatibile (FW only) |
| **Blackbox** | ADRC states debuggati | ADRC states debuggati |
| **CHIRP Mode** | Presente (sysid) | Opzionale (da implementare) |
| **Complessità** | ~100 righe | ~230 righe |

---

## 9. Testing e Validazione

### 9.1 Test in Simulazione (SITL)
1. Testare ADRC vs PID tradizionale in scenari identici
2. Verificare stabilità a diverse velocità di rotazione
3. Testare resistenza alle distorsioni (wind gust simulation)

### 9.2 Test su Hardware
1. **Test 1:** Stesso profilo, PID vs ADRC, volo libero
2. **Test 2:** Test di resistenza al vento (lanciare drone in vento leggero)
3. **Test 3:** Test con payload sbilanciato
4. **Test 4:** Test di recovery da manovre aggressive

### 9.3 Blackbox Analysis
Verificare che i seguenti segnali siano registrati correttamente:
- `adrc_z1` — stato stimato dall'osservatore
- `adrc_z2` — derivata della distorsione
- `adrc_z3` — distorsione totale stimata
- `adrc_lastOutput` — ultimo output di controllo

---

## 10. Roadmap di Implementazione

### Fase 1: Core ADRC (1-2 giorni)
- [ ] Aggiungere `USE_ADRC` a pid.h
- [ ] Implementare `adrcController()` in pid.c
- [ ] Implementare `adrcResetState()` in pid.c
- [ ] Integrare in `pidApplyMulticopterRateController()`

### Fase 2: Fixed-Wing (1 giorno)
- [ ] Integrare ADRC in `pidApplyFixedWingRateController()`
- [ ] Aggiungere supporto feedforward per FW

### Fase 3: Integrazione Features (1 giorno)
- [ ] Disabilitare anti-gravity/iterm-relax con ADRC
- [ ] Aggiungere reset ADRC in `pidController()`
- [ ] Aggiungere debug blackbox per ADRC states

### Fase 4: Testing e Tuning (2-3 giorni)
- [ ] Test in simulazione
- [ ] Test su hardware
- [ ] Documentazione UI/CLI

### Fase 5: CHIRP Mode (opzionale, 1-2 giorni)
- [ ] Implementare system identification mode
- [ ] Aggiungere CLI commands per sysid

---

## 11. Risorse e Riferimenti

- **ADRC Analysis (Betaflight):** `ADRC_ANALYSIS.md` — Analisi completa dell'implementazione Betaflight
- **Technical Reference:** `ADRC_TECHNICAL_REFERENCE.md` — Riferimento tecnico con dettagli implementativi
- **Paper di Riferimento:** Han, Q.-L. "From PID to Active Disturbance Rejection Controller"
- **INAV PID Source:** `src/main/flight/pid.c`, `src/main/flight/pid.h`
- **Betaflight ADRC Source:** `src/main/flight/pid.c` (ADRC-betaflight fork)

---

*Documento creato: 2025*
*Basato su analisi di ADRC-betaflight e INAV fork*
