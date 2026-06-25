# ADRC Integration - Stato Implementazione

## Riepilogo
- **Data inizio**: 2026-06-24
- **File target**: `src/main/flight/pid.h`, `src/main/flight/pid.c`
- **Righe stimate**: ~230
- **Ultimo aggiornamento**: 2026-06-24

## Fase 1: Core ADRC in pid.h ✅ COMPLETATA
- [x] Aggiungere enum `adrcMode_e`
- [x] Aggiungere costanti ADRC (`ADRC_WC_SCALE`, `ADRC_WO_SCALE`, `ADRC_B0_SCALE`, `ADRC_B0_FALLBACK`)
- [x] Aggiungere campi in `pidProfile_t` (`adrcMode`, `adrcWc`, `adrcWo`, `adrcB0`)
- [x] Aggiungere campi in `pidState_t` (`adrc_z1`, `adrc_z2`, `adrc_z3`, `adrc_lastOutput`)
- [x] Dichiarare funzioni API (`adrcResetState`, `adrcUpdateState`, `adrcComputeControl`)

## Fase 2: Core ADRC in pid.c ✅ COMPLETATA
- [x] Aggiungere stato globale `adrcGainsUpdateRequired`
- [x] Aggiungere `adrcUpdateState()` (ESO update con Euler forward)
- [x] Aggiungere `adrcComputeControl()` (control law con virtual PD)
- [x] Aggiungere `adrcResetState()`
- [x] Aggiungere `adrcShouldBeActive()`
- [x] Modificare `pidApplyMulticopterRateController()` con branch ADRC
- [x] Modificare `pidApplyFixedWingRateController()` con branch ADRC + feedforward
- [x] Reset template valori ADRC in PG_RESET_TEMPLATE

## Fase 3: Integrazione Features ✅ COMPLETATA
- [x] Branch ADRC nei controller (MC e FW) disabilita automaticamente PID tradizionale
- [x] Disabilitare anti-gravity quando ADRC attivo (ESO già gestisce disturbi)
- [x] Disabilitare ITerm Relax quando ADRC attivo (ESO già gestisce errori)
- [x] Disabilitare kCD (control derivative) quando ADRC attivo
- [x] Disabilitare kT (tracking anti-windup) quando ADRC attivo

## Fase 4: Target Configuration ✅ COMPLETATA
- [x] Aggiungere `#define USE_ADRC` in `src/main/target/common.h`
- [x] Aggiungere impostazioni ADRC in `src/main/fc/settings.yaml`:
  - [x] `adrc_mode` (OFF/MC/FW) con tabella `adrcModeTable`
  - [x] `adrc_wc` (control bandwidth, default 30)
  - [x] `adrc_wo` (observer bandwidth, default 90)
  - [x] `adrc_b0` (system gain, default 500)
- [x] Aggiungere `adrcModeTable` in settings.yaml

## Fase 5: Blackbox Fields ✅ COMPLETATA
- [x] Aggiungere `FLIGHT_LOG_FIELD_CONDITION_ADRC` in `blackbox_fielddefs.h`
- [x] Aggiungere campi struct `blackboxMainState_t` (`adrcZ1[3]`, `adrcZ2[3]`, `adrcZ3[3]`, `adrcOutput[3]`)
- [x] Aggiungere field definitions in `blackboxMainFields` array
- [x] Aggiungere condition handler per `FLIGHT_LOG_FIELD_CONDITION_ADRC` in `testBlackboxConditionUncached()`
- [x] Popolare campi ADRC in `loadMainState()` (solo quando ADRC attivo)
- [x] Nessun errore di compilazione

## Fase 6: Testing
- [ ] Compilazione pulita (CMake build)
- [ ] Test su hardware

## Note Implementazione
- **ESO Gains**: beta1=3*wo, beta2=3*wo², beta3=wo³ (standard second-order ESO)
- **Control Law**: kp=wc², kd=2*wc (virtual PD)
- **Fallback values**: wc<1→10.0, wo<1→30.0, b0<1→500.0
- **Default MC**: wc=30.0, wo=90.0, b0=500.0
- **Default FW**: wc=15.0, wo=45.0, b0=300.0 (da implementare in CMake)
- **Blackbox**: ESO states esportati per debugging (adrc_z3 come I, -adrc_z2 come D)
