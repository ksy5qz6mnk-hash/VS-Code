# Intelligent EQ Engine — High-Performance DSP Core + VST3/Standalone Design

**Date:** 2026-10-07  
**Status:** Design specification for review  
**Repository:** ksy5qz6mnk-hash/VS-Code  
**Scope:** Sub-project 1 — portable DSP core plus JUCE VST3/Standalone front ends. Windows APO and Linux PipeWire system-wide adapters are explicitly deferred to separate specs after the core passes verification.

## 1. Intent

Build a Windows 10 / Linux real-time Intelligent EQ engine that detects kick drums and basslines in mixed program audio, identifies which spectral components are perceptually important, and increases their apparent clarity/punch without the intelligent EQ stage ever requesting positive EQ gain.

The product must be useful for techno, hardtechno, hardstyle, hardcore and other bass-dense material, but it must remain generic enough for arbitrary music.

Primary user control:

- **Intelligent Gain / Intensity:** continuously variable 0–100% (internally a 0–12 dB contrast authority by default).
- The control determines how much subtractive spectral contrast the optimizer is allowed to create; it is not makeup gain and never commands a positive EQ boost.

Hard invariant:

```
intelligent_filter_gain_db <= 0.0 dB
```

A separate output-safety stage may attenuate further. No stage automatically adds makeup gain.

## 2. Research conclusions that bind this design

1. Real-time process callbacks must not allocate, block, lock, perform file I/O, or depend on UI activity. Windows APO guidance explicitly requires nonblocking real-time methods and minimal added latency.
2. FFT plans/tables and work buffers must be created ahead of processing. Intel IPP documents precomputed FFT state and external work buffers as performance optimizations. PFFFT supports allocation-free transforms and SIMD.
3. Onset detection benefits from complementary features. Essentia exposes HFC, complex-domain, spectral-flux, mel-flux, and RMS onset functions; this design uses a lightweight low-frequency spectral-flux path combined with time-domain envelope derivatives.
4. YIN is efficient and low-latency for F0 estimation, but mixed music is not a single clean periodic source. Therefore YIN/YIN-FFT is a confirmation stage after harmonic candidate generation, not the only bass tracker.
5. Frequency selectivity and masking should operate on perceptual bandwidths rather than linear Hz distance. The design uses ERB-spaced analysis and masking weights based on Moore/Glasberg auditory-filter work.
6. True-peak protection is distinct from the “no positive EQ gain” rule. Output safety follows ITU-R BS.1770-5 style true-peak measurement concepts.
7. VST3 hosts can vary process block length up to the configured maximum; all buffers are sized during prepare/setup, not process calls.
8. Windows and Linux system-wide integration are separate platform projects. Windows 10 APO deployment and Linux PipeWire graph scheduling impose different lifecycle and real-time constraints.

## 3. Chosen architecture

### 3.1 Split real-time and analysis paths

```
Input L/R
   |
   +--> Real-time DSP path --------------------------+
   |                                                |
   +--> Fast feature tap                            |
   |      kick envelope/onset                       |
   |                                                |
   +--> Lock-free analysis feed --> Deep analyzer   |
                                  |                 |
                                  +-> bass/F0       |
                                  +-> harmonics     |
                                  +-> ERB masking   |
                                  +-> beat phase    |
                                           |
                                           v
                                   Scene/Decision model
                                           |
                                           v
                                   immutable EQ plan
                                           |
                                           +----SPSC---->
                                                |
                                                v
                                      real-time filter bank
                                                |
                                         output safety
                                                |
                                              Output
```

The audio thread is never allowed to wait for the analyzer. If the analyzer misses a deadline, the audio thread continues with the last valid plan and smoothly relaxes toward neutral.

### 3.2 Core modules

```
src/core/
  Engine.h/.cpp
  RealtimeProcessor.h/.cpp
  AnalysisScheduler.h/.cpp
  SceneModel.h/.cpp
  SpectralPlanner.h/.cpp
  GainGovernor.h/.cpp
  SafetyLimiter.h/.cpp

src/analysis/
  KickDetector.h/.cpp
  BassCandidateGenerator.h/.cpp
  YinConfirmer.h/.cpp
  HarmonicTracker.h/.cpp
  BeatPhaseTracker.h/.cpp
  ErbMaskingModel.h/.cpp
  MultirateFrontend.h/.cpp

src/dsp/
  DynamicBell.h/.cpp
  DynamicFilterBank.h/.cpp
  EnvelopeFollower.h/.cpp
  Decimator.h/.cpp
  TruePeakMeter.h/.cpp
  SimdDispatch.h/.cpp
  FftBackend.h
  PffftBackend.cpp
  IppBackend.cpp (optional)

src/model/
  KickObject.h
  BassObject.h
  ProtectedRegion.h
  MaskingRegion.h
  EqPlan.h
  EngineTelemetry.h

apps/plugin/
  PluginProcessor.*
  PluginEditor.*

tests/
  unit/
  dsp/
  regression/
  realtime/
  benchmarks/
```

The `core`, `analysis`, `dsp`, and `model` targets must not depend on JUCE UI classes. JUCE is used by the plugin wrapper and may be used for host-facing utilities, but the DSP engine remains directly testable.

## 4. Performance architecture

### 4.1 Numeric format

- Audio path: float32.
- Analysis accumulation where precision materially helps: float32 by default; float64 only offline/tests.
- Aligned contiguous buffers (minimum 32-byte alignment for AVX2-capable builds).
- No per-sample polymorphism.
- No exceptions in the real-time path.
- Denormals disabled/flush-to-zero around processing.

### 4.2 SIMD and FFT

The FFT layer is abstracted.

Default backend:
- maintained PFFFT fork with SSE/AVX/AVX2 support and allocation-free transforms.

Optional x86 performance backend:
- Intel IPP when available.

Runtime/backend rules:
- baseline binary remains compatible with x86-64 CPUs lacking AVX2.
- AVX2/FMA optimized translation units are selected only when CPU capability is present.
- backend selection occurs during initialization, never inside process().
- all FFT plans and work buffers are preallocated.
- transforms are batched only when batching reduces total overhead without increasing control latency.

### 4.3 Multirate analysis

Do not run every detector at 48/96 kHz.

At 48 kHz:
- fast onset/envelope path: native sample rate.
- low-frequency analysis input: anti-alias low-pass then decimate to 12 kHz (factor 4).
- low-frequency pitch/harmonic window: 1024 samples at 12 kHz (~85.3 ms), equivalent temporal span to 4096 samples at 48 kHz at one quarter the sample count.
- short spectral/transient analysis: 1024 samples at native rate only when required.
- ERB energies update at a lower control rate than audio processing.

At other sample rates, choose an integer/polyphase ratio that keeps low-band analysis near 10–14 kHz.

### 4.4 Threading

Audio thread:
- input/output conversion
- fast envelope/onset primitives
- dynamic filters
- dry/wet interpolation
- true-peak safety
- push fixed-size analysis frames into SPSC ring
- consume newest complete EqPlan from SPSC mailbox

Analyzer thread:
- multirate preprocessing
- FFT/YIN
- harmonic candidate tracking
- beat phase
- ERB masking model
- scene model
- constrained spectral optimization

Overflow behavior:
- analysis frames may be dropped.
- audio is never dropped and never blocked.
- stale analysis reduces confidence and smoothly relaxes processing.

## 5. Analysis algorithms

### 5.1 Kick detector

Kick detection is not a single FFT-bin trigger.

Features:
- 30–180 Hz band-limited envelope
- envelope derivative / attack slope
- low-frequency half-wave spectral flux
- short-term crest factor
- optional high-frequency transient corroboration
- minimum inter-onset interval
- adaptive threshold from rolling robust statistics

Output:

```cpp
struct KickObject {
    double onsetTimeSamples;
    float bodyHz;
    float punchHz;
    float confidence;
    float onsetStrength;
    float decayMs;
    float spectralWidthErb;
};
```

The detector has states UNKNOWN -> CANDIDATE -> CONFIRMED -> TRACKED -> DECAYING -> RELEASED.

### 5.2 Bass tracker

Pipeline:
1. Build low-frequency spectral peaks from decimated signal.
2. Generate F0 candidates from harmonic relationships (f, 2f, 3f, 4f).
3. Score candidate salience and temporal stability.
4. Confirm/refine with YIN/YIN-FFT on the band-limited low-frequency signal.
5. Track pitch with hysteresis and confidence smoothing.
6. Reject very short/high-slope events already explained by the kick object.

Output:

```cpp
struct BassObject {
    float fundamentalHz;
    float confidence;
    float stability;
    float energyDb;
    float harmonics[6];
    uint8_t harmonicCount;
};
```

### 5.3 Beat-phase predictor

A lightweight periodicity/PLL-style tracker estimates beat period and phase from confirmed kick onsets.

Prediction may pre-arm filter movement only when confidence exceeds a strict threshold. Missed or syncopated beats immediately reduce prediction authority.

Prediction is an optimization; the engine must work correctly without it.

## 6. Psychoacoustic model

### 6.1 ERB representation

Convert frequency to ERB-rate for masking-distance calculations. Use a compact ERB-spaced energy representation concentrated on 20–1000 Hz, with optional coarse extension to 2 kHz for punch/harmonic masking.

### 6.2 Protected regions

Protected energy comes from:
- kick body
- kick punch
- bass F0
- perceptually relevant bass harmonics

Protection width is expressed in ERB units and varies with confidence and spectral width.

### 6.3 Masking score

For each candidate attenuation region:

```
maskScore =
    maskerEnergy
  * protectedImportance
  * overlapWeightERB
  * sceneConfidence
  * temporalCoincidence
```

Spectral overlap alone is insufficient. If a region is musically important bass content or a protected harmonic, attenuation is penalized.

## 7. Spectral planner

The planner selects at most six dynamic filters.

Objective:

```
maximize
    alpha * kickClarity
  + beta  * bassClarity
  - gamma * tonalDeviation
  - delta * filterMotion
  - epsilon * loudnessLoss
```

Constraints:
- every intelligent filter target gain <= 0 dB
- gain >= -maxCutDb
- protected regions have very large attenuation penalties
- no two filters may redundantly target the same narrow region unless a wider composite response is explicitly required
- gain slew and frequency slew are bounded
- total contrast budget is limited by Intelligent Gain
- filter plan changes must be sparse and stable

Default authority:
- Intelligent Gain: 0–12 dB
- Max instantaneous single-band cut: 6 dB
- Total simultaneous weighted cut budget: 10 dB
- Advanced mode may increase these limits, but defaults prioritize transparency.

## 8. Real-time dynamic filter bank

Use six continuously controllable bell/notch sections implemented with a topology-preserving/state-variable formulation suitable for parameter modulation.

Requirements:
- cut-only transfer for each steady-state section
- frequency, Q, and gain smoothed independently
- gain smoothing may run per sample
- frequency/Q update at bounded control intervals with interpolation
- no coefficient discontinuity
- stable at all supported sample rates
- hard clamp: targetGainDb <= 0.0f

The bank supports:
- stereo linked
- Mid/Side selective attenuation
- per-band bypass with state decay to avoid clicks

M/S mode is only used when the spatial masking score shows a clear benefit. Default behavior remains stereo linked.

## 9. Intelligent Gain semantics

User-facing control:
- 0% = bitwise/near-bitwise neutral processing aside from declared safety path.
- 100% = full configured contrast authority.

Internally:

```
authorityDb = curve(userIntensity) * maxAuthorityDb
requestedCut =
    authorityDb
  * detectorConfidence
  * maskSeverity
  * importanceWeight
```

The control must never be labeled as conventional positive EQ gain in telemetry. UI may display “Intensity” or “Intelligent Gain (contrast)”.

Optional wet/dry control is a convex blend and must not include makeup gain.

## 10. Room-correction interaction

Room correction and music intelligence are separate stages.

```
Static Room Correction -> Intelligent Music EQ -> Output Safety
```

The Intelligent EQ may consume a read-only room profile describing static magnitude correction so its scene scoring does not fight intentionally corrected regions.

It does not continuously adapt room correction from playback audio.

Room measurement/adaptive room correction belongs to a separate subsystem/spec.

## 11. Output safety

The intelligent EQ invariant does not guarantee sample-for-sample peak reduction because time-varying filters alter waveform phase and transient shape.

Therefore:
- true-peak meter follows the processing chain
- optional lookahead limiter only attenuates
- ceiling default: -1.0 dBTP
- limiter is independently bypassable for validation
- limiter telemetry is separate from Intelligent EQ attenuation
- no makeup gain

True-peak verification follows ITU-R BS.1770-5 principles.

## 12. Latency modes

### LIVE
- 0 ms explicit lookahead
- kick reaction is causal
- beat prediction may pre-arm only with sufficient confidence
- intended for low-latency use

### PRECISION
- configurable 3–10 ms audio lookahead
- default 7 ms
- VST3 wrapper reports latency to host
- analyzer is allowed to use future context within declared lookahead only for fast decisions; deep bass tracking remains asynchronous

No hidden latency.

## 13. Supported formats

Core:
- mono and stereo float32
- sample rates: 44.1, 48, 88.2, 96 kHz required
- 176.4/192 kHz supported if benchmark budgets pass

Plugin:
- VST3
- JUCE Standalone
- Windows 10 x64
- Linux x86-64

System-wide Windows APO and Linux PipeWire adapters are not part of this first implementation plan.

## 14. Real-time rules

Inside audio processing:
- zero heap allocation
- zero mutex/condition-variable use
- zero filesystem/network activity
- zero logging
- zero blocking
- no calling code with unknown real-time behavior
- no plan/FFT initialization
- no container growth
- no exceptions crossing the callback
- bounded loops only

All large/stateful preparation occurs in prepare()/setup.

## 15. Verification and tests

### 15.1 Hard-invariant tests

- every generated EQ target gain <= 0.0 dB
- no planner path produces NaN/Inf
- silence produces neutral plan
- invalid/stale analyzer state relaxes to neutral
- filter parameters remain bounded
- no allocation occurs in process() after prepare()

### 15.2 Detector tests

Synthetic corpus:
- kicks from 30–180 Hz with pitch sweeps
- short and long decays
- saturated/distorted kicks
- bass notes 25–220 Hz
- missing fundamentals
- kick + bass collisions
- offbeat bass
- noise/pads/vocals without kick
- double-kick and syncopated rhythms

Metrics:
- onset precision/recall/F1
- kick body frequency error
- bass F0 cents error
- octave error rate
- confidence calibration
- collision classification error

### 15.3 DSP regression

- swept-sine response for each filter state
- time-varying filter stress tests
- impulse response
- stereo/M/S reconstruction
- bypass/null behavior
- sample-rate changes after re-prepare
- random block sizes up to maxSamplesPerBlock

### 15.4 Safety

- worst-case intersample-peak material
- limiter ceiling verification
- no positive target EQ gain
- no unstable transient overshoot beyond safety ceiling when limiter enabled

### 15.5 Performance benchmarks

Measure release builds, pinned when possible:
- audio callback p50/p99/p99.9 wall time
- analyzer CPU time per hop
- SPSC backlog/dropped-frame count
- memory allocations after prepare
- total resident memory
- FFT backend throughput

Performance acceptance:
- audio-thread p99 <= 10% of the current block-duration budget on the reference test machine
- audio-thread p99.9 <= 25% of block-duration budget
- zero xruns attributable to the engine during a 30-minute stress test
- analyzer overload must degrade analysis quality, not audio continuity
- six-filter path must remain comfortably below detector/FFT CPU cost
- SIMD/FFT backend choice must be benchmarked rather than assumed

## 16. Build system

- CMake
- C++20
- JUCE fetched/pinned through CMake
- Catch2 or doctest for non-JUCE unit tests
- pluginval integration for VST3 validation
- sanitizers on supported non-real-time test builds
- warnings-as-errors in CI for core targets
- reproducible dependency versions

Build profiles:
- Debug
- RelWithDebInfo
- Release
- ReleaseNative (local benchmark-only CPU tuning; not redistributable baseline)

Portable release must not assume AVX2.

## 17. Milestones

### M1 — DSP skeleton and invariants
Core library, fixed buffers, SPSC state exchange, neutral processor, hard no-positive-gain tests.

### M2 — Kick detector
Time-domain envelope + LF flux + state machine + synthetic test corpus.

### M3 — Bass tracker
Multirate frontend + harmonic candidates + YIN confirmation + temporal tracker.

### M4 — Psychoacoustic scene model
ERB representation, protected regions, masking score, kick/bass collision arbitration.

### M5 — Dynamic filter bank
Six cut-only dynamic filters, smoothing, stereo/M/S, regression tests.

### M6 — Spectral planner
Constrained optimizer, contrast budget, stale-state behavior.

### M7 — Safety and latency modes
True-peak meter/limiter, Live and Precision lookahead, host latency reporting.

### M8 — JUCE VST3/Standalone
Parameters, state persistence, minimal diagnostic UI, pluginval.

### M9 — Performance optimization
Backend benchmarks, SIMD dispatch, profiler-guided optimization only after functional tests are green.

### M10 — Validation
Long-run stress, synthetic regression corpus, listening AB tests, documentation.

## 18. Deliberate non-goals for this spec

- neural-network detector
- online model training
- multichannel surround
- Windows APO packaging
- Linux PipeWire node
- microphone-driven adaptive room correction
- cloud service
- automatic loudness makeup

These may be separate follow-up specs after the core is validated.

## 19. Research references

- Microsoft: Implementing Audio Processing Objects — https://learn.microsoft.com/en-us/windows-hardware/drivers/audio/implementing-audio-processing-objects
- Microsoft: Audio Processing Object Architecture — https://learn.microsoft.com/en-us/windows-hardware/drivers/audio/audio-processing-object-architecture
- Microsoft: Deploying APOs (Windows 10 class requirements) — https://learn.microsoft.com/en-us/windows-hardware/drivers/dashboard/deploying-audio-processing-objects
- PipeWire real-time module — https://pipewire.pages.freedesktop.org/pipewire/page_module_rt.html
- PipeWire scheduling — https://pipewire.pages.freedesktop.org/pipewire/page_scheduling.html
- Steinberg VST3 processing FAQ — https://steinbergmedia.github.io/vst3_dev_portal/pages/FAQ/Processing.html
- Steinberg VST3 latency — https://steinbergmedia.github.io/vst3_dev_portal/pages/Technical%2BDocumentation/Change%2BHistory/3.1.0/IAudioPresentationLatency.html
- JUCE plugin tutorial — https://juce.com/tutorials/tutorial_create_projucer_basic_plugin/
- Essentia OnsetDetection — https://essentia.upf.edu/reference/streaming_OnsetDetection.html
- Essentia PitchYinFFT — https://essentia.upf.edu/reference/streaming_PitchYinFFT.html
- de Cheveigné & Kawahara, YIN — DOI 10.1121/1.1458024
- Moore & Glasberg, auditory filter / ERB work — DOI 10.1121/1.389861
- Wichern et al., AES 141, masking analysis — AES paper 9646
- ITU-R BS.1770-5 — https://www.itu.int/rec/R-REC-BS.1770-5-202311-I
- Intel IPP FFT reference — https://www.intel.com/content/www/us/en/docs/ipp/developer-guide-reference/2026-0/fast-fourier-transform-functions.html
- PFFFT maintained fork — https://github.com/marton78/pffft

## 20. Design decisions requiring no further clarification

- Deterministic DSP first; ML is deferred.
- Shared core + VST3/Standalone first; system-wide adapters later.
- Subtractive-only Intelligent EQ; no makeup gain.
- Analyzer may drop work; audio thread never waits.
- Portable baseline over architecture-specific assumptions; optional runtime/compile-time accelerated backends.
- Performance optimization is profiler/benchmark driven after correctness.

