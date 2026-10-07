# Intelligent EQ Engine Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Windows 10 / Linux C++20 Intelligent EQ engine that detects kick and bass objects in mixed music and increases their perceptual clarity using subtractive-only dynamic EQ, then expose it as JUCE VST3 and Standalone targets.

**Architecture:** A JUCE-independent DSP core owns the real-time processor, multirate analysis, kick/bass tracking, psychoacoustic scene model, constrained spectral planner, six-band cut-only filter bank, and safety stage. The audio thread never waits for analysis: fixed-capacity SPSC structures transfer frames and immutable EQ plans between the callback and an analyzer thread, with stale plans relaxing smoothly to neutral.

**Tech Stack:** C++20, CMake 3.25+, JUCE 8.0.14, Catch2 3.15.3, PFFFT pinned to commit `df4213c`, optional Intel IPP backend behind a build flag, pluginval for VST3 validation.

**Spec:** `docs/superpowers/specs/2026-10-07-intelligent-eq-engine-design.md`

## Global Constraints

- Intelligent EQ target gain is always `<= 0.0 dB`.
- Default Intelligent Gain authority is 0–12 dB; max single-band cut is 6 dB; total weighted simultaneous cut budget is 10 dB.
- Audio path is float32; float64 is test/offline only.
- Core supports mono/stereo and 44.1, 48, 88.2, 96 kHz; 176.4/192 kHz are benchmark-gated.
- Audio callback performs zero heap allocation, zero mutex/condition-variable use, zero filesystem/network I/O, zero logging, zero blocking, no FFT/plan initialization, no container growth, and no exceptions crossing the callback.
- Analyzer overload may drop analysis work; it must never block or drop audio.
- Six dynamic cut-only filters maximum.
- LIVE mode has 0 ms explicit lookahead; PRECISION mode supports 3–10 ms with 7 ms default and declared host latency.
- Safety ceiling defaults to -1.0 dBTP and may only attenuate; no automatic makeup gain anywhere.
- Room correction remains a separate read-only profile input; adaptive room correction is out of scope.
- Windows APO and Linux PipeWire system-wide adapters are out of scope for this plan.
- Deterministic DSP first; ML/online training are out of scope.
- Portable release must not assume AVX2.
- Dependency pins: JUCE `8.0.14`, Catch2 `v3.15.3`, PFFFT commit `df4213c`.

## Review Focus

1. **Non-power-of-two and rapidly varying host block sizes:** processing must remain correct for random block sizes from 1 to `maxBlockSize`; pinned in Task 1 regression tests.
2. **Analysis starvation/overflow:** dropped frames or a stalled analyzer must not stall audio and must relax EQ toward neutral; pinned in Tasks 2 and 7.
3. **Pathological numeric input:** NaN/Inf/subnormal samples must not poison persistent DSP state or emit NaN/Inf; pinned in Tasks 1, 6, and 8.
4. **Mono/stereo and M/S transitions:** switching spatial mode must not change channel count, explode level, or click; pinned in Task 6.
5. **Host re-prepare/sample-rate changes:** repeated `prepare()` at different rates/block sizes must rebuild state without leaks, stale latency, or invalid filters; pinned in Tasks 1, 8, and 9.

---

## File Structure

```
CMakeLists.txt
CMakePresets.json
cmake/
  Dependencies.cmake
  Warnings.cmake
  Sanitizers.cmake

src/model/
  EngineConfig.h
  KickObject.h
  BassObject.h
  ProtectedRegion.h
  MaskingRegion.h
  EqPlan.h
  EngineTelemetry.h

src/realtime/
  FixedSpscQueue.h
  LatestMailbox.h
  RealtimeGuards.h

src/dsp/
  EnvelopeFollower.h
  Decimator.h/.cpp
  FftBackend.h
  PffftBackend.h/.cpp
  IppBackend.h/.cpp
  DynamicBell.h/.cpp
  DynamicFilterBank.h/.cpp
  TruePeakMeter.h/.cpp
  SafetyLimiter.h/.cpp

src/analysis/
  AnalysisFrame.h
  MultirateFrontend.h/.cpp
  AnalysisScheduler.h/.cpp
  KickDetector.h/.cpp
  BassCandidateGenerator.h/.cpp
  YinConfirmer.h/.cpp
  HarmonicTracker.h/.cpp
  BeatPhaseTracker.h/.cpp
  ErbScale.h
  ErbMaskingModel.h/.cpp

src/core/
  SceneModel.h/.cpp
  GainGovernor.h/.cpp
  SpectralPlanner.h/.cpp
  RealtimeProcessor.h/.cpp
  Engine.h/.cpp

apps/plugin/
  PluginProcessor.h/.cpp
  PluginEditor.h/.cpp
  PluginParameters.h/.cpp

tests/
  TestSignalFactory.h/.cpp
  TestAllocationGuard.h/.cpp
  unit/
    CoreInvariantTests.cpp
    MultirateTests.cpp
    KickDetectorTests.cpp
    BassTrackerTests.cpp
    ErbMaskingTests.cpp
    PlannerTests.cpp
  dsp/
    DynamicFilterBankTests.cpp
    SafetyLimiterTests.cpp
  realtime/
    RealtimeContractTests.cpp
    AnalyzerStarvationTests.cpp
  regression/
    EngineRegressionTests.cpp
    PluginStateTests.cpp
  benchmarks/
    RealtimeBench.cpp
    AnalysisBench.cpp
    FftBench.cpp

docs/
  performance.md
  validation.md
```

---

### Task 1: Project foundation, model contracts, and real-time invariants

**Files:**
- Create: `CMakeLists.txt`
- Create: `CMakePresets.json`
- Create: `cmake/Dependencies.cmake`
- Create: `cmake/Warnings.cmake`
- Create: `cmake/Sanitizers.cmake`
- Create: `src/model/EngineConfig.h`
- Create: `src/model/EqPlan.h`
- Create: `src/model/EngineTelemetry.h`
- Create: `src/model/KickObject.h`
- Create: `src/model/BassObject.h`
- Create: `src/model/ProtectedRegion.h`
- Create: `src/model/MaskingRegion.h`
- Create: `src/realtime/FixedSpscQueue.h`
- Create: `src/realtime/LatestMailbox.h`
- Create: `src/realtime/RealtimeGuards.h`
- Create: `src/core/RealtimeProcessor.h`
- Create: `src/core/RealtimeProcessor.cpp`
- Create: `src/core/Engine.h`
- Create: `src/core/Engine.cpp`
- Create: `tests/TestAllocationGuard.h`
- Create: `tests/TestAllocationGuard.cpp`
- Create: `tests/unit/CoreInvariantTests.cpp`
- Create: `tests/realtime/RealtimeContractTests.cpp`

**Interfaces:**
- Produces: `struct EngineConfig { double sampleRate; uint32_t maxBlockSize; uint32_t channels; float intensity; float maxAuthorityDb; float maxBandCutDb; float totalCutBudgetDb; };`
- Produces: `struct EqBandPlan { float frequencyHz; float q; float gainDb; bool enabled; };`
- Produces: `struct EqPlan { std::array<EqBandPlan, 6> bands; uint64_t sequence; float confidence; };`
- Produces: `class Engine { void prepare(const EngineConfig&); void reset() noexcept; void setIntensity(float normalized) noexcept; void process(float* const* channels, uint32_t numChannels, uint32_t numSamples) noexcept; EngineTelemetry telemetry() const noexcept; };`
- Produces: fixed-capacity `FixedSpscQueue<T, Capacity>` with `bool tryPush(const T&) noexcept` and `bool tryPop(T&) noexcept`.
- Produces: `LatestMailbox<T>` with lock-free `publish(const T&) noexcept` and `bool consumeLatest(T&) noexcept`.

- [ ] **Step 1: Write failing core invariant and callback-shape tests**

Add tests named:
- `intensity_is_clamped_to_zero_one`
- `neutral_engine_preserves_silence`
- `process_accepts_block_sizes_1_to_max`
- `process_rejects_no_audio_by_safe_noop`
- `reprepare_changes_sample_rate_and_block_size_safely`
- `process_never_emits_nan_or_inf_for_finite_input`
- `spsc_queue_never_blocks_and_reports_full`

Assertions must use the spec values: 6 plan bands, 12 dB default authority, 6 dB max band cut, 10 dB total budget.

- [ ] **Step 2: Configure and run tests to verify RED**

Run:
```bash
cmake --preset dev
cmake --build --preset dev --target intelleq_tests
ctest --preset dev -R "CoreInvariant|RealtimeContract" --output-on-failure
```

Expected: configure succeeds, compile or tests fail because the core interfaces/implementation do not yet exist.

- [ ] **Step 3: Implement minimal project foundation and neutral engine**

Implement the exact interfaces above. `process()` is neutral in this task, uses only prepared fixed storage, handles random block sizes up to `maxBlockSize`, and sanitizes persistent state boundaries without allocating.

Dependency pins in `cmake/Dependencies.cmake`:
- JUCE tag `8.0.14`
- Catch2 tag `v3.15.3`
- PFFFT commit `df4213c`

- [ ] **Step 4: Run Task 1 tests to GREEN**

Run the same CMake/CTest commands.

Expected: all Task 1 tests pass.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt CMakePresets.json cmake src/model src/realtime src/core tests
git commit -m "feat: establish realtime-safe Intelligent EQ core"
```

---

### Task 2: Multirate frontend, FFT abstraction, and asynchronous analysis scheduler

**Files:**
- Create: `src/dsp/Decimator.h`
- Create: `src/dsp/Decimator.cpp`
- Create: `src/dsp/FftBackend.h`
- Create: `src/dsp/PffftBackend.h`
- Create: `src/dsp/PffftBackend.cpp`
- Create: `src/dsp/IppBackend.h`
- Create: `src/dsp/IppBackend.cpp`
- Create: `src/analysis/AnalysisFrame.h`
- Create: `src/analysis/MultirateFrontend.h`
- Create: `src/analysis/MultirateFrontend.cpp`
- Create: `src/analysis/AnalysisScheduler.h`
- Create: `src/analysis/AnalysisScheduler.cpp`
- Create: `tests/unit/MultirateTests.cpp`
- Create: `tests/realtime/AnalyzerStarvationTests.cpp`
- Modify: `src/core/Engine.h`
- Modify: `src/core/Engine.cpp`

**Interfaces:**
- Consumes: `EngineConfig`, `FixedSpscQueue`, `LatestMailbox`.
- Produces: `class FftBackend { virtual void prepare(uint32_t size)=0; virtual void realForward(const float* time, float* packedSpectrum) noexcept=0; };`
- Produces: `class MultirateFrontend { void prepare(double sampleRate, uint32_t maxBlockSize); uint32_t push(const float* mono, uint32_t n, float* decimatedOut) noexcept; double analysisRate() const noexcept; };`
- Produces: `struct AnalysisFrame { std::array<float, 1024> lowBand; uint64_t endSample; uint32_t validSamples; };`
- Produces: `class AnalysisScheduler { void prepare(const EngineConfig&); bool pushAudio(const float* mono, uint32_t n, uint64_t sampleClock) noexcept; void start(); void stop(); uint64_t droppedFrames() const noexcept; };`

- [ ] **Step 1: Write failing multirate and starvation tests**

Tests:
- 48 kHz maps low-band analysis to exactly 12 kHz.
- 44.1/88.2/96 kHz choose a ratio that keeps analysis rate in 10–14 kHz.
- A 4 kHz tone above the low-band anti-alias cutoff is attenuated before decimation.
- PFFFT plan/work buffers are created during `prepare()`, not `realForward()`.
- Saturating the analysis queue increments `droppedFrames()` while a concurrent audio loop continues processing.
- Analyzer stopped for 2 seconds does not block `Engine::process()`.

- [ ] **Step 2: Run tests to verify RED**

Run:
```bash
cmake --build --preset dev --target intelleq_tests
ctest --preset dev -R "Multirate|AnalyzerStarvation" --output-on-failure
```

Expected: FAIL because frontend/backend/scheduler are absent.

- [ ] **Step 3: Implement decimation, PFFFT backend, and scheduler**

Use a fixed-coefficient anti-alias low-pass/polyphase decimator for integer factors. Keep PFFFT state/workspace lifetime in `prepare()`. `IppBackend` compiles only under `INTELLEQ_ENABLE_IPP` and otherwise is not selected.

- [ ] **Step 4: Run Task 2 tests to GREEN**

Expected: all multirate and starvation tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/dsp src/analysis src/core tests/unit/MultirateTests.cpp tests/realtime/AnalyzerStarvationTests.cpp
git commit -m "feat: add multirate asynchronous analysis frontend"
```

---

### Task 3: Kick-object detector

**Files:**
- Create: `src/dsp/EnvelopeFollower.h`
- Create: `src/analysis/KickDetector.h`
- Create: `src/analysis/KickDetector.cpp`
- Create: `src/analysis/BeatPhaseTracker.h`
- Create: `src/analysis/BeatPhaseTracker.cpp`
- Create: `tests/TestSignalFactory.h`
- Create: `tests/TestSignalFactory.cpp`
- Create: `tests/unit/KickDetectorTests.cpp`

**Interfaces:**
- Consumes: `AnalysisFrame`, `KickObject`.
- Produces: `class KickDetector { void prepare(double nativeRate, double lowBandRate); std::optional<KickObject> process(const AnalysisFrame&, std::span<const float> nativeMono); void reset() noexcept; };`
- Produces: `class BeatPhaseTracker { void observe(const KickObject&); struct Prediction { uint64_t nextSample; float confidence; }; std::optional<Prediction> prediction() const noexcept; };`

- [ ] **Step 1: Write failing synthetic kick tests**

Generate deterministic test signals for:
- body fundamentals at 30, 45, 60, 90, 120, 180 Hz;
- downward pitch sweeps;
- short and long decays;
- clipped/saturated kicks;
- double kicks and syncopated onsets;
- pads/noise without kick.

Assert:
- onset F1 >= 0.90 on the synthetic corpus;
- median body-frequency error <= 5%;
- no false confirmed kick for the pad/noise fixtures;
- prediction confidence rises after four periodic kicks and drops after a deliberately missed/off-grid onset.

- [ ] **Step 2: Run detector tests to verify RED**

Run:
```bash
cmake --build --preset dev --target intelleq_tests
ctest --preset dev -R "KickDetector" --output-on-failure
```

Expected: FAIL because detector is absent.

- [ ] **Step 3: Implement kick features and state machine**

Implement 30–180 Hz envelope, derivative, LF half-wave spectral flux, crest factor, adaptive robust threshold, minimum inter-onset interval, and UNKNOWN→CANDIDATE→CONFIRMED→TRACKED→DECAYING→RELEASED state transitions. Keep high-frequency corroboration optional and disabled by default.

- [ ] **Step 4: Run tests to GREEN**

Expected: synthetic corpus metrics meet thresholds.

- [ ] **Step 5: Commit**

```bash
git add src/dsp/EnvelopeFollower.h src/analysis/KickDetector.* src/analysis/BeatPhaseTracker.* tests/TestSignalFactory.* tests/unit/KickDetectorTests.cpp
git commit -m "feat: detect and track kick objects"
```

---

### Task 4: Bass F0 candidate generation, YIN confirmation, and harmonic tracking

**Files:**
- Create: `src/analysis/BassCandidateGenerator.h`
- Create: `src/analysis/BassCandidateGenerator.cpp`
- Create: `src/analysis/YinConfirmer.h`
- Create: `src/analysis/YinConfirmer.cpp`
- Create: `src/analysis/HarmonicTracker.h`
- Create: `src/analysis/HarmonicTracker.cpp`
- Create: `tests/unit/BassTrackerTests.cpp`

**Interfaces:**
- Consumes: low-band spectrum from `FftBackend`, `KickObject`.
- Produces: `struct F0Candidate { float hz; float salience; float stability; };`
- Produces: `class BassCandidateGenerator { uint32_t generate(std::span<const float> magnitude, double analysisRate, std::span<F0Candidate> out) noexcept; };`
- Produces: `class YinConfirmer { void prepare(double analysisRate, uint32_t maxWindow); struct Result { float hz; float confidence; }; Result refine(std::span<const float> lowBand, float candidateHz) noexcept; };`
- Produces: `class HarmonicTracker { std::optional<BassObject> update(std::span<const F0Candidate>, const YinConfirmer::Result&, const std::optional<KickObject>&); void reset() noexcept; };`

- [ ] **Step 1: Write failing bass corpus tests**

Fixtures:
- notes from 25–220 Hz;
- second harmonic louder than the fundamental;
- missing fundamental;
- glides;
- kick and bass at near-identical F0;
- kick at F0 with bass at 2×F0;
- offbeat bass;
- no-bass noise/pad.

Assert:
- median F0 error <= 25 cents on clean notes;
- octave error rate <= 2% on clean/missing-fundamental fixtures;
- sustained near-identical kick+bass is classified as persistent bass after the kick decay;
- no stable BassObject on noise/pad fixtures;
- confidence decays when F0 becomes ambiguous.

- [ ] **Step 2: Run tests to verify RED**

Run:
```bash
cmake --build --preset dev --target intelleq_tests
ctest --preset dev -R "BassTracker" --output-on-failure
```

Expected: FAIL.

- [ ] **Step 3: Implement harmonic candidates + bounded YIN refinement**

Generate F0 candidates from peaks and 2×/3×/4× harmonic relationships, then use YIN only around candidate periods to reduce CPU and octave mistakes. Apply temporal hysteresis and kick-explained transient rejection.

- [ ] **Step 4: Run tests to GREEN**

Expected: metrics meet thresholds.

- [ ] **Step 5: Commit**

```bash
git add src/analysis/BassCandidateGenerator.* src/analysis/YinConfirmer.* src/analysis/HarmonicTracker.* tests/unit/BassTrackerTests.cpp
git commit -m "feat: track bass fundamentals and harmonics"
```

---

### Task 5: ERB psychoacoustic scene model and kick/bass collision arbitration

**Files:**
- Create: `src/analysis/ErbScale.h`
- Create: `src/analysis/ErbMaskingModel.h`
- Create: `src/analysis/ErbMaskingModel.cpp`
- Create: `src/core/SceneModel.h`
- Create: `src/core/SceneModel.cpp`
- Create: `tests/unit/ErbMaskingTests.cpp`

**Interfaces:**
- Consumes: `KickObject`, `BassObject`, low-band magnitude spectrum.
- Produces: `float hzToErb(float hz) noexcept;`
- Produces: `float erbToHz(float erb) noexcept;`
- Produces: `class ErbMaskingModel { void prepare(double sampleRate); SceneMask analyze(const KickObject*, const BassObject*, std::span<const float> magnitudes) const noexcept; };`
- Produces: `struct SceneMask { std::array<ProtectedRegion, 12> protectedRegions; uint8_t protectedCount; std::array<MaskingRegion, 24> maskers; uint8_t maskerCount; float confidence; };`
- Produces: `class SceneModel { SceneMask update(const std::optional<KickObject>&, const std::optional<BassObject>&, std::span<const float> magnitudes) noexcept; };`

- [ ] **Step 1: Write failing ERB and collision tests**

Assert:
- Hz↔ERB round-trip error < 0.1% from 20–2000 Hz;
- kick 52 Hz and bass 104 Hz produce two non-overlapping protected regions;
- kick 59 Hz and bass 61 Hz merge into one protected region;
- a protected bass harmonic cannot also be emitted as a masker;
- masker score increases with temporal coincidence and masker energy;
- stale/low-confidence objects reduce scene confidence.

- [ ] **Step 2: Run tests to verify RED**

Run:
```bash
cmake --build --preset dev --target intelleq_tests
ctest --preset dev -R "ErbMasking" --output-on-failure
```

Expected: FAIL.

- [ ] **Step 3: Implement ERB masking and protected-region merge logic**

Use fixed-capacity arrays only. No vector growth in analyzer hot paths.

- [ ] **Step 4: Run tests to GREEN**

- [ ] **Step 5: Commit**

```bash
git add src/analysis/ErbScale.h src/analysis/ErbMaskingModel.* src/core/SceneModel.* tests/unit/ErbMaskingTests.cpp
git commit -m "feat: model psychoacoustic bass masking"
```

---

### Task 6: Six-band cut-only dynamic filter bank with stereo/M/S processing

**Files:**
- Create: `src/dsp/DynamicBell.h`
- Create: `src/dsp/DynamicBell.cpp`
- Create: `src/dsp/DynamicFilterBank.h`
- Create: `src/dsp/DynamicFilterBank.cpp`
- Create: `tests/dsp/DynamicFilterBankTests.cpp`
- Modify: `src/core/RealtimeProcessor.h`
- Modify: `src/core/RealtimeProcessor.cpp`

**Interfaces:**
- Consumes: `EqPlan`.
- Produces: `class DynamicBell { void prepare(double sampleRate); void setTarget(float frequencyHz, float q, float gainDb) noexcept; float processSample(float x) noexcept; float targetGainDb() const noexcept; };`
- Produces: `enum class SpatialMode { StereoLinked, MidSide };`
- Produces: `class DynamicFilterBank { void prepare(double sampleRate, uint32_t maxBlockSize, uint32_t channels); void applyPlan(const EqPlan&) noexcept; void setSpatialMode(SpatialMode) noexcept; void process(float* const* channels, uint32_t numChannels, uint32_t numSamples) noexcept; };`

- [ ] **Step 1: Write failing filter regression tests**

Assert:
- any requested positive gain is clamped to exactly 0 dB;
- each enabled steady-state band has no magnitude response > +0.05 dB across a dense 20 Hz–20 kHz sweep;
- dynamic frequency/Q/gain changes do not generate NaN/Inf;
- random plan motion remains bounded;
- stereo linked preserves L/R relationship;
- M/S encode→process neutral→decode reconstructs within -120 dB RMS error;
- switching StereoLinked↔MidSide with a neutral plan does not click above -90 dBFS discontinuity;
- NaN/Inf input samples are prevented from corrupting persistent filter state.

- [ ] **Step 2: Run tests to verify RED**

Run:
```bash
cmake --build --preset dev --target intelleq_tests
ctest --preset dev -R "DynamicFilterBank" --output-on-failure
```

Expected: FAIL.

- [ ] **Step 3: Implement topology-preserving/state-variable cut filters and smoothing**

Use per-sample gain smoothing and bounded control-rate frequency/Q updates. Clamp all targets before they reach filter state.

- [ ] **Step 4: Run tests to GREEN**

- [ ] **Step 5: Commit**

```bash
git add src/dsp/DynamicBell.* src/dsp/DynamicFilterBank.* src/core/RealtimeProcessor.* tests/dsp/DynamicFilterBankTests.cpp
git commit -m "feat: add cut-only dynamic filter bank"
```

---

### Task 7: Constrained spectral planner, Intelligent Gain, and stale-state fallback

**Files:**
- Create: `src/core/GainGovernor.h`
- Create: `src/core/GainGovernor.cpp`
- Create: `src/core/SpectralPlanner.h`
- Create: `src/core/SpectralPlanner.cpp`
- Create: `tests/unit/PlannerTests.cpp`
- Modify: `src/analysis/AnalysisScheduler.cpp`
- Modify: `src/core/Engine.cpp`

**Interfaces:**
- Consumes: `SceneMask`, previous `EqPlan`, normalized user intensity.
- Produces: `class GainGovernor { float authorityDb(float normalizedIntensity) const noexcept; float clampBandCut(float requestedDb) const noexcept; };`
- Produces: `class SpectralPlanner { EqPlan plan(const SceneMask&, const EqPlan& previous, float intensity, uint64_t sequence) noexcept; EqPlan relaxTowardNeutral(const EqPlan&, float dtSeconds) noexcept; };`

- [ ] **Step 1: Write failing planner invariants**

Assert:
- 0% intensity emits neutral plan;
- 100% intensity never exceeds 12 dB authority, 6 dB single-band cut, or 10 dB weighted total cut budget;
- every emitted band gain <= 0 dB;
- protected regions are never selected as attenuation centers;
- redundant bands targeting the same narrow masker collapse to one band;
- filter movement is bounded between sequential plans;
- all NaN/Inf scene inputs yield a safe neutral/finite plan;
- analyzer starvation for a configurable stale timeout produces monotonic relaxation to neutral;
- queue overflow cannot change audio continuity, only plan freshness.

- [ ] **Step 2: Run tests to verify RED**

Run:
```bash
cmake --build --preset dev --target intelleq_tests
ctest --preset dev -R "Planner|AnalyzerStarvation" --output-on-failure
```

Expected: FAIL.

- [ ] **Step 3: Implement bounded greedy optimizer**

Use a deterministic bounded candidate scorer rather than a general-purpose solver in the real-time product. Score maskers by clarity benefit minus tonal deviation/filter-motion/loudness penalties, select at most six non-redundant bands, then apply hard budgets through `GainGovernor`.

- [ ] **Step 4: Run tests to GREEN**

- [ ] **Step 5: Commit**

```bash
git add src/core/GainGovernor.* src/core/SpectralPlanner.* src/analysis/AnalysisScheduler.cpp src/core/Engine.cpp tests/unit/PlannerTests.cpp tests/realtime/AnalyzerStarvationTests.cpp
git commit -m "feat: plan bounded subtractive spectral contrast"
```

---

### Task 8: True-peak safety stage and LIVE/PRECISION latency modes

**Files:**
- Create: `src/dsp/TruePeakMeter.h`
- Create: `src/dsp/TruePeakMeter.cpp`
- Create: `src/dsp/SafetyLimiter.h`
- Create: `src/dsp/SafetyLimiter.cpp`
- Create: `tests/dsp/SafetyLimiterTests.cpp`
- Modify: `src/core/RealtimeProcessor.h`
- Modify: `src/core/RealtimeProcessor.cpp`
- Modify: `src/core/Engine.h`
- Modify: `src/core/Engine.cpp`

**Interfaces:**
- Produces: `enum class LatencyMode { Live, Precision };`
- Produces: `class TruePeakMeter { void prepare(double sampleRate); float processBlock(float* const* channels, uint32_t numChannels, uint32_t numSamples) noexcept; };`
- Produces: `class SafetyLimiter { void prepare(double sampleRate, uint32_t maxBlockSize, uint32_t channels); void setCeilingDbTp(float) noexcept; void setLookaheadMs(float) noexcept; void process(float* const* channels, uint32_t numChannels, uint32_t numSamples) noexcept; uint32_t latencySamples() const noexcept; };`
- Adds: `Engine::setLatencyMode(LatencyMode, float precisionLookaheadMs = 7.0f) noexcept` and `Engine::latencySamples() const noexcept`.

- [ ] **Step 1: Write failing safety/latency tests**

Assert:
- limiter default ceiling is -1.0 dBTP;
- limiter only attenuates;
- worst-case intersample-peak fixtures remain <= -1.0 dBTP within meter tolerance;
- LIVE reports 0 samples explicit lookahead;
- PRECISION clamps requested lookahead to 3–10 ms and defaults to 7 ms;
- repeated re-prepare recalculates latency correctly at 44.1/48/96 kHz;
- non-finite input cannot leave non-finite limiter state/output.

- [ ] **Step 2: Run tests to verify RED**

Run:
```bash
cmake --build --preset dev --target intelleq_tests
ctest --preset dev -R "SafetyLimiter" --output-on-failure
```

Expected: FAIL.

- [ ] **Step 3: Implement true-peak meter, attenuation-only limiter, and delay line**

All delay memory is allocated in `prepare()`. Bypass/validation mode keeps limiter telemetry separate from Intelligent EQ attenuation.

- [ ] **Step 4: Run tests to GREEN**

- [ ] **Step 5: Commit**

```bash
git add src/dsp/TruePeakMeter.* src/dsp/SafetyLimiter.* src/core tests/dsp/SafetyLimiterTests.cpp
git commit -m "feat: add true-peak safety and latency modes"
```

---

### Task 9: JUCE VST3/Standalone wrapper, parameters, state, and diagnostics UI

**Files:**
- Create: `apps/plugin/PluginParameters.h`
- Create: `apps/plugin/PluginParameters.cpp`
- Create: `apps/plugin/PluginProcessor.h`
- Create: `apps/plugin/PluginProcessor.cpp`
- Create: `apps/plugin/PluginEditor.h`
- Create: `apps/plugin/PluginEditor.cpp`
- Create: `tests/regression/PluginStateTests.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `Engine`, `LatencyMode`, `EngineTelemetry`.
- Produces JUCE parameters:
  - `intensity`: 0.0–1.0, default 0.5
  - `kickFocus`: 0.0–1.0, default 0.8
  - `bassFocus`: 0.0–1.0, default 0.7
  - `focusMode`: Punch/Body/Sub, default Body
  - `character`: Transparent/Punch/Extreme, default Punch
  - `latencyMode`: Live/Precision, default Precision
  - `lookaheadMs`: 3.0–10.0, default 7.0
  - `safetyEnabled`: bool, default true
  - `ceilingDbTp`: -6.0 to -0.1, default -1.0
- Produces VST3 and Standalone targets named `IntelligentEQ`.

- [ ] **Step 1: Write failing plugin state tests**

Assert:
- parameter ranges/defaults exactly match the interfaces above;
- serialized state round-trips every parameter;
- processor reports `Engine::latencySamples()` after prepare and after mode/lookahead changes;
- mono and stereo bus layouts are accepted; unsupported layouts are rejected;
- random host block sizes up to prepared maximum process successfully;
- re-prepare after sample-rate/block-size changes updates engine and host latency.

- [ ] **Step 2: Run tests to verify RED**

Run:
```bash
cmake --build --preset dev --target intelleq_tests
ctest --preset dev -R "PluginState" --output-on-failure
```

Expected: FAIL because wrapper is absent.

- [ ] **Step 3: Implement JUCE processor, parameter tree, and minimal diagnostics editor**

UI shows Intensity prominently and read-only telemetry for detected kick Hz/confidence, bass Hz/confidence, active attenuation, latency mode, and safety activity. No visualization work that can block the audio thread.

- [ ] **Step 4: Build plugin and run state tests**

Run:
```bash
cmake --build --preset release --target IntelligentEQ_VST3 IntelligentEQ_Standalone
ctest --preset dev -R "PluginState" --output-on-failure
```

Expected: targets build; tests pass.

- [ ] **Step 5: Run pluginval**

Run pluginval at strictness level 10 against the generated VST3.

Expected: no validation errors.

- [ ] **Step 6: Commit**

```bash
git add apps/plugin CMakeLists.txt tests/regression/PluginStateTests.cpp
git commit -m "feat: expose Intelligent EQ as VST3 and standalone"
```

---

### Task 10: End-to-end regression, benchmarks, profiling gates, and documentation

**Files:**
- Create: `tests/regression/EngineRegressionTests.cpp`
- Create: `tests/benchmarks/RealtimeBench.cpp`
- Create: `tests/benchmarks/AnalysisBench.cpp`
- Create: `tests/benchmarks/FftBench.cpp`
- Create: `docs/performance.md`
- Create: `docs/validation.md`
- Modify: `CMakeLists.txt`
- Modify only profiler-proven hotspots from prior tasks.

**Interfaces:**
- Consumes all prior public interfaces.
- Produces benchmark executables `intelleq_bench_realtime`, `intelleq_bench_analysis`, `intelleq_bench_fft`.
- Produces release acceptance report format in `docs/validation.md`.

- [ ] **Step 1: Write failing end-to-end regression tests**

Fixtures combine:
- 160 BPM kick train + offbeat bass;
- 200+ BPM dense kick/bass sequence;
- kick and bass at same F0;
- missing-fundamental bass;
- silence/noise/pad-only;
- clipped hot master;
- mono and stereo content.

Assert:
- 0% Intensity is near-null against input with safety disabled (RMS residual <= -120 dBFS);
- no emitted planner gain > 0 dB;
- kick/bass protected-region collisions never create cuts inside the protected region;
- stale analyzer decays toward neutral while audio remains continuous;
- 30-minute synthetic stress harness reports zero engine-induced xruns in the controlled test runner;
- no allocations occur in `Engine::process()` after `prepare()`.

- [ ] **Step 2: Run regression tests to verify RED if any integration gap remains**

Run:
```bash
cmake --build --preset dev --target intelleq_tests
ctest --preset dev --output-on-failure
```

Expected: any integration gap is exposed before optimization; do not optimize around a failing functional test.

- [ ] **Step 3: Implement only missing integration glue until full suite is GREEN**

No algorithm expansion beyond the spec.

- [ ] **Step 4: Add benchmark harnesses and establish baseline**

Measure release:
- audio callback p50/p99/p99.9 time;
- analyzer time per hop;
- SPSC dropped-frame count;
- post-prepare allocation count;
- RSS;
- PFFFT throughput.

Acceptance:
- audio p99 <= 10% of current block-duration budget;
- audio p99.9 <= 25%;
- analyzer overload degrades analysis freshness, not audio continuity;
- six-filter processing cost remains below analysis cost.

- [ ] **Step 5: Compare FFT/SIMD backends and optimize only measured hotspots**

Baseline portable PFFFT first. Enable optional IPP only if benchmarked improvement is material and does not compromise portable builds. Record every benchmark before/after in `docs/performance.md`.

- [ ] **Step 6: Run full verification matrix**

Run:
```bash
cmake --build --preset release
ctest --preset release --output-on-failure
```

Then run:
- pluginval strictness 10;
- realtime benchmark;
- analysis benchmark;
- FFT benchmark;
- 30-minute stress test;
- sanitizer build/test for non-real-time targets.

Expected: all functional tests green and every performance acceptance threshold recorded.

- [ ] **Step 7: Write validation/performance documentation**

`docs/validation.md` must record:
- OS/compiler/CPU;
- sample rate/block size;
- test corpus version;
- detector metrics;
- latency;
- safety result;
- pluginval result;
- stress duration;
- known limitations.

`docs/performance.md` must record:
- compiler flags;
- backend;
- p50/p99/p99.9 callback time;
- analyzer hop time;
- FFT throughput;
- allocation count;
- RSS;
- profiler-derived optimizations.

- [ ] **Step 8: Commit**

```bash
git add tests docs CMakeLists.txt src
git commit -m "test: validate and benchmark Intelligent EQ engine"
```

---

## Final Completion Contract

Implementation is complete only when:

1. Every task's named tests has been observed RED before implementation and GREEN afterward.
2. Full CTest suite is green in Release.
3. VST3 and Standalone targets build on Windows 10 x64 and Linux x86-64.
4. pluginval strictness 10 reports no errors.
5. No Intelligent EQ target gain above 0 dB is observed in unit, randomized, or end-to-end tests.
6. No heap allocation occurs in the audio callback after `prepare()`.
7. Analyzer starvation/overflow never blocks audio.
8. LIVE reports 0 ms explicit lookahead; PRECISION reports configured 3–10 ms lookahead.
9. Safety limiter holds the validated -1.0 dBTP default ceiling within test tolerance.
10. Performance acceptance thresholds from the spec are measured and met on the recorded reference machine.
11. A whole-branch code review is performed after all tasks, with Critical/Important findings fixed under TDD before branch completion.
