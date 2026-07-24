# Host-timing-immune SITL scheduling — design notes

Problem (proven 2026-07-19): under host CPU load the firmware thread
can be descheduled for 1.5-2.5ms while sim time keeps advancing —
a state real silicon cannot reach — causing spurious desyncs.
SCHED_FIFO under identical load produces zero, but needs privileges.
Constraint: wall-clock divergence is acceptable; only simulation
fidelity matters, so the sim clock may stall or slow freely.

Two independent designs were produced (an in-session design agent and
a Codex review session). They agree on all of the following:

- ISR delivery and PRIMASK sections are already correctly gated
  (dispatch parks the firmware synchronously; grant-freeze v3); the
  unguarded path is MAINLINE progress.
- Timer reads must count as progress (the firmware intercepts them at
  ~500k/s) — this structurally kills the historic v1 failure where
  stalling on "no progress" choked polling-mode commutation.
- No accumulation of progress allowance (the historic v2 reservoir
  bug): allowances must expire or be capped per iteration.
- CPU-time gating (CLOCK_THREAD_CPUTIME_ID) is diagnostic only: it
  cannot distinguish pacing sleeps from starvation and imports host
  CPU behaviour into the timing model.
- Single-core affinity alone is insufficient (CFS can still preempt
  in favour of the load); strict cooperative handoff at intercept
  granularity is unaffordable (~10^5-10^6 context switches/s).
- QEMU-icount-style determinism is not achievable in this native
  build without instrumentation; treat as a possible future mode.
- After any stall the wall pacer must REBASE, never sprint to catch
  up (reuse the SET_SPEEDUP rebase mechanism).
- Validation: the load reproduction (full-core busy loops +
  esc_square sweeps) must show zero desyncs matching the SCHED_FIFO
  control, with t10-90 and chirp envelopes statistically identical
  across idle / loaded-gated / FIFO, plus the historic cold-start and
  steady-state regressions and existing CI.

They differ on coupling tightness:

## Design A: firmware progress lease (looser, lower risk)

Every firmware-thread intercept (timer read, per-loop watchdog
reload, PRIMASK set/clear, comparator read) renews
`fw_alive_until_ns = sim_now + sim.fw_lag_max_ns` (default ~20us =
one TENKHZ period). The sim stepping loop refuses to pass the lease,
blocking on a semaphore until renewed; the stall loop keeps
dispatching deliverable IRQs (a firmware spin on an ISR-set flag
would otherwise livelock); resume_firmware() renews on the
firmware's behalf after ISR delivery. Bounds sim-visible mainline
starvation at ~20us (vs 1.5-2.5ms), one atomic store per intercept,
unloaded behaviour untouched (renewals every <=2us vs a 20us lease).
Optional phase 2 = pace the firmware loop in sim time instead of the
2us wall sleep (converges toward design B).

## What implementation actually required (bring-up findings)

Design A as specified was implemented and did NOT reach zero desyncs
under load. Reaching the FIFO baseline required three refinements and
one discovery outside the scheduling layer entirely:

1. ISR time is lease-EXEMPT, not lease-renewing. The original
   "resume_firmware() renews on the firmware's behalf" idea is wrong:
   dense comparator edge bursts deliver ISRs continuously and a full
   renewal per delivery refuels the lease indefinitely with zero
   mainline progress. The dispatcher instead pushes the deadline out
   by exactly the sim time each base-level handler consumed, and
   stops tail-chaining when the lease is expired so the gate can wait
   for mainline progress between handlers.

2. Full lease re-arm only at the loop boundary (watchdog reload);
   ordinary intercepts (timer reads, comparator polls, PRIMASK
   set/clear) grant one loop quantum, capped at fw_lag_max_ns ahead.
   Otherwise a firmware thread preempted mid-loop stretches one loop
   iteration across many full leases of simulated time. The wall
   pacer must only rebase after stalls >1ms: rebasing on the
   thousands of micro-stalls per second forgives the pacer's sleep
   threshold each time and lets sim time run ahead of the wall clock.

3. The dominant desync driver was not mainline starvation but
   interrupt-delivery starvation inside PRIMASK windows. micros64()
   is disable/read/enable; the read grants sim time while PRIMASK is
   momentarily set. A sim thread running AHEAD consumes that grant
   later, in a window where PRIMASK is clear - interrupts deliver
   normally. A sim thread starved by the gate is HUNGRY and consumes
   every grant the instant it appears, inside the window - so nearly
   all simulated time advances "inside a critical section" and
   COMP/COM delivery latency stretches to the wall-clock length of
   the firmware's read runs (measured: 105us-20ms). The fix: when a
   deliverable interrupt is pending and the current section is still
   short (<2us granted), wait for the enable before consuming grants;
   genuine irqs-off delay loops cross the threshold and are consumed
   normally (interrupts blocked, as on silicon).

4. The desync INITIATION was a physics artifact all along: after each
   commutation the discrete 500ns solver chatters the freewheel diode
   on/off every step, toggling the comparator at ~2MHz for tens of
   degrees. Each delivered edge ISR advances sim time through its
   register reads and regenerates the next chatter step, so the
   chatter trajectory - and hence ZC accept timing - depends on host
   dispatch availability (edges coalesce when dispatch is busy). A
   late accept inflates commutation_interval, the ci/2 blanking
   window then swallows real crossings, and ci runs away (~4x) into
   desync. SCHED_FIFO "fixed" it only by maximizing dispatch
   availability. The real fix is sim.comparator_min_toggle_ns
   (default 2us): a comparator + EXTI path cannot retrigger at MHz
   rates, so the output is latched for its response time. This
   decouples the chatter from host timing. NOTE: suppressing the
   chatter removes an artifact the model v7 calibration had absorbed
   (steady rpm shifts ~+7% at mid throttle) - the model needs a refit
   against the bench data with the latch active.

Validation (24-core host, 22 pinned busy loops, tool on reserved
cores, patient settling): gate+latch = 0 desyncs across 3 esc_square
sweeps, matching the SCHED_FIFO control; idle also 0. Without the
latch, no gate variant got below ~90 errors per sweep. Loaded-run
validation requires the measurement tool to be protected from the
load and settle timeouts eliminated, else tool starvation and
wall-clock force-advance masquerade as SITL failures.

## Design B: bounded simulated-time credit (tighter, near-lockstep)

Monotonic issued/consumed credit accounts; the per-loop watchdog hook
issues exactly ONE loop quantum (sim.loop_time_ns, 2us) of sim-time
credit per mainline iteration; firmware timer reads issue synchronous
physics-step tickets; the sim thread consumes credit or blocks.
Runahead bounded at one loop quantum. Requires an explicit
cooperative-park/signal-park atomic state machine, makes loop_time_ns
a genuine timing-model parameter (eventually calibrated from G431
cycle counts), and adds synchronous handoffs on the hot timer-read
path (throughput risk to quantify). Also replaces the absolute
pacing clock with a maximum-speed limiter.

## Synthesis / recommendation

Implement A first: it is a small, low-risk change that removes the
proven failure mode with margin (20us << the harm scale, >> the
renewal interval). Instrument stall counts, max observed lag and
per-run maximum mainline gap. If trace comparison against the FIFO
baseline shows residual interleaving artefacts inside the 20us
window, tighten toward B (A's phase 2 is B's core idea); B's
park-state machine and pacer change remain the reference for that
step. Full determinism (CI lockstep mode) stays future work either
way.

## Follow-up: the physics-based resolution (2026-07-20)

The comparator min-toggle latch turned out to be treating a symptom.
A review pass (Codex, second opinion) identified the diode active-set
logic as the root defect: per-step |i|>10mA reclassification with
forced zeroing produced an off/on limit cycle at step rate. The
resolution, in order:

1. Persistent per-phase diode states with sub-step zero-crossing
   events (conduction ends exactly at zero current), complementarity
   checked rail-crossing turn-ons, star point with the resistive
   term, time-averaged torque/bus integrals.
2. The latch became an inertial propagation model: the input must
   stay across the threshold for the comparator's response time
   before the output commits - constant delay, no refractory jitter.
3. The front end is now modelled as the real boards have it: zero
   comparator hysteresis (neither target's firmware configures any),
   band-limited input noise through the divider RC. The occasional
   pre-crossing noise edges are load-bearing: AM32's blanking-window
   scheme needs repeated edge chances, and a perfectly clean
   one-edge-per-crossing comparator death-spirals ci during startup.

Results: reported rpm noise at or below the real ESCs' at every
operating point on both calibrated models (was up to 5x noisier),
desync marginality matching the bench on the 1404, best square-curve
fits of the campaign, CI green. Deferred (acceptance met without
them): exact-time ISR read advancement, TIM2 prescaler-phase
preservation, PWM/dead-time event segmentation - see the Codex plan
if the residual quantization ever matters.
