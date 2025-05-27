# 📘 PID vs PID+FLS Multi-Agent Formation Control Project Planner 🚀

**Overall Goal:** Systematically tune and compare PID-Only vs. PID+FLS controllers for multi-agent formation control under diverse dynamic conditions (setpoint changes and various wind disturbances).

---

## 📍 Phase I: Establish a Robust PID-Only Baseline Controller (Systematic Tuning)

### ⚙️ Step 1.1: Prepare for Ziegler-Nichols (Z-N) Ultimate Sensitivity Tuning
- **Code Modifications (`agent_control_main.cpp`):**
    - [X] Implement `ZN_TUNING_ACTIVE` boolean flag (e.g., `const bool ZN_TUNING_ACTIVE = true;`).
    - [X] Implement `ZN_KP_TEST_VALUE` constant (e.g., `const double ZN_KP_TEST_VALUE = 0.1;`).
    - [x] **Conditional PID Initialization:**
        - [x] If `ZN_TUNING_ACTIVE == true`:
            - [x] Initialize ALL PID controllers (all drones, X and Y axes) with `kp = ZN_KP_TEST_VALUE`, `ki = 0.0`, `kd = 0.0`.
            - [x] Ensure FLS is effectively disabled (e.g., `current_run_uses_fls` flag set to `false`).
    - [x] **File Naming for Z-N Iterations:**
        - [x] Modify CSV output filename to be unique for each Z-N test run, incorporating `ZN_KP_TEST_VALUE` (e.g., `multi_drone_zn_test_kp0.100.csv`).
        - [x] Modify metrics output filename similarly (e.g., `metrics_zn_test_kp0.100.txt`).
    - [x] **Simulation Settings for Z-N Test:**
        - [x] Ensure `simulation_time` is sufficient to observe sustained oscillations (e.g., 30-60s might be enough, focusing on the response to the first setpoint change in Phase 1).
        - [x] Wind disturbances should ideally be OFF or minimal during the Ku/Pu finding phase to get a clean system response. If wind *must* be on for the setpoint change to occur as desired, note its presence. *(Self-correction: For finding Ku/Pu, it's best to have a clean step response without other disturbances if possible. Your Phase 1 has a wind starting at t=5s. Consider if you can delay this wind or use a simpler setpoint change for Z-N if the wind interferes with observing pure P-controller oscillations).*

### 🔍 Step 1.2: Iteratively Find Ultimate Gain (`Ku`) and Ultimate Period (`Pu`)
- **Focus:** Drone 0, X-axis response to the first major setpoint change (e.g., initial position to Phase 1 target).
- **Iterative Process:**
    - [X] **Run 1:** Set `ZN_KP_TEST_VALUE` to a very low value (e.g., 0.1).
    - [X] Compile `agent_control_main.cpp`.
    - [X] Execute `pid_standalone_tester.exe`.
    - [X] **Analyze Plots:** Use `plot_pid_data.py` to visualize the generated CSV. Examine Drone 0 X-axis "Position Error" and "Position Tracking".
        - [X] Note: Response should be sluggish, heavily damped.
    - [X] **Subsequent Runs:**
        - [X] Gradually increase `ZN_KP_TEST_VALUE` (e.g., increments of 0.1, then 0.05, then 0.02 as you get closer).
        - [X] For each new `ZN_KP_TEST_VALUE`: Compile, Run, Plot, Analyze.
        - [X] **Look for:** Transition from damped oscillations to **sustained, uniform oscillations** (amplitude neither growing nor significantly shrinking over several cycles).
    - [X] **Identify `Ku_x`:** The `ZN_KP_TEST_VALUE` that produces these sustained oscillations is `Ku_x`.
        - [X] If oscillations grow, you've passed `Ku_x`; reduce `ZN_KP_TEST_VALUE`.
    - [X] **Measure `Pu_x`:** From the plot corresponding to `Ku_x`, measure the time period of one full cycle of the sustained oscillation (e.g., peak-to-peak time). Average over a few cycles if possible.
- **Deliverables for this Step:**
    - [X] Documented `Ku_x` value.
    - [X] Documented `Pu_x` value.
    - [X] Saved plot showing the sustained oscillation used to determine `Ku_x` and `Pu_x` (e.g., `zn_sustained_oscillation_KuX.X_PuY.Y.png`).

**Deliverables for Step 1.2:**
Documented Ku_x value: Ku = 3.5
Documented Pu_x value: Pu = 3.33 s
Saved plot showing the sustained oscillation used to determine Ku_x and Pu_x (You have this, e.g., zn_sustained_oscillation_Ku3.5_Pu3.33.png or similar).

### 🧮 Step 1.3: Calculate Initial PID Gains using Z-N Formulas
- **Action:** Apply classic Z-N formulas for PID:
    - [ ] `Kp_zn = 0.6 * Ku_x`
    - [ ] `Ti_zn = Pu_x / 2.0`
    - [ ] `Td_zn = Pu_x / 8.0`
- **Convert to Kp, Ki, Kd form:**
    - [ ] `Calculated_Kp = Kp_zn`
    - [ ] `Calculated_Ki = Kp_zn / Ti_zn`
    - [ ] `Calculated_Kd = Kp_zn * Td_zn`
- **Deliverable:** Documented `Calculated_Kp`, `Calculated_Ki`, `Calculated_Kd`.

**Deliverables for Step 1.3:**
Documented Calculated_Kp = 2.10
Documented Calculated_Ki = 1.26 (from 1.261)
Documented Calculated_Kd = 0.87 (from 0.874 or 0.8736)

### 🧪 Step 1.4: Implement and Test Initial Z-N Derived PID Gains (PID-Only Run)
- **Code Modifications (`agent_control_main.cpp`):**
    - [ ] Set `ZN_TUNING_ACTIVE = false;`.
    - [ ] Set `USE_FLS_NORMAL_RUN = false;` (or your equivalent for PID-Only mode).
    - [ ] In the "normal run" section, set the PID gains `kp, ki, kd` to your `Calculated_Kp, Calculated_Ki, Calculated_Kd`.
    - [ ] Ensure file naming uses `_pid_only_zn_initial` suffixes.
- **Action:**
    - [ ] Compile.
    - [ ] Run `pid_standalone_tester.exe` (full multi-phase simulation).
- **Analyze:**
    - [ ] Examine plots and metrics. Expect a potentially aggressive response (e.g., ~25% overshoot).
- **Deliverables:**
    - [ ] `multi_drone_pid_only_zn_initial.csv`
    - [ ] `performance_metrics_pid_only_zn_initial.txt`
    - [ ] Saved set of plots (overall and individual drones) for this run.

### 👌 Step 1.5: De-tune and Finalize PID-Only Baseline Controller
- **Goal:** Achieve a "good" PID-Only performance: reasonable overshoot, acceptable settling, and how it inherently handles disturbances. This is your benchmark.
- **Iterative Process:**
    - [ ] **Primary De-tuning:** Reduce `Calculated_Kp` by 30-50% (e.g., `Kp_baseline = Calculated_Kp * 0.7`). Start with this.
    - [ ] Keep `Calculated_Ki` and `Calculated_Kd` initially.
    - [ ] In `agent_control_main.cpp` (normal run, PID-Only mode), update `kp` to `Kp_baseline`.
    - [ ] Compile, Run, Plot, Analyze metrics and visual response across all phases.
    - [ ] **Fine-tune `Kp_baseline`, `Ki_baseline`, `Kd_baseline`:**
        - [ ] If overshoot is still too high, further reduce `Kp_baseline`.
        - [ ] If steady-state error (especially during wind in later phases for PID-Only) is problematic or settling is very slow, slightly increase `Ki_baseline`.
        - [ ] If the system is too oscillatory after Kp reduction, or if response is sluggish, slightly adjust `Kd_baseline` (often Z-N `Kd` is okay or might need slight increase for more damping if Kp was significantly reduced).
    - [ ] Repeat until satisfied with the PID-Only baseline performance.
- **Deliverables:**
    - [ ] Final `Kp_baseline`, `Ki_baseline`, `Kd_baseline` values.
    - [ ] `multi_drone_pid_only_FINAL_BASELINE.csv` (or similar distinct name)
    - [ ] `performance_metrics_pid_only_FINAL_BASELINE.txt`
    - [ ] Saved set of plots for this definitive PID-Only baseline.

---

## 🔬 Phase II: Evaluate and Refine PID+FLS Controller

### ▶️ Step 2.1: Run PID+FLS with Baseline PID Gains
- **Code Modifications (`agent_control_main.cpp`):**
    - [ ] Set `ZN_TUNING_ACTIVE = false;`.
    - [ ] Set `USE_FLS_NORMAL_RUN = true;` (or your equivalent to enable FLS).
    - [ ] Use the `Kp_baseline, Ki_baseline, Kd_baseline` (from Step 1.5) for ALL PID controllers.
    - [ ] Ensure your refined FLS rules (less aggressive during setpoint tracking) are active.
    - [ ] Ensure file naming uses `_pid_fls_with_baseline_pid` suffixes.
- **Action:**
    - [ ] Compile.
    - [ ] Run `pid_standalone_tester.exe`.
- **Deliverables:**
    - [ ] `multi_drone_pid_fls_with_baseline_pid.csv`
    - [ ] `performance_metrics_pid_fls_with_baseline_pid.txt`
    - [ ] Saved set of plots.

### 📊 Step 2.2: Initial Comparative Analysis: PID-Only (Baseline) vs. PID+FLS
- **Action:**
    - [ ] Use your Python script (`plot_pid_data.py`) modified to plot data from `..._pid_only_FINAL_BASELINE.csv` and `..._pid_fls_with_baseline_pid.csv` on the *same axes* for direct comparison.
- **Analyze Critical Performance Aspects:**
    - [ ] **Wind Disturbance Rejection:**
        - [ ] Peak error deviation during each distinct wind event (X, Y, oscillating, gusts, simultaneous).
        - [ ] Speed of recovery during and after wind.
        - [ ] Ability to maintain formation during wind.
    - [ ] **Setpoint Tracking Performance:**
        - [ ] Overshoot after large setpoint changes (t=0, 30, 60, 90s).
        - [ ] Rise time.
        - [ ] Settling time.
    - [ ] **Control Effort:**
        - [ ] Observe `Final Cmd` plots. Is PID+FLS smoother, or does it use less extreme commands during disturbances?
    - [ ] **Overall Path Quality:** Are PID+FLS paths visibly better in the "Formation Paths Comparison"?
- **Deliverable:** Initial assessment of FLS benefits and any new issues introduced by FLS (e.g., if Y-axis overshoot worsened with FLS).

### 🔧 Step 2.3: (If Needed) Targeted Fine-tuning of FLS Rules/Membership Functions
- **Goal:** Address specific shortcomings of FLS identified in Step 2.2, primarily related to wind compensation strength or unwanted interactions.
- **Action (Iterative, if necessary):**
    - [ ] **Identify Problem Scenario:** E.g., "FLS correction for +3.0 X-wind is only -1.5, resulting in residual error." or "FLS causes too much Y-overshoot at t=60s setpoint change."
    - [ ] **Hypothesize Cause & Solution:**
        - [ ] *Weak wind correction:* Strengthen consequent (e.g., SNC to LNC), or make LNC centroid more impactful, or adjust "wind" MF boundaries/peaks.
        - [ ] *FLS contributing to setpoint overshoot:* Weaken FLS consequents that fire during large error + high `dError` conditions (already done, but double-check if specific axes like Y need more).
    - [ ] Modify FLS rules/MFs in `agent_control_main.cpp` (and `fuzzy_test_main.cpp` for FLS surface plot consistency).
    - [ ] Re-run Step 2.1 (PID+FLS with baseline PID gains but updated FLS).
    - [ ] Re-do Step 2.2 (Comparative Analysis).
    - [ ] Repeat until FLS performance is satisfactory for wind rejection without negatively impacting setpoint tracking too much.
- **Deliverable:** Documented FLS changes and their impact. Updated PID+FLS CSVs, metrics, and plots for each iteration.

### ⚙️ Step 2.4: (Optional & Minor) Final PID Gain Tweaks for the PID+FLS System
- **Goal:** Small adjustments to PID gains if the optimized FLS allows for slightly different PID behavior for the *combined* system.
- **Action (Iterative, small changes):**
    - [ ] If FLS handles disturbances exceptionally well, you *might* be able to slightly reduce `Ki_baseline` if the system is too sluggish in eliminating very small final errors.
    - [ ] If the combined PID+FLS response to setpoint changes *still* has minor issues not fixable by FLS rules alone, make tiny adjustments to `Kp_baseline` or `Kd_baseline`.
    - [ ] Re-run Step 2.1 and 2.2 after any such PID tweaks.
- **Caution:** Avoid large PID changes here, as the primary PID tuning was done in Phase I. This is for subtle harmonization.
- **Deliverable:** Final `Kp_FLS_Optimized`, `Ki_FLS_Optimized`, `Kd_FLS_Optimized` (if different from `_baseline`).

---

## 📊 Phase III: Final Comparative Analysis, Visualization, and Reporting

### 💾 Step 3.1: Generate Final Definitive Datasets
- **Action:**
    1.  **PID-Only Final Run:**
        - [ ] `agent_control_main.cpp`: `ZN_TUNING_ACTIVE = false`, `USE_FLS_NORMAL_RUN = false`.
        - [ ] Use `Kp_baseline, Ki_baseline, Kd_baseline`.
        - [ ] Save outputs as `multi_drone_pid_only_FINAL.csv` and `performance_metrics_pid_only_FINAL.txt`.
    2.  **PID+FLS Final Run:**
        - [ ] `agent_control_main.cpp`: `ZN_TUNING_ACTIVE = false`, `USE_FLS_NORMAL_RUN = true`.
        - [ ] Use final PID gains (either `_baseline` or `_FLS_Optimized`) and final FLS configuration.
        - [ ] Save outputs as `multi_drone_pid_fls_FINAL.csv` and `performance_metrics_pid_fls_FINAL.txt`.

### 🖼️ Step 3.2: Create High-Quality Comparative Visualizations for Presentation
- **Action:** Use `plot_pid_data.py` to generate all comparative plots using the `..._FINAL.csv` files.
- **Enhancements for Presentation:**
    - [ ] **Consistent Styling:** Colors, line styles for PID-Only vs. PID+FLS.
    - [ ] **Clear Titles and Labels:** Ensure all plots are self-explanatory.
    - [ ] **Highlight Key Events:**
        - [ ] Use `axvspan` in matplotlib to shade regions for different wind events.
        - [ ] Use vertical lines (`axvline`) for setpoint change times.
    - [ ] **Zoomed-In Plots:** Create separate plots that zoom in on specific interesting events:
        - [ ] A major setpoint change (e.g., t=30s or t=60s) showing error and position tracking.
        - [ ] A significant wind disturbance (e.g., Phase 3 Y-gust) showing error and command responses.
    - [ ] **FLS Characteristic Plots:** Include `fls_membership_functions.png` and `fls_control_surface.png` (ensure they reflect the final FLS design used in `agent_control_main.cpp` by also updating `fuzzy_test_main.cpp` rules/MFs and re-running `fuzzy_standalone_tester.exe`).
- **List of Key Visualizations:**
    - [ ] Overall Formation: X-Position (PID vs PID+FLS)
    - [ ] Overall Formation: Y-Position (PID vs PID+FLS)
    - [ ] Overall Formation: Formation Paths (PID vs PID+FLS)
    - [ ] Overall Formation: Wind Profile (as reference)
    - [ ] Drone 0 (Representative): X-Position Error (PID vs PID+FLS)
    - [ ] Drone 0 (Representative): Y-Position Error (PID vs PID+FLS)
    - [ ] Drone 0 (Representative): X-Commands (showing PID, FLS Corr, Final for PID+FLS; and PID for PID-Only)
    - [ ] Drone 0 (Representative): Y-Commands (similar)
    - [ ] Drone 0 (Representative): X-Position Tracking (PID vs PID+FLS vs Target)
    - [ ] Drone 0 (Representative): Y-Position Tracking (PID vs PID+FLS vs Target)
    - [ ] (Optional) Repeat key individual drone plots for Drone 1 or 2 if they show notably different behavior.

### 🔢 Step 3.3: Compile Quantitative Performance Metrics
- **Action:** Extract data from `..._FINAL.txt` metric files and potentially calculate additional metrics from `..._FINAL.csv` files.
- **Metrics Table (PID-Only vs. PID+FLS for each relevant phase):**
    - [ ] **Setpoint Tracking:**
        - [ ] Overshoot % (for X and Y, for each major setpoint change)
        - [ ] Settling Time (2% or 5% criterion)
        - [ ] Rise Time (10-90%) (Optional, can calculate from CSV)
    - [ ] **Disturbance Rejection (for each significant wind event/phase):**
        - [ ] Peak Error during wind.
        - [ ] Integrated Absolute Error (IAE) during the wind period (`sum(abs(error[t])) * dt`).
        - [ ] Time to recover after wind subsides (if applicable).
    - [ ] **Control Effort (Optional):**
        - [ ] Max `Final Cmd` used.
        - [ ] Variance or sum of squares of `Final Cmd` (indication of "busyness").
- **Output:** Well-formatted tables suitable for a report/presentation.

### ✍️ Step 3.4: Draw Conclusions and Prepare Report/Presentation Narrative
- **Action:** Synthesize all findings.
    - [ ] **Introduction:** Briefly state project goal.
    - [ ] **Methodology:**
        - [ ] Describe the simulated multi-agent system.
        - [ ] Explain the PID controller.
        - [ ] Explain the Z-N tuning process used for the PID baseline.
        - [ ] Describe the GT2 FLS architecture (inputs, outputs, MFs, rule structure, KM type reduction).
        - [ ] Describe the multi-phase simulation scenarios (targets, wind profiles).
    - [ ] **Results - PID-Only Baseline:**
        - [ ] Show key plots and metrics for the tuned PID-Only controller.
        - [ ] Discuss its performance, strengths, and limitations (especially w.r.t. disturbances).
    - [ ] **Results - PID+FLS Controller:**
        - [ ] Show key comparative plots (PID-Only vs. PID+FLS).
        - [ ] Present metric comparison tables.
        - [ ] Discuss how FLS improved performance (be specific: "FLS reduced peak error from X-wind by Y%...").
        - [ ] Discuss any trade-offs or areas where FLS didn't improve or slightly worsened performance (e.g., initial Y-axis overshoot if that persists).
    - [ ] **Discussion:**
        - [ ] Why did FLS help where it did? (e.g., direct feedforward compensation for wind).
        - [ ] Why were certain challenges harder to overcome?
        - [ ] Limitations of the study/simulation.
    - [ ] **Conclusion:** Summarize key findings and the overall effectiveness of PID+FLS for this problem.
    - [ ] **Future Work:** Potential FLS rule enhancements, different FLS types, adaptive methods, more complex drone models, different optimization for PID/FLS.

---
