# 🧪 Systematic Testing Configuration Guide

## **Manual Configuration for Different Test Scenarios**

### **📍 Configuration File Location**
Your configuration file is located at:
```
C:/Users/ataka/OneDrive/Masaüstü/config/simulation_params.yaml
```

### **🎯 Key Parameters to Modify**

#### **1. Fuzzy Logic System (FLS) Control**
```yaml
controller_settings:
  fls:
    enable: true/false          # Toggle FLS on/off
```

#### **2. Wind Disturbance**
```yaml
scenario_settings:
  enable_wind: true/false       # Toggle wind disturbances
```

#### **3. Feed-Forward Control**
```yaml
controller_settings:
  pid:
    enable_feedforward: true/false  # Toggle feed-forward
```

#### **4. Derivative Filter**
```yaml
controller_settings:
  pid:
    enable_derivative_filter: true/false  # Toggle noise filtering
```

---

## **🧬 Test Scenarios Matrix**

### **Test 1: Baseline (Clean PID)**
```yaml
controller_settings:
  fls:
    enable: false
  pid:
    enable_feedforward: false
    enable_derivative_filter: true  # Keep critical improvements

scenario_settings:
  enable_wind: false
```
**Purpose:** Establish baseline performance with improved PID only

---

### **Test 2: FLS Effect**  
```yaml
controller_settings:
  fls:
    enable: true              # ← CHANGE: Enable FLS
  pid:
    enable_feedforward: false
    enable_derivative_filter: true

scenario_settings:
  enable_wind: false
```
**Purpose:** Measure FLS improvement over baseline

---

### **Test 3: Wind Challenge**
```yaml
controller_settings:
  fls:
    enable: false
  pid:
    enable_feedforward: false
    enable_derivative_filter: true

scenario_settings:
  enable_wind: true           # ← CHANGE: Enable wind
```
**Purpose:** Test system robustness under disturbances

---

### **Test 4: Feed-Forward Benefit**
```yaml
controller_settings:
  fls:
    enable: false
  pid:
    enable_feedforward: true  # ← CHANGE: Enable feed-forward
    enable_derivative_filter: true

scenario_settings:
  enable_wind: false
```
**Purpose:** Evaluate feed-forward tracking improvement

---

### **Test 5: FLS vs Wind**
```yaml
controller_settings:
  fls:
    enable: true              # ← CHANGE: FLS vs wind
  pid:
    enable_feedforward: false
    enable_derivative_filter: true

scenario_settings:
  enable_wind: true           # ← CHANGE: Both enabled
```
**Purpose:** Test FLS effectiveness against wind disturbances

---

### **Test 6: Full System Optimal**
```yaml
controller_settings:
  fls:
    enable: true              # ← CHANGE: All features
  pid:
    enable_feedforward: true  # ← CHANGE: All features
    enable_derivative_filter: true

scenario_settings:
  enable_wind: true           # ← CHANGE: All features
```
**Purpose:** Maximum system performance evaluation

---

## **⚡ Quick Testing Procedure**

### **Step-by-Step Manual Testing:**

1. **Build the system** (if not already built):
   ```powershell
   cd build
   cmake --build . --config Release
   ```

2. **For each test scenario:**
   ```powershell
   # 1. Edit the config file with the parameters above
   notepad "C:/Users/ataka/OneDrive/Masaüstü/config/simulation_params.yaml"
   
   # 2. Run the simulation
   ./build/Release/multi_drone_pid_tester.exe
   
   # 3. Save/rename the output files
   mv simulation_outputs/drone_metrics.csv simulation_outputs/test1_metrics.csv
   mv simulation_outputs/formation_data.csv simulation_outputs/test1_formation.csv
   ```

3. **Analyze results** using existing Python scripts:
   ```powershell
   python final_project_analysis.py
   python plot_simulation.py
   ```

---

## **📊 Expected Performance Comparisons**

### **Key Metrics to Compare:**
- **Overshoot %** - Lower is better
- **Settling Time** - Faster is better  
- **Steady-State Error** - Lower is better
- **Wind Recovery Time** - How quickly system recovers

### **Expected Results:**
- **Test 1 vs Test 2:** FLS should reduce overshoot ~10-20%
- **Test 1 vs Test 3:** Wind increases settling time ~2-3x
- **Test 1 vs Test 4:** Feed-forward improves tracking ~15-25%
- **Test 5:** FLS should maintain performance even with wind
- **Test 6:** Best overall performance combination

---

## **🚀 Automated Testing Option**

If you want to run all tests automatically, use the Python script:

```powershell
python systematic_testing.py
```

This will:
- ✅ Run all 6 test scenarios automatically
- 📁 Organize results in separate folders
- 📊 Generate comparison analysis report
- ⏱️ Complete testing in ~15-20 minutes

---

## **💡 Analysis Tips**

### **Comparing Results:**
1. **Use percentage improvements:**
   ```
   Improvement % = (Baseline - New) / Baseline × 100
   ```

2. **Focus on critical scenarios:**
   - Formation maintenance under wind
   - Setpoint tracking accuracy
   - System stability margins

3. **Document trade-offs:**
   - FLS complexity vs performance gain
   - Feed-forward benefits vs implementation cost
   - Overall system robustness

### **Visualization:**
- Use existing plotting scripts on new CSV data
- Create before/after comparison plots
- Generate performance improvement charts 