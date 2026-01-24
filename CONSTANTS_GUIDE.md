# 🎯 CONSTANTS GUIDE - Single Source of Truth

**Last Updated:** Competition prep - Week before competition

---

## ⚠️ IMPORTANT: WHERE TO UPDATE VALUES

### ✅ **USE THIS FILE FOR EVERYTHING:**
📁 **`RobotConstants.java`** (TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/constants/)

All competition teleop settings are here:
- PIDF gains (kP, kI, kD, kF)
- Default RPM
- All 7 shooting ranges (distances, RPM, hood positions)
- Hood positions
- Transfer ramp positions
- Drive control settings
- Turret control settings
- Limelight camera specs

### ❌ **DO NOT USE:**
📁 ~~`TeleOpConstants.java`~~ - **DEPRECATED** - Kept for backwards compatibility only

---

## 📝 HOW TO UPDATE SETTINGS:

### **1. After Tuning PIDF Gains:**

**Steps:**
1. Run `FlywheelPIDFTuner` OpMode
2. Tune gains using gamepad controls
3. Press **B** to save gains to telemetry log
4. Open **RobotConstants.java**
5. Go to **lines 20-23**
6. Update these values:
   ```java
   public static final double FLYWHEEL_KP = 0.00035;   // Copy from telemetry
   public static final double FLYWHEEL_KI = 0.000025;  // Copy from telemetry
   public static final double FLYWHEEL_KD = 0.00015;   // Copy from telemetry
   public static final double FLYWHEEL_KF = 0.000215;  // Copy from telemetry
   ```
7. Recompile and test in teleop

**Telemetry log format:**
```
=== TUNED GAINS - COPY TO RobotConstants.java ===
FLYWHEEL_KP = 0.000350;
FLYWHEEL_KI = 0.000025;
FLYWHEEL_KD = 0.000150;
FLYWHEEL_KF = 0.000215;
DEFAULT_TARGET_RPM = 3500;
Update these in RobotConstants.java lines 20-23!
```

---

### **2. After Testing Shooting Ranges:**

**Steps:**
1. Set up robot at test distance (e.g., 4.0 ft)
2. Use distance lock and shoot
3. If shots fall short → Increase RPM
4. If shots overshoot → Decrease RPM
5. Open **RobotConstants.java**
6. Find the appropriate range (e.g., RANGE_4 at line 60)
7. Update RPM value:
   ```java
   // Before:
   public static final double RANGE_4_FLYWHEEL_RPM = 3400;

   // After testing - shots fell short:
   public static final double RANGE_4_FLYWHEEL_RPM = 3500;  // Increased by 100
   ```
8. Recompile and re-test

**Range locations in RobotConstants.java:**
- Range 1 (2.47-2.84 ft): Lines 40-43
- Range 2 (2.84-3.2 ft): Lines 46-49
- Range 3 (3.21-4.0 ft): Lines 52-55
- Range 4 (4.0-4.5 ft): Lines 58-61
- Range 5 (4.6-5.0 ft): Lines 64-67
- Range 6 (4.84-5.25 ft): Lines 70-73
- Range 7 (5.2-5.7 ft): Lines 76-79
- Far Zone (5.7+ ft): Lines 82-84

---

### **3. Other Common Adjustments:**

| What to Change | Line Number | Constant Name |
|----------------|-------------|---------------|
| Default flywheel RPM | 20 | `DEFAULT_TARGET_RPM` |
| Default hood position | 32 | `HOOD_DEFAULT_POSITION` |
| Transfer ramp up position | 35 | `TRANSFER_UP_POSITION` |
| Transfer ramp down position | 36 | `TRANSFER_DOWN_POSITION` |
| Turret center position | 95 | `TURRET_CENTER_POSITION` |
| Driving speed when shooting | 78 | `SPEED_SHOOTING_MULTIPLIER` |
| Enable/disable PIDF mode | 15 | `USE_VELOCITY_CONTROL` |

---

## 🚨 EMERGENCY: Disable PIDF Mode

If PIDF velocity control causes problems during competition:

**RobotConstants.java line 15:**
```java
// Change this:
public static final boolean USE_VELOCITY_CONTROL = true;

// To this:
public static final boolean USE_VELOCITY_CONTROL = false;  // FALLBACK TO POWER MODE
```

Recompile - flywheel will use old power percentage mode instead.

---

## ✅ QUICK REFERENCE:

**ONLY UPDATE VALUES IN:**
```
TeamCode/
  └── src/main/java/org/firstinspires/ftc/teamcode/core/constants/
      └── RobotConstants.java  ← UPDATE HERE ONLY!
```

**Files that AUTO-USE these values:**
- ✅ `CompetitionTeleOpBase.java` (competition teleop)
- ✅ `Launcher.java` (flywheel component)
- ✅ `FlywheelPIDFTuner.java` (tuning program)
- ✅ `DistanceCalculator.java` (limelight distance)

**After updating RobotConstants.java:**
1. Save file
2. Recompile (Build → Make Project)
3. Deploy to robot
4. Test changes

---

## 📋 PRE-COMPETITION CHECKLIST:

- [ ] PIDF gains tuned and updated in RobotConstants.java
- [ ] All 7 shooting ranges tested and RPM values confirmed
- [ ] Hood positions verified for each range
- [ ] Default values set correctly
- [ ] Compiled and deployed to robot
- [ ] Full shooting workflow tested

---

**Remember:** RobotConstants.java is your SINGLE SOURCE OF TRUTH!
Update values there, recompile, and everything else updates automatically.
