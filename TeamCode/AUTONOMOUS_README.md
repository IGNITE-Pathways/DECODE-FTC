# Autonomous OpMode Implementation Guide

## Overview

This document describes the autonomous OpModes implemented using Pedro Pathing and what needs to be completed to make them fully functional.

## What Has Been Implemented

### Files Created

1. **AutonomousMain.java** - Main autonomous routine class
   - Handles alliance (RED/BLUE) and starting position (LEFT/RIGHT) configuration
   - Integrates Pedro Pathing for navigation
   - Integrates Robot components for shooting and intake
   - Implements the complete autonomous sequence

2. **AutonomousRedLeft.java** - Wrapper OpMode for Red Alliance, Left starting position
3. **AutonomousRedRight.java** - Wrapper OpMode for Red Alliance, Right starting position
4. **AutonomousBlueLeft.java** - Wrapper OpMode for Blue Alliance, Left starting position
5. **AutonomousBlueRight.java** - Wrapper OpMode for Blue Alliance, Right starting position

### Autonomous Sequence

The autonomous routine follows this sequence:
1. Shoot 3 pre-loaded balls
2. Drive to ball pickup location #1
3. Intake 3 balls
4. Drive to shooting position
5. Shoot 3 balls
6. Drive to ball pickup location #2
7. Intake 3 balls
8. Drive to shooting position
9. Shoot 3 balls

## What Needs to Be Done

### 1. Field Coordinates (CRITICAL - Must be done first)

**Location:** `AutonomousMain.java` → `calculateFieldPositions()` method

The current field coordinates are **placeholder values** and must be replaced with actual field measurements.

#### Starting Positions
- **Red Alliance, Left:** Currently `(72, 72, 0°)`
- **Red Alliance, Right:** Currently `(72, -72, 0°)`
- **Blue Alliance, Left:** Currently `(-72, 72, 180°)`
- **Blue Alliance, Right:** Currently `(-72, -72, 180°)`

**Action Required:**
- Measure actual starting positions on the field
- Update X, Y coordinates (in inches) and heading (in radians)
- Verify heading direction (which way the robot faces at start)

#### Shooting Position
- Currently set to `(0, 0)` for both alliances
- Red Alliance faces `0°` (forward)
- Blue Alliance faces `180°` (backward)

**Action Required:**
- Determine optimal shooting position on the field
- Measure X, Y coordinates where robot should shoot from
- Verify heading angle for proper target alignment

#### Ball Pickup Locations
- **Pickup Location #1:** Currently `(36, 36)` for Red Left, `(36, -36)` for Red Right
- **Pickup Location #2:** Currently `(36, -36)` for Red Left, `(36, 36)` for Red Right
- Similar pattern for Blue Alliance with negative X values

**Action Required:**
- Identify where balls are located on the field
- Measure exact X, Y coordinates for both pickup locations
- Adjust heading angles if robot needs to face a specific direction while intaking

### 2. Timing Adjustments

**Location:** `AutonomousMain.java` → `shootThreeBalls()` and `intakeThreeBalls()` methods

#### Shooting Timing
- **Flywheel spin-up time:** Currently `1500ms` - may need adjustment based on flywheel characteristics
- **Turret alignment timeout:** Currently `30 attempts` (3 seconds) - may need more time
- **Delay between shots:** Currently `800ms` - adjust based on spindexer kicker timing

**Action Required:**
- Test flywheel spin-up time and adjust if needed
- Verify turret alignment timing (may need to increase timeout)
- Test shooting sequence timing to ensure proper delays between shots

#### Intake Timing
- **Maximum wait time:** Currently `15000ms` (15 seconds) - may need adjustment
- **Ball detection check interval:** Currently `100ms` - may be too fast or too slow

**Action Required:**
- Test how long it takes to intake 3 balls
- Adjust maximum wait time if needed
- Verify ball detection is working correctly (check spindexer color sensor)

### 3. Spindexer Ball Detection

**Location:** `AutonomousMain.java` → `intakeThreeBalls()` method

The code uses `robot.getSpindexer().getBallCount()` to detect when 3 balls are intaked.

**Action Required:**
- Verify that `getBallCount()` method correctly counts balls in spindexer
- Test that color sensor detection is working during autonomous
- Ensure spindexer's `update()` method is being called frequently enough to detect balls
- May need to add delays or adjust spindexer rotation timing

### 4. Turret Alignment

**Location:** `AutonomousMain.java` → `shootThreeBalls()` method

The code waits for turret alignment before shooting.

**Action Required:**
- Verify Limelight is detecting AprilTag 24 correctly
- Test turret alignment timing - may need to increase timeout
- Ensure turret alignment is reliable before shooting
- Consider adding a check to verify alignment before proceeding

### 5. Path Following

**Location:** `AutonomousMain.java` → `pathToLocation()` method

The code uses Pedro Pathing to follow paths between locations.

**Action Required:**
- Test path following accuracy - robot should reach target positions precisely
- Verify heading interpolation is correct (robot faces correct direction at end of path)
- May need to adjust path constraints in `Constants.java` if paths are too fast/slow
- Test that `follower.isBusy()` correctly detects when path is complete

### 6. Robot Component Integration

**Location:** Throughout `AutonomousMain.java`

**Action Required:**
- Verify all Robot component methods work correctly:
  - `robot.startFlywheel()` / `robot.stopFlywheel()`
  - `robot.updateTurret()` - returns true if Limelight connected
  - `robot.shootSequence()` - handles one shot, returns true if should stop
  - `robot.startIntake()` / `robot.stopIntake()`
  - `robot.getSpindexer().getBallCount()` - counts balls
- Test that components don't interfere with each other during autonomous
- Ensure drive train stops properly when not following paths

## Testing Checklist

### Pre-Testing Setup
- [ ] Field coordinates measured and updated
- [ ] Robot hardware verified (motors, servos, sensors all working)
- [ ] Limelight connected and detecting AprilTag 24
- [ ] Spindexer color sensor working
- [ ] Pedro Pathing constants tuned (if needed)

### Step-by-Step Testing

1. **Test Starting Position**
   - [ ] Place robot at starting position
   - [ ] Run autonomous and verify robot starts at correct pose
   - [ ] Adjust starting pose coordinates if needed

2. **Test Pre-Loaded Shooting**
   - [ ] Load 3 balls into spindexer
   - [ ] Run autonomous and verify shooting sequence works
   - [ ] Adjust flywheel spin-up time if needed
   - [ ] Adjust turret alignment timeout if needed
   - [ ] Adjust delays between shots if needed

3. **Test Path to Pickup Location #1**
   - [ ] Run autonomous and verify robot reaches pickup location
   - [ ] Check that robot position matches target coordinates
   - [ ] Verify robot heading is correct at end of path
   - [ ] Adjust pickup location coordinates if needed

4. **Test Intake Sequence**
   - [ ] Place 3 balls at pickup location
   - [ ] Run autonomous and verify intake works
   - [ ] Check that ball detection is working
   - [ ] Adjust intake timing if needed
   - [ ] Verify all 3 balls are detected before moving to next step

5. **Test Path to Shooting Position**
   - [ ] Run autonomous and verify robot reaches shooting position
   - [ ] Check that robot position matches target coordinates
   - [ ] Verify robot heading is correct for shooting

6. **Test Shooting After Intake**
   - [ ] Run autonomous and verify shooting works after intaking
   - [ ] Check that all 3 balls are shot correctly
   - [ ] Adjust timing if needed

7. **Test Complete Sequence**
   - [ ] Run full autonomous sequence
   - [ ] Verify all steps complete successfully
   - [ ] Check timing - ensure sequence completes within autonomous period
   - [ ] Make final adjustments to coordinates and timing

## Configuration Files

### Constants.java
- Path constraints may need adjustment if paths are too fast/slow
- PIDF coefficients should already be tuned, but may need fine-tuning
- Drivetrain constants should already be configured

### Robot Components
- All component initialization should be working
- Verify hardware map names match actual hardware configuration

## Common Issues and Solutions

### Issue: Robot doesn't reach target positions accurately
**Solution:** 
- Check field coordinates are correct
- Verify Pedro Pathing constants are tuned
- May need to adjust path constraints

### Issue: Shooting doesn't work
**Solution:**
- Verify Limelight is connected and detecting AprilTag 24
- Check turret alignment timeout is sufficient
- Verify flywheel spin-up time is adequate
- Check spindexer shooting sequence timing

### Issue: Intake doesn't detect balls
**Solution:**
- Verify color sensor is working
- Check that spindexer's `update()` is being called
- Verify `getBallCount()` method is working correctly
- May need to adjust intake timing or add delays

### Issue: Path following is jerky or inaccurate
**Solution:**
- Check Pedro Pathing constants in `Constants.java`
- Verify odometry is working correctly
- May need to retune PIDF coefficients

## Notes

- All field coordinates are in **inches**
- All angles are in **radians** (use `Math.toRadians()` to convert from degrees)
- The autonomous sequence assumes 3 balls at each pickup location
- Make sure to test on actual field with correct field setup
- Autonomous period is typically 30 seconds - ensure sequence completes in time

## Next Steps

1. **Measure field coordinates** - This is the most critical step
2. **Test each step individually** - Don't try to test the full sequence first
3. **Tune timing** - Adjust delays and timeouts based on testing
4. **Verify ball detection** - Ensure spindexer correctly detects balls
5. **Final integration testing** - Test complete sequence once individual steps work

## Support

If you encounter issues:
1. Check telemetry output for error messages
2. Verify all hardware is connected and working
3. Test individual components separately
4. Review Pedro Pathing documentation if path following issues occur
5. Check Robot component methods are working correctly

Good luck with your autonomous implementation!
