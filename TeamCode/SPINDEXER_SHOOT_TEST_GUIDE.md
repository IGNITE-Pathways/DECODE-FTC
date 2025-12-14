# Spindexer Shoot Test - Testing Guide

## Overview
This test OpMode verifies that the `shoot_three_balls()` function works correctly with the spindexer's color sensor detection and ball indexing system.

## File Created
- **OpMode**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Main/SpindexerShootTest.java`
- **Group**: Test
- **Name**: "Spindexer Shoot Test"

## What This Test Does

1. **Initialization**: 
   - Initializes all robot components (spindexer, intake, launcher, etc.)
   - Starts the intake mechanism
   - Activates color sensor detection

2. **Ball Detection**:
   - Continuously monitors color sensor for incoming balls
   - Automatically assigns detected balls to divisions (0, 1, 2)
   - Rotates spindexer after each ball is detected

3. **Shooting Sequence**:
   - Waits for all 3 balls to be intaked
   - When gamepad1.x is pressed, executes `shoot_three_balls()`
   - Shoots balls in order: purple, purple, green (as defined in `need_colors` array)

## Testing Steps

### Pre-Test Setup
1. **Prepare Balls**: 
   - Have 3 balls ready (ideally 2 purple, 1 green to match the `need_colors` sequence)
   - Note: The spindexer will shoot in the order defined by `need_colors`, regardless of actual ball colors

2. **Robot Setup**:
   - Place robot on field with intake mechanism accessible
   - Ensure intake can receive balls manually
   - Make sure flywheel and kicker servos are functional
   - Verify color sensor is properly connected and positioned

3. **Driver Station**:
   - Connect gamepad1
   - Open Driver Station
   - Select "Spindexer Shoot Test" from the OpMode list

### Test Execution

1. **Start OpMode**:
   - Press INIT on Driver Station
   - Verify telemetry shows "Initialized" and "Ready to start!"
   - Press START to begin

2. **Intake Balls**:
   - After START, intake will automatically turn on
   - Telemetry will show "Intake started - Feed balls now!"
   - **Manually feed balls into intake one by one**
   - Watch telemetry to see:
     - Ball count increasing (0/3 → 1/3 → 2/3 → 3/3)
     - Division assignments (Div 0, Div 1, Div 2 showing ball colors)
     - "All Balls Ready" status changing from "NO" to "YES"

3. **Shoot Sequence**:
   - Once all 3 balls are intaked, telemetry will show:
     - ">>> READY TO SHOOT! Press gamepad1.x <<<"
   - **Press and hold gamepad1.x button**
   - Observe the shooting sequence:
     - Flywheel starts spinning
     - 1.5 second spin-up delay
     - First ball shoots (purple)
     - Spindexer rotates to next ball
     - Second ball shoots (purple)
     - Spindexer rotates to next ball
     - Third ball shoots (green)
     - Flywheel stops

4. **Verification**:
   - Check that all 3 balls were shot
   - Verify shooting order matches `need_colors` array: purple, purple, green
   - Check telemetry shows "Shooting complete!"

### Expected Behavior

- **Color Sensor**: Should detect each ball as it enters and assign it to a division
- **Spindexer Rotation**: Should automatically rotate after each ball detection
- **Ball Tracking**: Telemetry should show correct ball count and division assignments
- **Shooting Order**: Balls should shoot in the exact order: purple → purple → green
- **Flywheel Control**: Should start before first shot, stop after last shot

### Troubleshooting

**Issue**: Balls not being detected
- Check color sensor connection
- Verify color sensor is positioned correctly in intake path
- Check telemetry for hue values (should show values when ball passes sensor)

**Issue**: Wrong shooting order
- Verify `need_colors` array in Spindexer.java is `{"purple", "purple", "green"}`
- Check that `flag` variable is incrementing correctly after each shot

**Issue**: Flywheel not starting/stopping
- Check launcher/flywheel hardware connections
- Verify `startFlywheel()` and `stopFlywheel()` methods in Robot class

**Issue**: Spindexer not rotating to correct ball
- Check that `findDivisionWithColor()` is finding the correct division
- Verify `indexColors` HashMap is being updated correctly when balls are detected

## Telemetry Information

The OpMode displays:
- **Balls Intaked**: Current count (0-3)
- **All Balls Ready**: YES/NO status
- **Div 0, Div 1, Div 2**: Color of ball in each division ("purple", "green", or "none")
- **Status messages**: Current operation state

## Code Flow

```
1. Initialize Robot
2. Start Intake
3. Loop:
   - Update Spindexer (color sensor detection)
   - Check if all 3 balls intaked
   - If ready AND gamepad1.x pressed:
     - Call robot.shoot_three_balls()
     - Which calls:
       - startFlywheel(1.0)
       - Wait 1.5 seconds
       - spindexer.shoot_ball_new() (3 times)
       - stopFlywheel()
4. Stop intake on exit
```

## Notes

- The test uses `LinearOpMode` so all operations are blocking
- Color sensor detection happens continuously in the update loop
- Shooting only occurs once per OpMode run (tracked by `hasShot` flag)
- Intake runs continuously until OpMode stops
- All divisions are reset to "none" when OpMode initializes

