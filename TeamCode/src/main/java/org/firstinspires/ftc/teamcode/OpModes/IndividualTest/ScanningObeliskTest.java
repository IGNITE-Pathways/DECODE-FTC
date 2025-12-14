/*
 * FTC Into the Deep - Obelisk AprilTag Detection TeleOp
 * Uses Limelight 3A for AprilTag detection on the obelisk
 * Displays detected ball sequence based on AprilTag ID
 */
package org.firstinspires.ftc.teamcode.OpModes.IndividualTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * ScanningObeliskTest - Teleop mode for detecting obelisk AprilTag and reading ball sequence
 *
 * This OpMode uses Limelight 3A vision system to:
 * - Detect AprilTags on the obelisk
 * - Map AprilTag ID to corresponding ball sequence
 * - Display the detected ball sequence in Driver Station telemetry
 *
 * Three possible ball sequences:
 * - Purple - Purple - Green
 * - Purple - Green - Purple
 * - Green - Purple - Purple
 *
 * Press A button to start Limelight and begin scanning.
 * Configure the AprilTag IDs below to match your obelisk setup.
 */
@TeleOp(name = "Obelisk Ball Detection", group = "Into The Deep")
public class ScanningObeliskTest extends LinearOpMode {

    // Limelight device reference
    private Limelight3A limelight;

    // AprilTag ID to ball sequence mapping
    // TODO: Configure these IDs to match your obelisk AprilTag IDs
    private static final int APRILTAG_ID_PPG = 23;  // Purple - Purple - Green
    private static final int APRILTAG_ID_PGP = 22;  // Purple - Green - Purple
    private static final int APRILTAG_ID_GPP = 21;  // Green - Purple - Purple

    // Currently detected ball sequence
    private List<String> detectedBallSequence;

    // Track last detected AprilTag ID to prevent duplicate detections
    private int lastDetectedTagId = -1;

    // Track if Limelight has been started
    private boolean limelightStarted = false;

    // Button state tracking for edge detection
    private boolean prevA = false;
    private boolean prevY = false;

    @Override
    public void runOpMode() {
        // Initialize the Limelight camera system
        initLimelight();

        // Initialize the ball sequence list
        detectedBallSequence = new ArrayList<>();

        // Display initialization complete message
        telemetry.addData("Status", "Initialized - Waiting for START");
        telemetry.addData("Limelight Connected", limelight != null && limelight.isConnected() ? "Ready" : "Not Connected");
        telemetry.addLine("\nConfigured AprilTag IDs:");
        telemetry.addData("Tag " + APRILTAG_ID_PPG, "Purple - Purple - Green");
        telemetry.addData("Tag " + APRILTAG_ID_PGP, "Purple - Green - Purple");
        telemetry.addData("Tag " + APRILTAG_ID_GPP, "Green - Purple - Purple");
        telemetry.addLine("\n[A] = START SCANNING | [Y] = CLEAR SEQUENCE");
        telemetry.update();

        // Wait for driver to press START button
        waitForStart();

        // Main TeleOp loop - runs until STOP is pressed
        while (opModeIsActive()) {
            // Handle gamepad controls (must be checked first)
            handleGamepadInput();

            // Only scan if Limelight has been started
            if (limelightStarted) {
                // Detect AprilTag and update ball sequence
                detectObeliskAprilTag();
            }

            // Display telemetry data to Driver Station
            updateTelemetry();

            // Update telemetry display
            telemetry.update();

            // Small delay to prevent excessive CPU usage
            sleep(20);
        }

        // Clean up resources when OpMode stops
        if (limelight != null && limelightStarted) {
            limelight.stop();
        }
    }

    /**
     * Initializes the Limelight 3A vision system
     * Sets up camera connection and configuration
     */
    private void initLimelight() {
        try {
            // Get Limelight from hardware map (must be configured as "limelight" in config)
            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            if (limelight != null) {
                // Set telemetry update rate (100 Hz for responsive updates)
                limelight.setPollRateHz(100);
                // Note: Don't start limelight here - will start when A button is pressed
            }
        } catch (Exception e) {
            telemetry.addData("Error", "Limelight not found in hardware map: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Handles gamepad input for manual control
     * A button: Start Limelight and begin scanning
     * Y button: Clear the detected sequence
     */
    private void handleGamepadInput() {
        // A button pressed - start Limelight and begin scanning
        boolean currentA = gamepad1.a;
        if (currentA && !prevA) {
            if (limelight != null && !limelightStarted) {
                limelight.start();
                limelightStarted = true;
                telemetry.addData("Status", "Limelight started - Scanning for AprilTag...");
            }
        }
        prevA = currentA;

        // Y button pressed - clear the detected sequence
        boolean currentY = gamepad1.y;
        if (currentY && !prevY) {
            detectedBallSequence.clear();
            lastDetectedTagId = -1;
            telemetry.addData("Status", "Sequence cleared by driver");
        }
        prevY = currentY;
    }

    /**
     * Detects AprilTag on the obelisk and maps it to the corresponding ball sequence
     * Only updates the sequence when a new/different AprilTag is detected
     */
    private void detectObeliskAprilTag() {
        if (limelight == null || !limelight.isConnected()) {
            return;
        }

        try {
            // Get latest detection results from Limelight
            LLResult result = limelight.getLatestResult();

            // Verify result is valid and not null
            if (result != null && result.isValid()) {
                // Get all detected fiducials (AprilTags)
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                if (fiducialResults != null && !fiducialResults.isEmpty()) {
                    // Check each detected fiducial
                    for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
                        int tagId = fiducial.getFiducialId();

                        // Only process if this is a new detection (different from last detected)
                        if (tagId != lastDetectedTagId) {
                            lastDetectedTagId = tagId;
                            
                            // Map AprilTag ID to ball sequence
                            List<String> sequence = mapTagIdToSequence(tagId);
                            
                            if (sequence != null && !sequence.isEmpty()) {
                                detectedBallSequence = new ArrayList<>(sequence);
                            }
                            
                            // Break after processing first valid tag
                            break;
                        }
                    }
                } else {
                    // No fiducials detected - reset last detected tag
                    lastDetectedTagId = -1;
                }
            } else {
                // No valid result - reset last detected tag
                lastDetectedTagId = -1;
            }
        } catch (Exception e) {
            // Handle any errors gracefully
            telemetry.addData("Detection Error", e.getMessage());
        }
    }

    /**
     * Maps AprilTag ID to corresponding ball sequence
     * @param tagId The AprilTag ID detected
     * @return List of ball colors in sequence, or null if tag ID is not recognized
     */
    private List<String> mapTagIdToSequence(int tagId) {
        switch (tagId) {
            case APRILTAG_ID_PPG:
                // Purple - Purple - Green
                return Arrays.asList("purple", "purple", "green");
            
            case APRILTAG_ID_PGP:
                // Purple - Green - Purple
                return Arrays.asList("purple", "green", "purple");
            
            case APRILTAG_ID_GPP:
                // Green - Purple - Purple
                return Arrays.asList("green", "purple", "purple");
            
            default:
                // Unknown tag ID
                return null;
        }
    }

    /**
     * Updates telemetry display with current detection information
     * Shows the detected ball sequence and AprilTag information
     */
    private void updateTelemetry() {
        // Add header
        telemetry.addLine("\n========== OBELISK DETECTION ==========");

        // Display scanning status
        if (!limelightStarted) {
            telemetry.addData("Status", "Press [A] to start scanning");
            telemetry.addLine("\n[A] = START SCANNING | [Y] = CLEAR SEQUENCE");
            return;
        }

        // Display current AprilTag detection status
        if (limelight == null || !limelight.isConnected()) {
            telemetry.addData("Status", "Limelight not connected");
            telemetry.addLine("\n[Y] = CLEAR SEQUENCE");
            return;
        }

        LLResult result = limelight.getLatestResult();
        boolean tagDetected = false;
        int currentTagId = -1;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (fiducialResults != null && !fiducialResults.isEmpty()) {
                tagDetected = true;
                currentTagId = fiducialResults.get(0).getFiducialId();
            }
        }

        // Display AprilTag detection status
        if (tagDetected) {
            telemetry.addData("AprilTag Detected", "YES");
            telemetry.addData("AprilTag ID", currentTagId);
            
            // Show which sequence this tag maps to
            String sequenceName = getSequenceNameForTag(currentTagId);
            telemetry.addData("Sequence Type", sequenceName != null ? sequenceName : "Unknown Tag ID");
        } else {
            telemetry.addData("AprilTag Detected", "NO");
            telemetry.addData("AprilTag ID", "None");
            telemetry.addData("Status", "Scanning...");
        }

        // Display the detected ball sequence
        telemetry.addLine("\n--- Ball Sequence ---");
        if (detectedBallSequence == null || detectedBallSequence.isEmpty()) {
            telemetry.addData("Ball Sequence", "No sequence detected yet");
        } else {
            // Build a readable sequence string
            StringBuilder sequenceBuilder = new StringBuilder();
            for (int i = 0; i < detectedBallSequence.size(); i++) {
                String color = detectedBallSequence.get(i);
                sequenceBuilder.append(color.substring(0, 1).toUpperCase());
                if (i < detectedBallSequence.size() - 1) {
                    sequenceBuilder.append(" → ");
                }
            }
            telemetry.addData("Ball Sequence", sequenceBuilder.toString());

            // Display full sequence as comma-separated list
            telemetry.addData("Full Sequence", detectedBallSequence.toString());
        }

        telemetry.addLine("\n[Y] = CLEAR SEQUENCE");
    }

    /**
     * Gets a human-readable name for the sequence associated with a tag ID
     * @param tagId The AprilTag ID
     * @return String description of the sequence, or null if unknown
     */
    private String getSequenceNameForTag(int tagId) {
        switch (tagId) {
            case APRILTAG_ID_PPG:
                return "Purple - Purple - Green";
            case APRILTAG_ID_PGP:
                return "Purple - Green - Purple";
            case APRILTAG_ID_GPP:
                return "Green - Purple - Purple";
            default:
                return null;
        }
    }
}
