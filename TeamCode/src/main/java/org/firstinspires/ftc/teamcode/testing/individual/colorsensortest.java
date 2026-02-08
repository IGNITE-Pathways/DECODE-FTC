package org.firstinspires.ftc.teamcode.tests.individual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import android.graphics.Color;

@TeleOp(name="Purple/Green Detector V3 (Dark Filter Only)", group="Tests")
public class colorsensortest extends LinearOpMode {

    NormalizedColorSensor sensor;

    // Purple / Green HSV centers (tuned to your samples)
    float[] PURPLE = {240, 0.90f, 0.01f};
    float[] GREEN  = {160, 1.00f, 0.01f};

    // Distance weights for classification
    float WH = 2.0f;
    float WS = 1.0f;
    float WV = 1.0f;

    // Minimum brightness / clear alpha
    float MIN_LIGHT = 0.02f;

    // Number of samples to average
    int SAMPLE_COUNT = 6;

    @Override
    public void runOpMode() {

        sensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");

        telemetry.addLine("Purple/Green Detector V3 (Dark Filter Only)");
        telemetry.addLine("Averaging + alpha-based filtering.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            String result = detectColorWithConfidence();

            telemetry.addLine("=== DETECTED ===");
            telemetry.addData("Color", result);

            // Show raw sensor info for debugging
            NormalizedRGBA rgba = sensor.getNormalizedColors();
            telemetry.addLine("=== Raw Sensor Info ===");
            telemetry.addData("Alpha/Clear", rgba.alpha);
            telemetry.addData("Red", rgba.red);
            telemetry.addData("Green", rgba.green);
            telemetry.addData("Blue", rgba.blue);

            telemetry.update();
        }
    }

    private String detectColorWithConfidence() {

        float sumH = 0, sumS = 0, sumV = 0;
        int valid = 0;
        float sumClear = 0;

        for (int i = 0; i < SAMPLE_COUNT; i++) {
            NormalizedRGBA c = sensor.getNormalizedColors();
            float clear = c.alpha;  // brightness / reflectance

            // Skip too dark readings
            if (clear < MIN_LIGHT) {
                sleep(3);
                continue;
            }

            int argb = c.toColor();
            float[] hsv = new float[3];
            Color.colorToHSV(argb, hsv);

            sumH += hsv[0];
            sumS += hsv[1];
            sumV += hsv[2];
            sumClear += clear;
            valid++;

            sleep(3);
        }

        if (valid == 0) return "NEITHER";  // all samples too dark

        float avgH = sumH / valid;
        float avgS = sumS / valid;
        float avgV = sumV / valid;

        // Optional: small brightness boost to prevent near-zero value
        avgV = Math.max(avgV, 0.02f);
        avgS = Math.min(1.0f, avgS * (1.0f + 0.5f * (1 - avgV)));

        float[] hsv = {avgH, avgS, avgV};

        return classify(hsv);
    }

    private float distance(float[] a, float[] b) {
        float dh = Math.abs(a[0] - b[0]);
        if (dh > 180) dh = 360 - dh;

        float ds = Math.abs(a[1] - b[1]);
        float dv = Math.abs(a[2] - b[2]);
        return dh * WH + ds * WS + dv * WV;
    }

    private String classify(float[] hsv) {
        float dP = distance(hsv, PURPLE);
        float dG = distance(hsv, GREEN);

        float min = Math.min(dP, dG);

        if (min > 80) return "NEITHER";

        return (dP < dG) ? "PURPLE" : "GREEN";
    }
}
