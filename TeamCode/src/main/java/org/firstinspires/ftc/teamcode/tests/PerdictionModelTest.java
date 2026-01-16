package org.firstinspires.ftc.teamcode.tests;

import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtSession;
import ai.onnxruntime.OrtSession.Result;

import java.nio.FloatBuffer;
import java.util.HashMap;
import java.util.Map;

public class PerdictionModelTest {

    private OrtEnvironment env;
    private OrtSession session;

    /**
     * Initialize the ONNX model
     * Call this once before using predict()
     */
    public void initialize(String modelPath) throws Exception {
        env = OrtEnvironment.getEnvironment();
        session = env.createSession(modelPath, new OrtSession.SessionOptions());
    }

    /**
     * Run prediction with the three inputs as parameters
     * @param distance Distance value
     * @param botposeX Bot X position
     * @param botposeY Bot Y position
     * @return Array with [HoodAngle, FlywheelPowerRPM]
     */
    public double[] predict(double distance, double botposeX, double botposeY) throws Exception {
        // Create input tensor
        float[] inputArray = new float[]{
                (float) distance,
                (float) botposeX,
                (float) botposeY
        };

        // Create tensor from input array
        long[] shape = new long[]{1, 3};
        OnnxTensor inputTensor = OnnxTensor.createTensor(env, FloatBuffer.wrap(inputArray), shape);

        // Prepare inputs map
        Map<String, OnnxTensor> inputs = new HashMap<>();
        inputs.put("Distance", inputTensor);
        inputs.put("BotposeX", inputTensor);
        inputs.put("BotposeY", inputTensor);

        // Run inference
        Result result = session.run(inputs);

        // Get outputs: 'HoodAngle', 'FlywheelPowerRPM'
        float[] hoodAngleArray = (float[]) result.get(0).getValue();
        float[] flywheelPowerArray = (float[]) result.get(1).getValue();

        // Extract values
        double hoodAngle = hoodAngleArray[0];
        double flywheelPowerRPM = flywheelPowerArray[0];

        // Cleanup
        inputTensor.close();
        result.close();

        return new double[]{hoodAngle, flywheelPowerRPM};
    }

    /**
     * Clean up resources
     */
    public void close() throws Exception {
        if (session != null) {
            session.close();
        }
    }
}



/*
In order to use

// Initialize once
PerdictionModelTest model = new PerdictionModelTest();
model.initialize("perfect_svm_model.onnx");

// Use the predict function
double[] results = model.predict(10.0, 5.0, 3.0);
double hoodAngle = results[0];
double flywheelPowerRPM = results[1];

// Clean up when done
model.close();
 */