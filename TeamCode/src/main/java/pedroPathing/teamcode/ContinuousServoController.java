package pedroPathing.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class ContinuousServoController {
    private CRServo servo;
    private AnalogInput encoder; // Analog encoder
    //private double tolerance = 1; // Position tolerance in degrees (acceptable error)
    private double maxVoltage; // Max voltage of the analog encoder

    public ContinuousServoController(CRServo servo, AnalogInput encoder) {
        this.servo = servo;
        this.encoder = encoder;
        this.maxVoltage = encoder.getMaxVoltage(); // Store the encoder's max voltage
    }


    /*
    public void runToPosition(double targetPosition, boolean clockwise) {
        // Normalize the target position to within [0, 360)
        targetPosition = normalizePosition(targetPosition);

        // PID coefficients
        double kP = 0.025; // Medium response
        double kI = 0.001; // Small integral to avoid overshooting
        double kD = 0.0025; // Mild damping

        // PID variables
        double integralSum = 0;
        double lastError = 0;
        long lastTime = System.currentTimeMillis();

        //while (true) {
            // Get the current position in degrees
            double currentPosition = getCurrentPositionInDegrees();

            // Calculate the shortest error to the target position
            double error = calculateError(currentPosition, targetPosition);


            // Check if the target position is reached within tolerance
            //if (Math.abs(error) <= tolerance) {
                //servo.setPower(0); // Stop the servo
            //} else {


            // PID calculations
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastTime) / 1000.0; // Convert milliseconds to seconds
            lastTime = currentTime;

            // Proportional term
            double proportional = kP * error;

            // Integral term (accumulated error over time)
            integralSum += error * deltaTime;
            double integral = kI * integralSum;

            // Derivative term (rate of change of error)
            double derivative = kD * (error - lastError) / deltaTime;
            lastError = error;

            // PID output
            double pidOutput = proportional + integral + derivative;

            // Fine adjustment: If close to the target (within 5 degrees), allow direction change
            if (Math.abs(error) <= 5) {
                // Scale the output for finer control near the target
                pidOutput = Math.signum(pidOutput) * Math.min(Math.abs(pidOutput), 0.4); // Limit to 10% power
            } else {
                // Apply the specified direction if far from the target
                pidOutput = clockwise ? Math.max(0.7, pidOutput) : Math.min(-0.7, pidOutput);
            }

            // Clamp the PID output to [-1, 1] for servo power
            pidOutput = Math.max(-1, Math.min(1, pidOutput));

            // Set servo power
            servo.setPower(pidOutput);
        //}
    }
     */


    public void runToPosition(double targetPosition, boolean clockwise, double tolerance) {
        // Normalize the target position to within [0, 360)
        targetPosition = normalizePosition(targetPosition);

        //while (true) {
        // Get the current position in degrees
        double currentPosition = getCurrentPositionInDegrees();

        // Calculate the shortest error to the target position
        double error = calculateError(currentPosition, targetPosition);

        // Check if the target position is reached within tolerance
        if (Math.abs(error) <= tolerance) {
            servo.setPower(0); // Stop the servo
        } else {

            // Fine adjustment: If close to the target (within 5 degrees), allow direction change
            if (Math.abs(error) <= 5) {
                // Reverse direction if overshooting
                if (error > 0) {
                    servo.setPower(-0.09); // Move counterclockwise
                } else {
                    servo.setPower(0.09); // Move clockwise
                }
            } else {
                // Otherwise, move in the specified direction
                if (clockwise) {
                    servo.setPower(0.3); // Full speed clockwise
                } else {
                    servo.setPower(-0.3); // Full speed counterclockwise
                }
            }
        }
        //}
    }


    public double getCurrentPositionInDegrees() {
        // Convert the encoder voltage to degrees
        return (encoder.getVoltage() / maxVoltage) * 360.0;
    }

    private double normalizePosition(double position) {
        // Normalize position to within [0, 360)
        if (position < 0) {
            return position + 360;
        }
        if (position >= 360) {
            return position - 360;
        }
        return position;
    }

    private double calculateError(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;

        // Adjust for shortest path (account for wrap-around)
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        return error;
    }
}
