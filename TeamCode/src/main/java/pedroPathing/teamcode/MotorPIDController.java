package pedroPathing.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;

public class MotorPIDController {
    private PIDController controller;
    private DcMotor motor;
    private double f;
    private double ticksPerInch;
    private double ticksInDegree;

    public MotorPIDController(DcMotor motor, double p, double i, double d, double f, double ticksInDegree, double ticksInRotation, double spoolCircumference) {
        this.motor = motor;
        this.controller = new PIDController(p, i, d);
        this.f = f;
        this.ticksPerInch = (ticksInRotation / spoolCircumference);
        this.ticksInDegree = ticksInDegree;
    }

    public void setPIDF(double p, double i, double d, double f) {
        controller.setPID(p, i, d);
        this.f = f;
    }

    public void setTargetPosition(double target, double speed, String units) {
        switch (units) {
            case "Inches":
                int targetTicks = (int) (target * ticksPerInch);
                double pid = controller.calculate(motor.getCurrentPosition(), targetTicks);
                double ff = Math.cos(Math.toRadians(targetTicks / ticksInDegree)) * f;

                double value = (pid + ff);
                double max = 1.0;
                double min = 0.0;

                double cappedSpeed = Math.min(Math.max(value, min), max);

                motor.setPower(cappedSpeed * speed);
                break;
            case "Ticks":
                int targetTicks1 = (int) (target);
                double pid1 = controller.calculate(motor.getCurrentPosition(), targetTicks1);
                double ff1 = Math.cos(Math.toRadians(targetTicks1 / ticksInDegree)) * f;

                double value1 = (pid1 + ff1);
                double max1 = 1.0;
                double min1 = 0.0;

                double cappedSpeed1 = Math.min(Math.max(value1, min1), max1);

                motor.setPower(cappedSpeed1 * speed);
                break;
        }
    }

    public double getCurrentPosition(String units) {
        switch (units) {
            case "Inches":
                return motor.getCurrentPosition() / ticksPerInch;
            case "Ticks":
                return motor.getCurrentPosition();
            default:
                return motor.getCurrentPosition();
        }
    }
}
