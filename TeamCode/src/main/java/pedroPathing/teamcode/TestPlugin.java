package pedroPathing.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Map;

public class TestPlugin {

    public static void testMotor(LinearOpMode opMode, DcMotor motor, int minEncoder, int maxEncoder) throws InterruptedException {
        final double power = 0.5;
        final int tolerance = 20;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int startPos = motor.getCurrentPosition();

        // Move forward
        motor.setTargetPosition(maxEncoder);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        Thread.sleep(2000);
        int posAfterForward = motor.getCurrentPosition();

        // Move back
        motor.setTargetPosition(minEncoder);
        motor.setPower(power);
        Thread.sleep(2000);
        int posAfterBackward = motor.getCurrentPosition();

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean encoderChanged = Math.abs(posAfterForward - startPos) > tolerance &&
                Math.abs(posAfterBackward - posAfterForward) > tolerance;

        opMode.telemetry.addLine("Did the motor move?");
        opMode.telemetry.addLine("• Press A if it moved correctly.");
        opMode.telemetry.addLine("• Press B if it did NOT move.");
        opMode.telemetry.update();

        while (opMode.opModeIsActive() && !opMode.gamepad1.a && !opMode.gamepad1.b) opMode.idle();

        if (opMode.gamepad1.a && encoderChanged) {
            opMode.telemetry.addLine("✅ Motor + Encoder working.");
        } else if (opMode.gamepad1.a && !encoderChanged) {
            opMode.telemetry.addLine("⚠️ Encoder not changing.");
            opMode.telemetry.addLine("• Suggestion: Check encoder wiring or unplugged port.");
        } else if (opMode.gamepad1.b && encoderChanged) {
            opMode.telemetry.addLine("⚠️ Motor moved (based on encoder), but not visually.");
            opMode.telemetry.addLine("• Suggestion: Mechanical disconnection?");
        } else {
            opMode.telemetry.addLine("❌ Motor not moving + no encoder change.");
            opMode.telemetry.addLine("• Suggestion: Check motor wiring, dead motor, or bad config.");
        }

        opMode.telemetry.update();
        Thread.sleep(2000);
    }

    public static void testServo(LinearOpMode opMode, Servo servo) throws InterruptedException {
        opMode.telemetry.addLine("Sweeping servo: " + servo.toString());
        opMode.telemetry.update();

        try {
            servo.setPosition(0);
            Thread.sleep(750);
            servo.setPosition(1);
            Thread.sleep(750);
            servo.setPosition(0.5);
        } catch (Exception e) {
            opMode.telemetry.addLine("❌ Servo error: " + e.getMessage());
            opMode.telemetry.addLine("• Suggestion: Check wiring or port config.");
            opMode.telemetry.update();
            Thread.sleep(5000);
            return;
        }

        opMode.telemetry.addLine("Did the servo: " + servo +  " move?");
        opMode.telemetry.addLine("• Press A if yes.");
        opMode.telemetry.addLine("• Press B if no.");
        opMode.telemetry.update();

        while (opMode.opModeIsActive() && !opMode.gamepad1.a && !opMode.gamepad1.b) opMode.idle();

        if (opMode.gamepad1.a) {
            opMode.telemetry.addLine("✅ Servo working.");
        } else {
            opMode.telemetry.addLine("❌ Servo not moving.");
            opMode.telemetry.addLine("• Suggestion: Check wiring, broken horn, config issue.");
        }

        opMode.telemetry.update();
        Thread.sleep(2000);
    }

    public static void testMotorGroup(LinearOpMode opMode, MotorCoupling group, int minEncoder, int maxEncoder) throws InterruptedException {
        final double power = 0.5;
        final int tolerance = 20;

        for (DcMotor motor : group.motors.keySet()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Forward
        for (Map.Entry<DcMotor, Boolean> entry : group.motors.entrySet()) {
            DcMotor motor = entry.getKey();
            boolean reversed = entry.getValue();
            motor.setTargetPosition(reversed ? -maxEncoder : maxEncoder);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
        Thread.sleep(2000);

        // Back
        for (Map.Entry<DcMotor, Boolean> entry : group.motors.entrySet()) {
            DcMotor motor = entry.getKey();
            boolean reversed = entry.getValue();
            motor.setTargetPosition(reversed ? -minEncoder : minEncoder);
            motor.setPower(power);
        }
        Thread.sleep(2000);

        for (DcMotor motor : group.motors.keySet()) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        boolean anyMoved = group.motors.keySet().stream()
                .anyMatch(m -> Math.abs(m.getCurrentPosition()) > tolerance);

        opMode.telemetry.addLine("Did the coupled motors move?");
        opMode.telemetry.addLine("• Press A if yes.");
        opMode.telemetry.addLine("• Press B if no.");
        opMode.telemetry.update();

        while (opMode.opModeIsActive() && !opMode.gamepad1.a && !opMode.gamepad1.b) opMode.idle();

        if (opMode.gamepad1.a && anyMoved) {
            opMode.telemetry.addLine("✅ Coupled motors + encoders OK.");
        } else if (opMode.gamepad1.a && !anyMoved) {
            opMode.telemetry.addLine("⚠️ No encoder change, but user saw movement.");
            opMode.telemetry.addLine("• Suggestion: Encoder wiring issue?");
        } else if (opMode.gamepad1.b && anyMoved) {
            opMode.telemetry.addLine("⚠️ Encoders changed, but no movement seen.");
            opMode.telemetry.addLine("• Suggestion: Mechanically disconnected motors?");
        } else {
            opMode.telemetry.addLine("❌ Coupled motors not responding.");
            opMode.telemetry.addLine("• Suggestion: Wiring issue, config error, or controller problem.");
        }

        opMode.telemetry.update();
        Thread.sleep(2000);
    }
}
