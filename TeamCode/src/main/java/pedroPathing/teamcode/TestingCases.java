package pedroPathing.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

@TeleOp(name = "Test Case For: All Motors and Servos", group = "Testing")
public class TestingCases extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("🚦 Test Suite Ready");
        telemetry.addLine("Press ▶ to begin testing all connected motors and servos.");
        telemetry.update();
        waitForStart();

        telemetry.clearAll();

        // Track motors used in couplings so we don't test them individually
        Set<DcMotor> motorsAlreadyUsed = new HashSet<>();




        // === Motor Couplings ===
        MotorCoupling drivetrain = new MotorCoupling();
        drivetrain.addMotor(hardwareMap.get(DcMotor.class, "leftFront"), false);
        drivetrain.addMotor(hardwareMap.get(DcMotor.class, "leftRear"), false);
        drivetrain.addMotor(hardwareMap.get(DcMotor.class, "rightFront"), false); // reversed
        drivetrain.addMotor(hardwareMap.get(DcMotor.class, "rightRear"), false);  // reversed

        // Add to used set
        motorsAlreadyUsed.addAll(drivetrain.motors.keySet());

        telemetry.addLine("🔗 Testing Coupled Motors: drivetrain");
        telemetry.update();
        TestPlugin.testMotorGroup(this, drivetrain, 0, 500);



        // === Motor Couplings ===
        MotorCoupling lift = new MotorCoupling();
        lift.addMotor(hardwareMap.get(DcMotor.class, "verticalLeft"), true);
        lift.addMotor(hardwareMap.get(DcMotor.class, "verticalRight"), false);

        // Add to used set
        motorsAlreadyUsed.addAll(lift.motors.keySet());

        telemetry.addLine("🔗 Testing Coupled Motors: lift");
        telemetry.update();
        TestPlugin.testMotorGroup(this, lift, 0, 500);




        // === Test Remaining Motors Individually ===
        telemetry.addLine("🔧 Testing Uncoupled Motors...");
        telemetry.update();

        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            String name = entry.getKey();
            DcMotor motor = entry.getValue();

            // Skip motors that are part of a coupling
            if (motorsAlreadyUsed.contains(motor)) continue;

            telemetry.addLine("Now testing motor: " + name);
            telemetry.update();

            try {
                TestPlugin.testMotor(this, motor, 0, 500);
            } catch (Exception e) {
                telemetry.addLine("❌ Error testing motor " + name + ": " + e.getMessage());
                telemetry.update();
                sleep(5000);
            }
        }

        // === Test All Servos ===
        telemetry.addLine("🔄 Beginning Servo Tests...");
        telemetry.update();

        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            String name = entry.getKey();
            Servo servo = entry.getValue();

            telemetry.addLine("Now testing servo: " + name);
            telemetry.update();

            try {
                TestPlugin.testServo(this, servo);
            } catch (Exception e) {
                telemetry.addLine("❌ Error testing servo " + name + ": " + e.getMessage());
                telemetry.update();
                sleep(5000);
            }
        }

        telemetry.addLine("✅ All testing complete.");
        telemetry.update();
    }
}
