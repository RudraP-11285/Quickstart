package pedroPathing.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RGB Light Calibration", group = "Calibration")
public class lightTest extends LinearOpMode {

    private Servo indicatorServo;

    @Override
    public void runOpMode() {
        // Initialize hardware
        indicatorServo = hardwareMap.get(Servo.class, "indicator_servo");

        telemetry.addLine("Press Start to begin calibration");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Cycle through positions from 0.0 to 1.0 in steps
            for (double position = 0.0; position <= 1.0; position += 0.05) {
                indicatorServo.setPosition(position);
                telemetry.addData("Servo Position", "%.2f", position);
                telemetry.addLine("Observe the color output for this position.");
                telemetry.update();
                sleep(1000);  // Pause for observation
            }

            telemetry.addLine("Calibration complete.");
            telemetry.update();
            sleep(3000);  // Pause before ending
            break;
        }
    }
}