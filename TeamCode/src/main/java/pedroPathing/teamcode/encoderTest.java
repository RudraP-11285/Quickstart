package pedroPathing.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.acmerobotics.dashboard.FtcDashboard;


@Config
@TeleOp(name="Encoder Test", group="TeleOp")
public class encoderTest extends LinearOpMode {
    private AnalogInput armEncoder1; // First wire
    private AnalogInput deposEncoder1; // First wire
    private AnalogInput wristEncoder1; // First wire
    private DigitalChannel limitSwitch;
    private Servo intakeArm =  null; // Servo that rotates the claw up down
    private Servo intakeWrist =  null; // Servo that rotates the claw up down
    private Servo intakeRotate =  null; // Servo that rotates the claw left right
    private Servo rotate =  null; // Servo that rotates the claw up down
    private Servo deposLeft =  null; // Servo that rotates the claw up down
    private Servo deposRight =  null; // Servo that rotates the claw up downprivate Servo
    private Servo claw =  null; // Servo that rotates the claw up down
    private DcMotor verticalLeft = null;
    private DcMotor verticalRight = null;
    private ColorSensor colorSensor = null;
    private Servo indicatorServo;
    private Limelight3A limelight = null;

    int state = 0;
    boolean stateDebounce = false;

    boolean calculateDebounce = false;

    public static int direction = 1;

    String liftstate = "up";

    @Override
    public void runOpMode() {
        intakeArm = hardwareMap.get(Servo.class, "intakeArm"); // Exp. Hub P3
        deposLeft = hardwareMap.get(Servo.class, "deposLeft"); // Exp. Hub P3
        deposRight = hardwareMap.get(Servo.class, "deposRight"); // Exp. Hub P3

        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist"); // Exp. Hub P3
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate"); // Exp. Hub P2
        intakeArm = hardwareMap.get(Servo.class, "intakeArm"); // Exp. Hub P1

        claw = hardwareMap.get(Servo.class, "intakeClaw"); // Exp. Hub P3
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        indicatorServo = hardwareMap.get(Servo.class, "indicator_servo");

        rotate = hardwareMap.get(Servo.class, "intakeRotate"); // Exp. Hub P3
        armEncoder1 = hardwareMap.get(AnalogInput.class, "armEncoder1");
        deposEncoder1 = hardwareMap.get(AnalogInput.class, "depositEncoder1");
        wristEncoder1 = hardwareMap.get(AnalogInput.class, "wristEncoder1");
        verticalLeft = hardwareMap.get(DcMotor.class, "verticalLeft");
        verticalRight = hardwareMap.get(DcMotor.class, "verticalRight");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "magLimHorizontal1"); // 'magLimVert1' is the name in the config file
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        state = 0;
        stateDebounce = false;

        waitForStart();


        while (opModeIsActive()) {
            //verticalLeft.setPower(gamepad1.left_stick_y * direction);
            //verticalRight.setPower(-(gamepad1.left_stick_y * direction));


            switch (liftstate) {
                case "up":
                    if (verticalRight.getCurrentPosition() < 2500) {
                        verticalLeft.setPower(-1 * direction);
                        verticalRight.setPower(1 * direction);
                    } else {
                        sleep(500);
                        liftstate = "down";
                        verticalLeft.setPower(0);
                        verticalRight.setPower(0);
                    }
                    break;
                case "down":
                    if (verticalRight.getCurrentPosition() > 250) {
                        verticalLeft.setPower(1 * direction);
                        verticalRight.setPower(-1 * direction);
                    } else {
                        sleep(500);
                        liftstate = "up";
                        verticalLeft.setPower(0);
                        verticalRight.setPower(0);
                    }
                    break;
            }

            telemetry.addData("veritcal right pos", verticalRight.getCurrentPosition());
            telemetry.addData("veritcal left pos", verticalLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}
