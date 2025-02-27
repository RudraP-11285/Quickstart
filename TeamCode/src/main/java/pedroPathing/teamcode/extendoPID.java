package pedroPathing.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "PID Test Extendo", group = "! Testing")

public class extendoPID extends OpMode {
    private PIDController extendoController;
    public static double p= 0.015, i=0., d=0.0003;
    public static double f = 0.1;
    public static int target = 1;
    private final double ticks_in_degree = 700 / 180.0;
    private final double ticks_in_rotation = 537.6;
    private final double ticks_per_inch = 0.202 * ticks_in_rotation;
    //rotations required to move 1 inch is 1/(pi * wheel diameter)
    private DcMotor horizontalDrive;

    public void init() {
        extendoController = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        horizontalDrive = hardwareMap.get(DcMotor.class, "horizontalDrive");
    }

    public void loop() {
        extendoController.setPID(p, i, d);
        int position = horizontalDrive.getCurrentPosition();
        double pid = extendoController.calculate(position, target * ticks_per_inch);
        double ff = Math.cos(Math.toRadians((target * ticks_per_inch) / ticks_in_degree)) * f;

        double power = pid + ff;

        horizontalDrive.setPower(power);

        telemetry.addData("pos", position);
        telemetry.addData("target (ticks)", (target * ticks_per_inch));
        telemetry.addData("target (inches)", target);
        telemetry.update();
    }

}
