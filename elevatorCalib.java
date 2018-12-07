package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="elevator Calibration", group="Linear Opmode")
public class elevatorCalib extends LinearOpMode {
    private DcMotor elevatorDrive = null;
    static final double COUNTS_PER_MOTOR_REV_ELAVATOR = 288;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_ELAVATOR = 72/30;     // This is < 1.0 if geared UP
    static final double SPROCKET_DIAMETER = 1;     // For figuring circumference
    static final double COUNTS_PER_INCH_ELEVATOR = (COUNTS_PER_MOTOR_REV_ELAVATOR * DRIVE_GEAR_REDUCTION_ELAVATOR) /
            (SPROCKET_DIAMETER * 3.14159265358979323);
    static final double ELAVATOR_SENSETIVITY = 0.1;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode () {
        elevatorDrive = hardwareMap.get(DcMotor.class, "elevator_drive");
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();
        if (gamepad2.a) {
            elevatorDrive.setPower(1);
        } else if (gamepad2.b) {
            elevatorDrive.setPower(-1);
        } else {
            elevatorDrive.setPower(0);
        }
        while (opModeIsActive()) {
            telemetry.addData("elevatorPosition %7d", elevatorDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}
