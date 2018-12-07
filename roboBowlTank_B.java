package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="roboBowlTeleOp-B", group="Linear Opmode")
public class roboBowlTank_B extends LinearOpMode {
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    double leftPower;
    double rightPower;
    @Override
    public void runOpMode() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        while(opModeIsActive()) {
            leftPower = Range.clip(gamepad1.left_stick_y, -1, 1);
            rightPower = Range.clip(gamepad1.right_stick_y, -1, 1);
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

    }
}
