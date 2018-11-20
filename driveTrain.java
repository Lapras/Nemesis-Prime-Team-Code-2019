package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class driveTrain {
   public void teleopInput(double drive, double turn, double speedLimiter, DcMotor leftDrive, DcMotor rightDrive) {
       double leftPower = 0;
       double rightPower = 0;
       leftPower = Range.clip(drive + turn, -speedLimiter, speedLimiter);
       rightPower = Range.clip(drive - turn, -speedLimiter, speedLimiter);
       leftDrive.setPower(leftPower);
       rightDrive.setPower(rightPower);
   }
}
