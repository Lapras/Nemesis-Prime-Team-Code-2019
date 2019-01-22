package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
@Disabled
@Autonomous(name = "Variables", group = "Concept")
public class Variables {
    String startQuadrant;
    String trackableName;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // we have Core Hex motors, creating a different count value
    static final double COUNTS_PER_MOTOR_REV_ELAVATOR = 288;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_ELAVATOR = 72 / 30;     // This is < 1.0 if geared UP
    static final double DRIVE_GEAR_REDUCTION = 0.75;
    static final double ENCODER_MAX = 1563;
    static final double ELEVATION = 10.5;
    static final double COUNTS_PER_INCH_ELEVATOR = (ENCODER_MAX / ELEVATION);
    static final double WHEEL_DIAMETER_MM = 90;
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    static final double P_TURN_COEFF = 0.01;
    static final double HEADING_THRESHOLD = 0.5;
    static final double DRIVE_SPEED = 0.5; // higher power = faster traversal
    static final double BIG_TURN = 0.3;
    static final double TURN_SPEED = 0.3;
    static final double SMALL_TURN = 0.75; // higher power = faster traversal
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    static final String VUFORIA_KEY = "AcWJXfv/////AAABmXTvMEsBOEYkiGqMsGZyklUBGfk5cSsLyBZx0YTUz4txj9n9lF3yHQWwQIFc+gC2pdWkKk3iWgbSza68dp0T2zqc+1sG6S5G6VAXxnNBUsH6rjRa+6p1kPIsiEQdezRl4m5VeATR5SzGECvIbIhtc3nWjjquoM/d8+R7QCMOrAPRwf9bhK6Ah2tgIuPkwVwkp+G1q4/qaFrLMGcDxRDlwNTFMZfnmTLt18Xe6Q54eDPb6Bw2MHmfUXSXbeqiahjFGQz40bh0TH3vAp47L88U3oXSo+YgV2TyT1PhQE7SUE71ucVNQ9PUM0OfIKFTtHy2MaXsE2dOsj+WAJG5J3iGO530Gaod7DtKZSCPC02SyADM";
    //private DigitalChannel digitalTouch = null;
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;
    ElapsedTime runtime = new ElapsedTime();
    boolean servoPos = false;
    public static final float mmPerInch = 25.4f;
    private boolean targetVisible = false;
}
