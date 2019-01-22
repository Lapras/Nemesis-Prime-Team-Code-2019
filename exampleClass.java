package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Methods;
import org.firstinspires.ftc.teamcode.Variables;
@Disabled
@TeleOp(name="roverRuckusTeleopSingle", group="Linear Opmode")
public class exampleClass extends LinearOpMode {
    public Hardware hardware = new Hardware();
    public Methods methods = new Methods();
    public Variables variables = new Variables();
    public void runOpMode() {
        hardware.hardwareMap();
        waitForStart();
    }
}
