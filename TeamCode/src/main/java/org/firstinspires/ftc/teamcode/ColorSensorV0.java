package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@TeleOp
public class ColorSensorV0 extends OpMode {
    boolean stop = false;
    public ColorSensor colorSensor;
    private final ChestiiDeAutonomTeleOP c = new ChestiiDeAutonomTeleOP();
    SampleMecanumDrive drive;
    @Override
    public void init() {
        c.init(hardwareMap,telemetry,false);
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14.783464, -62.73622, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
    }
    @Override
    public void loop() {
        telemetry.addData("red:",colorSensor.red());
        telemetry.addData("green:",colorSensor.green());
        telemetry.addData("blue:",colorSensor.blue());
        telemetry.addData("alpha:",colorSensor.alpha());
        telemetry.update();
        if(gamepad1.x){
            drive.setMotorPowers(-0.4,0.4,-0.4,0.4);
            while(colorSensor.red() < 2000 && colorSensor.blue() < 2000 && colorSensor.green() < 2000 && !stop){
                telemetry.addData("aaaaa","aaaa");
                telemetry.update();
            }
        }
    }
    public void stop(){stop = true;}
}
