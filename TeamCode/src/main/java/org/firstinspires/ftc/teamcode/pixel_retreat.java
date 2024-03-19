package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sin;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Disabled
@TeleOp
public class pixel_retreat extends OpMode {
    public boolean stop=false;
    public double melcpoz = 0;
    ChestiiDeAutonomTeleOP c = new ChestiiDeAutonomTeleOP();
    double velo;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0,0,0);
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        c.init(hardwareMap);
        c.melctargetRealAngle(470, 1500, 10000);
//        c.setMelcPIDFCoefficients(configPID.kp,configPID.ki,configPID.kd,configPID.kf);
    }
    public void stop(){
        stop=true;
    }
    public void start(){
        Sistem.start();
    }
    private final Thread Sistem = new Thread(() -> {
        while (!stop) {
//            c.setMelcPower(gamepad2.right_stick_y);
//            c.setSliderVelocity(1000*gamepad2.left_stick_y);
//            if(gamepad2.right_bumper){
//                pid = new Pid_Controller_Adevarat(configPID.p, configPID.i, configPID.d);
//                pid.enable();
//                while(!stop&&!gamepad2.left_bumper) {
//                    pid.setPID(configPID.p,configPID.i,configPID.d,configPID.f * sin(517.8 - c.getCurrentPotentiometruAngle()));
//                    pid.setSetpoint(configPID.targetPoz);
//                    velo = pid.performPID(c.getCurrentPotentiometruAngle());
//                    c.setMelcPower(-velo);

//                    c.setMelcPIDFCoefficients(configPID.kp,configPID.ki,configPID.kd,configPID.kf);
//                    melcpoz = 520;
//                    c.melctarget(520, configPID.vel, 4000);
//                    telemetry.addData("ok","ok");
//                    melcpoz = 450;
//                    c.kdf(100);
//                    c.setMelcPIDFCoefficients(configPID.kp,configPID.ki,configPID.kd,configPID.kf);
//                    c.melctarget(450, configPID.vel, 4000);
               // }
           // }
        }
    });
    @Override
    public void loop() {
//        telemetry.addData("pozitie brat:",c.getCurrentPotentiometruAngle());
//        telemetry.addData("velocity melc jos:",c.getMelcJosVelocity());
//        telemetry.addData("velocity melc sus",c.getMelcSusVelocity());
//        telemetry.addData("target pozitie brat:",melcpoz);
//        telemetry.update();
        telemetry.addData("error", pid.getError());
        telemetry.addData("setpoint", pid.getSetpoint());
        telemetry.addData("pozitie", c.getCurrentPotentiometruAngle());
        telemetry.addData("velocity", c.getMelcsusPower());
        Log.d("Power", String.valueOf(c.getMelcsusPower()));
        Log.d("Pozitie", String.valueOf(c.getCurrentPotentiometruAngle()));
        telemetry.update();
    }

}
