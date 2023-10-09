package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp

public class sistem extends OpMode {
    public Servo ghearaL,ghearaR,plauncher;
    public CRServo maceta;
    public DcMotorEx melcjos,melcsus, slider;

    double y, x, rx, ghearaPoz=0.5, macetaPow=0;
    double max = 0,lastTime;
    double bval = 0;
    boolean stop = false, sliderState = true, aIntrat = false,aAjuns = true,aInchis = true;
    double intPoz = 0.4, servoPos = 0.0;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        melcjos = hardwareMap.get(DcMotorEx.class, "melcjos");
        melcsus = hardwareMap.get(DcMotorEx.class, "melcsus");

        ghearaL = hardwareMap.get(Servo.class, "ghearaL");
        ghearaR = hardwareMap.get(Servo.class, "ghearaR");

        slider = hardwareMap.get(DcMotorEx.class, "slider");

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        melcjos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        melcsus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        melcjos.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        melcsus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    public void start(){
        Systems.start();
    }
    private final Thread Systems = new Thread(new Runnable() {
        public void run() {
            boolean dpd = gamepad2.dpad_down;
            boolean dpu = gamepad2.dpad_up;
            slider.setPower(gamepad2.right_stick_y);
        }

    });
    public void slidertarget(int poz, double vel, double t, int tolerance) {
        if (melcsus.getCurrentPosition() < poz) {
            melcjos.setVelocity(-Math.abs(vel));
            melcsus.setVelocity(Math.abs(vel));
        } else {
            melcjos.setVelocity(Math.abs(vel));
            melcsus.setVelocity(-Math.abs(vel));
        }
        double lastTime = System.currentTimeMillis();
        while (Math.abs(poz - melcsus.getCurrentPosition()) > tolerance
                && Math.abs(poz - melcjos.getCurrentPosition()) > tolerance
                && lastTime + t > System.currentTimeMillis()) {
        }
        melcjos.setVelocity(0);
        melcsus.setVelocity(0);
    }
    @Override
    public void loop() {

        boolean bl;
        bl = gamepad2.b;
        if (bval == 0.5 && gamepad2.b) {
            bval += 0.0001;
            ghearaR.setPosition(0.15);
            ghearaL.setPosition(0.63);
            lastTime = System.currentTimeMillis();
            while (lastTime + 150 > System.currentTimeMillis()) {
            }
            ghearaL.setPosition(0.38);
            ghearaR.setPosition(0.38);
        }
        if (melcjos.getCurrentPosition() > -400) {
            ghearaR.setPosition(0.38);
            ghearaL.setPosition(0.38);
        } else {
            if (gamepad2.a) {
                ghearaR.setPosition(0.15);
                ghearaL.setPosition(0.63);
            }
            if (gamepad2.y) {
                ghearaR.setPosition(0.38);
                ghearaL.setPosition(0.38);
            }
        }

    }
    public void stop() {
        stop = true;
    }
}


