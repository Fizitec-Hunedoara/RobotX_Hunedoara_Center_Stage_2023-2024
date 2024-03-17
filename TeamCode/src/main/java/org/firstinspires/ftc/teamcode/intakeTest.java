/* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
 */
package org.firstinspires.ftc.teamcode;
import static java.lang.Math.abs;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;

@TeleOp
public class intakeTest extends OpMode {
    private RevBlinkinLedDriver.BlinkinPattern pattern;
    private double sm = 1, smm =1;
    private double bval = 0;
    private boolean bl;
    private double y, x, rx, max, intakeinatorPoz = 0.5,intakeinatorPoz2 = 0.13;
    private double lastTime;
    private double pmotorBL, pmotorBR, pmotorFL, pmotorFR;
    private boolean stop = false, aLansat = false, notEntered, aRetras = false, aIntrat = false,aInchis = false;
    private final ChestiiDeAutonomTeleOP c = new ChestiiDeAutonomTeleOP();
    public OpenCvCamera webcam;
    private boolean xLast;
    private enum OuttakeState{
        INCHIS,
        DESCHIS
    }
    private OuttakeState outtakeState = OuttakeState.INCHIS;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        c.init(hardwareMap, telemetry, true);
    }
    public void start() {
        Systems.start();
    }
    private final Thread AvionSiAGC = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                c.setLedPattern(pattern);
                if ((gamepad2.dpad_up && gamepad2.left_trigger > 0)||gamepad1.right_bumper) {
                    lastTime = System.currentTimeMillis();
                    c.setPlauncherPosition(0.5);
                    aLansat = true;
                }
                else {
                    c.setPlauncherPosition(0.35);
                }
                /*if (gamepad2.x) {
                    c.setExtensorPower(-1, 3000);
                }
                else if (!aLansat) {
                    if (!notEntered && c.getEncoderBrat() > 1.7) {
                        c.setExtensorPower(-1, 3000);
                        notEntered = true;
                    }
                    else if (c.getEncoderBrat() > 1.7) {
                        c.setExtensorPower(gamepad2.left_stick_x, 1);
                    }
                    else if (notEntered) {
                        c.setExtensorPower(1, 3000);
                        notEntered = false;
                    }
                }
                else {
                    if (!aRetras) {
                        c.setExtensorPower(-1, 3000);
                        aRetras = true;
                    }
                }*/
            }
        }
    });
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if(gamepad2.a){
                    c.spitPixel();
                }
                if(gamepad2.y){
                    c.setIntakeinatorPosition(0.44);
                    c.target(-630,1000,c.getSlider(),3000,20);
                    c.melctargetRealAngle(420,1200,3000);
                    c.setMacetaPower(1);
                    c.setMacetaPower(-1);
                    c.target(-700,1000,c.getSlider(),3000,5);
                    c.kdf(500);
                    c.setIntakeinatorPosition(0.6);
                    c.kdf(1500);
                }
                if(gamepad2.dpad_left && intakeinatorPoz > 0){
                    intakeinatorPoz -= 0.001;
                }
                if(gamepad2.dpad_right && intakeinatorPoz < 1){
                    intakeinatorPoz += 0.001;
                }
                if(gamepad2.dpad_down && intakeinatorPoz2 > 0){
                    intakeinatorPoz2 -= 0.001;
                }
                if(gamepad2.dpad_up && intakeinatorPoz2 < 1){
                    intakeinatorPoz2 += 0.001;
                }
                c.setIntakeinatorPosition(intakeinatorPoz2);
                c.setSliderPower(gamepad2.right_stick_y);
                if (xLast != gamepad2.x) {
                    if (outtakeState == OuttakeState.INCHIS) {
                        c.setExtensorPower(1, 3000);
                        outtakeState = OuttakeState.DESCHIS;
                    }
                    else {
                        c.setExtensorPower(-1, 1500);
                        outtakeState = OuttakeState.INCHIS;
                    }
                }
                xLast = gamepad2.x;
                if (gamepad2.dpad_up && gamepad2.right_trigger > 0) {
                    c.melctargetRealAngle(470, 1300, 10000);
                }
                else {
                    c.setMelcPower(-gamepad2.left_stick_y);
                }
                if (gamepad2.dpad_down) {
                    c.setMacetaPower(1.0);
                }
                if (gamepad2.dpad_up) {
                    c.setMacetaPower(-1.0);
                }
            }
        }
    });

    public void stop() {
        stop = true;
    }

    @Override
    public void loop() {
        telemetry.addData("motorBL", c.getMotorBLPower());
        telemetry.addData("motorFL", c.getMotorFLPower());
        telemetry.addData("motorBR", c.getMotorBRPower());
        telemetry.addData("motorFR", c.getMotorFRPower());
        telemetry.addData("slider:", c.getSliderPower());
        telemetry.addData("melcjos:", c.getMelcJosPosition());
        telemetry.addData("melcsus:", c.getMelcsusPower());
        telemetry.addData("macetaPow:", c.getMacetaPower());
        telemetry.addData("extensorPow:",c.getSlider().getCurrentPosition());
        telemetry.addData("bval:", bval);
        telemetry.addData("bl:", bl);
        telemetry.addData("sm", sm);
        telemetry.addData("gamepad2.b:", gamepad2.b);
        telemetry.addData("pozitiebrat:", c.getPotentiometruVoltage());
        telemetry.addData("pozitiebratencoder:",c.getMelcJosPosition());
        telemetry.addData("ghearaL:",c.getGhearaLPosition());
        telemetry.addData("ghearaR:",c.getGhearaRPosition());
        telemetry.addData("getBratAngle:",c.getCurrentPotentiometruAngle());
        telemetry.addData("intakeinator dreapta:",c.getIntakeinatorDreaptaPosition());
        telemetry.addData("automatizare",aInchis);
        telemetry.update();
    }
}