/* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
 */
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class TeleOPcenterstageRX extends OpMode {
    private double sm = 1, smm =1;
    private boolean lastx = false, lasty = false;
    private double bval = 0;
    private boolean bl;
    private double y, x, rx, max, ghearaLpos = 0.5, ghearaRpos = 0.5;
    private double lastTime;
    private double pmotorBL, pmotorBR, pmotorFL, pmotorFR;
    private boolean stop = false, aLansat = false, notEntered, aRetras = false, aIntrat = false,aInchis = false;
    private final ChestiiDeAutonom c = new ChestiiDeAutonom();
    public OpenCvCamera webcam;
    public PachetelNouAlbastru pipelineAlbastru = new PachetelNouAlbastru();

    public long lastDeschidereTimestamp, inchidereDelay = 500;
    private boolean rightBumperLast;

    private enum OuttakeState{
        INCHIS,
        ONE_PIXEL,
        TWO_PIXELS
    }
    private OuttakeState outtakeState = OuttakeState.INCHIS;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        c.init(hardwareMap, telemetry, true);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
//        webcam.setPipeline(pipelineAlbastru);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(Var_Red.Webcam_w, Var_Red.Webcam_h, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//        telemetry.addLine("waiting for start:");
//        telemetry.update();
//        FtcDashboard.getInstance().startCameraStream(webcam, 60);
    }

    public void start() {
        Chassis.start();
        Systems.start();
        AvionSiAGC.start();
    }

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                y = gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;

                pmotorFL = -y + x + rx;
                pmotorBL = -y - x + rx;
                pmotorBR = -y + x - rx;
                pmotorFR = -y - x - rx;

                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }

                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                if (gamepad1.x != lastx) {
                    if (gamepad1.x) {
                        if (sm == 1) {
                            sm = 4;
                        }
                        else {
                            sm = 1;
                        }
                    }
                    lastx = gamepad1.x;
                }
                if (gamepad1.y != lasty) {
                    if (gamepad1.y) {
                        if (sm == 1) {
                            sm = 2;
                        }
                        else {
                            sm = 1;
                        }
                    }
                    lasty = gamepad1.y;
                }

                c.POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            }
        }
    });

    private final Thread AvionSiAGC = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (gamepad2.dpad_up && gamepad2.left_trigger > 0) {
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
                c.setSliderPower(gamepad2.right_stick_y);
                if(gamepad2.dpad_up && gamepad2.right_trigger > 0){
                    c.melctarget(1.6,1300,10000);
                }
                else {
                    //c.setMelcPower(-gamepad2.left_stick_y/smm);
//
//                    if (gamepad2.right_trigger > 0.1){
//                        c.setMelcPower(-gamepad2.left_stick_y/6);
//                    }
//                    else c.setMelcPower(-gamepad2.left_stick_y);;
                    c.setMelcPower(-gamepad2.left_stick_y);
                }
                if (gamepad2.dpad_down) {
                    c.setMacetaPower(1.0);
                }
                if (gamepad2.dpad_up) {
                    c.setMacetaPower(-1.0);
                }
                if (c.getPotentiometruVoltage() > 1.7 && !aInchis) {
                    aInchis=true;
                    c.inchidere();
                }
                else if(c.getPotentiometruVoltage() < 1.7){
                    aInchis = false;
                }
                if (gamepad2.b != bl && !aIntrat) {
                    aIntrat = true;
                    Log.wtf("systems","check2");
                    if (gamepad2.b) {
                        //new Thread(() -> c.letPixelDrop(250)).run();
                    }
                    bl = false;
                }
                else if(gamepad2.b == bl && lastTime + 1000 < System.currentTimeMillis()) {
                    aIntrat = false;
                }

                if(gamepad2.left_bumper){
                    c.deschidere();
                }
                else if(gamepad2.right_bumper != rightBumperLast){
                    if(gamepad2.right_bumper){
                        if(outtakeState == OuttakeState.INCHIS){
                            c.deschidereJumate();
                            outtakeState = OuttakeState.ONE_PIXEL;
                        }
                        else if(outtakeState == OuttakeState.ONE_PIXEL){
                            c.deschidere();
                            lastDeschidereTimestamp = System.currentTimeMillis();
                            outtakeState = OuttakeState.TWO_PIXELS;
                        }
                    }
                    rightBumperLast = gamepad2.right_bumper;
                }
                else if(outtakeState == OuttakeState.TWO_PIXELS && System.currentTimeMillis() > lastDeschidereTimestamp + inchidereDelay){
                    c.inchidere();
                    outtakeState = OuttakeState.INCHIS;
                }
                else if(outtakeState == OuttakeState.INCHIS){
                    c.inchidere();
                }
            }
        }
    });

    public void stop() {
        stop = true;
        c.setIsStopRequested(true);
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
        telemetry.addData("bval:", bval);
        telemetry.addData("bl:", bl);
        telemetry.addData("sm", sm);
        telemetry.addData("gamepad2.b:", gamepad2.b);
        telemetry.addData("pozitiebrat:", c.getPotentiometruVoltage());
        telemetry.addData("pozitiebratencoder:",c.getMelcJosPosition());
        telemetry.addData("distanceL:", c.getDistanceL(DistanceUnit.CM));
        telemetry.addData("distanceR:", c.getDistanceR(DistanceUnit.CM));
        telemetry.addData("ghearaL:",c.getGhearaLPosition());
        telemetry.addData("ghearaR:",c.getGhearaRPosition());
        telemetry.update();
    }
}