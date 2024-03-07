package org.firstinspires.ftc.teamcode;
import java.util.Random;
import static org.firstinspires.ftc.teamcode.util.LogFiles.log;
import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import kotlin.math.UMathKt;

public class ChestiiDeAutonom{
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private DcMotorEx melcsus, melcjos, slider, motorBL, motorBR, motorFL, motorFR;
    private ServoImplEx ghearaR, ghearaL, plauncher, intakeinatorDreapta;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private CRServo maceta, extensorL, extensorR;
    private AnalogInput potentiometru;
    private DistanceSensor distanceL, distanceR;
    private RevBlinkinLedDriver led;
    private boolean sasiuInited;
    LinearOpMode opMode;
    public ChestiiDeAutonom(LinearOpMode opMode){
        this.opMode = opMode;
    }
    public void init(HardwareMap hard){
        this.init(hard, null, false);
    }
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(configPID.p,configPID.i,configPID.d);
    public void init(HardwareMap hard, Telemetry telemetry, boolean shouldInitSasiu) {
        this.hardwareMap = hard;
        this.telemetry = telemetry;
        if (shouldInitSasiu) {
            initSasiu(hard);
        }
        sasiuInited = shouldInitSasiu;

        potentiometru = hard.get(AnalogInput.class, "potentiometru");

        led = hard.get(RevBlinkinLedDriver.class, "led");

        melcjos = hard.get(DcMotorEx.class, "melcjos");
        melcsus = hard.get(DcMotorEx.class, "melcsus");
        slider = hard.get(DcMotorEx.class, "slider");

        ghearaL = hard.get(ServoImplEx.class, "gherutaL");
        ghearaR = hard.get(ServoImplEx.class, "gherutaR");
        plauncher = hard.get(ServoImplEx.class, "plauncher");
        intakeinatorDreapta = hard.get(ServoImplEx.class, "intakeinatorDreapta");

        maceta = hard.get(CRServo.class, "maceta");
        extensorL = hard.get(CRServo.class, "extensorL");
        extensorR = hard.get(CRServo.class, "extensorR");

        slider.setDirection(DcMotorEx.Direction.REVERSE);
        melcsus.setDirection(DcMotorEx.Direction.REVERSE);
        extensorL.setDirection(CRServo.Direction.REVERSE);

        melcjos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        melcsus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        melcjos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        melcjos.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        melcsus.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //distanceL = hard.get(DistanceSensor.class, "distanceL");
        //distanceR = hard.get(DistanceSensor.class, "distanceR");
    }

    public void initSasiu(HardwareMap hard) {
        motorBL = hard.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hard.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hard.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hard.get(DcMotorEx.class, "motorFR"); // Motor Back-Left

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public synchronized void POWER(double df1, double sf1,double ds1, double ss1) {
        if (sasiuInited) {
            motorFR.setPower(df1);
            motorBL.setPower(ss1);
            motorFL.setPower(sf1);
            motorBR.setPower(ds1);
        }
        else {
            throw new NullPointerException("Bro sasiul nu e initializat");
        }
    }

    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {pid.enable();
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!this.opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                kdf(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (this.opMode.opModeIsActive()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                kdf(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            kdf(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            kdf(20);
        }
    }

    public void initTaguriAprilie() {
        initAprilTag();
        setManualExposure(6, 250);
    }
    public void setLedPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        led.setPattern(pattern);
    }
    public void setRandomPattern(int t){
        Random rand = new Random();
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.fromNumber(rand.nextInt(100));
        setLedPattern(pattern);
        kdf(t);
    }
    public double getGhearaLPosition(){
        return ghearaL.getPosition();
    }
    public double getGhearaRPosition(){
        return ghearaR.getPosition();
    }
    public void setGhearaLPosition(double pos){ghearaL.setPosition(pos);}
    public void setGhearaRPosition(double pos){ghearaR.setPosition(pos);}
    public void detectieTaguriAprilie(int DESIRED_TAG_ID) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                if (detection.id == DESIRED_TAG_ID) {
                    telemetry.addData("April tag detection corners:", detection.corners);
                    telemetry.update();
                    break;
                }
            }
            else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
    }

    public void disableGhearaL(){ghearaL.setPwmDisable();}
    public void disableGhearaR(){ghearaR.setPwmDisable();}
    public void deschidere() {
        ghearaR.setPosition(0.25);
        ghearaL.setPosition(0.75);
    }

    public void deschidereJumate(){
        ghearaR.setPosition(0.5);
        ghearaL.setPosition(0.5);
    }

    public void inchidere() {
        ghearaR.setPosition(1);
        ghearaL.setPosition(0);
    }

    public synchronized void target(double poz, double vel, DcMotorEx motor, double t, int tolerance) {
        if (motor.getCurrentPosition() < poz) {
            motor.setVelocity(vel);
        }
        else {
            motor.setVelocity(-vel);
        }

        double lastTime = System.currentTimeMillis();
        while (!this.opMode.isStopRequested()
                && lastTime + t > System.currentTimeMillis()
                && (abs(motor.getCurrentPosition() - poz) > tolerance)) {
        }
        motor.setVelocity(0);
    }
    public double getPotentiometruVoltage(){return potentiometru.getVoltage();}
    public double getMotorBLPower() {
        return motorBL.getPower();
    }
    public double getMotorFLPower() {
        return motorFL.getPower();
    }
    public double getMotorBRPower() {
        return motorBR.getPower();
    }
    public double getMotorFRPower() {
        return motorFR.getPower();
    }
    public double getSliderPower() {
        return slider.getPower();
    }
    public double getMelcjosPower() {
        return melcjos.getPower();
    }
    public double getMelcsusPower() {
        return melcsus.getPower();
    }
    public double getMelcSusVelocity(){return melcsus.getVelocity();}
    public double getMelcJosVelocity(){return melcjos.getVelocity();}
    public double getMacetaPower() {
        return maceta.getPower();
    }
    public double getIntakeinatorDreaptaPosition(){return intakeinatorDreapta.getPosition();}
    public void setIntakeinatorPosition(double positionDreapta){
        intakeinatorDreapta.setPosition(positionDreapta);
    }
    public void setMelcPIDFCoefficients(double kp, double ki, double kd, double kf){
        melcsus.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(kp,ki,kd,kf));
        melcjos.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(kp,ki,kd,kf));
    }
    public double getDistanceL(DistanceUnit distanceUnit) {
        return distanceL.getDistance(distanceUnit);
    }

    public double getDistanceR(DistanceUnit distanceUnit) {
        return distanceR.getDistance(distanceUnit);
    }

    public DcMotorEx getSlider() {
        return slider;
    }
    public int getMelcJosPosition(){return melcjos.getCurrentPosition();}

    public void setExtensorPower(double pow, int t) {
        long lastTime = System.currentTimeMillis();
        while(lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested()) {
            extensorR.setPower(pow);
            extensorL.setPower(pow);
        }
        extensorL.setPower(0);
        extensorR.setPower(0);
    }

    public synchronized void setMelcPower(float melcPower) {
        melcsus.setPower(melcPower);
        melcjos.setPower(melcPower);
    }
    public double getAngleFromVoltage(double voltage){return getAngleFromVout(voltage);}
    public double getCurrentPotentiometruAngle(){return getAngleFromVout(getPotentiometruVoltage());}
    public void setSliderPower(double sliderPower) {
        slider.setPower(sliderPower);
    }

    public synchronized void setMacetaPower(double pow) {
        maceta.setPower(pow);
    }

    public synchronized void spitPixel(int t, double pow) {
        maceta.setPower(pow);
        kdf(t);
        maceta.setPower(0);
    }
    public void setMelcVelocity(double velocity){
        melcsus.setVelocity(velocity);
        melcjos.setVelocity(velocity);
    }
    public void setSliderVelocity(double velocity){
        slider.setVelocity(velocity);
    }
    public void letPixelDrop(long delay) {
        ghearaL.setPosition(0.445);
        ghearaR.setPosition(0.14);
        try {
            Thread.sleep(delay);
        }
        catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        ghearaR.setPosition(0.38);
        ghearaL.setPosition(0.38);
    }
    public void deschidereLeft(){
        setGhearaLPosition(0.64);
        setGhearaRPosition(0.285);
    }
    public void deschidereRight(){
        setGhearaLPosition(0.465);
        setGhearaRPosition(0.14);
    }
    public void melctarget(double poz, double vel, double t) {
        double lastTime = System.currentTimeMillis();
        if (getPotentiometruVoltage() > poz && this.opMode.opModeIsActive()) {
            melcsus.setVelocity(-vel);
            melcjos.setVelocity(-vel);
            while (getPotentiometruVoltage() > poz && lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested()) {
            }
        }
        else{
            melcsus.setVelocity(vel);
            melcjos.setVelocity(vel);
            while (getPotentiometruVoltage() < poz && lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested()) {
            }
        }
        melcjos.setVelocity(0);
        melcsus.setVelocity(0);
    }
    public void melctargetRealAngle(double angle, double vel, double t) {
        double lastTime = System.currentTimeMillis();
        if (getCurrentPotentiometruAngle() < angle) {
            melcsus.setVelocity(-vel);
            melcjos.setVelocity(-vel);
            while (getCurrentPotentiometruAngle() < angle && lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested()) {
            }
        }
        else {
            melcsus.setVelocity(vel);
            melcjos.setVelocity(vel);
            while (getCurrentPotentiometruAngle() > angle && lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested()) {
            }
        }
        melcjos.setVelocity(0);
        melcsus.setVelocity(0);
    }
    public void melctargetRealAngleAdaptive(double angle, double pow, double t) {
        double lastTime = System.currentTimeMillis();
        if (getCurrentPotentiometruAngle() < angle) {
            melcsus.setVelocity(-1 / abs(getCurrentPotentiometruAngle()-angle) * pow);
            melcjos.setVelocity(-1 / abs(getCurrentPotentiometruAngle()-angle) * pow);
            while (getCurrentPotentiometruAngle() < angle && lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested()) {
            }
        }
        else {
            melcsus.setVelocity(1 / abs(getCurrentPotentiometruAngle()-angle) * pow);
            melcjos.setVelocity(1 / abs(getCurrentPotentiometruAngle()-angle) * pow);
            while (getCurrentPotentiometruAngle() > angle && lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested()) {
            }
        }
        melcjos.setVelocity(0);
        melcsus.setVelocity(0);
    }
    public void melctargettolerance(double poz, double vel, double t, double tolerance) {
        double lastTime = System.currentTimeMillis();
        if (getPotentiometruVoltage() > poz) {
            melcsus.setVelocity(-vel);
            melcjos.setVelocity(-vel);
            while (getPotentiometruVoltage() > poz + tolerance && lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested()) {
            }
        }
        else {
            melcsus.setVelocity(vel);
            melcjos.setVelocity(vel);
            while (getPotentiometruVoltage() < poz - tolerance && lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested()) {
            }
        }
        melcjos.setVelocity(0);
        melcsus.setVelocity(0);
    }
    public void setMacetaTimp(long t, double pow){
        long lastTime = System.currentTimeMillis();
        maceta.setPower(pow);
        while(lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested());
        maceta.setPower(0);
    }
    public void melctargetencoder(double poz, double vel, double t, double tolerance){
        if (melcjos.getCurrentPosition() > poz) {
            melcjos.setVelocity(vel);
            melcsus.setVelocity(vel);
        }
        else {
            melcjos.setVelocity(-vel);
            melcsus.setVelocity(-vel);
        }
        double lastTime = System.currentTimeMillis();
        while (!this.opMode.isStopRequested()
                && lastTime + t > System.currentTimeMillis()
                && (abs(melcjos.getCurrentPosition() - poz) > tolerance)){
            Log.wtf("pozitie:",Integer.toString(melcjos.getCurrentPosition()));}
        melcsus.setVelocity(0);
        melcjos.setVelocity(0);
    }
    public void pixel_advance(){
            melctargetRealAngle(440, 1500, 3000);
            target(-1120, 2000, getSlider(), 3000, 20);
            melctargetRealAngle(424, 1500, 3000);
            setMacetaPower(-1);
    }
    public void pixel_advance_1(){
        melctargetRealAngle(440,1500,3000);
        target(-1120, 2000, getSlider(), 3000, 20);
        melctargetRealAngle(432,1500,3000);
        setMacetaPower(-1);
    }
    public void pixel_retreat(){
        melctargetRealAngle(435,1500,3000);
        target(-70, 2000, getSlider(), 3000, 20);
        setMacetaPower(0);
    }
    public synchronized void setPlauncherPosition(double position) {
        plauncher.setPosition(position);
    }

    public double getAngleFromVout(double vout){
        return (270 * vout + 445.5 + 23.382 * sqrt(400 * vout * vout - 440 * vout + 363)) / (2 * vout);
    }

    public PIDFCoefficients getMelcPIDFCoefficients() {
        return melcjos.getPIDFCoefficients(melcjos.getMode());
    }
    public void operation_pixel(){
        setMacetaPower(-1);
        setIntakeinatorPosition(0.39);
        target(-630,1000,getSlider(),3000,20);
        melctargetRealAngle(420,1200,3000);
        target(-730,1000,getSlider(),3000,5);
        kdf(1500);
        setIntakeinatorPosition(0.58);
        kdf(500);
    }
    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis() && !this.opMode.isStopRequested());
    }
}
