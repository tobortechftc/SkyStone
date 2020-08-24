package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;


import java.util.List;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Thread.sleep;

/**
 * Swerve chassis consists of 4 wheels with a Servo and DcMotor attached to each.
 * Track (distance between left and right wheels), wheelbase (distance between front and back wheels)
 * and wheel radius are adjustable.
 * Expected hardware configuration is:<br />
 * Servos: servoFrontLeft, servoFrontRight, servoBackLeft, servoBackRight.<br />
 * Motors: motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight.<br />
 * Orientation sensors (optional): imu, imu2
 */
public class MechChassis extends Logger<MechChassis> implements Configurable {

    final private CoreSystem core;

    public enum DriveMode {
        STOP,      // not moving
        STRAIGHT,  // driving in a straight line utilizing orientation sensor to correct itself,
        //  all servos are set to the same position
        ROTATE,    // rotating in place, all servos are set on tangent lines to a circle drawn
        //  through the centers of 4 wheels.
        STEER      // motor power and servos in the direction of movement are controlled by
        //  the driver; opposite servos are in central position
    }

    // the following 4 ratio values are used to normalize 4 wheel motors to the same speed
    // whenever changing a wheel motor, it must be calibrated again
    /* for GoBilda 435 motor set:
    private double ratioFL = 1.0;
    private double ratioFR = 0.8847;
    private double ratioBL = 0.9348;
    private double ratioBR = 0.9315;
    */
    /* for GoBilda 1125 motor set: */
    private double ratioFL = 0.9890;
    private double ratioFR = 0.9890;
    private double ratioBL = 1.0;
    private double ratioBR = 0.9941;

    private double left_ratio = 1.0; // slow down ratio for left wheels to go straight
    private double right_ratio = 1.0; // slow down ratio for right wheels to go straight
    private double front_ratio = 0.95; // slow down ratio for front wheels to go 90 degree
    private double back_ratio = 1.0; // slow down ratio for front wheels to go 90 degree

    private double fixedStopDist = 15; // stop distance for 20 cm /sec


    // distance between the centers of left and right wheels, inches
    private double track = 11.5;
    // distance between the centers of front and back wheels, inches
    private double wheelBase = 10.7;
    // wheel radius, inches
    private double wheelRadius = 2.0;
    // minimum power that should be applied to the wheel motors for robot to start moving
    private double minPower = 0.2;
    private double minPowerHorizontal = 0.3;

    // maximum power that should be applied to the wheel motors
    private double maxPower = 0.99;
    private double maxRange = 127; // max range sensor detectable
    private double defaultScale = 1.0;
    private double mecanumForwardRatio = 0.8;
    private double chassisAligmentPower = 0.2;

    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;

    // array contains the same wheel assemblies as above variables
    //  and is convenient to use when actions have to be performed on all 4
    public CombinedOrientationSensor orientationSensor;
    public double auto_target_x = 0;
    public double auto_target_y = 0;
    public double auto_power = 0;
    public double auto_loop_time = 0;
    public  double auto_travel_p = 0;

    private DriveMode driveMode = DriveMode.STOP;      // current drive mode
    private double targetHeading;     // intended heading for DriveMode.STRAIGHT as reported by orientation sensor
    private double headingDeviation;  // current heading deviation for DriveMode.STRAIGHT as reported by orientation sensor
    private double servoCorrection;   // latest correction applied to leading wheels' servos to correct heading deviation
    private double curHeading = 0;
    private boolean useScalePower = true;//
    private boolean setImuTelemetry = false;//unless debugging, don't set telemetry for imu
    private boolean setRangeSensorTelemetry = false;//unless debugging, don't set telemetry for range sensor

    private boolean normalizeMode = true;

    public void toggleNormalizeMode(){
        normalizeMode = !normalizeMode;
    }

    public boolean getNormalizeMode(){
        return normalizeMode;
    }

    private void configure_IMUs(Configuration configuration) {
        orientationSensor = new CombinedOrientationSensor().configureLogging(logTag + "-sensor", logLevel);
        orientationSensor.configure(configuration.getHardwareMap(), "imu", "imu2");
    }

    List<LynxModule> allHubs;

    //odometry motors
    DcMotorEx verticalLeftEncoder;
    DcMotorEx verticalRightEncoder;
    DcMotorEx horizontalEncoder;
    OdometryGlobalCoordinatePosition globalPositionUpdate;
    final double ODO_COUNTS_PER_INCH = 307.699557;
    final double ODO_COUNTS_PER_CM = ODO_COUNTS_PER_INCH / 2.54;


    String rfName = "motorFR" , lfName = "motorFL";
    String rbName = "motorBR";
    String lbName = "motorBL";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    final double TICKS_PER_CM = 16.86;//number of encoder ticks per cm of driving
    final double WHEEL_CM_PER_ROTATION = 4*Math.PI*2.54;


    public void setGlobalPosUpdate(OdometryGlobalCoordinatePosition val) { globalPositionUpdate=val;}
    public OdometryGlobalCoordinatePosition globalPositionUpdate() { return globalPositionUpdate; }
    public double odo_count_per_inch() {return ODO_COUNTS_PER_INCH;}
    public double odo_count_per_cm() {return ODO_COUNTS_PER_CM;}
    public DcMotorEx verticalLeftEncoder(){ return verticalLeftEncoder; }
    public DcMotorEx verticalRightEncoder(){ return verticalRightEncoder; }
    public DcMotorEx horizontalEncoder(){ return horizontalEncoder; }

    public double getMecanumForwardRatio() {
        return mecanumForwardRatio;
    }

    public void configureOdometry() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeftEncoder(), verticalRightEncoder(), horizontalEncoder(), odo_count_per_inch(), 75);
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseLeftEncoder();
    }

    public double odo_x_pos_inches() {
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.returnXCoordinate()/odo_count_per_inch();
    }
    public double odo_x_pos_cm() {
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.returnXCoordinate()/odo_count_per_cm();
    }

    public double odo_y_pos_inches() {
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.returnYCoordinate()/odo_count_per_inch();
    }
    public double odo_y_pos_cm() {
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.returnYCoordinate()/odo_count_per_cm();
    }

    public double odo_x_speed_cm() { // horizontal speed as cm/sec
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.getXSpeedDegree() / 360.0 * WHEEL_CM_PER_ROTATION;
    }

    public double odo_y_speed_cm() { // forward speed as cm/sec
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.getYSpeedDegree() / 360.0 * WHEEL_CM_PER_ROTATION;
    }

    public double odo_speed_cm() {
       double speed = 0;
       // Praveer to-do: combine odo_x_speed_cm() and odo_y_speed_cm() into chassis speed
       speed = Math.hypot(odo_x_speed_cm(), odo_x_speed_cm());
       return speed;
    }

    public double odo_heading() {
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.returnOrientation();
    }

    public double getLeft_ratio() { return left_ratio; }
    public double getRight_ratio() { return left_ratio; }
    public double getFront_ratio() { return front_ratio; }
    public double getBack_ratio() { return back_ratio; }

    public double powerScale() { return defaultScale; }
    public double getDefaultScale() {
        return defaultScale;
    }
    public void setDefaultScale(double val) {
        defaultScale = val;
    }

    public void enableRangeSensorTelemetry() { // must be call before reset() or setupTelemetry()
        setRangeSensorTelemetry = true;
    }

    public void enableImuTelemetry(Configuration configuration) {
        setImuTelemetry = true;
        if (orientationSensor==null) {
            configure_IMUs(configuration);
        }
    }

    @Adjustable(min = 8.0, max = 18.0, step = 0.02)
    public double getTrack() {
        return track;
    }

    public void setTrack(double track) {
        this.track = track;
    }

    @Adjustable(min = 8.0, max = 18.0, step = 0.02)
    public double getWheelBase() {
        return wheelBase;
    }

    public void setWheelBase(double wheelBase) {
        this.wheelBase = wheelBase;
    }

    @Adjustable(min = 1.0, max = 5.0, step = 0.02)
    public double getWheelRadius() {
        return wheelRadius;
    }

    public void setWheelRadius(double wheelRadius) {
        this.wheelRadius = wheelRadius;
    }

    @Adjustable(min = 0.0, max = 0.4, step = 0.01)
    public double getMinPower() {
        return minPower;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    @Adjustable(min = 0.2, max = 1.0, step = 0.01)
    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    @Override
    public String getUniqueName() {
        return "chassis";
    }

    /**
     * SwerveChassis constructor
     */
    public MechChassis(CoreSystem core) {
        this.core = core;
    }

    /**
     * Used only for ToboRuckus, old code
     */
    @Deprecated
    public MechChassis() {
        this.core = new CoreSystem(); // Prevents null pointer exception
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    public void configure(Configuration configuration, boolean auto) {
        // set up motors / sensors as wheel assemblies

        motorFL = configuration.getHardwareMap().tryGet(DcMotorEx.class, lfName);
        motorFR = configuration.getHardwareMap().tryGet(DcMotorEx.class, rfName);
        motorBL = configuration.getHardwareMap().tryGet(DcMotorEx.class, lbName);
        motorBR = configuration.getHardwareMap().tryGet(DcMotorEx.class, rbName);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // map odometry encoders
        verticalLeftEncoder = configuration.getHardwareMap().tryGet(DcMotorEx.class, verticalLeftEncoderName);
        verticalRightEncoder = configuration.getHardwareMap().tryGet(DcMotorEx.class, verticalRightEncoderName);
        horizontalEncoder = configuration.getHardwareMap().tryGet(DcMotorEx.class, horizontalEncoderName);

        verticalLeftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //verticalLeftEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        //verticalRightEncoder.setDirection(DcMotorEx.Direction.REVERSE);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Enable bulk read mode to speed up the encoder reads
        allHubs = configuration.getHardwareMap().getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (auto || setImuTelemetry) {
            configure_IMUs(configuration);
        }

        // register chassis as configurable component
        configuration.register(this);
    }


    public void driveStraight(double power, double cm, double degree, long timeout_sec) throws InterruptedException {
        driveStraight(power, cm, degree, 0.8, timeout_sec);
    }

    public void driveStraight(double power, double cm, double degree, double slowDownPercent, long timeout_sec) throws InterruptedException {
        double fixed_heading = orientationSensor.getHeading();
        // to-do: Sasha to compute x_dist/y_dist based on current heading
        double x_dist = cm * Math.sin(Math.toRadians(degree)) * Math.signum(power);
        double y_dist = cm * Math.cos(Math.toRadians(degree)) * Math.signum(power);
        double target_x = x_dist + odo_x_pos_cm();
        double target_y = y_dist + odo_y_pos_cm();
        auto_target_x = target_x;
        auto_target_y = target_y;

        power = power * Math.signum(cm);

        driveTo(power, target_x, target_y, fixed_heading, slowDownPercent,timeout_sec);
    }

    public void driveTo(double power, double target_x, double target_y, double target_heading, long timeout_sec) throws InterruptedException {
        driveTo(power, target_x, target_y, target_heading, .8, timeout_sec);
    }

    public void driveTo(double power, double target_x, double target_y, double target_heading, double slowDownPercent, double timeout_sec) throws InterruptedException {
        long iniTime = System.currentTimeMillis();
        double current_heading = orientationSensor.getHeading();
        double cur_x = odo_x_pos_cm(), prev_x = cur_x, init_x=cur_x;
        double cur_y = odo_y_pos_cm(), prev_y = cur_y, init_y=cur_y;
        double desiredDegree = Math.toDegrees(Math.atan2(target_x - cur_x, target_y - cur_y));
        double currentAbsDiff = abs(desiredDegree - current_heading) > 180 ? 360 - abs(desiredDegree - current_heading) : abs(desiredDegree - current_heading);
        double movementAngle = Math.min(180-currentAbsDiff, currentAbsDiff); // movementaAngle for robot from 0 to 90
        boolean rotateFirst = (movementAngle>45); // if diff(target_heading-current_heading)>45
        if (rotateFirst && (Math.abs(current_heading - target_heading) > 20)) {
            double stage_one_heading = target_heading - 10;
            if (current_heading > target_heading)
                stage_one_heading = target_heading + 10;
            rawRotateTo(power, stage_one_heading, true, timeout_sec);
        }

        double error_cm = 0.5;  // to-do: error_cm should depend on degree
        double powerUsed = (Math.abs(power)<minPower?minPower*Math.signum(power):power);
        double x_dist = target_x - cur_x;
        double y_dist = target_y - cur_y;
        boolean x_reached = false;
        boolean y_reached = false;
        double traveledPercent = 0;
        double save_target_heading = target_heading;

        target_heading = orientationSensor.getHeading();
        double init_loop_time = System.currentTimeMillis();
        int loop_count = 0;

        double[] motorPowers = {0, 0, 0, 0};
        // slow down stuff
        double[] s = getSlowDownParameters(target_heading, getCurHeading(), power);
        slowDownPercent = s[0];
        double decreasePercent = s[1];
        double minPowerForAngle = s[2];

        double cur_s;
        double expectedStopDist;
        double remDistance;

        while((!x_reached || !y_reached) && (System.currentTimeMillis() - iniTime < timeout_sec * 1000)) {
            if (Math.abs(y_dist) > 0.01 && Math.abs(x_dist) > 0.01) {
                traveledPercent = Math.max(Math.abs(cur_y - init_y) / Math.abs(y_dist), Math.abs(cur_x - init_x) / Math.abs(x_dist));
            } else if (Math.abs(y_dist)>0.01) {
                traveledPercent = Math.abs(cur_y - init_y) / Math.abs(y_dist);
            } else if (Math.abs(x_dist)>0.01) {
                traveledPercent = Math.abs(cur_x - init_x) / Math.abs(x_dist);
            }
            auto_travel_p = traveledPercent;
//            if (traveledPercent > slowDownPercent) {
//               powerUsed = getSlowDownPower(traveledPercent, slowDownPercent, decreasePercent, power, minPowerForAngle);
//            }
            if (traveledPercent<0.9) {
                desiredDegree = Math.toDegrees(Math.atan2(target_x - cur_x, target_y - cur_y));
            }
            //move
            motorPowers = angleMove(desiredDegree, powerUsed, true, target_heading);

            remDistance = Math.hypot(target_x- cur_x, target_y - cur_y);
            cur_s = odo_speed_cm();
            expectedStopDist = Math.pow(cur_s / 20., 2) * fixedStopDist;
            if (remDistance - (expectedStopDist + .01 * cur_s)  < .001){ // we need to measure fixedStopDist ( overshoot fot any speed, then we need to change 20. to that speed
                break;
            }



            if (Math.abs(cur_y-target_y)<=error_cm)
                y_reached=true;
            else if (y_dist>0){
                if (cur_y >= target_y - error_cm)
                    y_reached=true;
            } else{
                if (cur_y <= target_y + error_cm)
                    y_reached=true;
            }
            if (Math.abs(cur_x-target_x)<=error_cm)
                x_reached=true;
            else if (x_dist>0){
                if (cur_x >= target_x - error_cm)
                    x_reached=true;
            } else{
                if (cur_x <= target_x + error_cm)
                    x_reached=true;
            }
            prev_x = cur_x;
            cur_x = odo_x_pos_cm();
            prev_y = cur_y;
            cur_y = odo_y_pos_cm();
            loop_count ++;
        }
        double end_loop_time = System.currentTimeMillis();
        if (loop_count>0)
            auto_loop_time = (end_loop_time-init_loop_time)/(double)loop_count;

        target_heading = save_target_heading;

        current_heading = orientationSensor.getHeading();
        currentAbsDiff = abs(target_heading - current_heading) > 180 ? 360 - abs(target_heading - current_heading) : abs(target_heading - current_heading);
        if ((currentAbsDiff > 1.2) && !Thread.interrupted()) {
            rotateTo(Math.abs(power), target_heading, timeout_sec);
        }
        stopNeg(motorPowers);
        //tl.addData("speed: ", odo_speed_cm());
        //tl.update();

    }
    public double getSlowDownPower(double traveledPercent, double slowDownPercent, double decreaseP, double power, double minPowerForAngle){
        double apower = Math.abs(power);
        double pow;
        double percentPow;
        if (traveledPercent < .25 + .75 * slowDownPercent)
            //pow = .25 * minPower + .75 * apower;
            percentPow = decreaseP;
        else if (traveledPercent < .5 + .5 * slowDownPercent)
            //pow = .5 * minPower + .5 * apower;
            percentPow = decreaseP * .6;
        else if (traveledPercent < .75 + .25 * slowDownPercent) {
            //pow = .75 * minPower + .25 * apower;
            percentPow = decreaseP * .3;
        } else {
            //pow = minPower;
            percentPow = 0;
        }
        pow = percentPow * apower + (1-percentPow) * minPowerForAngle;
        if (pow < minPowerForAngle) pow = minPowerForAngle;
        return  pow * Math.signum(power);
    }

    public double[] getSlowDownParameters(double directionAngle, double heading, double power){
        double movementAngle = Math.abs(directionAngle - heading);
        movementAngle = Math.min(180-movementAngle, movementAngle);// movementaAngle but from 0 to 90
        movementAngle = 1 - movementAngle / 90.;// movemebt angle from 0 (horizontal, slowest) to 1(vertical, fastest
        double minPowerForAngle = minPowerHorizontal - movementAngle * (minPowerHorizontal - minPower);
        double powWeight = 1, movementAngleWeight = 1;// these numebrs need to be tested and maybe changed
        double fast = (powWeight * power + movementAngleWeight * movementAngle) / (powWeight + movementAngleWeight);  // how fast the robot is going to go from 0 to 1
        double fastSlowDownP = .7, slowSlowDownP = .85;
        double fastDecreaseP = .75, slowDecreaseP = .4;
        double slowDownP = fastSlowDownP + (1-fast) * (slowSlowDownP - fastSlowDownP);// slow down percent - slow down sooner when going faster
        double decreaseP = fastDecreaseP + (1-fast) * (slowDecreaseP - fastDecreaseP); // how much we decrease power in the first step - slower has steeper decrease

        return new double[] {slowDownP, decreaseP, minPowerForAngle};
    }
    /**
     * move in the vertical direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void yMove(int sgn, double power) {
        motorFL.setPower(sgn * power * left_ratio * ratioFL);
        motorFR.setPower(sgn * power * right_ratio * ratioFR);
        motorBL.setPower(sgn * power * left_ratio * ratioBL);
        motorBR.setPower(sgn * power * right_ratio * ratioBR);
    }

    /**
     * move in the horizontal direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void xMove(int sgn, double power) {
        motorFL.setPower(sgn * power * front_ratio * ratioFL);
        motorFR.setPower(-sgn * power * front_ratio * ratioFR);
        motorBL.setPower(-sgn * power * back_ratio * ratioBL);
        motorBR.setPower(sgn * power * back_ratio * ratioBR);
    }
    public double[] angleMove(double directionAngle, double power, boolean headingCorrection, double fixed_heading){

        auto_travel_p = directionAngle;

        double cur_heading = orientationSensor.getHeading();
        double degree_diff = Math.abs(cur_heading-fixed_heading);
        // to-do: need to handle gap from 179 to -179
        double cur_left_to_right_ratio = 1.0;
        boolean slow_down_left=false, slow_down_right=false;
        if (headingCorrection && (degree_diff>1.0)) { // for Y axle correction
            slow_down_left = ((cur_heading-fixed_heading>0) && directionAngle>-45 && directionAngle<45) ||
                    (((cur_heading-fixed_heading<0) && directionAngle>135 && directionAngle<-135));
            slow_down_right = ((cur_heading-fixed_heading<0) && directionAngle>-45 && directionAngle<45) ||
                    (((cur_heading-fixed_heading>0) && directionAngle>135 && directionAngle<-135));
            if (slow_down_left||slow_down_right) {
                cur_left_to_right_ratio = (1.0 - degree_diff * 0.05);
                if (cur_left_to_right_ratio<0.9) cur_left_to_right_ratio=0.9;
            }
        }
        double cur_front_to_back_ratio = 1.0;
        boolean slow_down_front=false, slow_down_back=false;
        if (headingCorrection && (degree_diff>1.0)) { // for Y axle correction
            slow_down_front = ((cur_heading-fixed_heading>0) && directionAngle>=45 && directionAngle<=135) ||
                    (((cur_heading-fixed_heading<0) && directionAngle>=-135 && directionAngle<=-45));
            slow_down_back = ((cur_heading-fixed_heading<0) && directionAngle>=45 && directionAngle<=135) ||
                    (((cur_heading-fixed_heading>0) && directionAngle>=-135 && directionAngle<=-45));
            if (slow_down_front||slow_down_back) {
                cur_front_to_back_ratio = (1.0 - degree_diff * 0.05);
                if (cur_front_to_back_ratio<0.9) cur_front_to_back_ratio=0.9;
            }
        }

        double[] motorPowers  = new double[4];
        motorPowers[0] = (Math.sin(Math.toRadians(directionAngle - fixed_heading))+ Math.cos(Math.toRadians(directionAngle- fixed_heading)));
        motorPowers[1] = (Math.cos(Math.toRadians(directionAngle- fixed_heading))- Math.sin(Math.toRadians(directionAngle - fixed_heading)));
        motorPowers[2] = motorPowers[1];
        motorPowers[3] = motorPowers[0];
        double max = Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1]));
        if (slow_down_left) { // slow-down left
            motorPowers[0] *= cur_left_to_right_ratio;
            motorPowers[2] *= cur_left_to_right_ratio;
        } else if (slow_down_right) { // slow down right
            motorPowers[1] *= cur_left_to_right_ratio;
            motorPowers[3] *= cur_left_to_right_ratio;
        }
        if (slow_down_front) { // make front motors slower
            motorPowers[0] = motorPowers[0] * Math.abs(power) * ratioFL * cur_front_to_back_ratio / max;
            motorPowers[1] = motorPowers[1] * Math.abs(power) * ratioFR * cur_front_to_back_ratio / max;
            motorPowers[2] = motorPowers[2] * Math.abs(power) * ratioBL / max;
            motorPowers[3] = motorPowers[3] * Math.abs(power) * ratioBR / max;
            motorFL.setPower(motorPowers[0]);
            motorFR.setPower(motorPowers[1]);
            motorBL.setPower(motorPowers[2]);
            motorBR.setPower(motorPowers[3]);
        } else { // slow down back (or keep same)
            motorPowers[0] = motorPowers[0] * Math.abs(power) * ratioFL / max;
            motorPowers[1] = motorPowers[1] * Math.abs(power) * ratioFR / max;
            motorPowers[2] = motorPowers[2] * Math.abs(power) * ratioBL * cur_front_to_back_ratio / max;
            motorPowers[3] = motorPowers[3] * Math.abs(power) * ratioBR * cur_front_to_back_ratio / max;

            motorFL.setPower(motorPowers[0]);
            motorFR.setPower(motorPowers[1]);
            motorBL.setPower(motorPowers[2]);
            motorBR.setPower(motorPowers[3]);
        }
        return motorPowers;
    }

    public void freeStyle(double fl, double fr, double bl, double br, boolean normalized) {
        if (normalized) {
            motorFL.setPower(fl * ratioFL);
            motorFR.setPower(fr * ratioFR);
            motorBL.setPower(bl * ratioBL);
            motorBR.setPower(br * ratioBR);
        } else {
            motorFL.setPower(fl);
            motorFR.setPower(fr);
            motorBL.setPower(bl);
            motorBR.setPower(br);
        }
    }

    public void forward(double power, double inches, long timeout_sec) {
        // power should always be positive
        // inches > 0 forward
        //        < 0 backward

        power = Math.abs(power);
        boolean count_up = (Math.signum(inches)>0);
        double error_inches = 0.1;
        double target_y = inches+odo_y_pos_inches();
        long iniTime = System.currentTimeMillis();
        double cur_y = odo_y_pos_inches(), prev_y=cur_y;
        while ((Math.abs(cur_y-target_y) > error_inches) && (System.currentTimeMillis()-iniTime<timeout_sec*1000)) {
            yMove((int) Math.signum(inches), power);
            if (count_up) {
                if (cur_y>=target_y-error_inches) break;
            } else {
                if (cur_y<=target_y+error_inches) break;
            }
                prev_y=cur_y;
            cur_y = odo_y_pos_inches();
        }
        stop();
    }

    public void crab(double power, double inches, long timeout_sec) {
        // power should always be positive
        // inches > 0, crab right 90 degree
        //        < 0, crab left 90 degree
        power = Math.abs(power);

        boolean count_up = (Math.signum(inches)>0);

        double error_inches = 0.1;
        double target_x = inches+odo_x_pos_inches();
        long iniTime = System.currentTimeMillis();
        double cur_x = odo_x_pos_inches(), prev_x=cur_x;
        while ((Math.abs(cur_x-target_x) > error_inches) && (System.currentTimeMillis()-iniTime<timeout_sec*1000)) {
            xMove((int) Math.signum(inches), power);
            if (count_up) {
                if (cur_x>=target_x-error_inches) break;
            } else {
                if (cur_x<=target_x+error_inches) break;
            }
            prev_x=cur_x;
            cur_x = odo_x_pos_inches();
        }
        stop();
    }

    /**
     * pivot and turn
     *
     * @param sgn   can be either +1 or -1(+1 for clockwise, -1 for counter clockwise)
     * @param power must be in range [0,1]
     */
    public void turn(int sgn, double power) {
        motorFL.setPower(sgn * power);
        motorFR.setPower(-sgn * power);
        motorBL.setPower(sgn * power);
        motorBR.setPower(-sgn * power);
    }

    /**
     * turning while driving
     *
     * @param power         must be in range [0,1]
     * @param turningFactor int range [-1,+1] (+1 for turning right, -1 for turning left)
     */
    public void carDrive(double power, double turningFactor) {

        if (turningFactor > 0) {
            turningFactor = 1 - turningFactor;
            if (power>0) { // drive forward
                motorFL.setPower(power);
                motorFR.setPower(turningFactor * power);
                motorBL.setPower(power);
                motorBR.setPower(turningFactor * power);
            } else {
                motorFL.setPower(turningFactor * power);
                motorFR.setPower(power);
                motorBL.setPower(turningFactor * power);
                motorBR.setPower(power);
            }
        } else {
            turningFactor = 1 + turningFactor;
            if (power>0) { // drive forward
                motorFL.setPower(turningFactor * power);
                motorFR.setPower(power);
                motorBL.setPower(turningFactor * power);
                motorBR.setPower(power);
            } else {
                motorFL.setPower(power);
                motorFR.setPower(turningFactor * power);
                motorBL.setPower(power);
                motorBR.setPower(turningFactor * power);
            }
        }

    }

    public void driveStraightAuto(double power, double cm, int timeout) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;

        if (power < 0 || power > 1) {
            throw new IllegalArgumentException("Power must be between 0 and 1");
        }

        double distance = TICKS_PER_CM * cm;

        if (power == 0) {
            //driveMode = SwerveChassis.DriveMode.STOP;
            targetHeading = 0;
            headingDeviation = 0;
            servoCorrection = 0;
            //for (SwerveChassis.WheelAssembly wheel : wheels) wheel.motor.setPower(0);
            orientationSensor.enableCorrections(false);
            return;
        }

        if (distance < 0) {
            power = -power;
            distance = -distance;
        }

        //motor settings
        //driveMode = SwerveChassis.DriveMode.STRAIGHT;
        int[] startingCount = new int[4];
        for (int i = 0; i < 4; i++) {
            //wheels[i].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //wheels[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //startingCount[i] = wheels[i].motor.getCurrentPosition();
        }

        //imu initialization
        orientationSensor.enableCorrections(true);
        targetHeading = orientationSensor.getHeading();

        //start powering wheels
        yMove(cm > 0 ? +1 : -1, power);
        //record time
        long iniTime = System.currentTimeMillis();

        //waiting loop
        while (true) {
            // check and correct heading as needed
            double sensorHeading = orientationSensor.getHeading();
            headingDeviation = targetHeading - sensorHeading;

            //determine if target distance is reached
            int maxTraveled = Integer.MIN_VALUE;
            for (int i = 0; i < 4; i++) {
                //maxTraveled = Math.abs(Math.max(maxTraveled, wheels[i].motor.getCurrentPosition() - startingCount[i]));
            }
            if (distance - maxTraveled < 10)
                break;
            //determine if time limit is reached
            if (System.currentTimeMillis() - iniTime > timeout)
                break;
            if (Thread.currentThread().isInterrupted())
                break;
            // yield handler
            this.core.yield();
        }
        stop();
        driveMode = DriveMode.STOP;
    }



    public void setRunMode(DcMotor.RunMode rm) {
        motorFL.setMode(rm);
        motorFR.setMode(rm);
        motorBL.setMode(rm);
        motorBR.setMode(rm);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        motorFL.setZeroPowerBehavior(zpb);
        motorFR.setZeroPowerBehavior(zpb);
        motorBL.setZeroPowerBehavior(zpb);
        motorBR.setZeroPowerBehavior(zpb);
    }

    public void stop() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
    public void stopNeg(double[] motorPowers) throws InterruptedException {
        motorFL.setPower(Math.signum(motorPowers[0] * .2));
        motorFR.setPower(Math.signum(motorPowers[1] * .2));
        motorBL.setPower(Math.signum(motorPowers[2] * .2));
        motorBR.setPower(Math.signum(motorPowers[3] * .2));
        sleep(30);
        stop();
    }

    public void reset() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        driveMode = DriveMode.STOP;
        targetHeading = 0;
    }


    public enum Direction {
        FRONT, LEFT, RIGHT, BACK;
    }

    public enum Wall {
        LEFT, RIGHT;
    }

    public double getCurHeading() {
        return curHeading;
    }

    /**
     * Scales power according to <code>minPower</code> and <code>maxPower</code> settings
     */
    private double scalePower(double power) {
        double adjustedPower = Math.signum(power) * minPower + power * (maxPower - minPower);
        return Math.abs(adjustedPower) > 1.0 ? Math.signum(adjustedPower) : adjustedPower;
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        line.addData("Pwr/Scale", new Func<String>() {
            @Override
            public String value() {
                return String.format("%.2f / %.1f / %s\n", motorFL.getPower(), getDefaultScale(),
                        (getNormalizeMode()?"Normalized":"Speedy"));
            }
        });
        /*
        if (motorFL != null) {
            line.addData("FL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorFL.getCurrentPosition();
                }
            });
        }
        if (motorFR != null) {
            line.addData("FR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorFR.getCurrentPosition();
                }
            });
        }
        if (motorBL != null) {
            line.addData("BL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorBL.getCurrentPosition();
                }
            });
        }
        if (motorBR != null) {
            line.addData("BR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorBR.getCurrentPosition();
                }
            });
        }

        if (horizontalEncoder != null) {
            line.addData("row X", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return horizontalEncoder.getCurrentPosition();
                }
            });
        }
        if (verticalLeftEncoder != null) {
            line.addData("row Y-Left", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return verticalLeftEncoder.getCurrentPosition();
                }
            });
        }
        if (verticalRightEncoder != null) {
            line.addData("row Y-Right", "%d\n", new Func<Integer>() {
                @Override
                public Integer value() {
                    return verticalRightEncoder.getCurrentPosition();
                }
            });
        }
        */

        if (globalPositionUpdate!=null) {
            line.addData("Odo (speed,x,ly,ry)", new Func<String>() {
                @Override
                public String value() {
                    return String.format("(%5.0f,%5.0f,%5.0f,%5.0f)\n", odo_speed_cm(), globalPositionUpdate.XEncoder(),
                            globalPositionUpdate.leftYEncoder(),
                            globalPositionUpdate.rightYEncoder());
                }
            });
            line.addData("Odo-pos (x,y,angle)", new Func<String>() {
                @Override
                public String value() {
                    return String.format("(%2.0f, %2.0f, %2.2f)\n", odo_x_pos_cm(), odo_y_pos_cm(), odo_heading());
                }
            });
        }
        //set up imu telemetry
        if (orientationSensor != null && setImuTelemetry) {
            line.addData("imuC", "%.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return orientationSensor.getHeading();
                }
            });
            orientationSensor.setupTelemetry(line);
        }

        telemetry.addLine().addData("M", new Func<String>() {
            @Override
            public String value() {
                return driveMode.name();
            }
        }).addData("Head", new Func<String>() {
            @Override
            public String value() {
                if (driveMode != DriveMode.STRAIGHT) return "N/A";
                return String.format("%+.1f (%+.2f)", targetHeading, headingDeviation);
            }
        }).addData("Adj", new Func<String>() {
            @Override
            public String value() {
                if (driveMode != DriveMode.STRAIGHT) return "N/A";
                return String.format("%+.1f", servoCorrection);
            }
        });
    }
    public void setupEncoders(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        if (motorFL != null) {
            line.addData("FL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorFL.getCurrentPosition();
                }
            });
        }
        if (motorFR != null) {
            line.addData("FR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorFR.getCurrentPosition();
                }
            });
        }
        if (motorBL != null) {
            line.addData("BL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorBL.getCurrentPosition();
                }
            });
        }
        if (motorBR != null) {
            line.addData("BR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorBR.getCurrentPosition();
                }
            });
        }
    }
    public void resetOrientation() {
        orientationSensor.reset();
    }

    public boolean hasRollStabalized(int inputIndex, double minDiff) {
        return orientationSensor.hasRollStabalized(inputIndex, minDiff);
    }

    public void rawRotateTo(double power, double finalHeading, boolean stopEarly, double timeout_sec) throws InterruptedException {
        if (Thread.interrupted()) return;
        debug("rotateT0(pwr: %.3f, finalHeading: %.1f)", power, finalHeading);
        double iniHeading = orientationSensor.getHeading();
        double deltaD = finalHeading - iniHeading;
        debug("iniHeading: %.1f, deltaD: %.1f)", iniHeading, deltaD);
        //do not turn if the heading is close enough the target
        if (Math.abs(deltaD) < 0.5)
            return;
        //resolve the issue with +-180 mark
        if (Math.abs(deltaD) > 180) {
            finalHeading = finalHeading + (deltaD > 0 ? -360 : +360);
            deltaD = 360 - Math.abs(deltaD);
            deltaD = -deltaD;
            debug("Adjusted finalHeading: %.1f, deltaD: %.1f)", finalHeading, deltaD);
        }
        //break on reaching the target
        if (Thread.interrupted()) return;
        //for (WheelAssembly wheel : wheels)
          //  wheel.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ensure the condition before calling rotate()
        //driveMode = DriveMode.STOP;
        useScalePower = false;
        //***** routine to start the wheels ******//
        turn((int)Math.signum(deltaD), power);
        //***** End routine to start the wheels ******//
        //record heading for checking in while loop
        double lastReading = orientationSensor.getHeading();
        long iniTime = System.currentTimeMillis();
        while (true) {
            if (Thread.interrupted()) return;
            double currentHeading = orientationSensor.getHeading();
            //we cross the +-180 mark if and only if the product below is a very negative number
            if ((currentHeading * lastReading < -100.0) || (Math.abs(currentHeading - lastReading) > 180.0)) {
                //deltaD>0 => cross the mark clockwise; deltaD<0 => cross the mark anticlockwise
                finalHeading = finalHeading + (deltaD > 0 ? -360.0 : +360.0);
                debug("Crossing180, finalHeading: %.1f, deltaD:%.1f)", finalHeading, deltaD);
            }
            debug("currentHeading: %.1f, finalHeading: %.1f)", currentHeading, finalHeading);
            //if within acceptable range, terminate
            if (Math.abs(finalHeading - currentHeading) < (stopEarly ? power * 10.0 : 0.6)) break;
            //if overshoot, terminate
            if (deltaD > 0 && currentHeading - finalHeading > 0) break;
            if (deltaD < 0 && currentHeading - finalHeading < 0) break;

            if (System.currentTimeMillis() - iniTime > timeout_sec*1000) break;
            //stop pressed, break
            if (Thread.interrupted()) return;
            lastReading = currentHeading;
//            sleep(0);
            // yield handler
            //TaskManager.processTasks();
            this.core.yield();
        }
        if (Thread.interrupted()) return;
        //stop
        stop();

        useScalePower = true;
    }

    static final double degreeToRad = PI / 180;
    static final double radToDegree = 180 / PI;

    public void rotateTo(double power, double finalHeading) throws InterruptedException {
        rotateTo(power, finalHeading, 4000);
    }

    public void rotateTo(double power, double finalHeading, double timeout_sec) throws InterruptedException {
        rotateTo(power, finalHeading, timeout_sec, true,true);
    }

    public void rotateTo(double power, double finalHeading, double timeout_sec, boolean changePower, boolean finalCorrection) throws InterruptedException {
        if (Thread.interrupted()) return;
        double iniHeading = orientationSensor.getHeading();
        if (power <= chassisAligmentPower || Math.abs(iniHeading-finalHeading)<1.0) {//was 0.3
            rawRotateTo(power, finalHeading, false, timeout_sec);//was power
            if (Thread.interrupted()) return;
            if (power > chassisAligmentPower)
                rawRotateTo(chassisAligmentPower, finalHeading, false, timeout_sec);
            return;
        }
        double iniAbsDiff = abs(finalHeading - iniHeading) > 180 ? 360 - abs(finalHeading - iniHeading) : abs(finalHeading - iniHeading);
        if (iniAbsDiff < 0.5)//if within 0.5 degree of target, don't rotate
            return;

        int direction;
        if (cross(-iniHeading, -finalHeading) >= 0) {//revert sign and take cross product
            direction = -1;//rotating ccw
        } else {
            direction = +1;//rotating cw
        }
        if (Thread.interrupted()) return;
        double lowPowerDegree = 8 + (power - chassisAligmentPower) * 60;

        //ensure the condition before calling rotate()
        useScalePower = false;
        //power the wheels
        if (Thread.interrupted()) return;
        turn(1, direction * power);
        double currentHeading;
        double crossProduct;
        double currentAbsDiff;
        boolean lowerPowerApplied = false;
        long iniTime = System.currentTimeMillis();
        int loop = 0;
        while (true) {
            if (Thread.interrupted()) return;
            currentHeading = orientationSensor.getHeading();
//            info("RotateTo-%.1f, heading =%.3f, pw=%.2f(%s)", finalHeading,currentHeading,power,(lowerPowerApplied?"low":"hi"));
            crossProduct = cross(-currentHeading, -finalHeading);
            //break if target reached or exceeded
            if (direction == -1) {//rotating ccw
                if (crossProduct <= 0) break;
            } else {//rotating cw
                if (crossProduct >= 0) break;
            }
            currentAbsDiff = abs(finalHeading - currentHeading) > 180 ? 360 - abs(finalHeading - currentHeading) : abs(finalHeading - currentHeading);
            if (changePower && !lowerPowerApplied && currentAbsDiff <= lowPowerDegree) {//damp power to alignment power if in last 40%, (currentAbsDiff / iniAbsDiff < 0.40)
                if (Thread.interrupted()) return;
                turn(1, 0.0);
                sleep(50);
                if (Thread.interrupted()) return;
                turn(1, direction * (chassisAligmentPower-0.05));
                lowerPowerApplied = true;
            }
            if (currentAbsDiff / iniAbsDiff < 0.20 && abs(crossProduct) * radToDegree < 1.0)//assume sinx=x, stop 1 degree early
                break;//stop if really close to target
            if (Thread.interrupted()) break;
            if (System.currentTimeMillis() - iniTime > timeout_sec*1000) break;
            TaskManager.processTasks();
            loop++;
        }
        if (Thread.interrupted()) return;
        stop();
        if (!finalCorrection) {
            driveMode = DriveMode.STOP;
            useScalePower = true;
            return;
        }
        if (Thread.interrupted()) return;
        sleep(100);
        //**************Check for overshoot and correction**************
        currentHeading = orientationSensor.getHeading();
        currentAbsDiff = abs(finalHeading - currentHeading) > 180 ? 360 - abs(finalHeading - currentHeading) : abs(finalHeading - currentHeading);
        if ((currentAbsDiff > 1.2) && !Thread.interrupted()) {
            rawRotateTo(chassisAligmentPower, finalHeading, false, 0.5);
        }
        if (Thread.interrupted()) return;
        //**************End correction**************
        driveMode = DriveMode.STOP;
        useScalePower = true;
//        tl.addData("iteration",loop);
//        tl.update();
//        sleep(3000);
    }

    //final heading needs to be with in range(-180,180]


    //cross two unit vectors whose argument angle is given in degree
    public static double cross(double theta, double phi) {
        return cos(theta * degreeToRad) * sin(phi * degreeToRad) - sin(theta * degreeToRad) * cos(phi * degreeToRad);
    }
}
