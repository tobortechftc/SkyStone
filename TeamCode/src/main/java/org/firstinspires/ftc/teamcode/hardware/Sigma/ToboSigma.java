package org.firstinspires.ftc.teamcode.hardware.Sigma;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import javax.xml.transform.Source;

public class ToboSigma extends Logger<ToboSigma> implements Robot2 {
    private Telemetry telemetry;
    public SwerveChassis chassis;
    public CameraStoneDetector cameraStoneDetector;
    public FoundationHook foundationHook;
    public Intake intake;
    public StoneGrabber stoneGrabber;

    public enum SkystoneLocation {
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    public enum CameraSource {
        INTERNAL, WEBCAM_RIGHT, WEBCAM_LEFT
    }

    public enum AutoTeamColor {
        NOT_AUTO, AUTO_RED, AUTO_BLUE, DIAGNOSIS
    }

    public CoreSystem core;
    public ElapsedTime runtime = new ElapsedTime();
    public double rotateRatio = 0.7; // slow down ratio for rotation
    public double motor_count = 0;
    public double auto_chassis_power = .6;
    public double auto_chassis_power_slow = .3;

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, AutoTeamColor autoColor) {
        runtime.reset();
        double ini_time = runtime.seconds();
        this.telemetry = telemetry;

        this.core = new CoreSystem();
        info("RoboSigma configure() after new CoreSystem()(run time = %.2f sec)", (runtime.seconds() - ini_time));
        chassis = new SwerveChassis(this.core).configureLogging("Swerve", logLevel); // Log.DEBUG

        // Warning: MUST disable the following line during competition
        // chassis.enableRangeSensorTelemetry();//Comment out later

        chassis.configure(configuration, (autoColor != AutoTeamColor.NOT_AUTO), true);
        info("RoboSigma configure() after init Chassis (run time = %.2f sec)", (runtime.seconds() - ini_time));
        if (autoColor != AutoTeamColor.NOT_AUTO && autoColor != AutoTeamColor.DIAGNOSIS) {
            cameraStoneDetector = new CameraStoneDetector().configureLogging("CameraStoneDetector", logLevel);
            // cameraStoneDetector.configure(configuration, CameraSource.INTERNAL);
            cameraStoneDetector.configure(configuration, (autoColor == AutoTeamColor.AUTO_RED ? CameraSource.WEBCAM_LEFT : CameraSource.WEBCAM_RIGHT));
            info("RoboSigma configure() after init cameraStoneDetector (run time = %.2f sec)", (runtime.seconds() - ini_time));
        }
        foundationHook = new FoundationHook(this.core).configureLogging("FoundationHook", logLevel);
        foundationHook.configure(configuration, (autoColor != AutoTeamColor.NOT_AUTO));
        if (autoColor==AutoTeamColor.DIAGNOSIS) {
            foundationHook.hookUp();
        }

        stoneGrabber = new StoneGrabber(this.core).configureLogging("StoneGrabber", logLevel);
        stoneGrabber.configure(configuration, (autoColor != AutoTeamColor.NOT_AUTO));
        intake = new Intake(this.core).configureLogging("Intake", logLevel);
        intake.configure(configuration, (autoColor!=AutoTeamColor.NOT_AUTO));

    }

    @Override
    public void reset(boolean auto) {
        if (Thread.currentThread().isInterrupted()) return;
        if (chassis != null) {
            chassis.reset();
            if (auto) {
                // Display all sensors in auto only for debugging
                // chassis.setupTelemetry(telemetry);
            }
        }
        if (foundationHook!=null) {
            foundationHook.reset(auto);
        }
        if (stoneGrabber!=null)
            stoneGrabber.reset(auto);

        if (intake!=null) {
            intake.reset(auto);
        }
    }

    @MenuEntry(label = "TeleOp", group = "Competition")
    public void mainTeleOp(EventManager em, EventManager em2) {
        telemetry.addLine().addData("(RS)", "4WD").setRetained(true)
                .addData("(RS) + (LS)", "2WD / Steer").setRetained(true);
        telemetry.addLine().addData("< (LS) >", "Rotate").setRetained(true)
                .addData("[LB]/[LT]", "Slow / Fast").setRetained(true);
        telemetry.addLine().addData("motor_count=", new Func<String>() {
            @Override
            public String value() {
                return String.format("%2.0f", motor_count);
            }
        });
        if (chassis!=null)
            chassis.setupTelemetry(telemetry);
        if (stoneGrabber!=null)
            stoneGrabber.setupTelemetry(telemetry);
        if (intake!=null)
            intake.setupTelemetry(telemetry);
        if (foundationHook!=null)
            foundationHook.setupTelemetry(telemetry);

        em.updateTelemetry(telemetry, 100);

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.BOTH)) < 0.1) {
                    // right stick with idle left stick operates robot in "crab" mode
                    double power = Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH));
                    power *= power; // square power to stick
                    double heading = toDegrees(currentX, currentY);
                    double cur_heading = chassis.getCurHeading();
                    // invert headings less than -90 / more than 90
                    if ((Math.abs(cur_heading - heading) < 10) || (Math.abs(currentX) + Math.abs(currentY) < 0.1)) { // keep original heading
                        heading = cur_heading;
                    }
                    // dead zone mapping: [-120, -75] to -90
                    // dead zone mapping: [75, 120] to 90
                    if (heading > -120 && heading < -75) heading = -90;
                    if (heading > 75 && heading < 120) heading = 90;
                    if ((Math.abs(cur_heading - heading) == 180) && Math.abs(heading) <= 90) {
                        heading = cur_heading;
                        power = -1 * power;
                    }
                    if (Math.abs(heading) > 90) { // reduce rotation angle
                        heading -= Math.signum(heading) * 180;
                        power = -1 * power;
                    }
                    debug("sticksOnly(): straight, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, true);
                } else if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY)) > 0.2 && Math.abs(currentY) < 0.1) {
                    // Orbit mode: right-stickY is small and both right-stickX left-StickY have value
                    double power = Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY));
                    double curvature = Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY));
                    power *= power * source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY); // square power to stick
                    chassis.orbit(power * powerAdjustment(source), curvature);
                } else {
                    // right stick with left stick operates robot in "car" mode
                    double heading = source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY) * 90;
                    double power = currentY * Math.abs(currentY);
                    debug("sticksOnly(): right / steer, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH)) < 0.2 &&
                        Math.abs(currentX) > 0.1) {
                    // left stick with idle right stick rotates robot in place
                    chassis.rotate(currentX * Math.abs(currentX) * powerAdjustment(source) * rotateRatio);
                } else if (source.getTrigger(Events.Side.RIGHT) < 0.2 && Math.abs(currentX) > 0.1) {
                    // right stick with left stick operates robot in "car" mode
                    double heading = currentX * 90;
                    double power = source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY);
                    debug("sticksOnly(): left / steer, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                } else if (chassis!=null){
                    chassis.stop();
                }
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);


        // em: [RB] + [Y] for mineral dump combo (move slider to dump, intake box up, open box gate)
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.START) && source.isPressed(Button.BACK)) { // testing chassis speed
                    motor_count = chassis.driveStraightSec(1.0, 10);
                    return;
                } else if (source.isPressed(Button.LEFT_BUMPER) && source.isPressed(Button.RIGHT_BUMPER)) { // testing chassis speed
                    motor_count = chassis.driveStraightSec(1.0, 2);
                    return;
                } else if (source.isPressed(Button.BACK)) { // default scale up
                    chassis.setDefaultScale(1.0);
                    return;
                }

            }
        }, Button.Y);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.Y) && source.isPressed(Button.BACK)) {
                    // intake drop In/out
                    if (intake!=null) intake.intakeDropAuto();
                } else if (source.isPressed(Button.BACK)) { // default scale back to 0.5
                    chassis.setDefaultScale(0.7);
                } else if(!source.isPressed(button.START)){
                    foundationHook.hookAuto();
                }
            }
        }, Button.A);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (intake != null) intake.intakeIn(!source.isPressed(Button.BACK));
            }
        }, Button.LEFT_BUMPER);

        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                if (intake != null) intake.intakeStop();
            }
        }, Button.LEFT_BUMPER);

        em.onTrigger(new Events.Listener() {
            @Override
            public void triggerMoved(EventManager source, Events.Side side, float current, float change) throws InterruptedException {
                // 0.2 is a dead zone threshold for the trigger

                if (current > 0.2) {
                    if (intake != null) intake.intakeOut(!source.isPressed(Button.BACK));
                } else {
                    if (intake != null) intake.intakeStop();
                }
            }
        }, Events.Side.LEFT);

        //The following are all events for Driver 2

    /*    em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.deliverStoneCombo();
            }
        }, new Button[]{Button.Y});
*/
        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.grabStoneCombo();
            }
        }, new Button[]{Button.A});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.grabStoneInsideCombo();
            }
        }, new Button[]{Button.Y});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armInCombo(source.isPressed(Button.BACK), false);
                else
                    stoneGrabber.grabberAuto();
            }
        }, new Button[]{Button.X});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armOutCombo();
                else if (!source.isPressed(Button.START))
                    stoneGrabber.wristAuto();
            }
        }, new Button[]{Button.B});

        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY)) > 0.2) {
                    // left stick with idle right stick rotates robot in place
                    if (source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY) > 0)
                        stoneGrabber.armUpInc();
                    else
                        stoneGrabber.armDownInc();
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.LEFT);

        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY)) > 0.2) {
                    // left stick with idle right stick rotates robot in place
                    if (source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY) > 0)
                        stoneGrabber.liftUp();
                    else
                        stoneGrabber.liftDown();
                } else
                    stoneGrabber.liftStop();
            }
        }, Events.Axis.Y_ONLY, Events.Side.RIGHT);

    }

    @MenuEntry(label = "Auto Straight", group = "Test Chassis")
    public void testStraightSkyStone(EventManager em) {

        telemetry.addLine().addData(" < (BACK) >", "Power(%.2f)", auto_chassis_power).setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else {
                    chassis.driveStraightAuto(auto_chassis_power, 100, 0, 10000);
                    //chassis.driveStraightAuto(auto_chassis_power, -7, 0, 10000);
                    //chassis.driveStraightAuto(auto_chassis_power, 220, -90, 15000);
                    //chassis.driveStraightAuto(auto_chassis_power, 260, 90, 15000);
                    //chassis.driveStraightAuto(auto_chassis_power, 20, 0, 10000);
                    //chassis.driveStraightAuto(auto_chassis_power, -5, 0, 10000);
                    //chassis.driveStraightAuto(auto_chassis_power, 243, -90, 15000);//245?
                }
            }
        }, new Button[]{Button.Y});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power -= 0.1;
                    if (auto_chassis_power < 0.1) auto_chassis_power = 0.1;
                }
            }
        }, new Button[]{Button.A});


    }
    @MenuEntry(label = "New Auto Straight", group = "Test Chassis")
    public void testStraightNewSkyStone(EventManager em) {

        try {
            chassis.tl=telemetry;
            //chassis.driveStraightAutoRunToPosition(.4, 200, 0, 10000, telemetry);
            chassis.tl=telemetry;
            chassis.driveStraightAutoRunToPosition(.6, 150, -90, 10000);
            Thread.sleep(500);
            chassis.driveStraightAutoRunToPosition(.6, 150, 90, 10000);
            Thread.sleep(500);
            chassis.driveStraightAutoRunToPosition(.6, 150, 0, 10000);
            Thread.sleep(500);
            chassis.driveStraightAutoRunToPosition(.6, -150, 0, 10000);
        }catch (InterruptedException e){

        }


        /*
        telemetry.addLine().addData(" < (BACK) >", "Power(%.2f)", auto_chassis_power).setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else {
                    chassis.driveStraightAutoNew(auto_chassis_power, 100, 0, 10000);

                }
            }
        }, new Button[]{Button.Y});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power -= 0.1;
                    if (auto_chassis_power < 0.1) auto_chassis_power = 0.1;
                }
            }
        }, new Button[]{Button.A});
        *
         */
    }

        @MenuEntry(label = "Drive Straight", group = "Test Chassis")
    public void testStraight(EventManager em) {
        telemetry.addLine().addData("(LS)", "Drive").setRetained(true)
                .addData("Hold [LB]/[RB]", "45 degree").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 1000);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                double power = Math.max(Math.abs(currentX), Math.abs(currentY));
                double heading = toDegrees(currentX, currentY);
                debug("testStraight(): x: %+.2f, y: %+.2f, pow: %+.3f, head: %+.1f",
                        currentX, currentY, power, heading);

                if (source.isPressed(Button.LEFT_BUMPER) || source.isPressed(Button.RIGHT_BUMPER)) {
                    // constrain to 45 degree diagonals
                    heading = Math.signum(heading) * (Math.abs(heading) < 90 ? 45 : 135);
                } else {
                    // constrain to 90 degrees
                    heading = Math.round(heading / 90) * 90;
                }

                // adjust heading / power for driving backwards
                if (heading > 90) {
                    chassis.driveStraight(-1.0 * power, heading - 180);
                } else if (heading < -90) {
                    chassis.driveStraight(-1.0 * power, heading + 180);
                } else {
                    chassis.driveStraight(power, heading);
                }
            }
        }, Events.Axis.BOTH, Events.Side.LEFT);
    }

    @MenuEntry(label = "Rotate in Place", group = "Test Chassis")
    public void testRotate(EventManager em) {
        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                chassis.rotate(currentX);
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);
    }

    @MenuEntry(label = "Crab", group = "Test Chassis")
    public void testCrab(EventManager em) {
        telemetry.addLine().addData(" < (LS) >", "Direction").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                chassis.driveStraightAuto(.5, 120, 90 * Math.signum(currentX), 100000);
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);
    }

    /**
     * Returns angle (-180 to 180 degrees) between positive Y axis
     * and a line drawn from center of coordinates to (x, y).
     * Negative values are to the left of Y axis, positive are to the right
     */
    private double toDegrees(double x, double y) {
        if (x == 0) return y >= 0 ? 0 : 180;
        return Math.atan2(x, y) / Math.PI * 180;
    }

    /**
     * Returns power adjustment based on left bumper / left trigger state
     * Normal mode = 60% power
     * Left bumper down = slow mode (30% power)
     * Left trigger down = turbo mode (up to 100%) power
     */
    private double powerAdjustment(EventManager source) {
        double adjustment = chassis.getDefaultScale();  // default adjustment
        double trig_num = 0.0;
        if (source.isPressed(Button.RIGHT_BUMPER)) {
            // slow mode uses 30% of power
            adjustment = 0.25;
        } else if ((trig_num = source.getTrigger(Events.Side.RIGHT)) > 0.2) {
            // 0.2 is the dead zone threshold
            // turbo mode uses (100% + 1/2 trigger value) of power
            adjustment = 0.9 + 0.1 * trig_num * trig_num;
        }
        return adjustment;
    }

    public void getFirstSkyStone(ToboSigma.SkystoneLocation skyStonePosition, boolean isBlue, boolean isLeft) throws InterruptedException {
        int side = isBlue ? 1 : -1;

        //arm out
        stoneGrabber.armOutComboAuto();
        stoneGrabber.grabberOpen();

        // go to stones
        foundationHook.hookUp();

        chassis.driveStraightAutoRunToPosition(.4, 46, 0, 10000);
        chassis.rotateTo(.2, 0);
        double dist = chassis.getDistance(SwerveChassis.Direction.FRONT) - 11;

        if (dist>20) dist=20;
        chassis.driveStraightAutoRunToPosition(.3,  dist, 0,1000);
        chassis.rotateTo(.2, 0);
        core.yield_for(0.2);
        if (skyStonePosition==SkystoneLocation.UNKNOWN)
           skyStonePosition = chassis.skyStoneLocation(isBlue); // using color sensors need to be close enough to the stones

        chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow,  -2, 0,1000);
        if(!isBlue){ // Red side
            if((skyStonePosition == SkystoneLocation.LEFT)||skyStonePosition == SkystoneLocation.UNKNOWN){
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 8, 90 * side, 3000);  // test to get exact numbers
            } else if(skyStonePosition == ToboSigma.SkystoneLocation.CENTER){
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 13, -90 * side, 3000);  // test to get exact numbers
            } else { // if (skyStonePosition == ToboSigma.SkystoneLocation.RIGHT) {
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 33, -90 * side, 3000);  // test to get exact numbers
            }
        } else { // Blue side
            if (skyStonePosition == ToboSigma.SkystoneLocation.RIGHT || skyStonePosition == SkystoneLocation.UNKNOWN) {
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 5, 90 * side, 3000);  // test to get exact numbers
            } else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER) {
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 15, -90 * side, 3000); // test to get exact numbers
            } else { // skyStonePosition == ToboSigma.SkystoneLocation.LEFT
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 35, -90 * side, 3000);  // test to get exact numbers
            }
        }

        //grab stone
        if (Thread.currentThread().isInterrupted()) return;

        stoneGrabber.grabStoneComboAuto();
        //chassis.driveStraightAutoRunToPosition(.3, -8, 0, 1000);
        chassis.driveStraightAutoRunToPosition(.3, -13, 0, 1000);
        chassis.rotateTo(.2, 0);
        stoneGrabber.armInComboAuto(true);

        // go to foundation

        int ss_pos = 1;
        if (isBlue) {
            if (skyStonePosition == ToboSigma.SkystoneLocation.RIGHT)
                ss_pos = 3;
            else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER)
                ss_pos = 2;
        } else { // red
            if (skyStonePosition == ToboSigma.SkystoneLocation.LEFT)
                ss_pos = 3;
            else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER)
                ss_pos = 2;
        }

        chassis.rotateTo(.2, 0);
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 150 + 20 * ss_pos, -90 * side, 15000);

        //align

        chassis.rotateTo(.2, 0);

        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        dist = dist > 128 ? 40 : dist;

        chassis.driveStraightAutoRunToPosition(auto_chassis_power, dist - 40, -90 * side, 15000);

        if (Thread.currentThread().isInterrupted()) return;

        //place stone

        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 20, 0, 1000);
        stoneGrabber.armOutComboAuto();
        stoneGrabber.wristPerpendicular();
    }


    public void getAnotherSkyStone(SkystoneLocation StoneLoc, int stoneNum, boolean isBlue) throws InterruptedException{//stoneNum - how many stones ara we going to have after this trip
        int side = 1;
        if(!isBlue){
            side = -1;
        }
        int toTake;
        if (StoneLoc == SkystoneLocation.RIGHT) {
            int[] a = {6, 1, 2, 4, 5};
            toTake = a[stoneNum - 2];
        } else if (StoneLoc == SkystoneLocation.CENTER) {
            int[] a = {5, 1, 3, 4, 6};
            toTake = a[stoneNum - 2];
        } else { // left or unknown
            int[] a = {4, 2, 3, 5, 6};
            toTake = a[stoneNum - 2];
        }

        //go to stones

        chassis.rotateTo(.2, 0);

        // align


         double dist;
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 180 + 20 * stoneNum, 90* side, 15000);
        chassis.rotateTo(.2, 0);
        if (isBlue) {
            dist = chassis.getDistance(SwerveChassis.Direction.RIGHT);
        } else {
            dist = chassis.getDistance(SwerveChassis.Direction.LEFT);
        }
        if (dist > 128) {
            dist = 0;
        }

        //arm out
        stoneGrabber.armOutComboAuto();
        stoneGrabber.grabberOpen();

        // get close to stones

        chassis.driveStraightAutoRunToPosition(auto_chassis_power, dist +15 -20 * stoneNum, 90* side, 15000);
        dist = chassis.getDistance(SwerveChassis.Direction.FRONT);
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, (dist - 20), 0, 10000);

        //grab stone
        stoneGrabber.grabStoneComboAuto();

        //go back
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, -30, 0, 10000);

         chassis.rotateTo(.2, 0);
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 175 + 20 * stoneNum, -90 * side, 15000);
        chassis.rotateTo(.2, 0);
        if (isBlue) {
            dist = chassis.getDistance(SwerveChassis.Direction.LEFT);
        } else {
            dist = chassis.getDistance(SwerveChassis.Direction.RIGHT);
        }
        if (dist > 128) {
            dist = 20;
        }
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, dist - 40, -90 * side, 15000);

        stoneGrabber.deliverStoneComboAuto();
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, -20, 0, 10000);
        stoneGrabber.armInComboAuto(false);

    }

    public void grabAndPark(boolean isBlue) throws InterruptedException{
        if (Thread.currentThread().isInterrupted()) return;
        int side = 1;
        if(!isBlue){
            side = -1;
        }
        double dist;
        //= chassis.getDistance(SwerveChassis.Direction.FRONT);

       // chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 15, 0, 10000);
        foundationHook.hookDown();
        Thread.sleep(500);
        dist = chassis.getDistance(SwerveChassis.Direction.BACK);
        int max_dist = 100;
        if (dist > max_dist) {
            dist = max_dist;
        }

        //chassis.driveStraightAutoRunToPosition(auto_chassis_power/2, -dist - 10*(1- auto_chassis_power) , 7* side, 10000);
        chassis.driveStraightAutoRunToPosition(auto_chassis_power/1.5, -dist - 10*(1- auto_chassis_power) , 7* side, 4000);
        foundationHook.hookUp();
        stoneGrabber.deliverStoneComboAuto();
        chassis.rotateTo(.3, 0);

        if (isBlue) {
            dist = chassis.getDistance(SwerveChassis.Direction.LEFT);
        } else {
            dist = chassis.getDistance(SwerveChassis.Direction.RIGHT);
        }
        if (dist > 128) {
            dist = 20;
        }

        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 90-dist, 90* side, 1500);
        stoneGrabber.armInComboAuto(false);

        if ((isBlue && chassis.getDistance(SwerveChassis.Direction.RIGHT) > 40) || (!isBlue && chassis.getDistance(SwerveChassis.Direction.LEFT) > 40)){
               //chassis.driveStraightAutoRunToPosition(.4, 40, 90* side, 1500);
               chassis.driveStraightAutoRunToPosition(.6, (isBlue?50:60), 90* side, 1500);
        } else {
             chassis.driveStraightAutoRunToPosition(.6, 45, 0, 1500);
             //chassis.driveStraightAutoRunToPosition(.4, 40, 90* side, 1500);
             chassis.driveStraightAutoRunToPosition(.6, (isBlue?50:60), 90* side, 1500);
        }

    }
}
