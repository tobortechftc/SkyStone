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
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import javax.xml.transform.Source;

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

public class ToboSigma extends Logger<ToboSigma> implements Robot2 {
    private Telemetry telemetry;
    public SwerveChassis chassis;
    public CameraStoneDetector cameraStoneDetector;
    public FoundationHook foundationHook;
    public IntakeV3 intake;
    public StoneGrabber stoneGrabber;

    public class AutoPara {
        boolean isBlue = true;
        boolean laneFront = true;
        boolean offensive = false;
        boolean isDone = false;

        public boolean isDone() {
            return isDone;
        }

        public boolean isBlue() {
            return isBlue;
        }

        public boolean isLaneFront() {
            return laneFront;
        }

        public boolean isOffensive() {
            return offensive;
        }
    }

    ;

    public AutoPara autoPara = null;

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
    public ElapsedTime runtimeAuto = new ElapsedTime();
    public double rotateRatio = 0.7; // slow down ratio for rotation
    public double motor_count = 0;
    public double auto_chassis_power = .7;
    public double auto_chassis_power_slow = .5;
    public double auto_chassis_align_power = .22;


    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, AutoTeamColor autoColor) {
        runtime.reset();
        double ini_time = runtime.seconds();
        this.telemetry = telemetry;

        autoPara = new AutoPara();

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
        if (autoColor == AutoTeamColor.DIAGNOSIS) {
            foundationHook.hookUp();
        }

        stoneGrabber = new StoneGrabber(this.core).configureLogging("StoneGrabber", logLevel);
        stoneGrabber.configure(configuration, (autoColor != AutoTeamColor.NOT_AUTO));
        intake = new IntakeV3(this.core).configureLogging("IntakeV3", logLevel);
        intake.configure(configuration, (autoColor != AutoTeamColor.NOT_AUTO));

    }

    @Override
    public void reset(boolean auto) {
        reset(auto, false);
    }

    public void reset(boolean auto, boolean armOut) {
        if (Thread.currentThread().isInterrupted()) return;
        if (chassis != null) {
            chassis.reset();
            if (auto) {
                // Display all sensors in auto only for debugging
                // chassis.setupTelemetry(telemetry);
            } else {
                chassis.changeChassisDrivingDirection();
            }
        }
        if (foundationHook != null) {
            foundationHook.reset(auto);
        }
        if (stoneGrabber != null)
            stoneGrabber.reset(auto, armOut);

        if (intake != null) {
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
        if (chassis != null)
            chassis.setupTelemetry(telemetry);
        if (stoneGrabber != null)
            stoneGrabber.setupTelemetry(telemetry);
        if (intake != null)
            intake.setupTelemetry(telemetry);
        if (foundationHook != null)
            foundationHook.setupTelemetry(telemetry);

        em.updateTelemetry(telemetry, 100);

        // -----------------------------------
        // Teleop driver-1 control:
        // ------------------------------------
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.BOTH)) < 0.1) {
                    // right stick with idle left stick operates robot in "crab" mode
                    double power = Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH));
                    power *= power; // square power to stick
                    double heading = toDegrees(currentX, currentY);
                    if (chassis.isReversed()) power *= -1;
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
                    if (chassis.isReversed()) power *= -1;
                    // chassis.orbit(power * powerAdjustment(source), curvature);
                } else {
                    // right stick with left stick operates robot in "car" mode
                    double heading = source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY) * 90;
                    double power = currentY * Math.abs(currentY);
                    if (chassis.isReversed()) power *= -1;
                    debug("sticksOnly(): right / steer, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH)) < 0.2) {
                    if (Math.abs(currentX) > 0.1) {
                        // left stick with idle right stick rotates robot in place
                        chassis.rotate(currentX * Math.abs(currentX) * powerAdjustment(source) * rotateRatio);
                    } else {
                        chassis.stop();
                    }
                }
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                double power = (source.isPressed(Button.RIGHT_BUMPER) ? -auto_chassis_power_slow : -auto_chassis_power);
                double curvature = Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY));
                chassis.orbit(power, curvature, source.isPressed(Button.START));
            }
        }, Button.DPAD_LEFT);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                chassis.stop();
            }
        }, Button.DPAD_LEFT);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                double power = (source.isPressed(Button.RIGHT_BUMPER) ? auto_chassis_power_slow : auto_chassis_power);
                double curvature = Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY));
                chassis.orbit(power, curvature, source.isPressed(Button.START));
            }
        }, Button.DPAD_RIGHT);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                chassis.stop();
            }
        }, Button.DPAD_RIGHT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.A) || source.isPressed(Button.B) || source.isPressed(Button.Y) || source.isPressed(Button.X)) {
                    if (intake != null) intake.intakeStop();
                    return;
                }
                if (intake != null) {
                    intake.intakeIn(!source.isPressed(Button.BACK));
                    chassis.setDefaultScale(chassis.DEFAULT_SLOW_SCALE);
                }
            }
        }, Button.LEFT_BUMPER);

        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                if (intake != null) {
                    intake.intakeStop();
                    chassis.setDefaultScale(chassis.DEFAULT_FAST_SCALE);
                }
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

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) { // default scale up
                    chassis.setDefaultScale(1.0);
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    stoneGrabber.armInReadyGrabCombo();
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    chassis.incDefaultScale();
                }
            }
        }, Button.Y);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.Y) && source.isPressed(Button.BACK)) {
                    // intake drop In/out
                    if (intake != null) intake.intakeDropAuto();
                } else if (source.isPressed(Button.BACK)) { // default scale toggle slow and fast
                    chassis.toggleSlowMode();
                } else if (source.isPressed(Button.LEFT_BUMPER)) { // same for driver-2 grab stone combos
                    if (stoneGrabber.isArmInside())
                        stoneGrabber.grabStoneInsideCombo();
                    else
                        stoneGrabber.grabStoneCombo();
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    chassis.decDefaultScale();
                } else if (!source.isPressed(button.START)) {
                    foundationHook.hookAuto();
                }
            }
        }, Button.A);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) { // back-X swap driving direction
                    chassis.changeChassisDrivingDirection();
                } else if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armInCombo(!source.isPressed(Button.BACK), false);
                else
                    stoneGrabber.grabberAuto();
            }
        }, new Button[]{Button.X});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armOutCombo(0, false);
                else if (!source.isPressed(Button.START))
                    stoneGrabber.wristAuto();
            }
        }, new Button[]{Button.B});


        // ---------------------------------
        // Teleop Driver-2 control:
        // ---------------------------------

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.Y) && source.isPressed(Button.BACK)) {
                    // intake drop In/out
                    if (intake != null) intake.intakeDropAuto();
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (stoneGrabber.isArmInside())
                        stoneGrabber.grabStoneInsideCombo();
                    else
                        stoneGrabber.grabStoneCombo();
                } else if (source.isPressed(Button.BACK)) // reset lifter encoder to 0
                    stoneGrabber.liftResetEncoder();
            }
        }, new Button[]{Button.A});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armInReadyGrabCombo();
                else if (source.isPressed(Button.X))
                    stoneGrabber.releaseStoneCombo();
            }
        }, new Button[]{Button.Y});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armInCombo(!source.isPressed(Button.BACK), false);
                else
                    stoneGrabber.grabberAuto();
            }
        }, new Button[]{Button.X});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER))
                    stoneGrabber.armOutCombo(1.0, true);
                else if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armOutCombo(0, false);
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
                        stoneGrabber.armUpInc(source.isPressed(Button.BACK));
                    else
                        stoneGrabber.armDownInc();
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.LEFT);

        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY)) > 0.2 &&
                        (!source.isPressed(Button.LEFT_BUMPER))) {
                    // left stick with idle right stick rotates robot in place
                    if (source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY) > 0)
                        stoneGrabber.liftUp(source.isPressed(Button.BACK), source.isPressed(Button.RIGHT_BUMPER));
                    else
                        stoneGrabber.liftDown(source.isPressed(Button.BACK), source.isPressed(Button.RIGHT_BUMPER));
                } else
                    stoneGrabber.liftStop();
            }
        }, Events.Axis.Y_ONLY, Events.Side.RIGHT);

        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY)) > 0.2) {
                    // left stick with idle right stick rotates robot in place
                    if (source.isPressed(Button.LEFT_BUMPER)) {
                        if (source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY) < 0)
                            stoneGrabber.wristLeftInc();
                        else stoneGrabber.wristRightInc();
                    }
                }
            }
        }, Events.Axis.X_ONLY, Events.Side.RIGHT);

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER))
                    stoneGrabber.capstoneRightInc();
                else if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.regrabStoneCombo(true);
                else
                    stoneGrabber.capstoneServoAuto();
            }
        }, new Button[]{Button.DPAD_RIGHT});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER))
                    stoneGrabber.capstoneLeftInc();
                else if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.regrabStoneCombo(false);
            }
        }, new Button[]{Button.DPAD_LEFT});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER)) {
                    stoneGrabber.grabCapStoneCombo();
                }
            }
        }, new Button[]{Button.DPAD_UP});
    }

    public void setupTelemetry(Telemetry telemetry) {
        if (Thread.currentThread().isInterrupted()) return;
        Telemetry.Line line = telemetry.addLine();
        line.addData(" | Team", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n",  (autoPara.isBlue() ? "Blue" : "Red"));
            }
        });
        line.addData("Lane", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n",  (autoPara.isLaneFront() ? "Front" : "Back"));
            }
        });

        line.addData("Offensive", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n",  (autoPara.isOffensive() ? "Yes" : "No"));
            }
        });

    }

    @MenuEntry(label = "Auto Backup", group = "Competition")
    public void AutoBackup(EventManager em) {
        telemetry.addLine().addData(" <A|B|Y|X>", "Lane/Team/Offsesive/Done").setRetained(true);
        setupTelemetry(telemetry);

        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                autoPara.laneFront = !autoPara.laneFront;
            }
        }, new Button[]{Button.A});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                autoPara.offensive = !autoPara.offensive;
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                autoPara.isDone = true;
            }
        }, new Button[]{Button.X});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                autoPara.isBlue = !autoPara.isBlue;
            }
        }, new Button[]{Button.B});

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
            chassis.tl = telemetry;
            //chassis.driveStraightAutoRunToPosition(.4, 200, 0, 10000, telemetry);
            chassis.tl = telemetry;
            chassis.driveStraightAutoRunToPosition(.6, 150, -90, 10000);
            sleep(500);
            chassis.driveStraightAutoRunToPosition(.6, 150, 90, 10000);
            sleep(500);
            chassis.driveStraightAutoRunToPosition(.6, 150, 0, 10000);
            sleep(500);
            chassis.driveStraightAutoRunToPosition(.6, -150, 0, 10000);
        } catch (InterruptedException e) {

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

    @MenuEntry(label = "Wheel Intake", group = "Test Chassis")
    public void testWheelIntake(EventManager em) {
        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                intake.intakeDropAuto();
                chassis.driveStraightAutoRunToPosition(auto_chassis_power, 0, -40, 2000);
                chassis.rotateTo(.5, 45);
                chassis.driveStraightAutoRunToPosition(auto_chassis_power, 0, -20, 2000);
                intake.intakeIn(true);
                chassis.driveStraightAutoRunToPosition(auto_chassis_power, 0, 20, 2000);

            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.Y) && source.isPressed(Button.BACK)) {
                    // intake drop In/out
                    if (intake != null) intake.intakeDropAuto();
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (stoneGrabber.isArmInside())
                        stoneGrabber.grabStoneInsideCombo();
                    else
                        stoneGrabber.grabStoneCombo();
                } else if (source.isPressed(Button.BACK)) // reset lifter encoder to 0
                    stoneGrabber.liftResetEncoder();
            }
        }, new Button[]{Button.A});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armInReadyGrabCombo();
                else if (source.isPressed(Button.X))
                    stoneGrabber.releaseStoneCombo();
                else if (source.isPressed(Button.BACK)) {
                    if (stoneGrabber != null) stoneGrabber.deliverStoneThrowCombo();
                }
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armInCombo(!source.isPressed(Button.BACK), false);
                else
                    stoneGrabber.grabberAuto();
            }
        }, new Button[]{Button.X});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER))
                    stoneGrabber.armOutCombo(1.0, true);
                else if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armOutCombo();
                else if (!source.isPressed(Button.START))
                    stoneGrabber.wristAuto();
            }
        }, new Button[]{Button.B});
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

    public void waitAndProcessTask(long timeMillis){
        long iniTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - iniTime < timeMillis) {
            TaskManager.processTasks();
        }
    }

    //assume: arm is out, grabber is open
    public double grabStoneAndDeliverOnFoundation(double stoneX, boolean firstStone, boolean isBlue) throws InterruptedException {
        if (firstStone) {
//            stoneGrabber.grabStoneComboAuto();
            stoneGrabber.grabStoneComboHigher();
            while (!TaskManager.isComplete("Grab Stone Combo")) {
                TaskManager.processTasks();
            }
            //====================parallelized region===================
//        stoneGrabber.armInCombo(true, true);
//            chassis.driveStraightAuto(.35, isBlue ? -7 : -7, 0, 1000);
        } else {

            double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT),chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT));
            if (dist - 14 > 1) {
                chassis.driveStraightAuto(.35, dist - 14, 0, 1000);////?
            } else if (dist - 14 < -1) {
                chassis.driveStraightAuto(.35, dist - 9, 0, 1000);////?
            }
            stoneGrabber.grabberOpenAuto();
            stoneGrabber.grabStoneComboHigher();
            while (!TaskManager.isComplete("Grab Stone Combo")) {
                TaskManager.processTasks();
            }
            //====================parallelized region===================
//            chassis.driveStraightAutoRunToPosition(.35, firstStone ? -8 : -10, 0, 1000);

        }
        double distanceToBack = chassis.getDistance(SwerveChassis.Direction.BACK);
        chassis.driveStraightAuto(0.35, 65 - distanceToBack, 0, 1000);
        chassis.rotateTo(.6, isBlue ? -90 : +90);
        stoneGrabber.lifterDownCombo();
        waitAndProcessTask(200);
        chassis.driveStraightAutoRunToPositionNoIMU(0.75, 135 - stoneX, 0, 5000);//used to use IMU
        //==========================arriving before foundation=================================
        
        double disLeft = chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT);
        double disRight = chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT);
        double distanceToFoundation = Math.min(disLeft, disRight) > 70 ? 0 : Math.min(disLeft, disRight);
//        telemetry.addData("distance left",disLeft);
//        telemetry.addData("distance right",disRight);
//        telemetry.addData("dis to foundation",distanceToFoundation);
//        telemetry.update();
//        sleep(100);
        if (distanceToFoundation > 5) {
            chassis.driveStraightAuto(0.5, distanceToFoundation - 5, 0, 2000);
        }

        if (firstStone) {
            stoneGrabber.deliverStoneThrowComboAuto();//throw if first stone
            //stoneGrabber.armInComboAuto(false);//no para
            stoneGrabber.armInCombo(false,true);
            waitAndProcessTask(1000);
        } else {
            stoneGrabber.deliverStoneComboAuto();//don't throw if second stone
            stoneGrabber.armInCombo(false,true);
        }

        //let it finish


//        chassis.rotateTo(auto_chassis_align_power,-90);
        return 135 + distanceToFoundation;
    }

    public int getFirstSkyStone(ToboSigma.SkystoneLocation skyStonePosition, boolean isBlue, boolean isLeft) throws InterruptedException {
        int side = isBlue ? 1 : -1;

        //arm out

        //=================================Parallelized region=======================================
//        stoneGrabber.armOutComboAuto();
        stoneGrabber.armOutCombo();
        chassis.driveStraightAutoRunToPosition(.4, 46, 0, 10000);
        while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }
        //===========================================================================================
        stoneGrabber.grabberOpenAuto();
        // go to stones
        foundationHook.hookUp();

        chassis.rotateTo(.25, 0);
        double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT)) - 12;
        if (skyStonePosition != SkystoneLocation.UNKNOWN)
            dist -= 2;

        if (dist > 17) dist = 17;

        chassis.driveStraightAutoRunToPosition(.3, dist, 0, 1000);

        if (skyStonePosition == SkystoneLocation.UNKNOWN) { // use color sensor to detect the skystone
            chassis.rotateTo(auto_chassis_align_power, 0);
            sleep(200);
            skyStonePosition = chassis.skyStoneLocation(isBlue); // using color sensors need to be close enough to the stones
            chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, -2, 0, 1000);
        }

//        telemetry.addData("skeystone=","%s",skyStonePosition.toString());
//        telemetry.update();
//        core.yield_for(5);

        if (!isBlue) { // Red side
            if ((skyStonePosition == SkystoneLocation.LEFT) || skyStonePosition == SkystoneLocation.UNKNOWN) {
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 13, 90 * side, 3000);  // test to get exact numbers
            } else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER) {
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 7, -90 * side, 3000);  // test to get exact numbers
            } else { // if (skyStonePosition == ToboSigma.SkystoneLocation.RIGHT) {
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 27, -90 * side, 3000);  // test to get exact numbers
            }
        } else { // Blue side
            if (skyStonePosition == ToboSigma.SkystoneLocation.RIGHT || skyStonePosition == SkystoneLocation.UNKNOWN) {
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 8, 90 * side, 3000);  // test to get exact numbers
            } else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER) {
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 13, -90 * side, 3000); // test to get exact numbers
            } else { // skyStonePosition == ToboSigma.SkystoneLocation.LEFT
                chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, 33, -90 * side, 3000);  // test to get exact numbers
            }
        }

        //grab stone
        if (Thread.currentThread().isInterrupted()) return 0;

        stoneGrabber.grabStoneComboAuto();
        //chassis.driveStraightAutoRunToPosition(.3, -8, 0, 1000);

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


        // go to foundation


//=================================Parallelized region=======================================
//        stoneGrabber.armInComboAuto(true);
        stoneGrabber.armInCombo(true, true);
        chassis.driveStraightAutoRunToPosition(.3, -13, 0, 1000);
        chassis.rotateTo(auto_chassis_align_power, 0);
        if (ss_pos == 1) {
            long iniTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - iniTime < 250) {
                TaskManager.processTasks();
            }
        } else if (ss_pos == 2) {
            long iniTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - iniTime < 150) {
                TaskManager.processTasks();
            }
        }
        chassis.driveStraightAutoRunToPosition(.6, 160 + 20 * ss_pos, -90 * side, 15000);
        while (!TaskManager.isComplete("Arm In Combo")) {
            TaskManager.processTasks();
        }
//===========================================================================================
        //align

        chassis.rotateTo(auto_chassis_align_power, 0);

        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        dist = dist > 40 ? 35 : dist;
        if (dist - 30 > 5) {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, dist - 30, -90 * side, 15000);
        }
        if (Thread.currentThread().isInterrupted()) return 0;

//=================================Parallelized region=======================================

        return ss_pos;
    }


    public void approachStone(ToboSigma.SkystoneLocation skyStonePosition, boolean isBlue, boolean isLeft) throws InterruptedException {
        //=================================Parallelized region=======================================
//        stoneGrabber.armOutComboAuto();
        stoneGrabber.armOutCombo();
        chassis.driveStraightAutoRunToPosition(.6, 46, 0, 10000);
        while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }
        //===========================================================================================
        stoneGrabber.grabberOpenAuto();
        // go to stones
        foundationHook.hookUp();

//        chassis.rotateTo(.25, 0);
        double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT)) - 12;
//        if (skyStonePosition != SkystoneLocation.UNKNOWN)
//            dist -= 2;
//        if (dist > 17) dist = 17;
//        chassis.driveStraightAutoRunToPosition(.6, dist, 0, 1000);
        if (dist - 14 > 1) {
            chassis.driveStraightAuto(.35, dist - 14, 0, 1000);////?
        } else if (dist - 14 < -1) {
            chassis.driveStraightAuto(.35, dist - 9, 0, 1000);////?
        }
    }

    //assumption: arm is out when the function is called
    public void oneMoreStone(boolean onQarry, double initialX, double stoneX, double dumpingX, int StoneId) throws InterruptedException {
        if (!onQarry) {
            //if in building zone, retract arm to pass sky bridge
            stoneGrabber.armInComboAuto(false);
        }

        if (!onQarry) {
            //if in building zone, passing the bridge and take out the arm midway
            //===================parallelized region===================
            stoneGrabber.armOutCombo();
            chassis.driveStraightAutoRunToPosition(0.75, Math.abs(initialX - stoneX), initialX > stoneX ? 90 : -90, StoneId == 5 ? 0.6 : 0.4, 5000);
            while (!TaskManager.isComplete("Arm Out Combo")) {
                TaskManager.processTasks();
            }
            //=========================================================
            stoneGrabber.grabberOpenAuto();
            chassis.driveStraightAutoRunToPosition(.35, 13, 0, 1000);
        } else {
            chassis.driveStraightAutoRunToPosition(0.75, Math.abs(initialX - stoneX), initialX > stoneX ? 90 : -90, 5000);
        }

        stoneGrabber.grabStoneComboAuto();
        //====================parallelized region===================
        stoneGrabber.armInCombo(true, true);
        chassis.driveStraightAutoRunToPosition(.35, -13, 0, 1000);
//        chassis.rotateTo(auto_chassis_align_power, 0);
        long iniTime = System.currentTimeMillis();
        if (StoneId == 5 || StoneId == 4)
            while (System.currentTimeMillis() - iniTime < 150) {
                TaskManager.processTasks();
            }
        chassis.driveStraightAutoRunToPosition(0.75, dumpingX - stoneX, -90, 5000);
        //===========================================================

        stoneGrabber.armOutComboAuto();
        stoneGrabber.deliverStoneComboAuto();
    }

    public void getAnotherSkyStone(int ss_pos, int stoneNum, boolean isBlue) throws InterruptedException {//stoneNum - how many stones ara we going to have after this trip
        int side = isBlue ? 1 : -1;

        int toTake;
        if (ss_pos == 3) {
            int[] a = {5, 1, 2, 4, 6};
            toTake = a[stoneNum - 2];
        } else if (ss_pos == 2) {
            int[] a = {5, 1, 3, 4, 6};
            toTake = a[stoneNum - 2];
        } else { // left or unknown
            int[] a = {4, 2, 3, 5, 6};
            toTake = a[stoneNum - 2];
        }
        //telemetry.addData("ssPos",ss_pos);
        //telemetry.addData("toTake",toTake);
        // stoneGrabber.armInComboAuto(false);// ASK CHARLIE
        // chassis.driveStraightAutoRunToPosition(.6, 55, 0, 1500);
        chassis.rotateTo(auto_chassis_align_power, 0);
        stoneGrabber.armOutCombo(2.0, true);
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 210, 90 * side, 5000);
        stoneGrabber.grabberOpenAuto();
        chassis.rotateTo(auto_chassis_align_power, 0);
        double dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.RIGHT : SwerveChassis.Direction.LEFT);
        //  telemetry.addData("distance", dist);
        //telemetry.addData("goLeft",-dist - 10 + 20 * (6 - toTake));
        //telemetry.addData("goLeft",telemetry.addData("goLeft",-dist - 10 + 20 * (6 - toTake)));
        //telemetry.update();
        chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, -dist - 10 + 20 * (6 - toTake), -90 * side, 15000);
        dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT));
        chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, dist - 13, 0, 10000);

        //stoneGrabber.grabberOpen();
        //grab stone
        stoneGrabber.grabStoneComboAuto();
        stoneGrabber.armInCombo(true, true);
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, -5, 0, 1000);
        chassis.rotateTo(auto_chassis_align_power, 0);
        chassis.driveStraightAutoRunToPosition(.7, 100 + 20 * (6 - toTake), -90 * side, 15000);
        stoneGrabber.armOutComboAuto();
        stoneGrabber.deliverStoneComboAuto();
        stoneGrabber.armInCombo(true, true);
        long iniTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - iniTime < 500) {
            TaskManager.processTasks();
        }
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 40, 90 * side, 15000);

    }

    public void repositioning(boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        foundationHook.hookDown();

        Thread.sleep(400);
        double dist = Math.min(130, Math.max(chassis.getDistance(SwerveChassis.Direction.BACK), 115));

        //chassis.driveStraightAutoRunToPosition(auto_chassis_power/2, -dist - 10*(1- auto_chassis_power) , 7* side, 10000);
        //=================================Parallelized region=======================================
//        stoneGrabber.deliverStoneComboAuto();
        stoneGrabber.deliverStoneCombo(true);
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, -dist, 7 * side, 4000);//        dist = chassis.getDistance(SwerveChassis.Direction.BACK);
        while (!TaskManager.isComplete("Deliver Stone Combo")) {
            TaskManager.processTasks();
        }
        //===========================================================================================
        foundationHook.hookUp();

        chassis.rotateTo(auto_chassis_align_power, 0);

        dist = chassis.getDistance(SwerveChassis.Direction.BACK);
        if (dist > 1) {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, -Math.min(dist, 10), 0, 300);
        }

        dist = isBlue ? chassis.getDistance(SwerveChassis.Direction.LEFT) : chassis.getDistance(SwerveChassis.Direction.RIGHT);

        if (dist > 128) {
            dist = 20;
        }

        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 90 - dist, 90 * side, 1500);
        // stoneGrabber.armInComboAuto(false);
    }


    public void grabAndPark(boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;

        foundationHook.hookDown();
        sleep(400);
        double dist = Math.max(chassis.getDistance(SwerveChassis.Direction.BACK), 118);

        //chassis.driveStraightAutoRunToPosition(auto_chassis_power/2, -dist - 10*(1- auto_chassis_power) , 7* side, 10000);
        //=================================Parallelized region=======================================
//        stoneGrabber.deliverStoneComboAuto();
        stoneGrabber.deliverStoneCombo(true);
//        chassis.driveStraightAutoRunToPosition(0.7, -1.05*dist/Math.cos(Math.PI*7.0/180.0), 7 * side, 5000);
        chassis.driveStraightAutoRunToPositionNoIMU(.4, -dist, 5 * side, 5000);//        dist = chassis.getDistance(SwerveChassis.Direction.BACK);
//        chassis.driveStraightAutoRunToPosition(0.6, -dist, 0, 5000);
        while (!TaskManager.isComplete("Deliver Stone Combo")) {
            TaskManager.processTasks();
        }
        //===========================================================================================

        foundationHook.hookUp();

        chassis.rotateTo(.25, 0);

        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);

        if (dist > 128) {
            dist = 20;
        }

        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 100 - dist, 90 * side, 1500);
        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        if (dist < 95) {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, Math.min(50, 95 - dist), 90 * side, 1500);
            dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        }
        stoneGrabber.armInComboAuto(false);

        if ((isBlue && chassis.getDistance(SwerveChassis.Direction.RIGHT) > 60) || (!isBlue && chassis.getDistance(SwerveChassis.Direction.LEFT) > 60)) {
            //chassis.driveStraightAutoRunToPosition(.4, 40, 90* side, 1500);
            chassis.driveStraightAutoRunToPosition(.6, 55, 90 * side, 1500);
        } else {
            chassis.driveStraightAutoRunToPosition(.6, 50, 0, 1500);
            chassis.rotateTo(auto_chassis_align_power, 0);
            chassis.driveStraightAutoRunToPosition(.6, 55, 90 * side, 1500);
        }

    }

    public void rotateFoundation(boolean isBlue, boolean moveArm) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        if (moveArm)
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, isBlue ? 23 : 28, 0, 1000);
        foundationHook.hookDown();
        chassis.driveStraightSec(0.25, 0.3, false);
        if (moveArm)
            stoneGrabber.armOutCombo();
        chassis.driveStraightAutoRunToPositionNoIMU(auto_chassis_power, -40, -45 * side, 1500);
        /*while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }*/
        //stoneGrabber.wristPerpendicular();
        chassis.changeStopBehavior(false);
        chassis.driveStraightSec(-.5, .2, true);
        if (moveArm)
            stoneGrabber.deliverStoneCombo(true);
        chassis.rawRotateTo(.65, -90 * side, true, 3000);
        //stoneGrabber.deliverStoneCombo(true);
        chassis.changeStopBehavior(true);
        chassis.driveStraightAutoRunToPosition(.6, 35, 0, 1000);
        foundationHook.hookUp();
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, -10, 0, 1500);
        if (moveArm) {
            stoneGrabber.armInCombo(false, true);
            chassis.rotateTo(.5, 0);
            double dist = Math.max(10, Math.min(70, chassis.getDistance(SwerveChassis.Direction.BACK)));
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, 53 - dist, 0, 1500);
            while (!TaskManager.isComplete("Deliver Stone Combo")) {
                TaskManager.processTasks();
            }
        }

    }


    public void parkAfterRotate(boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        chassis.driveStraightAutoRunToPosition(.4, 85, side * 90, 3000);

    }

    public int getFirstSkyStoneDefense(ToboSigma.SkystoneLocation skyStonePosition, boolean isBlue, boolean safe) throws InterruptedException {
        int side = isBlue ? 1 : -1;
        stoneGrabber.armOutCombo();
        chassis.driveStraightAutoRunToPosition(.6, 46, 0, 10000);
        stoneGrabber.grabberOpenAuto();
        double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT)) - 12;
        if (skyStonePosition != SkystoneLocation.UNKNOWN)
            dist -= 2;
        if (dist > 19) dist = 19;
        while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }

        // go to stones
        foundationHook.hookUp();
        chassis.driveStraightAutoRunToPositionNoIMU(.6, dist, 0, 1000);

        if (skyStonePosition == SkystoneLocation.UNKNOWN) { // use color sensor to detect the skystone
            chassis.rotateTo(auto_chassis_align_power, 0);
            sleep(200);
            skyStonePosition = chassis.skyStoneLocation(isBlue); // using color sensors need to be close enough to the stones
            chassis.driveStraightAutoRunToPosition(auto_chassis_power_slow, -2, 0, 500);
        }

//        telemetry.addData("skeystone=","%s",skyStonePosition.toString());
//        telemetry.update();
//        core.yield_for(5);

        if (!isBlue) { // Red side
            if ((skyStonePosition == SkystoneLocation.LEFT) || skyStonePosition == SkystoneLocation.UNKNOWN) {
                chassis.driveStraightAutoRunToPositionNoIMU(auto_chassis_power_slow, 13, 90 * side, 500);  // test to get exact numbers
            } else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER) {
                chassis.driveStraightAutoRunToPositionNoIMU(auto_chassis_power_slow, 7, -90 * side, 700);  // test to get exact numbers
            } else { // if (skyStonePosition == ToboSigma.SkystoneLocation.RIGHT) {
                chassis.driveStraightAutoRunToPositionNoIMU(auto_chassis_power_slow, 27, -90 * side, 1000);  // test to get exact numbers
            }
        } else { // Blue side
            if (skyStonePosition == ToboSigma.SkystoneLocation.RIGHT || skyStonePosition == SkystoneLocation.UNKNOWN) {
                chassis.driveStraightAutoRunToPositionNoIMU(auto_chassis_power_slow, 8, 90 * side, 500);  // test to get exact numbers
            } else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER) {
                chassis.driveStraightAutoRunToPositionNoIMU(auto_chassis_power_slow, 13, -90 * side, 700); // test to get exact numbers
            } else { // skyStonePosition == ToboSigma.SkystoneLocation.LEFT
                chassis.driveStraightAutoRunToPositionNoIMU(auto_chassis_power_slow, 30, -90 * side, 1000);  // test to get exact numbers
            }
        }

        //grab stone
        if (Thread.currentThread().isInterrupted()) return 0;

        stoneGrabber.grabStoneComboAuto();
        //chassis.driveStraightAutoRunToPosition(.3, -8, 0, 1000);

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


        // go to foundation


        //=================================Parallelized region=======================================
        //        stoneGrabber.armInComboAuto(true);
        stoneGrabber.armInCombo(true, true);
        chassis.driveStraightAutoRunToPosition(.7, -10, 0, 1000);
        chassis.rotateTo(auto_chassis_align_power, 0);
        if (ss_pos == 1) {
            long iniTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - iniTime < 450) {
                TaskManager.processTasks();
            }
        } else if (ss_pos == 2) {
            long iniTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - iniTime < 250) {
                TaskManager.processTasks();
            }
        }
        // the following step acrossing the bridge
        chassis.driveStraightAutoRunToPosition(.7, 90 + 20 * ss_pos, -90 * side, 15000);
        while (!TaskManager.isComplete("Arm In Combo")) {
            TaskManager.processTasks();
        }
        chassis.rotateTo(auto_chassis_align_power, 0);
        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        if (safe) {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, -10, 0, 2000);
        }
        int dForward = 0;
        if (safe) {
            dForward = 10;
        }
        if (dist < 40) {
            dForward = findFoundtaion(isBlue);
            dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        }
        dist = Math.min(dist-20, 55); // min for 55 cm as the range sensor is not accurate for long distance (over 50 cm) here.
        if (dist > 3) {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, dist, -90 * side, 3000);
        }
        if (dForward != 0) {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, dForward, 0, 2000);
        }
        if (Thread.currentThread().isInterrupted()) return 0;

        //=================================Parallelized region=======================================

        return ss_pos;
    }


    public int findFoundtaion(boolean isBlue) throws InterruptedException {
        int side = isBlue ? 1 : -1;
        int count = 0;
        while (chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT) < 40) {
            chassis.driveStraightAutoRunToPosition(.8, -10, 0, 1000);
            count++;
            if (count > 6) {
                break;
            }
        }
        chassis.driveStraightAutoRunToPosition(.8, -10, 0, 1000);
        return count * 10 + 30;
    }

    public void getAnotherSkyStoneNew(int ss_pos, int stoneNum, boolean isBlue) throws InterruptedException {//stoneNum - how many stones ara we going to have after this trip
        int side = isBlue ? 1 : -1;

        int toTake;
        //if (ss_pos == 3 && isBlue) {
        //  int[] a = {6, 1, 2, 4, 5};
        //toTake = a[stoneNum - 2];
        //} else
        if (ss_pos == 2 || (ss_pos == 3)) {
            int[] a = {5, 1, 3, 4, 6};
            toTake = a[stoneNum - 2];
        } else { // left or unknown
            int[] a = {4, 2, 3, 5, 6};
            toTake = a[stoneNum - 2];
        }
        //telemetry.addData("ssPos",ss_pos);
        //telemetry.addData("toTake",toTake);
        // stoneGrabber.armInComboAuto(false);// ASK CHARLIE
        // chassis.driveStraightAutoRunToPosition(.6, 55, 0, 1500);
        //chassis.rotateTo(auto_chassis_align_power, 0);
        stoneGrabber.armOutCombo(1, true);
        if (Thread.interrupted()) return;
        stoneGrabber.moveWrist(false);
        if (Thread.interrupted()) return;
        chassis.driveStraightAutoRunToPosition(.8, 210, 90 * side, 5000);
        if (Thread.interrupted()) return;
        stoneGrabber.grabberOpenAuto();
        if (Thread.interrupted()) return;
        chassis.rotateTo(auto_chassis_align_power, 0);
        if (Thread.interrupted()) return;
        double dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.RIGHT : SwerveChassis.Direction.LEFT);
        //  telemetry.addData("distance", dist);
        //telemetry.addData("goLeft",-dist - 10 + 20 * (6 - toTake));
        //telemetry.addData("goLeft",telemetry.addData("goLeft",-dist - 10 + 20 * (6 - toTake)));
        //telemetry.update();
        int timeout = 1000;
        if (isBlue) {
            if (toTake != 6) {
                chassis.driveStraightAutoRunToPositionNoIMU(.5, -dist - 5 + 20 * (6 - toTake), -90 * side, timeout);
            } else {
                chassis.driveStraightAutoRunToPositionNoIMU(.5, -dist, -90 * side, timeout);
            }
        } else {
            if (toTake != 6) {
                chassis.driveStraightAutoRunToPositionNoIMU(.5, -dist - 15 + 20 * (6 - toTake), -90 * side, timeout);
            } else {
                chassis.driveStraightAutoRunToPositionNoIMU(.5, -dist, -90 * side, timeout);
            }
        }
        if (Thread.interrupted()) return;
        dist = (Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT)));
        if (Thread.interrupted()) return;
        if (dist - 14 > 1) {
            chassis.driveStraightAuto(.35, dist - 14, 0, 1000);////?
        } else if (dist - 14 < -1) {
            chassis.driveStraightAuto(.35, dist - 9, 0, 1000);////?
        }
        if (toTake == 6) {
            chassis.rotateTo(.4, 10 * side, 50);
        }
        if (Thread.interrupted()) return;
        stoneGrabber.grabStoneComboAutoHigher();
        if (Thread.interrupted()) return;
        if (toTake == 6) {
            chassis.driveStraightAutoRunToPosition(.8, 20, -90 * side, 1000);
        }
        if (Thread.interrupted()) return;
        chassis.rotateTo(.6, -90 * side);
        if (Thread.interrupted()) return;
        sleep(200);
        if (Thread.interrupted()) return;
        stoneGrabber.lifterDownCombo();
        if (Thread.interrupted()) return;
        foundationHook.hookDown();
        //stoneGrabber.grabberOpen();
        if (toTake != 6) {
            chassis.driveStraightAutoRunToPositionNoIMU(.8, 120 + 20 * (toTake - 3), 0, 15000);
        } else {
            chassis.driveStraightAutoRunToPositionNoIMU(.8, 100 + 20 * (toTake - 3), 0, 15000);
        }
        if (Thread.interrupted()) return;
        stoneGrabber.grabberOpenAuto();
        if (Thread.interrupted()) return;
        chassis.driveStraightAutoRunToPosition(.8, -40, 0, 15000);

    }

    public void rotateFoundationNew(boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT));
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, dist + 5, 0, 1000);//isBlue ? 23 : 28
        foundationHook.hookDown();
        chassis.driveStraightSec(0.25, 0.2, false);

        stoneGrabber.armOutCombo();
        chassis.driveStraightAutoRunToPositionNoIMU(.8, -50, -50 * side, 1500);//.7?
        /*while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }*/
        //stoneGrabber.wristPerpendicular();
        chassis.changeStopBehavior(false);
        chassis.driveStraightSec(-.7, .2, true);
        stoneGrabber.deliverStoneCombo(true);
        chassis.rawRotateTo(.8, -85 * side, true, 3000);
        //stoneGrabber.deliverStoneCombo(true);
        chassis.changeStopBehavior(true);
        chassis.driveStraightAutoRunToPosition(.9, 30, 0, 1000);
        foundationHook.hookUp();
        chassis.driveStraightAutoRunToPosition(.9, -10, 0, 1500);
        stoneGrabber.armInCombo(false, true);
        stoneGrabber.grabberClose();
        chassis.rotateTo(.7, 0);
        sleep(100);
        dist = Math.max(10, Math.min(70, chassis.getDistance(SwerveChassis.Direction.BACK)));
        if (isBlue) {
            chassis.driveStraightAuto(.5, 60 - dist, 0, 1000);
        } else {
            chassis.driveStraightAuto(.5, 55 - dist, 0, 1000);
        }
        chassis.rotateTo(auto_chassis_align_power, 0);
        //while (!TaskManager.isComplete("Deliver Stone Combo")) {
        //  TaskManager.processTasks();
        //}

    }

    public void rotateFoundationOnly(boolean isBlue, boolean l2) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT));
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, dist + 5, 0, 1000);//isBlue ? 23 : 28
        foundationHook.hookDown();
        chassis.driveStraightSec(0.25, 0.2, false);
        chassis.driveStraightAutoRunToPositionNoIMU(.8, -50, -50 * side, 1500);//.7?
        chassis.changeStopBehavior(false);
        chassis.driveStraightSec(-.7, .2, true);
        chassis.rawRotateTo(.8, -85 * side, true, 3000);
        chassis.changeStopBehavior(true);
        chassis.driveStraightAutoRunToPosition(.9, 30, 0, 1000);
        foundationHook.hookUp();
        chassis.driveStraightAutoRunToPosition(.9, -10, 0, 1500);

        dist = Math.min(75, chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT));
        if (l2) {
            chassis.driveStraightAutoRunToPosition(.6, dist, -90 * side, 5000);
        } else {
            chassis.driveStraightAutoRunToPosition(.6, dist - 65, -90 * side, 5000);
        }
        chassis.driveStraightAutoRunToPosition(.6, -80, 0, 5000);


    }

    public void parkAfterRotateNew(boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        chassis.driveStraightAutoRunToPosition(.4, 85, side * 90, 3000);

    }

    public void autoFoundationOnly(boolean isBlue, boolean laneFront, boolean offensive) throws InterruptedException {
        int side = isBlue ? 1 : -1;
        chassis.driveStraightAutoRunToPosition(auto_chassis_power,20,-90*side,1500);
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 70, 0, 1500);
        if (offensive) {
            foundationHook.hookDown();
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, 70, 0, 1500);
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, -70, 0, 1500);
        }
        chassis.rotateTo(.6,0);
        double dist2 = chassis.getDistance(SwerveChassis.Direction.BACK);
        if(dist2>128)dist2=70;
        chassis.driveStraightAutoRunToPosition(auto_chassis_power,70-dist2,0,1500);
        rotateFoundation(isBlue, false);
        double dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        if (dist > 45) dist = 45;
        if (laneFront) {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, 60 - dist, 90 * side, 1500);
        } else {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, 10 - dist, 90 * side, 1500);
        }
        chassis.driveStraightAutoRunToPositionNoIMU(.4, -85, 0, 3000);
    }

    public void getFoundation(boolean isBlue, boolean laneTwo) throws InterruptedException {
        int side = isBlue ? 1 : -1;
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 70, 0, 1500);
        rotateFoundationOnly(isBlue, laneTwo);
    }
}
