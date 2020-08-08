package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.MechChassis;
import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import static java.lang.Thread.sleep;

public class ToboMech extends Logger<ToboMech> implements Robot2 {
    private Telemetry telemetry;
    public MechChassis chassis;
    public CoreSystem core;
    public ElapsedTime runtime = new ElapsedTime();
    public double rotateRatio = 0.7; // slow down ratio for rotation


    public double auto_chassis_power = .4;
    public double auto_chassis_dist = 150;
    public double auto_chassis_heading = -90;
    public double auto_chassis_power_slow = .2;
    public double auto_chassis_align_power = .22;

    public double auto_rotate_degree = 90;



    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, ToboSigma.AutoTeamColor autoColor) {
        runtime.reset();
        double ini_time = runtime.seconds();
        this.telemetry = telemetry;

        this.core = new CoreSystem();
        info("RoboRuck configure() after new CoreSystem()(run time = %.2f sec)", (runtime.seconds() - ini_time));
        chassis = new MechChassis(core).configureLogging("Mecanum", logLevel); // Log.DEBUG
        if (autoColor== ToboSigma.AutoTeamColor.DIAGNOSIS) {
            // enable imu for diagnosis
            chassis.enableImuTelemetry(configuration);
        }
        chassis.configure(configuration, (autoColor!= ToboSigma.AutoTeamColor.NOT_AUTO));
        info("RoboRuck configure() after init Chassis (run time = %.2f sec)", (runtime.seconds() - ini_time));
    }


    @Override
    public void reset(boolean auto) {
        chassis.reset();
        if (auto) {
            chassis.setupTelemetry(telemetry);
        }
    }


    @MenuEntry(label = "TeleOp", group = "Test Chassis")
    public void mainTeleOp(EventManager em) {
        //telemetry.addLine().addData("(LS)", "Drive").setRetained(true)
        //        .addData("Hold [LB]/[RB]", "45 degree").setRetained(true);
        // chassis.setupTelemetry(telemetry);
        Thread positionThread;
        if (chassis!=null && chassis.globalPositionUpdate()==null) {
            chassis.configureOdometry();
            chassis.setupTelemetry(telemetry);
            positionThread = (chassis.globalPositionUpdate()==null? null: new Thread(chassis.globalPositionUpdate()));
            if (positionThread!=null)
                positionThread.start();
        }
        setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 1000);
        em.onStick(new Events.Listener() { // Left-Joystick
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY))> 0.2 )
                    return; // avoid conflicting drives
                double normalizeRatio = 0.43;
                // Left joystick for forward/backward and turn
                if (Math.abs(currentY)>0.2) { // car mode
                    chassis.carDrive(currentY*Math.abs(currentY) * normalizeRatio, currentX);
                }else if (Math.abs(currentX) > 0.2) {
                    chassis.turn((currentX > 0 ? 1 : -1), Math.abs(currentX * currentX) * chassis.powerScale());
                } else if (Math.abs(currentY)>0.2) {
                    chassis.yMove((currentY>0?1:-1), Math.abs(currentY * currentY) * chassis.powerScale() * normalizeRatio);
                } else {
                    chassis.stop();
                }
            }
        }, Events.Axis.BOTH, Events.Side.LEFT);

        em.onStick(new Events.Listener() { // Right-Joystick
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                double movingAngle = 0;
                double normalizeRatio = 0.43; // minimum 0.43 when moving forward, and maximum 1.0 when crabbing 90 degree
                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY))>0.2 )
                    return; // avoid conflicting drives
                double left_x = source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY);
                // right joystick for free crabbing
                if (Math.abs(left_x)>0.1 && Math.abs(currentY)>0.1) {
                    // car drive
                    chassis.carDrive(currentY*Math.abs(currentY)*normalizeRatio, left_x);
                } else if (Math.abs(currentX)+Math.abs(currentY)>0.2) {
                    movingAngle = Math.toDegrees(Math.atan2(currentX, currentY));
                    if (movingAngle>=-90 && movingAngle<=90) {
                        normalizeRatio = 0.43 + 0.57 * (Math.abs(movingAngle)/90.0);
                    } else { // movingAngle is < -90 or > 90
                        normalizeRatio = 0.43 + 0.57 * ((180-Math.abs(movingAngle))/90.0);
                    }
                    double lsx = currentX;
                    double lsy = currentY;
                    double power_lf = (lsy+lsx) * normalizeRatio;
                    double power_lb = (lsy-lsx) * normalizeRatio;
                    double power_rf = (lsy-lsx) * normalizeRatio;
                    double power_rb = (lsy+lsx) * normalizeRatio;

                    power_lf = Range.clip(power_lf, -1, 1);
                    power_lb = Range.clip(power_lb, -1, 1);
                    power_rf = Range.clip(power_rf, -1, 1);
                    power_rb = Range.clip(power_rb, -1, 1);

                    power_lf *= Math.abs(power_lf)* chassis.powerScale();
                    power_lb *= Math.abs(power_lb)* chassis.powerScale();
                    power_rf *= Math.abs(power_rf)* chassis.powerScale();
                    power_rb *= Math.abs(power_rb)* chassis.powerScale();
                    chassis.freeStyle(power_lf, power_rf, power_lb, power_rb);
                } else {
                    chassis.stop();
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (chassis!=null) {
                    chassis.forward(0.3, 30, 3);
                    // chassis.forward(1.0, 10, 3);
                }
            }
        }, new Button[]{Button.DPAD_UP});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (chassis!=null) {
                    chassis.forward(0.3, -30, 3);
                }
            }
        }, new Button[]{Button.DPAD_DOWN});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (chassis!=null) {
                    chassis.crab(0.3, 30, 3);
                }
            }
        }, new Button[]{Button.DPAD_RIGHT});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (chassis!=null) {
                    chassis.crab(0.3, -30, 3);
                }
            }
        }, new Button[]{Button.DPAD_LEFT});
    }

    public void setupTelemetryDiagnostics(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        line.addData("Test ", new Func<String>() {
            @Override
            public String value() {
                return String.format("Power=%.2f, dist=%.1f, rotate_degree=%.1f\n",
                        auto_chassis_power, auto_chassis_dist, auto_rotate_degree);
            }
        });
    }

    public void setupTelemetry(Telemetry telemetry) {
        if (Thread.currentThread().isInterrupted()) return;
        Telemetry.Line line = telemetry.addLine();
        if (chassis==null) return;
        /*
        line.addData(" | Odometry (vl,vr,h) =", new Func<String>() {
            @Override
            public String value() {
                return String.format("(%5d,%5d,%5d)\n", chassis.verticalLeftEncoder().getCurrentPosition(),
                        chassis.verticalRightEncoder().getCurrentPosition(),chassis.horizontalEncoder().getCurrentPosition());
            }
        });
        line.addData("Odo (x, y, angle) =", new Func<String>() {
            @Override
            public String value() {
                return String.format("(%4.0f, %4.0f, %4.0f)\n", chassis.odo_x_pos(),chassis.odo_y_pos(),chassis.odo_heading());
            }
        });

        line.addData("Offensive", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n",  (autoPara.isOffensive() ? "Yes" : "No"));
            }
        });
        */
    }


    private double toDegrees(double x, double y) {
        if (x == 0) return y >= 0 ? 0 : 180;
        return Math.atan2(x, y) / Math.PI * 180;
    }


    @MenuEntry(label = "Drive Straight", group = "Test Chassis")
    public void testStraight(EventManager em) {
        Thread positionThread;
        if (chassis!=null && chassis.globalPositionUpdate()==null) {
            chassis.configureOdometry();
            chassis.setupTelemetry(telemetry);
            positionThread = (chassis.globalPositionUpdate()==null? null: new Thread(chassis.globalPositionUpdate()));
            if (positionThread!=null)
                positionThread.start();
        }
        if (Thread.interrupted()) return;
        telemetry.addLine().addData("(BACK) Y/A", "+/- Power(%.2f)", auto_chassis_power).setRetained(true);
        telemetry.addLine().addData("(BACK) X/B", "+/- heading(%.2f)", auto_rotate_degree).setRetained(true);
        // chassis.setupTelemetry(telemetry);
        setupTelemetryDiagnostics(telemetry);
        // chassis.enableImuTelemetry();
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else {
                    chassis.driveStraight(auto_chassis_power, 120,  auto_rotate_degree, 5);
                }
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power -= 0.1;
                    if (auto_chassis_power < 0.1) auto_chassis_power = 0.1;
                } else {
                    chassis.driveStraightNew(auto_chassis_power, 120,  auto_rotate_degree, 0.8, 5);
                }
            }
        }, new Button[]{Button.A});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree -= 5;
                }
            }
        }, new Button[]{Button.X});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree += 5;
                }
            }
        }, new Button[]{Button.B});
    }

    @MenuEntry(label = "Auto Rotation", group = "Test Chassis")
    public void testRotationSkyStone(EventManager em) {
        Thread positionThread;
        if (chassis!=null && chassis.globalPositionUpdate()==null) {
            chassis.configureOdometry();
            chassis.setupTelemetry(telemetry);
            positionThread = (chassis.globalPositionUpdate()==null? null: new Thread(chassis.globalPositionUpdate()));
            if (positionThread!=null)
                positionThread.start();
        }
        if (Thread.interrupted()) return;
        telemetry.addLine().addData("(BACK) Y/A", "+/- Power(%.2f)", auto_chassis_power).setRetained(true);
        telemetry.addLine().addData("(BACK) X/B", "+/- degree(%.2f)", auto_rotate_degree).setRetained(true);
        chassis.setupTelemetry(telemetry);
        setupTelemetryDiagnostics(telemetry);
        // chassis.enableImuTelemetry();
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else {
                    chassis.rotateToOld(auto_chassis_power, auto_rotate_degree, 5000);
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
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree -= 10;
                    if (auto_rotate_degree > 150) auto_rotate_degree = 150;
                }
            }
        }, new Button[]{Button.X});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree += 10;
                    if (auto_rotate_degree < -150) auto_rotate_degree = -150;
                }
            }
        }, new Button[]{Button.B});

    }

     @MenuEntry(label = "New Auto Straight", group = "Test Chassis")
    public void testStraightNewSkyStone(EventManager em) {
        if (Thread.interrupted()) return;
         Thread positionThread;
         if (chassis!=null && chassis.globalPositionUpdate()==null) {
             chassis.configureOdometry();
             chassis.setupTelemetry(telemetry);
             positionThread = (chassis.globalPositionUpdate()==null? null: new Thread(chassis.globalPositionUpdate()));
             if (positionThread!=null)
                 positionThread.start();
         }
        try {

            chassis.driveStraight(.6, 50, -90, 5);
            sleep(500);
            chassis.driveStraight(.6, 50, 90, 5);
            sleep(500);
            chassis.driveStraight(.6, 50, 0, 5);
            sleep(500);
            chassis.driveStraight(.6, -50, 0, 5);
        } catch (InterruptedException e) {

        }

    }


}
