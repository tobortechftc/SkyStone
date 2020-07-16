/**
 * Put all common hardware configuration and common motor APIs
 * History
 *   1. New for advanced robot control, 2020/02/23 
 */

package org.firstinspires.ftc.teamcode.opmodes.ggMecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

//import com.qualcomm.robotcore.util.Range;


/**
 * Common codes 
 */

//HUB 2 = SIDE HUB
//HUB 3 = BOTTOM HUB
public class Y20CommonEx extends OpMode {


   //static final boolean START_AS_RED = true;  // default as RED
   static final boolean START_AS_RED = false;   // default as BLUE
   boolean color_flipped_ = false;      // flip the default team color, used to select AR programs

   /// Own main timer, reset at START
   ElapsedTime timer_ = new ElapsedTime();
   double curr_time_ = 0.0;        // own timer, reset at START

   int loop_cnt_ = 0;              // loop count for monitoring/debugging

   double battery_voltage_ = 0.0;  // battery monitoring


   /// Enable motor bulk reading (MANUAL mode), 2020/02/23
   enum MotorBulkReadMode { BULK_READ_OFF, BULK_READ_AUTO, BULK_READ_MANUAL };
   //MotorBulkReadMode motor_read_mode_ = MotorBulkReadMode.BULK_READ_OFF;
   MotorBulkReadMode motor_read_mode_ = MotorBulkReadMode.BULK_READ_MANUAL;
   List<LynxModule> all_hubs_ ;
   enum MotorName { MOTOR_LF, MOTOR_LB, MOTOR_RF, MOTOR_RB, MOTOR_LEFT_INTAKE, MOTOR_RIGHT_INTAKE, MOTOR_LIFT };  // list all motors
   long last_motor_read_loop_id_ = -1;    // last loop ID when any motor encoder is read

   /// Drive motors
   static final boolean WORKAROUND_V53_BUG = true;
   //static final boolean WORKAROUND_V53_BUG = false ;   // V5.4 revert FPID mod back to V5.2, still needed for whatever reason, 2020/02/15

   static final boolean USE_ENCODER_FOR_TELEOP = true;   // ALWAYS TRUE
   DcMotorEx motorLF_; //port 0 HUB3
   DcMotorEx motorLB_; //port 1
   DcMotorEx motorRF_; //port 2
   DcMotorEx motorRB_; //port 3

    //odometry motors
    DcMotorEx verticalLeftEncoder;
    DcMotorEx verticalRightEncoder;
    DcMotorEx horizontalEncoder;

    String rfName = "motorFR" , lfName = "motorFL";
    String rbName = "motorBR";
    String lbName = "motorBL";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = rbName;

   double ENCODER_MAX_DRIVE_POWER = 0.5;
   double ENCODER_MAX_ROTATE_POWER = 0.7;
   //double ENCODER_MAX_SIDEWALK_POWER = 0.5;
   double ENCODER_MAX_SIDEWALK_POWER = 0.5;

   /// Intake motors
   static boolean USE_INTAKE = false;
   DcMotorEx motor_left_intake_;
   DcMotorEx motor_right_intake_;
   static double INTAKE_POWER = -1.0;
   static double OUTTAKE_POWER = 1.0;
   double power_left_intake_ = 0.0;
   double power_right_intake_ = 0.0;


   ///  Lift control
   static boolean USE_LIFT = false;
   static boolean USE_RUN_TO_POS_FOR_LIFT = true;        // init version, slide go down too small
   //static boolean USE_RUN_TO_POS_FOR_LIFT = false;     // direct control for fast slide speed, no obvious speedup is observed, REVERTED, 2020/01/16; tried again with 3in spool, buggy, 2020/01/26
   //static double LIFT_RUN_TO_POS_RISE_TIME = 1.0;        // increase the encoder count by 2inch to speedup the lift rise time if >0.0, successfully fool the FPID, 2020/01/26
   //static double LIFT_RUN_TO_POS_RISE_TIME = 2.0;        // increase the encoder count by 20% to speedup the lift rise time if >0.0, successfully fool the FPID, 2020/02/01
   static double LIFT_RUN_TO_POS_RISE_TIME = 0.0;          // turn it off after change to V5.4, 2020/02/19

   DcMotorEx motor_lift_;
   static double LIFT_UP_POWER = 1.0;
   static double LIFT_DOWN_POWER = -LIFT_UP_POWER;
   static double LIFT_HOLD_POWER = 0.1*LIFT_UP_POWER;    // power for holding position
   static final int LIFT_ENC_HOLD_COUNT = 50;            // encoder count to hold position
   double power_lift_ = 0.0;
   static final double  LIFT_ENC_POWER = 1.0;            // ~2sec for full-length
   //static final int LIFT_ENC_COUNT_PER_STONE = 120;    // 120 for 4in; YJ3.7 motor, 1.5in diameter spool
   //static final int MAX_LIFT_ENC_COUNT = 1100 ;        // max encoder count to prevent over-extension
   //static final int LIFT_ENC_COUNT_PER_STONE = 168;    // YJ5.2 motor, 1.5in diameter spool; factor: 5.2/3.7=1.405
   //static final int MAX_LIFT_ENC_COUNT = 1520 ;        // max encoder count to prevent over-extension
   //static final int LIFT_ENC_COUNT_PER_STONE = 440;      // YJ13.7 motor, 1.5in diameter spool; factor: 13.7/3.7=3.7
   //static final int MAX_LIFT_ENC_COUNT = 4070 ;        // max encoder count to prevent over-extension
   //static final int LIFT_ENC_COUNT_PER_STONE = 185;      // YJ13.7 motor, 3in diameter spool (62mm/26mm=2.385)
   //static final int LIFT_ENC_COUNT_PER_STONE = 194;      // YJ13.7 motor, 3in diameter spool (62mm/26mm=2.385); +5%, 2020/02/01
   static final int LIFT_ENC_COUNT_PER_STONE = 200;      // YJ13.7 motor, 3in diameter spool (62mm/26mm=2.385); +5%, 2020/02/01; +5% after V5.4, 2020/02/19
   //static final int MAX_LIFT_ENC_COUNT = 1706 ;         // max encoder count to prevent over-extension; 2~3in short to max
   //static final int MAX_LIFT_ENC_COUNT = 1800 ;          // max encoder count to prevent over-extension; 2020/01/26
   static final int MAX_LIFT_ENC_COUNT = 2300 ;          // max encoder count to prevent over-extension; 4-stage, SAR330, 2020/02/08
   static final int LIFT_ENC_COUNT_FIRST_STONE = LIFT_ENC_COUNT_PER_STONE*3/4;   // rise 3inch
   //static final int LIFT_ENC_COUNT_FIRST_STONE = LIFT_ENC_COUNT_PER_STONE*5/8;   // rise 2.4inch, still too much, 2020/02/08
   //static final int LIFT_ENC_COUNT_FIRST_STONE = LIFT_ENC_COUNT_PER_STONE*2/4;     // rise 2inch, 2020/02/08; too tight; V5.4, hit the foundation, 02/29
   //static final int LIFT_ENC_COUNT_FIRST_STONE = LIFT_ENC_COUNT_PER_STONE*9/10;   // rise ~4inch for 3in spool, 2020/01/26; too much, 2020/01/30

   //static final int MIN_LIFT_ENC_COUNT = 0;            // min lift encoder count to prevent reverse
   //static final int MIN_LIFT_ENC_COUNT = -50;            // min lift encoder count to prevent reverse; negative for full speed down; string may come out
   //static final int MIN_LIFT_ENC_COUNT = -20;            // min lift encoder count to prevent reverse; negative for full speed down; too slow for 3in spool, 2020/01/25
   static final int MIN_LIFT_ENC_COUNT = -LIFT_ENC_COUNT_PER_STONE/2;   // min lift encoder count to prevent reverse; negative for full speed down; increased to fast reset with 3in spool, 2/9*360=80deg, min risk to revert, 2020/01/26
   //static final int MIN_LIFT_ENC_COUNT = -20;            // min lift encoder count to prevent reverse; negative for full speed down; too slow for 3in spool, 2020/01/25; restore for V5.4
   static final double LIFT_FAST_FALL_TIME = 2.0;             // give lift motor a negative encoder to make it go down faster, Thx, 2020/02/28; increased to 2.0 to ensure full speed fall, 03/07

   int last_lift_tg_enc_ = 0;
   double last_lift_power_ = 0.0;
   int last_stone_lift_enc_ = -MAX_LIFT_ENC_COUNT;
   int last_stone_level_ = 0;                            // 0, no stone yet; 1 one stone placed

   /// Servo Variables
   ///  Two continuous servos are used to lower/raise intake system
   static final double  CR_SERVO_STOP = 0.5;
   static double LOWER_INTAKE = 0.0;
   static double RAISE_INTAKE = 1-LOWER_INTAKE;
   Servo servo_left_intake_;
   double servo_left_intake_pos_;
   Servo servo_right_intake_;
   double servo_right_intake_pos_;

   /// Stone pusher
   static boolean USE_STONE_PUSHER = false;
   Servo servo_pusher_;
   double servo_pusher_pos_;
   static double PUSHER_INIT = 0.95;
   //static double PUSHER_HOLD = 0.6;
   static double PUSHER_HOLD = 0.58;
   static double PUSHER_UNLOAD = 0.4;
   double last_auto_hold_time_ = 0.0;

   /// Stone gater
   static boolean USE_STONE_GATER = false;
   Servo servo_gater_;
   double servo_gater_pos_ ;
   //static double GATER_CLOSE = 0.10;       // double beam slide, gater servo moved, 01/15
   static double GATER_CLOSE = 0.15;       // rise the gater close position, 03/07
   //static double GATER_OPEN = 0.65;
   static double GATER_OPEN = 0.55;        // 2020/02/29
   static double GATER_INIT = GATER_CLOSE;
   //static double AUTO_PUSH_TIME = 2.0;                // if >0, auto close the gater and reset pusher after this time period; too long
   static double AUTO_PUSH_TIME = 1.0;                // if >0, auto close the gater and reset pusher after this time period
   static boolean AUTO_START_INTAKE = true;                    // auto start intake if true
   static double AUTO_START_INTAKE_TIME = AUTO_PUSH_TIME+1.0;  // if >0, auto start intake after the stone is unloaded

   /// Arm for deliver/stack stone
   static boolean USE_ARM = false;
   static boolean AUTO_GRAB_STONE = true;
   static boolean AUTO_LIFT_STONE = true;
   Servo servo_arm_;
   double servo_arm_pos_;
   //static double ARM_DELIVER = 0.7;
   //static double ARM_DELIVER = 0.68;    // after replacing arm servo, 03/04
   //static double ARM_DELIVER = 0.65;    // before replacing arm servo on 03/04
   static double ARM_DELIVER = 0.67;      // 03/11, tweaking
   //static double ARM_GRAB = 0.3;
   static double ARM_GRAB = 0.32;       // push arm lower against the stone
   static double ARM_COLLECT = 0.2;
   static double ARM_INIT = ARM_COLLECT;

   /// Claw control
   Servo servo_claw_;
   double servo_claw_pos_;
   static double CLAW_OPEN = 0.45;
   static double CLAW_CLOSE = 0.55;
   static boolean USE_CAPSTONE = false;
   //static double CLAW_RELEASE_CAPSTONE = 0.0;    // pull the pin to release capstone
   static double CLAW_RELEASE_CAPSTONE = 0.10;    // to avoid hitting the right lift supporting bar, 2020/02/11

   /// Foundation hooks
   static boolean USE_HOOKS = false;
   Servo servo_left_hook_;
   double servo_left_hook_pos_;
   //static double LEFT_HOOK_UP = 0.25;
   static double LEFT_HOOK_UP = 0.05;
   //static double LEFT_HOOK_DOWN = 0.75;
   static double LEFT_HOOK_DOWN = 0.80;
   Servo servo_right_hook_;
   double servo_right_hook_pos_;
   //static double RIGHT_HOOK_UP = 0.75;
   static double RIGHT_HOOK_UP = 0.95;
   //static double RIGHT_HOOK_DOWN = 0.25;
   static double RIGHT_HOOK_DOWN = 0.20;

   /// Parking sticks
   static boolean USE_PARKING_STICKS = false;
   Servo servo_left_park_;
   double servo_left_park_pos_;
   static double LEFT_PARK_OUT = 0.18;
   static double LEFT_PARK_IN = 0.85;
   Servo servo_right_park_;
   double servo_right_park_pos_;
   static double RIGHT_PARK_OUT = 0.85;
   static double RIGHT_PARK_IN = 0.18;

   /////***************************** SENSORS *************************************/////

   enum RangeName { RANGE_STONE, RANGE_RIGHT, RANGE_LEFT, RGB_RANGE_STONE } ;  // list all range sensors
   double rgb_range_stone_dist_ = 0.0;
   int last_rgb_range_stone_read_loop_id_ = -1;        // loop_id for last stone range reading
   double last_rgb_range_stone_read_time_ = 0.0;       // last stone range reading time
   double range_stone_dist_ = 0.0;
   int last_range_stone_read_loop_id_ = -1;            // loop_id for last stone range reading
   double last_range_stone_read_time_ = 0.0;           // last stone range reading time
   double range_stone_dist_init_min_ = 0.0;            // min reading for RANGE_STONE in init_loop()
   boolean range_stone_error_detected_ = false;        // monitor the stone range reading in init_loop(), and set this flag if readings are unstable, AR will disable the use of it
   double range_right_dist_ = 0.0;                     // last/cached reading for RANGE_RIGHT
   int last_range_right_read_loop_id_ = -1;            // loop_id for last right range reading
   double last_range_right_read_time_ = 0.0;           // last right range reading time
   double range_right_dist_init_min_ = 0.0;            // min reading for RANGE_RIGHT in init_loop()
   boolean range_right_error_detected_ = false;        // monitor the right range reading in init_loop(), and set this flag if readings are unstable, AR will disable the use of it
   double range_left_dist_ = 0.0;
   int last_range_left_read_loop_id_ = -1;             // loop_id for last left range reading
   double last_range_left_read_time_ = 0.0;            // last right range reading time
   double range_left_dist_init_min_ = 0.0;            // min reading for RANGE_LEFT in init_loop()
   boolean range_left_error_detected_ = false;         // monitor the left range reading in init_loop(), and set this flag if readings are unstable, AR will disable the use of it

   static final int MAX_RANGE_READ_FREQ = 10 ;         // how often range sensor should be read; 10Hz=>100ms
   //static final int MAX_RANGE_READ_FREQ = 100 ;        // how often range sensor should be read; 100Hz=>10ms; read every loop
   boolean last_range_read_failed_ = false;            // flag to indicate the last read failed
   double last_range_failed_read_time_ = 0.0;          // time for last failed range read

   /// REV range sensor on right side of robot
   static final boolean USE_RIGHT_RANGE = false;        // use range sensor to measure distance to wall
   Rev2mDistanceSensor range_right_;                   // REV 2m range sensor
   double range_right_dist_init_ = -1.0;               // init value should be ~25cm

   static final boolean USE_LEFT_RANGE = false;         // use range sensor to measure distance to wall
   Rev2mDistanceSensor range_left_;                    // REV 2m range sensor
   double range_left_dist_init_ = -1.0;                // init value should be ~25cm

   /// REV range sensor for detecting stone
   static final boolean USE_RANGE_FOR_STONE = false;    // true for detecting stone
   static final boolean AUTO_CALIBRATE_RANGE = false;  // automatically calibrate REV RANGE sensor during init; false reading (~11cm) from claw; caused AR init() timeout, 2020/02/29
   Rev2mDistanceSensor range_stone_;                   // REV 2m range sensor; range: min=5cm, max=80~200cm, resolution: 1mm
   static final double MIN_RANGE_DIST = 0.03;          // minimal range reading, in meter
   static final double MAX_RANGE_DIST = 1.50;          // maximal range reading, in meter
   static final double MAX_RANGE_DIST_STONE = 0.12;    // distance to sensor: near >7~8cm; far <18cm
   static final double DELTA_RANGE_DIST_STONE = 0.10;  // distance delta used to detect stone
   double range_stone_dist_init_ = -1.0;               // init value should be ~25cm

   /// REV RGB/range sensor for detecting stone
   static final boolean USE_RGB_FOR_STONE = false;     // true for detecting stone; not good for V2
   LynxI2cColorRangeSensor rev_rgb_range_;             // REV color/range sensor2
   static final int MIN_RGB_ALPHA = 10;                // min alpha for RGB color
   static final int MIN_RGB_ALPHA_STONE = 80;          // typical value for alpha: 55 no stone, 90~100 with stone
   static final double MIN_RGB_RANGE_DIST = 0.01;      // minimal range reading for V3
   static final double MAX_RGB_RANGE_DIST = 0.10*1.5;  // maximal range reading for V3
   static final double MAX_RGB_DIST_STONE = 0.08;      // tpyical value for distance: 25+ no stone, 11~12 with stone

   static final boolean AUTO_CALIBRATE_RGB = false;     // automatically calibrate REV RGB sensor during init
   static final int DELTA_RGB_ALPHA_STONE = 25;         // alpha delta used to detect stone
   int rev_rgb_alpha_init_ = -1;
   static final boolean AUTO_CALIBRATE_RGB_RANGE = false;   // automatically calibrate REV RGB/range sensor during init
   double rev_rgb_dist_init_ = -1.0;
   static final boolean AUTO_SPIT_STONE2 = false;     // automatically spit the 2nd stone; DON'T work yet
   static final double AUTO_SPIT_STONE2_TIME = 0.5;   // the amount of time to fully spit the 2nd stone
   double last_double_stone_time_ = 0.0;              // last time double stones occur

   static final boolean USE_RGBV3_FOR_STONE = false;   // REV Color V3, 2020/02/29; proximity range: 1~10cm
   static final double  MAX_RGBV3_STONE_DIST = 0.085; // no stone: 9~10cm; regular stone 3~5cm; Skystone: 5~7cm
   ColorSensor rgb_color_stone_;                      // Color sensor from REV Color V3
   DistanceSensor rgb_range_stone_;                   // proximity sensor from REV Color V3
   double rgb_range_stone_dist_init_ = 0.0;

   static final boolean USE_INTAKE_MAG_SWITCH = false;
   static final double  LOWER_INTAKE_TIME = 0.35;     // period to lower intake after switch is untriggered for protection
   DigitalChannel intake_mag_switch_;                 // REV magnetic switch to determine intake motor position
   boolean intake_mag_prev_state_ ;                   // previous state
   double  intake_mag_change_time_ ;                  // time for last state change

   static final boolean USE_STONE_LIMIT_SWITCH = false;  // use limit switch for reliable stone detection
   DigitalChannel stone_limit_switch_;                // goBilda limit switch to detect stone
   boolean stone_limit_switch_prev_state_ ;           // previous state

   /// IMU, only use for AR
   static final boolean USE_IMU  = true;              // use IMU sensor to turn
   BNO055IMU imu_;                                    // RevHub IMU
   boolean imu_init_ok_ = false;
   double imu_init_time_ = 0.0;
   BNO055IMU.Parameters imu_parameters = new BNO055IMU.Parameters();
   Orientation imu_angles_;
   //Acceleration imu_gravity_;

   //static final boolean USE_IMU2  = true;            // OK for init now after put it into a separate thread, 02/20
   static final boolean USE_IMU2  = false;            // failed IMU2 may still increase looptime, so keep it off unless we need it, 02/23
   BNO055IMU imu2_;                                   // IMU from Hub2
   boolean imu2_init_ok_ = false;
   double imu2_init_time_ = 0.0;
   BNO055IMU.Parameters imu2_parameters = new BNO055IMU.Parameters();
   Orientation imu2_angles_;
   //
   double heading_ = 0.0;                                  // current heading

   static final boolean CACHE_IMU_READING = true;
   int last_imu_read_loop_id_ = -1;                         // loop_id for last IMU reading

   // Initialize IMU in a separate thread to avoid timeout, see IMUInitTest.java, 2020/02/09
   static final boolean USE_IMU_LOADER = true;      // initialize IMU in init_loop() with retry, see IMUInitTest.java, 2020/02/09
   static final boolean INIT_IMU_IN_INIT_LOOP = false;      // initialize IMU in init_loop() with retry, see IMUInitTest.java, 2020/02/09
   enum ImuInitState { NOT_STARTED, STARTING, INIT_STARTED, INIT_SUCCESS, INIT_ERROR, RETRY_ON_ERROR, INTEGRATION_STARTED };
   ImuInitState imuInitState = ImuInitState.NOT_STARTED;
   ImuInitState imu2InitState = ImuInitState.NOT_STARTED;
   boolean imu_stress_test_ = false;



   /////***************************** JOY STICKS *************************************/////

   static final float JOYSTICK_DEAD_ZONE = 0.1f;
   static final double  MIN_BUTTON_INTERVAL = 0.3;

   int a1_cnt_;                 // number of times pad1/A is pressed
   int b1_cnt_;                 // number of times pad1/B is pressed
   int x1_cnt_;                 // number of times pad1/X is pressed
   int y1_cnt_;                 // number of times pad1/Y is pressed
   int lb1_cnt_;                // number of times pad1/left_bumper is pressed
   int rb1_cnt_;                // number of times pad1/right_bumper is pressed
   int lsb1_cnt_;               // number of times pad1/left_joystick is pressed
   int rsb1_cnt_;               // number of times pad1/right_joystick is pressed
   int lt1_left_cnt_ ;          // number of times pad1 left_trigger+dpad_left are pressed at the same time
   int lt1_right_cnt_ ;         // number of times pad1 left_trigger+dpad_left are pressed at the same time

   int a2_cnt_;                 // number of times pad2/A is pressed
   int b2_cnt_;                 // number of times pad2/B is pressed
   int x2_cnt_;                 // number of times pad2/X is pressed
   int y2_cnt_;                 // number of times pad2/Y is pressed
   int lb2_cnt_;                // number of times pad2/left_bumper is pressed
   int rb2_cnt_;                // number of times pad2/right_bumper is pressed
   int lsb2_cnt_;               // number of times pad2/left_joystick is pressed
   int rsb2_cnt_;               // number of times pad2/right_joystick is pressed

   double last_button_time_;
   double last_button_a1_time_;
   double last_button_b1_time_;
   double last_button_rb1_time_;
   double last_button_time2_;

   ///  Constructor
   public Y20CommonEx() {
   }


   ///  Code to run when the op mode is initialized goes here
   @Override public void init() {
      /// Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
      all_hubs_ = hardwareMap.getAll(LynxModule.class);
      /// Important Step 3: Option B. Set all Expansion hubs to use the MANUAL Bulk Caching mode
      for (LynxModule module : all_hubs_ ) {
         switch (motor_read_mode_) {
            case BULK_READ_AUTO:
               module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
               break;
            case BULK_READ_MANUAL:
               module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
               break;
            case BULK_READ_OFF:
            default:
               module.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
               break;
         }
      }

      /// Use the hardwareMap to get the dc motors and servos by name.

      motorLF_ = hardwareMap.get(DcMotorEx.class, lfName);
      motorLB_ = hardwareMap.get(DcMotorEx.class, lbName);
      motorRF_ = hardwareMap.get(DcMotorEx.class, rfName);
      motorRB_ = hardwareMap.get(DcMotorEx.class, rbName);
      motorLF_.setDirection(DcMotor.Direction.REVERSE);
      motorLB_.setDirection(DcMotor.Direction.REVERSE);

      // map odometry encoders
       verticalLeftEncoder = hardwareMap.get(DcMotorEx.class, verticalLeftEncoderName);
       verticalRightEncoder = hardwareMap.get(DcMotorEx.class, verticalRightEncoderName);
       horizontalEncoder = hardwareMap.get(DcMotorEx.class, horizontalEncoderName);

      if( USE_ENCODER_FOR_TELEOP ) {
         motorLF_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorLF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorLB_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorLB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorRF_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorRF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorRB_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorRB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);

         motorLF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         motorLB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         motorRF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         motorRB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      }

      if( USE_INTAKE ) {
         motor_left_intake_ =  hardwareMap.get(DcMotorEx.class, "motorLeftIntake");
         motor_right_intake_ =  hardwareMap.get(DcMotorEx.class, "motorRightIntake");
         motor_right_intake_.setDirection(DcMotor.Direction.REVERSE) ;

         motor_left_intake_.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
         motor_left_intake_.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
         motor_right_intake_.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
         motor_right_intake_.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

         servo_left_intake_ = hardwareMap.servo.get("servoLeftIntake");
         servo_left_intake_pos_ = CR_SERVO_STOP ;
         servo_left_intake_.setPosition(CR_SERVO_STOP);
         servo_right_intake_ = hardwareMap.servo.get("servoRightIntake");
         servo_right_intake_pos_ = CR_SERVO_STOP ;
         servo_right_intake_.setPosition(CR_SERVO_STOP);
      }
      if( USE_LIFT ) {
         motor_lift_ = hardwareMap.get(DcMotorEx.class, "motorLift");
         motor_lift_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         motor_lift_.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
         power_lift_ = 0.0;
         if( USE_RUN_TO_POS_FOR_LIFT ) {
            motor_lift_.setTargetPosition(0);
            motor_lift_.setMode( DcMotor.RunMode.RUN_TO_POSITION );  // must call setTargetPosition() before switching to RUN_TO_POSISTION
         } else {
            motor_lift_.setMode( DcMotor.RunMode.RUN_USING_ENCODER);
         }
         last_stone_lift_enc_ = -1;
      }

      if( USE_STONE_PUSHER ) {
         servo_pusher_ = hardwareMap.servo.get("servoPusher");
         servo_pusher_.setPosition(PUSHER_INIT);
         servo_pusher_pos_ = PUSHER_INIT;
      }
      if( USE_STONE_GATER ) {
         servo_gater_ = hardwareMap.servo.get("servoGater");
         servo_gater_.setPosition(GATER_INIT);
         servo_gater_pos_ = GATER_INIT;
      }

      if( USE_ARM ) {
         servo_arm_ = hardwareMap.servo.get("servoArm");
         servo_arm_.setPosition(ARM_INIT);
         servo_arm_pos_ = ARM_INIT;
         servo_claw_ = hardwareMap.servo.get("servoClaw");
         servo_claw_.setPosition(CLAW_OPEN);
         servo_claw_pos_ = CLAW_OPEN;
      }

      if( USE_HOOKS ) {
         servo_left_hook_ = hardwareMap.servo.get("servoLeftHook");
         servo_left_hook_.setPosition(LEFT_HOOK_UP);
         servo_left_hook_pos_ = LEFT_HOOK_UP;
         servo_right_hook_ = hardwareMap.servo.get("servoRightHook");
         servo_right_hook_.setPosition(RIGHT_HOOK_UP);
         servo_right_hook_pos_ = RIGHT_HOOK_UP;
      }

      if( USE_PARKING_STICKS ) {
         servo_left_park_ = hardwareMap.servo.get("servoLeftPark");
         servo_left_park_.setPosition(LEFT_PARK_IN);
         servo_left_park_pos_ = LEFT_PARK_IN;
         servo_right_park_ = hardwareMap.servo.get("servoRightPark");
         servo_right_park_.setPosition(RIGHT_PARK_IN);
         servo_right_park_pos_ = RIGHT_PARK_IN;
      }

      if( USE_RGB_FOR_STONE ) {
         rev_rgb_range_ = hardwareMap.get(LynxI2cColorRangeSensor.class, "stoneColor");
         //rev_rgb_range_ = hardwareMap.get(LynxI2cColorRangeSensor.class, "stoneColorV3");  // different interface for V3, can't define it as LynxI2cColorRangeSensor anymore, 2020/02/29
         if( rev_rgb_range_!=null ) {
            if( AUTO_CALIBRATE_RGB ) {
               int alpha = rev_rgb_range_.alpha();
               //double dist = rev_rgb_range_.getDistance(DistanceUnit.CM);
               double dist = rev_rgb_range_.getDistance(DistanceUnit.METER);
               if( alpha>=MIN_RGB_ALPHA && alpha<100000 ) {
                  rev_rgb_alpha_init_ = alpha;
               }
               if( AUTO_CALIBRATE_RGB_RANGE && !Double.isNaN(dist) ) {
                  if( dist>MIN_RGB_RANGE_DIST && dist<MAX_RGB_RANGE_DIST ) {
                     rev_rgb_dist_init_  = dist;
                  }
               }
            }
         }
      }
      if( USE_RGBV3_FOR_STONE ) {
         //rgb_color_stone_ = hardwareMap.get(ColorSensor.class, "stoneColorV3");
         rgb_range_stone_ = hardwareMap.get(DistanceSensor.class, "stoneColorV3");
         if( AUTO_CALIBRATE_RANGE && rgb_range_stone_!=null ) {
            while(true) { // wait till range sensor gets a valid reading
               double dis = getRangeDist(RangeName.RGB_RANGE_STONE);
               if( dis>0.0 && dis<0.2 ) {
                  rgb_range_stone_dist_init_ = dis;
                  break;
               }
            }
         }
      }

      if( USE_RIGHT_RANGE ) {
         range_right_ = (Rev2mDistanceSensor) (hardwareMap.get(DistanceSensor.class, "rightRange"));
         if( AUTO_CALIBRATE_RANGE && range_right_!=null ) {
            while(true) { // wait till range sensor gets a valid reading
               double dis = getRangeDist(RangeName.RANGE_RIGHT);
               if( dis>0.01 && dis<2.0 ) {
                  range_right_dist_init_ = dis;
                  break;
               }
            }
         }
      }
      if( USE_LEFT_RANGE ) {
         range_left_ = (Rev2mDistanceSensor) (hardwareMap.get(DistanceSensor.class, "leftRange"));
         if( AUTO_CALIBRATE_RANGE && range_left_!=null ) {
            while(true) { // wait till range sensor gets a valid reading
               double dis = getRangeDist(RangeName.RANGE_LEFT);
               if( dis>0.01 && dis<2.0 ) {
                  range_left_dist_init_ = dis;
                  break;
               }
            }
         }
      }

      if( USE_RANGE_FOR_STONE) {
         range_stone_ = (Rev2mDistanceSensor) (hardwareMap.get(DistanceSensor.class, "stoneRange"));
         if( AUTO_CALIBRATE_RANGE && range_stone_!=null ) {
            while(true) { // wait till range sensor gets a valid reading
               double dis = getRangeDist(RangeName.RANGE_STONE);
               if( dis>0.01 && dis<0.5 ) {
                  range_stone_dist_init_ = dis;
                  break;
               }
            }
         }
      }

      if( USE_INTAKE_MAG_SWITCH ) {
         intake_mag_switch_ =  hardwareMap.get(DigitalChannel.class, "intake_mag_switch");
         intake_mag_switch_.setMode(DigitalChannelController.Mode.INPUT);
         intake_mag_prev_state_ = intake_mag_switch_.getState();
         intake_mag_change_time_ = 0.0;
      }
      if( USE_STONE_LIMIT_SWITCH ) {
         stone_limit_switch_ =  hardwareMap.get(DigitalChannel.class, "stone_limit_switch");
         stone_limit_switch_.setMode(DigitalChannelController.Mode.INPUT);
         stone_limit_switch_prev_state_ = stone_limit_switch_.getState();
      }


      /////***************************** JOY STICKS *************************************/////

      /// Set joystick deadzone, any value below this threshold value will be considered as 0; moved from init() to init_loop() to aovid crash
      if(gamepad1!=null) gamepad1.setJoystickDeadzone( JOYSTICK_DEAD_ZONE );
      if(gamepad2!=null) gamepad2.setJoystickDeadzone( JOYSTICK_DEAD_ZONE );

      resetControlVariables();
   }


   /// This method will be called once before entering loop()
   @Override public void init_loop() {
      loop_cnt_++;
   }

   /// This method will be called once when the PLAY button is first pressed.
   @Override public void start() {
      timer_.reset();

      resetControlVariables();   // need reset all key counters bcz they are used in init_loop()
   }

   /// This method will be called repeatedly in a loop
   @Override public void loop () {
   }


   /// Code to run when the op mode is first disabled goes here
   @Override public void stop () {
   }

   /// Reset all variables before entering init_loop() and loop()
   void resetControlVariables() {
      a1_cnt_ = 0;
      b1_cnt_ = 0;
      x1_cnt_ = 0;
      y1_cnt_ = 0;
      lb1_cnt_ = 0;
      rb1_cnt_ = 0;
      lsb1_cnt_ = 0;
      rsb1_cnt_ = 0;
      lt1_left_cnt_ = 0;
      lt1_right_cnt_ = 0;

      a2_cnt_ = 0;
      b2_cnt_ = 0;
      x2_cnt_ = 0;
      y2_cnt_ = 0;
      lb2_cnt_ = 0;
      rb2_cnt_ = 0;
      lsb2_cnt_ = 0;
      rsb2_cnt_ = 0;

      curr_time_ = 0.0;
      loop_cnt_ = 0;
      last_imu_read_loop_id_ = -1;
      last_motor_read_loop_id_ = -1;
      last_rgb_range_stone_read_loop_id_ = -1;
      last_range_stone_read_loop_id_ = -1;
      last_range_right_read_loop_id_ = -1;
      last_range_left_read_loop_id_ = -1;
      last_range_stone_read_time_ = 0;
      last_range_right_read_time_ = 0;
      last_range_left_read_time_ = 0;
      range_stone_dist_init_min_ = 0;
      range_right_dist_init_min_ = 0;
      range_left_dist_init_min_ = 0;
      range_stone_error_detected_ = false;
      range_right_error_detected_ = false;
      range_left_error_detected_ = false;
      battery_voltage_ = 0.0;

      last_button_time_ = 0.0;
      last_button_a1_time_ = 0.0;
      last_button_b1_time_ = 0.0;
      last_button_rb1_time_ = 0.0;
      last_button_time2_ = 0.0;
      last_double_stone_time_ = 0.0;
   }

   /// Return true if it's RED team
   boolean isRedTeam() {
      return ( START_AS_RED ? (!color_flipped_) : color_flipped_ );
   }

   /// Return current robot heading based on gyro/IMU reading
   double getHeading() {
      if( CACHE_IMU_READING && loop_cnt_==last_imu_read_loop_id_ ) {
         // return the existing heading when getHeading() is called in the same loop
         return heading_ ;
      }

      heading_ = 0;
      if( USE_IMU && imu_!=null && imu_init_ok_ ) {
         imu_angles_  = imu_.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);  // acquiring angles are expensive, keep it minimal
         // IMU can automatically detect orientation, and return heading as first angle, 2020/01/25
         heading_ = AngleUnit.DEGREES.normalize(imu_angles_.firstAngle);
         last_imu_read_loop_id_ = loop_cnt_;
      } else if( USE_IMU2 && imu2_!=null && imu2_init_ok_ ) {
         // use IMU2 if IMU failed
         imu2_angles_  = imu2_.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
         heading_ = AngleUnit.DEGREES.normalize(imu2_angles_.firstAngle);
         last_imu_read_loop_id_ = loop_cnt_;
      }
      return heading_;
   }

   /// Return heading diff for a given init heading
   double getHeadingDiff(double init_h) {
      double curr_h = getHeading();
      double diff_h = init_h - curr_h;
      if( diff_h>=360.0 ) diff_h -= 360.0;
      else if( diff_h<=-360.0 ) diff_h += 360.0;
      return diff_h ;
   }


   /// Gets the heading from REV IMU/IMU2
   double readImuHeading() {
      double h = -360.0;
      if( USE_IMU && imu_!=null && imu_init_ok_ ) {
         h = imu_.getAngularOrientation().firstAngle;
      }
      return h;
   }
   /// Gets the heading from REV IMU2
   double readImu2Heading() {
      double h = -360.0;
      if( USE_IMU2 && imu2_!=null && imu2_init_ok_ ) {
         h = imu2_.getAngularOrientation().firstAngle;
      }
      return h;
   }

   /// Normalize a given heading with value of (-360,360] to range (-180,180]
   double normalizeHeading(double h) {
      double nh = h;
      if( h>-180 && h<180 ) {
      } else if( h <= -180 ) {
         nh = h+360;
      } else if( h > 180 ) {
         nh = h-360;
      }
      return nh;
   }

   /// Computes the current battery voltage
   // Copy from /robotcontroller/external/samples/ConceptTelemetry.java
   double getBatteryVoltage(boolean first_reading) {
      double result = Double.POSITIVE_INFINITY;
      for (VoltageSensor sensor : hardwareMap.voltageSensor) {
         double voltage = sensor.getVoltage();
         if (voltage > 0) {
            result = Math.min(result, voltage);
            if(first_reading) break;    // return the first valid reading (>0.0) to minimize overhead
         }
      }
      return result;
   }

   int numVoltageSensors() {
      int num = 0;
      for( VoltageSensor sensor : hardwareMap.voltageSensor ) {
         ++num;
      }
      return num;
   }

   /// Class to initialize IMU using a separate thread
   class IMULoader implements Runnable {
      public void run() {
         if( USE_IMU && imu_!=null && !imu_init_ok_ ) {
            imu_init_ok_ = imu_.initialize(imu_parameters);
            if( imu_init_ok_ ) { imu_init_time_ = timer_.seconds(); }
         }
         if( USE_IMU2 && imu2_!=null && !imu2_init_ok_ ) {
            imu2_init_ok_ = imu2_.initialize(imu_parameters);
            if( imu2_init_ok_ ) { imu2_init_time_ = timer_.seconds(); }
         }
      }
   }

   /// Return true if all drive motors' encoder are 0s
   boolean areAllDriveMotorsReset() {
      return ( getMotorEncoder(MotorName.MOTOR_LF)==0
            && getMotorEncoder(MotorName.MOTOR_LB)==0
            && getMotorEncoder(MotorName.MOTOR_RF)==0
            && getMotorEncoder(MotorName.MOTOR_RB)==0
            );
   }

   /// Return motor encoder value
   int getMotorEncoder( MotorName m ) {
      int enc = 0;
      if( motor_read_mode_==MotorBulkReadMode.BULK_READ_MANUAL && loop_cnt_!=last_motor_read_loop_id_ ) {
        // Important Step 4: If you are using MANUAL mode, you must clear the BulkCache once per control cycle
        for( LynxModule module : all_hubs_ ) {
            module.clearBulkCache();
        }
      }
      switch ( m ) {
         case MOTOR_LF:
            enc = motorLF_.getCurrentPosition();
            break;
         case MOTOR_LB:
            enc = motorLB_.getCurrentPosition();
            break;
         case MOTOR_RF:
            enc = motorRF_.getCurrentPosition();
            break;
         case MOTOR_RB:
            enc = motorRB_.getCurrentPosition();
            break;
         case MOTOR_LEFT_INTAKE:
            enc = motor_left_intake_.getCurrentPosition();
            break;
         case MOTOR_RIGHT_INTAKE:
            enc = motor_right_intake_.getCurrentPosition();
            break;
         case MOTOR_LIFT:
            enc = motor_lift_.getCurrentPosition();
            break;
         default:
            break;
      }
      last_motor_read_loop_id_ = loop_cnt_ ;
      return enc;
   }

   /// Return motor velocity value; TBD
   double getMotorVelocity( MotorName m ) {
      double vol = 0.0;
      if( motor_read_mode_==MotorBulkReadMode.BULK_READ_MANUAL && loop_cnt_!=last_motor_read_loop_id_ ) {
        // Important Step 4: If you are using MANUAL mode, you must clear the BulkCache once per control cycle
        for( LynxModule module : all_hubs_ ) {
            module.clearBulkCache();
        }
      }
      switch ( m ) {
         case MOTOR_LF:
            vol = motorLF_.getVelocity();
            break;
         case MOTOR_LB:
            vol = motorLB_.getVelocity();
            break;
         case MOTOR_RF:
            vol = motorRF_.getVelocity();
            break;
         case MOTOR_RB:
            vol = motorRB_.getVelocity();
            break;
         case MOTOR_LEFT_INTAKE:
            vol = motor_left_intake_.getVelocity();
            break;
         case MOTOR_RIGHT_INTAKE:
            vol = motor_right_intake_.getVelocity();
            break;
         case MOTOR_LIFT:
            vol = motor_lift_.getVelocity();
            break;
         default:
            break;
      }
      last_motor_read_loop_id_ = loop_cnt_ ;
      return vol;
   }

   /// Determine wether a reading from a REV 2M range sensor is valid
   boolean isValidRangeReading(double dist) {
      return ( !Double.isNaN(dist) && dist>MIN_RANGE_DIST && dist<MAX_RANGE_DIST ) ;
   }

   /// Return reading for a given range sensor in meters
   double getRangeDist( RangeName r ) {
      double dis = 0.0;
      double period = 1.0/MAX_RANGE_READ_FREQ ;   // dflt 10Hz => one read per 100ms
      last_range_read_failed_ = false;
      switch ( r ) {
         case RGB_RANGE_STONE :
            if( rgb_range_stone_ != null ) {
               if( (last_rgb_range_stone_read_loop_id_ == loop_cnt_) || (last_rgb_range_stone_read_time_>0 && curr_time_-last_rgb_range_stone_read_time_<period) ) {
                  dis = rgb_range_stone_dist_;
               } else {
                  dis = rgb_range_stone_.getDistance( DistanceUnit.METER );
                  if( isValidRangeReading(dis) ) {
                     last_rgb_range_stone_read_loop_id_ = loop_cnt_;
                     last_rgb_range_stone_read_time_ = curr_time_;
                     rgb_range_stone_dist_ = dis;
                     last_range_failed_read_time_ = 0.0;
                  } else {  // invalid reading, use the last value
                     dis = rgb_range_stone_dist_;
                     last_range_read_failed_ = true;
                     last_range_failed_read_time_ = curr_time_;
                  }
               }
            }
            break;
         case RANGE_STONE :
            if( range_stone_ != null ) {
               if( (last_range_stone_read_loop_id_ == loop_cnt_) || (last_range_stone_read_time_>0 && curr_time_-last_range_stone_read_time_<period) ) {
                  dis = range_stone_dist_;
               } else {
                  dis = range_stone_.getDistance( DistanceUnit.METER );
                  if( isValidRangeReading(dis) ) {
                     last_range_stone_read_loop_id_ = loop_cnt_;
                     last_range_stone_read_time_ = curr_time_;
                     range_stone_dist_ = dis;
                     last_range_failed_read_time_ = 0.0;
                  } else {  // invalid reading, use the last value
                     dis = range_stone_dist_;
                     last_range_read_failed_ = true;
                     last_range_failed_read_time_ = curr_time_;
                  }
               }
            }
            break;
         case RANGE_RIGHT :
            if( range_right_ != null ) {
               if( (last_range_right_read_loop_id_ == loop_cnt_) || (last_range_right_read_time_>0 && curr_time_-last_range_right_read_time_<period) ) {
                  dis = range_right_dist_;
               } else {
                  dis = range_right_.getDistance( DistanceUnit.METER );
                  if( isValidRangeReading(dis) ) {
                     last_range_right_read_loop_id_ = loop_cnt_;
                     last_range_right_read_time_ = curr_time_;
                     range_right_dist_ = dis;
                     last_range_failed_read_time_ = 0.0;
                  } else {  // invalid reading, use the last value
                     dis = range_right_dist_ ;
                     last_range_read_failed_ = true;
                     last_range_failed_read_time_ = curr_time_;
                  }
               }
            }
            break;
         case RANGE_LEFT :
            if( range_left_ != null ) {
               if( (last_range_left_read_loop_id_ == loop_cnt_) || (last_range_left_read_time_>0 && curr_time_-last_range_left_read_time_<period) ) {
                  dis = range_left_dist_;
               } else {
                  dis = range_left_.getDistance( DistanceUnit.METER );
                  if( isValidRangeReading(dis) ) {
                     last_range_left_read_loop_id_ = loop_cnt_;
                     last_range_left_read_time_ = curr_time_;
                     range_left_dist_ = dis;
                     last_range_failed_read_time_ = 0.0;
                  } else {  // invalid reading, use the last value
                     dis = range_left_dist_ ;
                     last_range_read_failed_ = true;
                     last_range_failed_read_time_ = curr_time_;
                  }
               }
            }
            break;
         default:
            break;
      }
      return dis;
   }
}
