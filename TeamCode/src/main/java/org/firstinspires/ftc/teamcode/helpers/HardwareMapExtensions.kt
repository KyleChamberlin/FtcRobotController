package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.ftc.LazyImu
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS
import org.firstinspires.ftc.teamcode.drivetrain.DriveMotor
import org.firstinspires.ftc.teamcode.drivetrain.DriveMotors
import org.firstinspires.ftc.teamcode.helpers.vision.ViewId

fun HardwareMap.getMotor(name: String): DcMotorEx = this.get(DcMotorEx::class.java, name)
fun HardwareMap.getServo(name: String): ServoImplEx = this.get(ServoImplEx::class.java, name)
fun HardwareMap.leftFrontMotor(): LeftFrontMotor = this.getMotor("left_front_motor")
fun HardwareMap.rightFrontMotor(): RightFrontMotor = this.getMotor("right_front_motor")
fun HardwareMap.leftBackMotor(): LeftBackMotor = this.getMotor("left_back_motor")
fun HardwareMap.rightBackMotor(): RightBackMotor = this.getMotor("right_back_motor")
fun HardwareMap.mecanumDrive(): MecanumDrive = MecanumDrive(
    this.leftFrontMotor(),
    this.rightFrontMotor(),
    this.leftBackMotor(),
    this.rightBackMotor()
)

fun HardwareMap.webcam(): WebcamName = this.get(WebcamName::class.java, "webcam_1")
fun HardwareMap.cameraMonitorViewId(): ViewId = this.appContext.resources.getIdentifier(
    "cameraMonitorViewId",
    "id",
    this.appContext.packageName
)

var HardwareMap.bulkCachingMode: BulkCachingMode
    get() = TODO()
    set(value) {
        for (module in getAll<LynxModule>(LynxModule::class.java)) {
            module.setBulkCachingMode(value)
        }
    }

fun HardwareMap.imu(
    name: String = "imu", orientation: ImuOrientationOnRobot = RevHubOrientationOnRobot(
        LogoFacingDirection.UP, UsbFacingDirection.BACKWARD
    )
): IMU {
    return LazyImu(this, name, orientation).imu
}

val LazyImu.imu: IMU
    get() = this.get()

val IMU.heading: Rotation2d
    get() = Rotation2d.exp(
        this.robotYawPitchRollAngles.getYaw(
            RADIANS
        )
    )

val IMU.yaw: Double get() = this.robotYawPitchRollAngles.yaw

val HardwareMap.nextVoltageSensor: VoltageSensor get() = voltageSensor.iterator().next()

fun HardwareMap.driveMotors(
    leftFront: String = "left_front",
    leftFrontDirection: Direction = REVERSE,
    rightFront: String = "right_front",
    rightFrontDirection: Direction = FORWARD,
    leftBack: String = "left_back",
    leftBackDirection: Direction = REVERSE,
    rightBack: String = "right_back",
    rightBackDirection: Direction = REVERSE,
): DriveMotors {
    return DriveMotors(
        leftFront = DriveMotor(this.getMotor(leftFront), leftFrontDirection),
        rightFront = DriveMotor(this.getMotor(rightFront), rightFrontDirection),
        leftBack = DriveMotor(this.getMotor(leftBack), leftBackDirection),
        rightBack = DriveMotor(this.getMotor(rightBack), rightBackDirection)
    )
}
