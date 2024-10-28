# robot2024

The code for our 2024 robot

## How it works

### What's a Kotlin?

- Java, but better
- No, seriously, it was made by JetBrains to make Java better again.
- Kotlin can interop with Java (i.e. Java code can call Kotlin, and vice versa).
- It's a lot easier to work with.

#### Why is half of the code still in Java then?

- You all know how to use Java, not Kotlin, therefore anything that may need to be changed
  on-the-fly (other than constants) have been kept in Java.

### Spark Motor Stuff

#### `SparkPIDController`

- A lot of boilerplate for managing the speeds of each motor with PID.
- They use the integrated relative encoder as a feedback device by default.
- Can be set to use absolute encoders if they exist (ask assembly/control systems people)
- They can be managed by position or velocity setpoints.
    - To set a position setpoint, use the encoder values.
    - Set a scaling factor/zero offset on the encoder, THE PID CONTROLLER DOES NOT HAVE ONE
- SmartMotion also exists, but I don't know how it works yet. Will update.

### Subsystems

#### Arm

- Both the arm pivot and extension is controlled via a position-based feedback loop
- This means that you set a position (angle degrees or ext percent) and the subsystem will try to meet that setpoint

  ##### Arm Pivot
    - Setpoints are set via `SparkPIDController`'s position setpoints
    - There are two motors controlling the arm, configured via a master (left) and slave (right) config

  ##### Arm Extension
    - The extension is a bit different. You still set a percent setpoint but the code that "meets" the setpoint is
      handled separately, as `SparkPIDController` was being weird. See `Arm.extendSetpoint()` for more information.
    - There is only one motor being used for arm extension.
    - PID is not used.

#### Camera

- Still an untested work-in-progress. Will update this later, hopefully.

#### Drivetrain

- This needs a bit more explanation, so buckle up

  ##### `SwerveModule`
    - There are four of these in a drivetrain, each with one turning and one driving motor
    - The `angularOffset` constructor parameter is the angle (rad) each module is offset by,
      relative to the front right one. These are +/- quarter circles and half circles
    - Modules have a `desiredState`, which includes the drive velocity and a rotation setpoint in radians
    - The desired states, once set, are first corrected then optimized before being set via the `SparkPIDControllers`

  ##### Swerve
    - Swerve is a type of drivetrain where there are four wheels whose rotations and positions can be controlled
      separately.
    - To make the robot drive in a field-centric manner (i.e. forward is always field-north no matter which way
      the robot is facing), a gyroscope is used to measure the robot's current rotation, and fix motion accordingly

  ##### Slew and rate-limiting
    - I did not write this (I copied it from whoever did write it), nor do I know how it works. It is off by default and
      the robot seems to work fine.
      if someone figures it out, update this, please.

  ##### `Drivetrain.drive()`
    - Converts controller inputs to `ChassisSpeeds`, which when set, update the desired speeds using math
    - Don't ask, 99% of it is library code.

  ##### Other Drivetrain stuff
    - `resetGyro()`: Makes it so that field-north is the direction the robot is facing
    - `stop()`: Kills the motors
    - `lock()`: Turns motors into an X-formation, effectively killing them

#### Notifier

- This subsystem houses the controllers and LED, and is used to notify the drivers of stuff using the XboxController's
  rumbling stuff.
- Pass in the Notifier subsystem as a constructor argument for commands, and call `Notifer.notify()` to
  buzz the driver's and operator's controllers for one second.

#### Climber

- Each climber has two `ClimberModule`s, each of which has a servo to lock and unlock the climbers, and a motor
- One of the motors need to be inverted. Do not change that.
- I'm still working on an automatic climbing command, therefore the gyro and `.bias()` is not yet used

#### Shintake

- Shooter + Intake combined subsystem.
- Contains methods to set the intake and shooter motors.
- The shooter has two motors, right (master) and left (slave).
- A beam brake is used to check if the note is in place.

#### Led

- Contains both an `AddressableLED` and an `AddressableLEDBuffer`
- Write colors to the buffers, then write the buffer to the LED.