Described Subsystems : 

    Shintake (shooter/intake)
        Commands : takeIn, spitOut

        Periodic - displays LEFT,RIGHT set velocity, set power
                    BeamBreak status
    Arm
        Commands : Manual Angle - Adjusts turret angle manually
                   AutoAngle - Based on photonvision

        Periodic -  displays LEFT,RIGHT PID + angle
                    Uses PID + feedforward to hold angle
    Climber
        Commands :  Climb - Climbs parallel to ground using robot gyro
                    ClimberActivate - Extends climber
                    ClimbRelease - releases climber to bring robot down
                    Manual Climb - climbs manually

        Periodic - displays LEFT, RIGHT current
                   displays climber activation

    PoseEstimator
        methods - estimates robot pose based on photonvision
        Periodic - displays x,y,theta

AprilTag Commands:
    AprilTagDistance - puts calculated distance on smartDashboard
    


