```mermaid

graph LR;
    PDH{{Power Distribution Hub}};
    RoboRIO==>|CAN Bus|CoralSubsystem;
    RoboRIO==>|CAN Bus|ClimberSystem

    subgraph IntakeGroup
        direction LR
        Intake((Intake)) ==> Funnel
        CoralSubsystem((Coral Subsystem))==> Elevator;
        IntakeOSystem((Outake));
        Funnel(Funnel) ==> CoralSubsystem
        subgraph CoralGroup
                direction LR
                Elevator(Elevator) ==> ElevatorUp;
                ElevatorUp(Elevator Up) ==> ElevatorDown;
                ElevatorDown(ElevatorDown) ==> ElevatorStop;
                ElevatorStop(Elevator Stop) ==> ElevatorL1;
                ElevatorL1(ElevatorL1) ==> ElevatorL2;
                ElevatorL2(ElevatorL2) ==> ElevatorL3;
                ElevatorL3(ElevatorL3) ==> ElevatorL4;
                ElevatorL4(ElevatorL4) ==> IntakeOSystem;
        end
    end

    subgraph ClimberGroup
        direction LR
        ClimberSystem(Climber);
        DeployClimber(Deploy Climber);
        ClimberSystem==>DeployClimber;
        DeployClimber ==> CageHug;
        CageHug(hug cage);
        CageHug ==> FUNCCLIMBER1;
        FUNCCLIMBER1[/Climb up/]==>FUNCCLIMBER2
        FUNCCLIMBER2[/Climb down/]
    end

    subgraph DriveTrainGroup
        direction LR
        DriveTrain(Drive Train);
        DriveTrain==>FUNCDRIVE[/Drive Robot/];
        DriveTrain==>FrontRightSwerve(Front Right Swerve);
        DriveTrain==>FrontLeftSwerve(Front Left Swerve);
        DriveTrain==>BackRightSwerve(Back Right Swerve);
        DriveTrain==>BackLeftSwerve(Back Left Swerve);

subgraph SwerveGroup
        direction LR
        FrontRightSwerve==>MaxFR(Max *1*);
        FrontLeftSwerve==>MaxFL(Max *2*);
        BackRightSwerve==>MaxBR(Max *3*);
        BackLeftSwerve==>MaxBL(Max *4*);
        FrontRightSwerve==>VortexFR(Vortex *5*);
        FrontLeftSwerve==>VortexFL(Vortex *6*);
        BackRightSwerve==>VortexBR(Vortex *7*);
        BackLeftSwerve==>VortexBL(Vortex *8*);

        subgraph SwerveGroupFR
        direction LR
        MaxFR
        VortexFR
        MaxFR==>EncoderFR[[Absolute Encoder]];
        end

        subgraph SwerveGroupFL
        direction LR
        MaxFL
        VortexFL
        MaxFL==>EncoderFL[[Absolute Encoder]];
        end

        subgraph SwerveGroupBR
        direction LR
        MaxBR
        VortexBR
        MaxBR==>EncoderBR[[Absolute Encoder]];
        end

        subgraph SwerveGroupBL
        direction LR
        MaxBL
        VortexBL
        MaxBL==>EncoderBL[[Absolute Encoder]];
        end
    end
    end

    

    RADIO[(RADIO FUNCTION : communicate between robot and drive station)]==>RoboRIO;
    RoboRIO{{RoboRIO FUNCTION : Brain of the robot, controls all subsystems}}==>|CAN Bus|DriveTrain;
    
    PDH==>|Slot 1|MaxFR
    PDH==>|Slot 2|MaxFL
    PDH==>|Slot 3|MaxBR
    PDH==>|Slot 4|MaxBL
    PDH==>|Slot 5|VortexFR
    PDH==>|Slot 6|VortexFL
    PDH==>|Slot 7|VortexBR
    PDH==>|Slot 8|VortexBL
    PDH==>|Slot 9|RoboRIO


    PDH ==> FunnelMotor
    FunnelPivot((Funnel Pivot))
    FunnelMotor((Funnel Motor)) ==> FunnelPivot ==> Funnel;
    DeepClimbMotor1((Deep Motor 1))
    DeepClimbMotor2((Deep Motor 2))
    ElavatorMotor((Elevator Motor))


    linkStyle 22,23,24,25,26,27,28,29,30,31,32,33,36,37,38,39,40,41,42,43,44,45 stroke-width:4px,fill:none,stroke:orange;
    linkStyle 17,18,19,20,21 stroke-width:4px,fill:none,stroke:red; 
    linkStyle 2,3,4,5,6,7,8,9,10,11,12,46,47 stroke-width:4px,fill:none,stroke:cyan;
    linkStyle 13,14,15,16 stroke-width:4px,fill:none,stroke:lime; 
    linkStyle 0,1,34,35 stroke-width:4px,fill:none,stroke:purple;

```

<b> This was last updated by Anthony on Feb 18, 2025</b>
