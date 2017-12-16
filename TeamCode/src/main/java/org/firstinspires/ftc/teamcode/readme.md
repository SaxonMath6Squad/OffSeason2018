## Team 11528: The Bots of Prey 2017-2018 Code

## Robot specifications:
    4 Omi Wheels, each driven by a NeveRest 40 Motor. 

## Autonomous Outline
    OpMode:
      NavigationGuidance: 
            DriveEngine: 
      RobotTelemetry: 
      ActionHandler: 
    
Every Autonomous OpMode will be given a file with scripted movement and action commands. The Autonomous planner is given this file, which it parses. Movement commands will be parsed into an AutonomousMovementCommand child, which is specialized for "drive on heading"(with a degree heading, speed, etc.) "turn to orientation" (with a degree) and other commands. These will also have an arraylist of AutonomousInterrupts, which will be used to stop the robot while performing an operation. The most basic one will be to stop it when the stop button is pressed. Further examples include a max time limit for an operation, a distance to go, or for some hardware trigger to be fired. 

    
The ActionHandler is likewise treated with an AutonomousInterrupts Arraylist. This will also be given a AutonomousActionCommand child, which is likewise specialized per action. ActionHandler will also include a way to thread off operations, each linked to a specific child type, that can be stopped at a later time in the scripting. 


## User Controlled Outline
    OpMode
        DriveEngine 
        RobotTelemetry 
        ActionHandler        

The tele-op will remove most of the autonomous structure. The OpMode will directly read the controller input and directly pass in command to the DriveEngine and ActionHandler. These will not include an AutonomousInterrupt, but instead a check on the opMode status to ensure they are alive. The DriveEngine, via a setting which can be toggled, can flip between using the IMU and GeoLocator for position and orientation updates and not. Parts of the Threadable ActionHandler will still be utilized . 


## Class Descriptions
    MotorHandler:
        Handles Speed and Distance control on motors
        Replaces the standard Modern Robotics supplied algorithms
        Runs own thread to automatically update Tachometer data and to adjust power to match Speed
        Cuttently uses P control on Motors, good enough so no need for PID control
        Does have an acceleration though, will fix this later
    
    PIDController
       Pid controller, what else needs to be said? Has option for I_Cap, use if needed

    JsonConfigReader
        Used for any config files, enables other classes to retrieve info about motor values, etc.

    JoystickHandler
        used to simplify joystick controls, computes angle and magintude of joysticks
        0 deg is straight ahead on joystick controller

    MotorTachometer
        Utlizes encoder data to compute rps
        requires to be called frequently, ideally utilized in a thread in a MotorHandler
