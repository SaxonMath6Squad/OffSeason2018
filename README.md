##Welcome to the Bots Of Prey Github Repository! 

This code is for the 2017-2018 Relic Recovery code. A few key features of are code: 

    Holonomic Drive System: 
        Our robot operates on a 4 wheel 90 degree offset drive using NeverRest 40:1 Motors. All drive kinematics is built to only operate
        on this setup. 
        
    Vector Location System (VLS): 
        Running in conjunction with our kinematic system is a vector location system. This uses information from the wheel encoders and
        rev IMU to extrapolate our current position on the field. 
        
    Drive to Location System: 
        Using our VLS and game element locations defined in the RelicRecovery.java, our robot is able to drive to locations to place or 
        park elements. 
        
    Cryptobox Image Detection (CID): 
        Using our own algorithms, and access to the camera images thanks to Vuforia, we are able to detect the XY locations of 
        cryptoboxes. This allows us to automatically center the robot on cryptobox columns during autonomous and user controlled for
        easier placement. 
        
        
