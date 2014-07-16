Gearticks-IMU-MARG
==================

Repo for MARG/IMU arduino files we develop

####Prerequisites:
  - Download the files [here: https://github.com/sparkfun/MPU-9150_Breakout](https://github.com/sparkfun/MPU-9150_Breakout)
  - Follow the instructions [here: https://github.com/sparkfun/MPU-9150_Breakout/tree/master/firmware](https://github.com/sparkfun/MPU-9150_Breakout/tree/master/firmware)

> To use the library, copy both I2Cdev and MPU6050 into your libraries folder in your main Arduino directory. If you don't have a folder called libraries, create one and drop both files in there. Now you can goto the example sketch named MPU9150_raw (in MPU6050 -> Examples), open and run the main .ino file.

####FAQ:

 - Why does my serial monitor spit out gibberish?
     - Set its frequency to 38400 baud

####Useful readings
 - Algorithm we are using: [http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf](http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf)
 - Quaternions: [http://www.x-io.co.uk/res/doc/quaternions.pdf](http://www.x-io.co.uk/res/doc/quaternions.pdf)
 - Euler angles to Roll/Pitch/Yaw: [http://personal.maths.surrey.ac.uk/st/T.Bridges/SLOSH/3-2-1-Eulerangles.pdf](http://personal.maths.surrey.ac.uk/st/T.Bridges/SLOSH/3-2-1-Eulerangles.pdf)
