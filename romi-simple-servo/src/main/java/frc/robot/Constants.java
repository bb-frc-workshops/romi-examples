// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class Joystick {
        // Button mapping for a Logitech F310 gamepad. 
        public static final int X = 1;
        public static final int A = 2;
        public static final int B = 3;
        public static final int Y = 4;
        public static final int TOPLEFT = 5;
        public static final int TOPRIGHT = 6;
        public static final int BOTTOMLEFT = 7;
        public static final int BOTTOMRIGHT = 8;
        public static final int BACK = 9;
        public static final int START = 10;
        public static final int LEFT_ANALOG = 11;
        public static final int RIGHT_ANALOG = 12;
        public static final int LEFT_ANALOG_LEFT_RIGHT_AXIS = 0;
        public static final int LEFT_ANALOG_UP_DOWN_AXIS = 1;
        public static final int RIGHT_ANALOG_LEFT_RIGHT_AXIS = 2;
        public static final int RIGHT_ANALOG_UP_DOWN_AXIS = 3;   
    }
    // Taken from "Robot Port" specified in the Romi configuration 
    // on the Web UI at http://10.0.0.2 or http://wpilibpi.local
    public final class GPIO {
        public static final int EXT0_DIGITAL = 8;
        public static final int EXT1_ANALOG_IN = 0;
        public static final int EXT2_PWM = 2;
        public static final int EXT3_PWM = 3;
        public static final int EXT4_PWM = 4;
    }
}
