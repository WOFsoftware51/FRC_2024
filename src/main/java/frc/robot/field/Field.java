package frc.robot.field;

import edu.wpi.first.math.geometry.Pose3d;

public class Field {
    
    private Field(){

    }
    public class Blue { 
        public static AprilTag kAprilTag7 = new AprilTag(7, new Pose3d());
        public static AprilTag kAprilTag8 = new AprilTag(8, new Pose3d());
        public static AprilTag kAprilTag6 = new AprilTag(6, new Pose3d());

    }

    public class Red {
        public static AprilTag kAprilTag3 = new AprilTag(3, new Pose3d());
        public static AprilTag kAprilTag4 = new AprilTag(4, new Pose3d());
        public static AprilTag kAprilTag5 = new AprilTag(5, new Pose3d());
    }

    public static AprilTag kAprilTag13 = new AprilTag(13, new Pose3d());
    public static AprilTag kAprilTag14 = new AprilTag(14, new Pose3d());
    public static AprilTag kAprilTag15 = new AprilTag(15, new Pose3d());
    public static AprilTag kAprilTag1 = new AprilTag(1, new Pose3d());
    public static AprilTag kAprilTag2 = new AprilTag(2, new Pose3d());
    public static AprilTag kAprilTag9 = new AprilTag(9, new Pose3d());
    public static AprilTag kAprilTag10 = new AprilTag(10, new Pose3d());
}
