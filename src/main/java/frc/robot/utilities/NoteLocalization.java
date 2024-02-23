// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;


/** In reality this class doesn't actually localize the Note in 2D Space but instead
 * calcualtes the disance from the nearest point one a measured line that represents 
 * the heading of the robot. Make sure to locale the center of your camera to the 
 * bottom left hand corner
 */
public class NoteLocalization {

    //Point A is the position of the head of the robot relative to the camera in photon vision
    private static double[] pointA = convertAnglesToPixels(-11.8, -8.63);
    //Point B is a point along a perpendicule line to the front edge of the robot
    private static double[] pointB = convertAnglesToPixels(-4.62, 5.32);



    /** takes the angle displacements from photon vision and converts them to an arbitrary
     * coordinate system of 1500 x 1000. Note that this is specific to a field of view of 70 degrees and
     * may require tuning if using any other FOV. Also in this case the x axis is 60 degrees in full and
     * the y axis is 40 degrees in full. 
     * @param yawAngle the displacement as an angle in the x direction that photon vision gives you of some point
     * @param pitchAngle the displacement as an angle in the y direction that photon vision gives you of some point
    */
    public static double[] convertAnglesToPixels(double yawAngle, double pitchAngle){
        double xCoordinate = 0, yCoordintate = 0;

        yawAngle = yawAngle + 30; // Adjust yaw to range 0 to 60
        pitchAngle = pitchAngle + 20; // Adjust pitch to range 0 to 40
        
        xCoordinate = (yawAngle / 60) * 1500;
        yCoordintate = (pitchAngle / 40) * 1000;
        return new double[]{xCoordinate, yCoordintate};
        
    }

    /**
     * 
     * @param pointC the coordinate of the note. Should already be converted to the relative coordinate system
     * @return returns a distance from the nearest point of the line of heading
     */
    public static double getSignedDistanceFromNearestPathToNote(double[] pointC){
    //turn point C into its relative Coordinate frame

    double[] pointD = calculatePointD(pointC);
    double lengthCD = calculateDistanceBetweenTwoPoints(pointC, pointD);
    // Calculate the slope of line AB
    double slopeAB = (pointB[1] - pointA[1]) / (pointB[0] - pointA[0]);
    // Calculate the y-intercept of line AB
    double interceptAB = pointA[1] - slopeAB * pointA[0];
    // Calculate the equivalent Y along the line AB for the X coordinate of point C
    double equivalentYAlongLine = slopeAB * pointC[0] + interceptAB;

    //if the point is to the left of the line make the distance from the line negative
    if(equivalentYAlongLine > pointC[1])
        lengthCD *= -1;

    return lengthCD;
    }

    /**
     * Calcultes the point D along the line AB that creates a perpendicular line CD. Assumes that 
     * point the point C has already been converted 
     * @param pointC an array representing the position of the
     * @return
     */
    public static double[] calculatePointD(double[] pointC){
            //calculate some needed values which come from parametric equations
    double a = pointB[0] - pointA[0];
    double b = pointB[1] - pointA[1];

    //calculate the paramter t
    double numerator = (pointB[0] - pointA[0]) * (pointC[0] - pointA[0]) + (pointB[1] - pointA[1]) * (pointC[1] - pointA[1]);
    double denominator = Math.pow(pointB[0] - pointA[0], 2) + Math.pow(pointB[1] - pointA[1], 2);
    double t = numerator / denominator;

    //create the point D and its consequential line CD
    return new double[]{pointA[0] + a * t, pointA[1]+ b * t};
    }


    /**
     * performs the same calculation as getSignedDistanceFromNearestPathToNote but is unsigned
     * @param pointC the coordinate of the note. Should already be converted to the relative coordinate system
     * @return the unsigned distance from the note to a perpindicular point along line AB
     */
    public static double calculateDistanceFromNearestNoteToPath(double[] pointC){
        return Math.abs(getSignedDistanceFromNearestPathToNote(pointC));
    }

    /**
     * provides a custom implementation for finding the best note to target in an array of visible notes.
     * prioritizes the vertical distance from the heading of the robot but still takes horizontal factor into
     * account.
     * @param notes an array containing all the relative positions of the notes from photonvision. Should not 
     * be parameterized to relative coordinate system
     * @return the best note in the array to target
     */
    public static double[] calculateBestTarget(double[][] notes){
        int bestNote = 0;
        double smallestDistance = Double.POSITIVE_INFINITY;
        for(int i = 0; i < notes.length; i++){
            notes[i] =  convertAnglesToPixels(notes[i][0], notes[i][1]);
            double horizontalDistance = calculateDistanceFromNearestNoteToPath(notes[i]);
            double verticalDistance = calculateDistanceBetweenTwoPoints(calculatePointD(notes[i]), pointA);
            double verticalScaler = 1;
            double scaledDistance = Math.pow(Math.pow(verticalDistance * verticalScaler, 2) + Math.pow(horizontalDistance, 2), 0.5);
            if(scaledDistance < smallestDistance){
                bestNote = i;
            }
        }
        return notes[bestNote];
    }

    /**
     * the distance between two points given such as line AB
     * @param point1 some 2D point
     * @param point2 another 2D point
     * @return the length between two points
     */
    public static double calculateDistanceBetweenTwoPoints(double[] point1, double[] point2){
        return Math.sqrt(Math.pow(point2[0] - point1[0], 2) + Math.pow(point2[1] - point1[1], 2));
    }


}
