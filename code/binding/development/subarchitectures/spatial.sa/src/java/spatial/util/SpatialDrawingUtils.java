/**
 * 
 */
package spatial.util;

import java.awt.Color;

import org.cognitivesystems.common.autogen.Math.*;

import cast.cdl.guitypes.*;
import cast.core.components.InspectableComponent;

/**
 * @author nah
 */
public class SpatialDrawingUtils {

    private static final int POINT_SIZE = 1;
    private static final int ORIGIN_OFFSET = 300;
    private static final int ARROW_LENGTH = 40;

    private static double length(Vector2D v) {
        return Math.sqrt(v.m_x * v.m_x + v.m_y * v.m_y);
    }

    private static double length(Vector3D v) {
        return Math.sqrt(v.m_x * v.m_x + v.m_y * v.m_y + v.m_z * v.m_z);
    }

    private static double[][] rodrigues(Vector3D r) {

        double th = length(r);

        if (th != 0) {
            double[][] R = new double[3][3];
            double x = r.m_x / th, y = r.m_y / th, z = r.m_z / th;
            double co = Math.cos(th), si = Math.sin(th);
            R[0][0] = x * x * (1. - co) + co;
            R[0][1] = x * y * (1. - co) - z * si;
            R[0][2] = x * z * (1. - co) + y * si;
            R[1][0] = x * y * (1. - co) + z * si;
            R[1][1] = y * y * (1. - co) + co;
            R[1][2] = y * z * (1. - co) - x * si;
            R[2][0] = x * z * (1. - co) - y * si;
            R[2][1] = y * z * (1. - co) + x * si;
            R[2][2] = z * z * (1. - co) + co;
            return R;
        }
        else {
            return identity3x3();
        }
    }

    private static double[][] identity3x3() {
        double[][] M = new double[3][3];
        int i, j;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                M[i][j] = 0.;
            }
        }
        M[0][0] = M[1][1] = M[2][2] = 1.;

        return M;
    }

    private static Vector3D rotate(Vector3D r, Vector3D p) {

        double R[][];
        R = rodrigues(r);
        Vector3D q = new Vector3D();
        q.m_x =
                (float) (R[0][0] * p.m_x + R[0][1] * p.m_y + R[0][2]
                    * p.m_z);
        q.m_y =
                (float) (R[1][0] * p.m_x + R[1][1] * p.m_y + R[1][2]
                    * p.m_z);
        q.m_z =
                (float) (R[2][0] * p.m_x + R[2][1] * p.m_y + R[2][2]
                    * p.m_z);
        return q;
    }

    public static void drawPose(InspectableComponent _comp,
                                Pose3D _pose,
                                Color _color) {

        // START HERE

//         System.out.println("orientation: " + _pose.m_orientation.m_x
//         + " " + _pose.m_orientation.m_y + " "
//         + _pose.m_orientation.m_z);
//        
//         System.out.println("position: " + _pose.m_position.m_x + " "
//         + _pose.m_position.m_y + " " + _pose.m_position.m_z);

        drawPoint(_comp, _pose.m_position.m_x, -_pose.m_position.m_y,
            _color);

        // System.out.println("length: " + length(_pose.m_orientation));
        //        
        // double theta =
        // Math.atan(_pose.m_orientation.m_z
        // / _pose.m_orientation.m_x);
        //
        // double diffX = Math.cos(theta) * ARROW_LENGTH;
        // double diffZ = Math.sin(theta) * ARROW_LENGTH * -1; // invert
        //
        // System.out.println(diffX + " " + diffZ);
        //
        // int startX = world2Pixel(_pose.m_position.m_x);
        // // y to y
        // int startY = world2Pixel(-1 * _pose.m_position.m_y);
        //
        // int endX = startX + Math.round((float) diffX);
        // int endY = startY+ Math.round((float) diffZ);
        //
        // _comp.drawLine2D(startX, startY, endX, endY, _color.getRed(),
        // _color.getGreen(), _color.getBlue(), NONE.value);
        // // _comp.drawLine2D(startX, starty, world2Pixel(0f),
        // // world2Pixel(0f), _color.getRed(), _color.getGreen(),
        // _color
        // // .getBlue(), NONE.value);

    }

    /**
     * @param _comp
     * @param _i
     * @param _i2
     */
    public static void drawPoint(InspectableComponent _comp,
                                 float _x,
                                 float _y,
                                 Color _colour) {

        drawPoint(_comp, world2Pixel(_x), world2Pixel(_y), _colour);
    }

    /**
     * @param _x
     * @return
     */
    private static int world2Pixel(float _x) {
        return Math.round(_x * 600) + ORIGIN_OFFSET;
    }

    /**
     * @param _comp
     * @param _i
     * @param _i2
     */
    private static void drawPoint(InspectableComponent _comp,
                                  int _x,
                                  int _y,
                                  Color _colour) {

//        System.out.println("draw: " + _x + " " + _y);

        _comp.drawPoint2D(_x, _y, _colour.getRed(), _colour.getGreen(),
            _colour.getBlue(), FAT.value);
    }

    public static Point2D bottomLeft(Vector3D _bbox) {
        float centreX = _bbox.m_x;
        float centreY = _bbox.m_y;

        float halfSizeX = 10;
        float halfSizeY = 10;

        return new Point2D(centreX - halfSizeX, centreY - halfSizeY,
            null, 0);
    }

    public static Point2D bottomRight(Vector3D _bbox) {
        float centreX = _bbox.m_x;
        float centreY = _bbox.m_y;

        float halfSizeX = 10;
        float halfSizeY = 10;

        return new Point2D(centreX + halfSizeX, centreY - halfSizeY,
            null, 0);
    }
    
    public static Point2D topRight(Vector3D _bbox) {
        float centreX = _bbox.m_x;
        float centreY = _bbox.m_y;

        float halfSizeX = 10;
        float halfSizeY = 10;

        return new Point2D(centreX + halfSizeX, centreY + halfSizeY,
            null, 0);
    }

    public static Point2D topLeft(Vector3D _bbox) {
        float centreX = _bbox.m_x;
        float centreY = _bbox.m_y;

        float halfSizeX = 10;
        float halfSizeY = 10;

        return new Point2D(centreX - halfSizeX, centreY + halfSizeY,
            null, 0);
    }
    
    /**
     * @param _comp
     * @param _bbox
     * @param _colour
     */
    public static void drawBBox(InspectableComponent _comp,
                                Vector3D _bbox,
                                Color _colour) {
        float centreX = _bbox.m_x;
        float centreY = -_bbox.m_y;

        float halfSizeX = 10;
        float halfSizeY = 10;

        _comp.drawRect2D(world2Pixel(centreX - halfSizeX),
            world2Pixel(centreY - halfSizeY), world2Pixel(centreX
                + halfSizeX), world2Pixel(centreY + halfSizeY), _colour
                .getRed(), _colour.getGreen(), _colour.getBlue(),
            NONE.value);

    }

    /**
     * @param _component
     * @param _x1
     * @param _y1
     * @param _x2
     * @param _y2
     * @param _value
     * @param _colour
     */
    public static void drawLine(InspectableComponent _comp,
                                float _x1,
                                float _y1,
                                float _x2,
                                float _y2,
                                float _value,
                                Color _colour) {

        _comp.drawLine2D(world2Pixel(_x1), world2Pixel(-_y1),
            world2Pixel(_x2), world2Pixel(-_y2), _colour.getRed(),
            _colour.getGreen(), _colour.getBlue(), FAT.value);

    }

}
