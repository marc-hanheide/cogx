/**
 * 
 */
package org.cognitivesystems.spatial.pf;

/**
 * @author nah
 */
public class PotentialFieldPoint {

    private float m_x;
    private float m_y;
    private float m_val;

    public PotentialFieldPoint(float _x, float _y, float _val) {
        m_x = _x;
        m_y = _y;
        m_val = _val;
    }

    public float getValue() {
        return m_val;
    }

    public float getX() {
        return m_x;
    }

    public float getY() {
        return m_y;
    }

    public void setValue(float _val) {
        m_val = _val;
    }

    public void setX(float _x) {
        m_x = _x;
    }

    public void setY(float _y) {
        m_y = _y;
    }

}
