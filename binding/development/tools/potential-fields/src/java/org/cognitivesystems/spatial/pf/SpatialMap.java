/**
 * 
 */
package org.cognitivesystems.spatial.pf;

/**
 * @author nah
 */
public class SpatialMap {

    protected float m_blx;
    protected float m_bly;
    protected float m_trx;
    protected float m_try;

    /**
     * 
     */
    public SpatialMap(float _blx, float _bly, float _trx, float _try) {
        m_blx = _blx;
        m_bly = _bly;
        m_trx = _trx;
        m_try = _try;
    }

    protected static float centimetres2Metres(float _metres) {
        return _metres / 100f;
    }

    protected static float metres2Centimetres(float _metres) {
        return _metres * 100f;
    }

    protected float translateXToMap(float _x) {
        return _x - m_blx;
    }

    protected float translateYToMap(float _y) {
        return _y - m_bly;
    }

    protected float translateXFromMap(float _x) {
        return _x + m_blx;
    }

    protected float translateYFromMap(float _y) {
        return _y + m_bly;
    }

    
    protected boolean isInMap(float _x, float _y) {
    	return (_x > m_blx && _x < m_trx) && (_y > m_bly && _y < m_try);
    }
    
}
