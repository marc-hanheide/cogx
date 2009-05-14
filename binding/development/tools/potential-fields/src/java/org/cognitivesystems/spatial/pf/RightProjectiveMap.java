package org.cognitivesystems.spatial.pf;


/**
 * 
 */

/**
 * Input into this class is in metres, so 1.0 is one metre. The
 * underlying map representation uses 1 cm grid squares, so objects
 * specified in greater detail will be abstracted over.
 * 
 * @author nah
 */
public class RightProjectiveMap extends ProjectiveMap {


    /**
     * The constructor needs to specify the bounding box of the scene.
     */
    public RightProjectiveMap(float _blx, float _bly, float _trx,
            float _try) {
        super(_blx, _bly, _trx, _try, ProjectiveDirection.RIGHT);
    }

}
