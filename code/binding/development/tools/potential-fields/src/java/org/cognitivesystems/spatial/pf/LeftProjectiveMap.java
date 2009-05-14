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
public class LeftProjectiveMap extends ProjectiveMap {


    /**
     * The constructor needs to specify the bounding box of the scene.
     */
    public LeftProjectiveMap(float _blx, float _bly, float _trx,
            float _try) {
        super(_blx, _bly, _trx, _try, ProjectiveDirection.LEFT);
    }

    /**
     * @param args
     * @throws SpatialMapException 
     */
    public static void main(String[] args) throws SpatialMapException {

        ProjectiveMap projMap = new LeftProjectiveMap(-0.5f, -0.5f, 0.5f,
            0.5f);

        projMap.setViewerPosition(0.0f, -1f, 0);
        projMap.addObject("obj0", 0.05f, -0.05f, 0);
        projMap.addObject("obj1", -0.05f, -0.05f, 0);
        projMap.addObject("obj2", -0.05f, 0.05f, 0);
        projMap.addObject("obj3", -0.05f, -0.02f, 0);

        // proxMap.proximityMap("obj2");
        //
        // for (int i = 0; i < 10; i++) {
        // PotentialFieldPoint pfp = proxMap.nextSweetSpot();
        // System.out.println("sweet spot: " + pfp.getX() + ","
        // + pfp.getY() + " " + pfp.getValue());
        // }

        System.out.println(projMap.projectiveValue("obj3", "obj2"));
        System.out.println(projMap.projectiveValue("obj3", "obj1"));
        System.out.println(projMap.projectiveValue("obj3", "obj0"));

        System.out.println(projMap.projectiveValue("obj2", "obj3"));
        System.out.println(projMap.projectiveValue("obj2", "obj1"));
        System.out.println(projMap.projectiveValue("obj2", "obj0"));

        System.out.println(projMap.projectiveValue("obj1", "obj3"));
        System.out.println(projMap.projectiveValue("obj1", "obj2"));
        System.out.println(projMap.projectiveValue("obj1", "obj0"));

        System.out.println(projMap.projectiveValue("obj0", "obj3"));
        System.out.println(projMap.projectiveValue("obj0", "obj2"));
        System.out.println(projMap.projectiveValue("obj0", "obj1"));

        projMap = new RightProjectiveMap(-0.5f, -0.5f, 0.5f,
            0.5f);

        projMap.setViewerPosition(0.0f, -1f, 0);
        projMap.addObject("obj0", 0.05f, -0.05f, 0);
        projMap.addObject("obj1", -0.05f, -0.05f, 0);
        projMap.addObject("obj2", -0.05f, 0.05f, 0);
        projMap.addObject("obj3", -0.05f, -0.02f, 0);

        // proxMap.proximityMap("obj2");
        //
        // for (int i = 0; i < 10; i++) {
        // PotentialFieldPoint pfp = proxMap.nextSweetSpot();
        // System.out.println("sweet spot: " + pfp.getX() + ","
        // + pfp.getY() + " " + pfp.getValue());
        // }

        System.out.println(projMap.projectiveValue("obj3", "obj2"));
        System.out.println(projMap.projectiveValue("obj3", "obj1"));
        System.out.println(projMap.projectiveValue("obj3", "obj0"));

        System.out.println(projMap.projectiveValue("obj2", "obj3"));
        System.out.println(projMap.projectiveValue("obj2", "obj1"));
        System.out.println(projMap.projectiveValue("obj2", "obj0"));

        System.out.println(projMap.projectiveValue("obj1", "obj3"));
        System.out.println(projMap.projectiveValue("obj1", "obj2"));
        System.out.println(projMap.projectiveValue("obj1", "obj0"));

        System.out.println(projMap.projectiveValue("obj0", "obj3"));
        System.out.println(projMap.projectiveValue("obj0", "obj2"));
        System.out.println(projMap.projectiveValue("obj0", "obj1"));

    }

}
