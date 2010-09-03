/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package celm.plugins;

import java.util.Vector;

/**
 *
 * @author harmish
 */
public interface Plugin
{
    public Vector<Object> toVector(Ice.ObjectImpl iceObject);
}
