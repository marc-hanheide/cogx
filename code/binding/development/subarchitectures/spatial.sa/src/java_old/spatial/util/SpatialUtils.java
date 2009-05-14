/**
 * 
 */
package spatial.util;

import org.cognitivesystems.common.autogen.Math.Vector3D;

import spatial.autogen.SpatialRelationshipType;
import spatial.autogen.SpatialScene;

/**
 * @author nah
 */
public class SpatialUtils {

	/**
	 * 
	 */
	public static final String NEAR = "near";

	/**
	 * 
	 */
	public static final String BACK_OF = "back";

	/**
	 * 
	 */
	public static final String FRONT_OF = "front";

	/**
	 * 
	 */
	public static final String RIGHT_OF = "right";

	/**
	 * 
	 */
	public static final String LEFT_OF = "left";

	public static float distance(Vector3D _v1, Vector3D _v2) {
		return (float) Math.sqrt(Math.pow(_v1.m_x - _v2.m_x, 2)
				+ Math.pow(_v1.m_y - _v2.m_y, 2)
				+ Math.pow(_v1.m_z - _v2.m_z, 2));
	}

	/**
	 * @param _scene
	 * @return
	 */
	public static String toString(SpatialScene _scene) {
		String out = "[SpatialScene:";
		out += " [" + _scene.m_proximityRelationships.length + " prox]";
		out += " [" + _scene.m_frontRelationships.length + " front]";
		out += " [" + _scene.m_backRelationships.length + " back]";
		out += " [" + _scene.m_leftRelationships.length + " left]";
		out += " [" + _scene.m_rightRelationships.length + " right]";
		out += " ";
		for (int i = 0; i < _scene.m_locations.length; i++) {
			out += _scene.m_locations[i] + " ";
		}
		out += "]";
		return out;
	}

	public static String enum2string(SpatialRelationshipType _label)
			throws SpatialSubarchitectureException {
		if (_label == SpatialRelationshipType.SPATIAL_LEFT) {
			return LEFT_OF;
		}
		else if (_label == SpatialRelationshipType.SPATIAL_RIGHT) {
			return RIGHT_OF;
		}
		else if (_label == SpatialRelationshipType.SPATIAL_FRONT) {
			return FRONT_OF;
		}
		else if (_label == SpatialRelationshipType.SPATIAL_BACK) {
			return BACK_OF;
		}
		else if (_label == SpatialRelationshipType.SPATIAL_PROXIMAL) {
			return NEAR;
		}

		throw new SpatialSubarchitectureException("unknown rel type: " + _label);
	}

	public static SpatialRelationshipType string2enum(String _label)
			throws SpatialSubarchitectureException {
		if (_label.equals(LEFT_OF)) {
			return SpatialRelationshipType.SPATIAL_LEFT;
		}
		else if (_label.equals(RIGHT_OF)) {
			return SpatialRelationshipType.SPATIAL_RIGHT;
		}
		else if (_label.equals(FRONT_OF)) {
			return SpatialRelationshipType.SPATIAL_FRONT;
		}
		else if (_label.equals(BACK_OF)) {
			return SpatialRelationshipType.SPATIAL_BACK;
		}
		else if (_label.equals(NEAR)) {
			return SpatialRelationshipType.SPATIAL_PROXIMAL;
		}

		throw new SpatialSubarchitectureException("unknown rel type: " + _label);
	}

	// public static boolean isSpatialRelation(RelationLabel _rl) {
	// return isSpatialRelation(_rl.m_label);
	// }

	public static boolean isSpatialRelation(String _rl) {
		return _rl.equals(SpatialUtils.LEFT_OF)
				|| _rl.equals(SpatialUtils.RIGHT_OF)
				|| _rl.equals(SpatialUtils.FRONT_OF)
				|| _rl.equals(SpatialUtils.BACK_OF)
				|| _rl.equals(SpatialUtils.NEAR);
	}
}
