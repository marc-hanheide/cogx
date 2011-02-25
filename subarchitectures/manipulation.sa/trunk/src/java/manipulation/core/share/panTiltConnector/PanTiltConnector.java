package manipulation.core.share.panTiltConnector;

import manipulation.core.share.types.PTZPosition;

/**
 * represents a connection to a pan tilt unit
 * 
 * @author ttoenige
 * 
 */
public interface PanTiltConnector {
	public void setPoseRad(PTZPosition position);

	public void setPoseDeg(PTZPosition position);

	public PTZPosition getPoseRad();

	public PTZPosition getPoseDeg();
}