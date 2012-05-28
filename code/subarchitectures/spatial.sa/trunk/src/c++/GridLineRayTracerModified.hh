//
// = FILENAME
//    GridLineRayTracer.hh
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 1997 Patric Jensfelt
//                  1999 Patric Jensfelt
//                  2005 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef Cure_GridLineRayTracer_hh
#define Cure_GridLineRayTracer_hh

#include "Navigation/LocalGridMap.hh"
#include "Utils/HelpFunctions.hh"
#include "SensorData/SICKScan.hh"
#include "Transformation/Pose3D.hh"

namespace Cure {

/**
 * Class that helps you update a gridmap along a ray
 *
 * @author Patric Jensfelt
 * @see
 */
template<class MAPDATA>
class GridLineRayTracer {
public:
	/**
	 * Constructor
	 *
	 * @param m reference to a LocalGridMap where we want to perform ray
	 * tracing
	 */
	GridLineRayTracer(LocalGridMap<MAPDATA> &m);

	/**
	 * Initiate the starting point for the line
	 * 
	 * @param x x-coordinate for start of the ray
	 * @param y y-coordinate for start of the ray
	 * @return 0 if inside map, else error code
	 */
	int setStart(double x, double y, double angle);

	/**
	 * Use this function to access the current cell data in the ray
	 * tracing process.
	 *
	 * @return reference to cell data in the underlying LocalGridMap
	 * or error cell LocalGridMap:getMapdataError() if you are outside
	 * for example. You check for thsi by comparing the address of the
	 * cell
	 */
	MAPDATA& data();

	/** 
	 * Get the coordinates (in cells) of the current ray trace position
	 * @return 0 if inside map, else error code
	 */
	int getGridCoords(int& x, int& y) const;

	/**
	 * Get the global coordinates (in mm) of the current cursor position
	 * @return 0 if inside map, otherwise error code
	 */
	int getWorldCoords(double& x, double& y) const;

	/**
	 * @return the distance from the start point to the current
	 * ray trace position in m
	 */
	double getDistToStart() const;

	/**
	 * Use this function to step to the next cell in th eray trace process
	 * @eturns 0 if inside map, otherwise error code
	 */
	int stepRay();

	/**
	 * @retur true if ray is outside
	 */
	int isOutside() const;

	/**
	 * Add the data from a laser scan to the map
	 * 
	 * @param scan the scan data
	 * @param sp laser scanner sensor pose at time of the scan
	 * @param maxRange the max range to use when updating the map [m]
	 * @param FOV: width of the scan wedge, if other than the scan's own [rads]
	 */
	void addScan(SICKScan &scan, const Pose3D &sp, double maxRange);
	void addScan(SICKScan &scan, const Pose3D &sp, double maxRange, double FOV);

private:

	/// A reference to the map object
	LocalGridMap<MAPDATA> &m_Map;

	/// The size of the map object
	long m_Size;

	/// The size of the cells in the map
	double m_CellSize;

	/// Map coordinates as floating point values
	float m_Xf, m_Yf;

	/// Map coordinates as integer values
	int m_Xi, m_Yi;

	/// Map coordinates of the start point as floating point values
	float m_Xs, m_Ys;

	/// The direction of the ray [rad]
	double m_Angle;

	/// The direction of the ray as k in y=kx+m or x=km+y
	double m_K;

	/// Pointer to the ray tracing function best suited for the current
	/// direction
	int (GridLineRayTracer<MAPDATA>::* drawLineFcn)(void);

	/// Draw a line between -45 and +45 degs or 135 and 225 degs
	int drawLineXp(void);

	/// Draw a line between 135 and 225 degs
	int drawLineXn(void);

	/// Draw a line between 45 and 135 degs
	int drawLineYp(void);

	/// Draw a line between -135 and -45 degs
	int drawLineYn(void);

}; // class GridLineRayTracer //

template<class MAPDATA>
GridLineRayTracer<MAPDATA>::GridLineRayTracer(LocalGridMap<MAPDATA> &m) :
	m_Map(m) {
	m_Size = m_Map.getSize();
	m_CellSize = m_Map.getCellSize();
}

template<class MAPDATA> int GridLineRayTracer<MAPDATA>::setStart(double x,
		double y, double angle) {
	// Set the map coordinates
	m_Xf = (x - m_Map.getCentXW()) / m_CellSize;
	m_Yf = (y - m_Map.getCentYW()) / m_CellSize;
	m_Xi = (m_Xf < 0 ? int(m_Xf - 0.5) : int(m_Xf + 0.5));
	m_Yi = (m_Yf < 0 ? int(m_Yf - 0.5) : int(m_Yf + 0.5));

	m_Xs = m_Xf;
	m_Ys = m_Yf;

	m_Angle = Cure::HelpFunctions::mod2PI(angle);

	if ((0 <= m_Angle) && (m_Angle < M_PI / 4.0)) {
		drawLineFcn = &(GridLineRayTracer::drawLineXp);
		m_K = tan(m_Angle);
	} else if ((M_PI / 4.0 <= m_Angle) && (m_Angle < 3.0 * M_PI / 4.0)) {
		drawLineFcn = &(GridLineRayTracer::drawLineYp);
		m_K = 1 / tan(m_Angle);
	} else if ((3.0 * M_PI / 4.0 <= m_Angle) && (m_Angle < 5.0 * M_PI / 4.0)) {
		drawLineFcn = &(GridLineRayTracer::drawLineXn);
		m_K = tan(m_Angle);
	} else if ((5.0 * M_PI / 4.0 <= m_Angle) && (m_Angle < 7.0 * M_PI / 4.0)) {
		drawLineFcn = &(GridLineRayTracer::drawLineYn);
		m_K = 1 / tan(m_Angle);
	} else {
		drawLineFcn = &(GridLineRayTracer::drawLineXp);
		m_K = tan(m_Angle);
	}

	return 0;
}

template<class MAPDATA> MAPDATA&
GridLineRayTracer<MAPDATA>::data() {
	return m_Map(m_Xi, m_Yi);
}

template<class MAPDATA> int GridLineRayTracer<MAPDATA>::getGridCoords(int& x,
		int& y) const {
	x = m_Xi;
	y = m_Yi;

	// Don't move outside the map
	if ((m_Xi < -m_Size) || (m_Size < m_Xi) || (m_Yi < -m_Size)
			|| (m_Size < m_Yi)) {
		return 1;
	} else {
		return 0;
	}
}

template<class MAPDATA> int GridLineRayTracer<MAPDATA>::getWorldCoords(
		double& x, double& y) const {
	x = long(m_Map.getCentXW() + m_Xf * m_CellSize);
	y = long(m_Map.getCentYW() + m_Yf * m_CellSize);

	return 0;
}

template<class MAPDATA> double GridLineRayTracer<MAPDATA>::getDistToStart() const {
	return hypot(m_Yf - m_Ys, m_Xf - m_Xs) * m_CellSize;
}

template<class MAPDATA> int GridLineRayTracer<MAPDATA>::isOutside() const {
	if ((m_Xi < -m_Size) || (m_Size < m_Xi) || (m_Yi < -m_Size)
			|| (m_Size < m_Yi))
		return true;
	else
		return false;
}

template<class MAPDATA> int GridLineRayTracer<MAPDATA>::stepRay() {
	return (this->*drawLineFcn)();
}

template<class MAPDATA> int GridLineRayTracer<MAPDATA>::drawLineXp() {
	m_Xi++;
	m_Xf++;
	m_Yf += m_K;
	m_Yi = (0 < m_Yf ? int(m_Yf + 0.5) : int(m_Yf - 0.5));

	// Don't move outside the map
	if ((m_Xi < -m_Size) || (m_Size < m_Xi) || (m_Yi < -m_Size)
			|| (m_Size < m_Yi))
		return false;
	else
		return true;
}

template<class MAPDATA> int GridLineRayTracer<MAPDATA>::drawLineXn() {
	m_Xi--;
	m_Xf--;
	m_Yf -= m_K;
	m_Yi = (0 < m_Yf ? int(m_Yf + 0.5) : int(m_Yf - 0.5));

	// Don't move outside the map
	if ((m_Xi < -m_Size) || (m_Size < m_Xi) || (m_Yi < -m_Size)
			|| (m_Size < m_Yi))
		return false;
	else
		return true;
}

template<class MAPDATA> int GridLineRayTracer<MAPDATA>::drawLineYp() {
	m_Yi++;
	m_Yf++;
	m_Xf += m_K;
	m_Xi = (0 < m_Xf ? int(m_Xf + 0.5) : int(m_Xf - 0.5));

	// Don't move outside the map
	if ((m_Xi < -m_Size) || (m_Size < m_Xi) || (m_Yi < -m_Size)
			|| (m_Size < m_Yi))
		return false;
	else
		return true;
}

template<class MAPDATA> int GridLineRayTracer<MAPDATA>::drawLineYn() {
	m_Yi--;
	m_Yf--;
	m_Xf -= m_K;
	m_Xi = (0 < m_Xf ? int(m_Xf + 0.5) : int(m_Xf - 0.5));

	// Don't move outside the map
	if ((m_Xi < -m_Size) || (m_Size < m_Xi) || (m_Yi < -m_Size)
			|| (m_Size < m_Yi))
		return false;
	else
		return true;
}

template<class MAPDATA>
void GridLineRayTracer<MAPDATA>::addScan(Cure::SICKScan &scan,
		const Cure::Pose3D &sp, double maxRange) {
	for (int i = 0; i < scan.getNPts(); i++) {
		setStart(sp.getX(), sp.getY(), sp.getTheta() + scan.getStartAngle()
				+ scan.getAngleStep() * i);

		while (!isOutside() && (getDistToStart() < scan.getRange(i)
				|| scan.getRange(i) == 0) && getDistToStart() < maxRange) {
			data() = '0';
			stepRay();
		}
		if (!isOutside() && getDistToStart() < maxRange) {
			data() = '1';
		}
	}
}

template<class MAPDATA>
void GridLineRayTracer<MAPDATA>::addScan(Cure::SICKScan &scan,
		const Cure::Pose3D &sp, double maxRange, double FOV) {
	double startAngle = -FOV * 0.5;
	startAngle = startAngle < scan.getStartAngle() ? scan.getStartAngle()
			: startAngle;
	double fieldFraction = startAngle / scan.getStartAngle();
	int startIndex = (1 - fieldFraction) * 0.5 * scan.getNPts();
	int endIndex = scan.getNPts() - startIndex;

	for (int i = startIndex; i < endIndex; i++) {
		setStart(sp.getX(), sp.getY(), sp.getTheta() + startAngle
				+ scan.getAngleStep() * i);

		while (!isOutside() && getDistToStart() < scan.getRange(i)
				&& getDistToStart() < maxRange) {
			data() = '0';
			stepRay();
		}
		if (!isOutside() && getDistToStart() < maxRange) {
			data() = '1';
		}
	}
}

} // namespace Cure

#endif // ISR_GridLineRayTracer_h
