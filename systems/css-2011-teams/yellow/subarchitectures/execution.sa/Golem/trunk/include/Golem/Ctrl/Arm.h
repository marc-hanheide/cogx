/** @file Arm.h
 * 
 * Implementation of arm controller interface.
 * 
 * Joint Control interface is a set of classes modelling a generic open-chain manipulator. 
 * The robotic arm (master) is assumed to consist of a number of independent 
 * joints (slaves).
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_ARM_H_
#define _GOLEM_CTRL_ARM_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Math/Math.h>
#include <Golem/Math/Mat34.h>
#include <Golem/Math/Quat.h>
#include <Golem/Math/Bounds.h>
#include <Golem/Math/Queue.h>
#include <Golem/Tools/Library.h>
#include <Golem/Tools/Context.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Ctrl/Profile.h>
#include <vector>
#include <list>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Time-indexed state.
 * 
 * <code>State</code> generates the input and output states of the 
 * system basing on the template description <code>State</code>.
 */
template <typename Type>
class State : public Type {
public:
	typedef std::vector<State> Seq;

	/** time stamp */
	SecTmReal t;

	State(SecTmReal t = SEC_TM_REAL_ZERO) : t(t) {
	}
	State(const Type& type, SecTmReal t = SEC_TM_REAL_ZERO) : Type(type), t(t) {
	}

	inline bool operator < (const State<Type>& s) const {
		return this->t < s.t;
	}
	inline bool operator <= (const State<Type>& s) const {
		return this->t <= s.t;
	}

	inline friend bool operator < (const State<Type>& s, SecTmReal t) {
		return s.t < t;
	}
	inline friend bool operator < (SecTmReal t, const State<Type>& s) {
		return t < s.t;
	}
	inline friend bool operator <= (const State<Type>& s, SecTmReal t) {
		return s.t <= t;
	}
	inline friend bool operator <= (SecTmReal t, const State<Type>& s) {
		return t <= s.t;
	}
};

//------------------------------------------------------------------------------

/** (Extended) generalised coordinates.
 */
struct GenCoord {
	/** position */
	Real pos;
	/** velocity */
	Real vel;
	/** acceleration */
	Real acc;

	/** Default constructor does not do anything */
	GenCoord() {
	}
	
	/** Constructs GenCoord with specified pose, pose velocity and pose acceleration */
	GenCoord(Real pos, Real vel, Real acc) : pos(pos), vel(vel), acc(acc) {
	}

	/** sets zero */
	inline void setZero() {
		pos = REAL_ZERO;
		vel = REAL_ZERO;
		acc = REAL_ZERO;
	}

	/** tests for finite vector
	*/
	inline bool isFinite() const {
		if (!Math::isFinite(pos) || !Math::isFinite(vel) || !Math::isFinite(acc))
			return false;
		return true;
	}
};

/** Generalised coordinates in time.
 */
typedef State<GenCoord> GenState;

/** Generalised Joint trajectory is a set of velocity profiles */
class GenCoordTrj : public Polynomial4 {
public:
	/** Trajectory description */
	class Desc : public Polynomial4::Desc {
	public:
		/** Creates object from the description. */
		CREATE_FROM_OBJECT_DESC0(GenCoordTrj, Profile::Ptr)
	};
	
	/** Default constructor */
	GenCoordTrj();

	GenCoordTrj(Real t0, Real t1, const GenCoord& c0, const GenCoord& c1);
	
	void set(Real t0, Real t1, const GenCoord& c0, const GenCoord& c1);

	GenCoord get(Real t) const;
};

//------------------------------------------------------------------------------

/** Max configuration space dimensions */
const U32 CONFIG_SPACE_DIM = 6;

/* SI Units:
 * - position/angle: meter [m], radian [rad]
 * - time: second [sec]
 * - velocity: [m/sec], [rad/sec]
 * - acceleration: [m/sec*sec], [rad/sec*sec]
 * - weight: kilogram [kg]
 * - force: Newton [N]
 */

/** Configuration space coordinates of the arm.
 */
struct ConfigspaceCoord {
	typedef std::vector<ConfigspaceCoord> Seq;

	/** configuration space coordinates */
	Real c[CONFIG_SPACE_DIM];

	/**	Sets configuration space coordinates to the specified value. */
	inline void set(U32 index, Real value) {
		c[index] = value;
	}
	
	/**	Sets configuration space coordinates to the specified value. */
	inline void set(Real value) {
		for (U32 i = 0; i < CONFIG_SPACE_DIM; i++)
			c[i] = value;
	}

	/** sets zero */
	inline void setZero() {
		set(REAL_ZERO);
	}

	/**	Sets configuration space coordinates to the specified value. */
	inline Real get(U32 index) const {
		return c[index];
	}

	/** tests for exact zero vector
	*/
	inline bool isZero() const {
		for (U32 i = 0; i < CONFIG_SPACE_DIM; i++)
			if (!Math::abs(c[i]))
				return false;
		return true;
	}

	/** tests for positive vector
	*/
	inline bool isPositive() const {
		for (U32 i = 0; i < CONFIG_SPACE_DIM; i++)
			if (c[i] <= REAL_ZERO)
				return false;
		return true;
	}

	/** tests for finite vector
	*/
	inline bool isFinite() const {
		for (U32 i = 0; i < CONFIG_SPACE_DIM; i++)
			if (!Math::isFinite(c[i]))
				return false;
		return true;
	}
	
	/**	Array subscript operator. */
	inline const Real &operator [] (U32 index) const {
		return c[index];
	}

	/**	Array subscript operator. */
	inline Real &operator [] (U32 index) {
		return c[index];
	}
};

/** Joint coordinates of the arm in time.
 */
typedef State<ConfigspaceCoord> JointState;

/** (Extended) generalised coordinates of the arm joints.
 */
struct GenConfigspaceCoord {
	typedef std::vector<GenConfigspaceCoord> Seq;
	
	/** pose */
	ConfigspaceCoord pos;
	/** pose velocity */
	ConfigspaceCoord vel;
	/** pose acceleration */
	ConfigspaceCoord acc;

	/**	Sets generalised configuration space coordinates to the specified value. */
	inline void set(U32 index, const GenCoord &value) {
		pos.set(index, value.pos);
		vel.set(index, value.vel);
		acc.set(index, value.acc);
	}
	
	/**	Sets configuration space coordinates to the specified value. */
	inline void set(const GenCoord &value) {
		pos.set(value.pos);
		vel.set(value.vel);
		acc.set(value.acc);
	}

	/** sets zero */
	inline void setZero() {
		pos.setZero();
		vel.setZero();
		acc.setZero();
	}

	/**	Sets configuration space coordinates to the specified value. */
	inline GenCoord get(U32 index) const {
		return GenCoord(pos.get(index), vel.get(index), acc.get(index));
	}

	/** tests for finite vector
	*/
	inline bool isFinite() const {
		if (!pos.isFinite() || !vel.isFinite() || !acc.isFinite())
			return false;
		return true;
	}
};

/** Generalised configuration space coordinates of the arm in time.
 */
typedef State<GenConfigspaceCoord> GenConfigspaceState;

//------------------------------------------------------------------------------

/** Workspace coordinates.
 */
typedef Mat34 WorkspaceCoord;

/** Workspace velocity.
 */
typedef Twist WorkspaceVel;

/** Joint coordinates of the arm in time.
 */
typedef State<WorkspaceCoord> WorkspaceState;

/** Generalised workspace coordinates.
 */
struct GenWorkspaceCoord {
	typedef std::vector<GenWorkspaceCoord> Seq;
	
	/** tool frame pose */
	WorkspaceCoord pos;
	/** tool frame velocity */
	WorkspaceVel vel;

	/** sets Id transformation and zero velocity */
	inline void setZero() {
		pos.p.setZero();
		pos.R.setId();
		vel.setZero();
	}

	/** tests for finite vector
	*/
	inline bool isFinite() const {
		if (!pos.isFinite() || !vel.isFinite())
			return false;
		return true;
	}
};

/** Generalised configuration space coordinates of the arm in time.
 */
typedef State<GenWorkspaceCoord> GenWorkspaceState;

//------------------------------------------------------------------------------

/** Exponential coordinates of rigid body transformation (exponential mapping: se(3) -> SE(3)).
*/
struct ExpCoord {
	/** Twist coordinates of rigid body transformation (generator of exponential mapping: se(3) -> SE(3)) */
	Twist twist;
	/** Transformation magnitude */
	Real theta;
};

/** Manipulator Jacobian (in twist coordinates).
*/
struct Jacobian {
	Twist j[CONFIG_SPACE_DIM];

	/**	Array subscript operator. */
	inline const Twist &operator [] (U32 index) const {
		return j[index];
	}

	/**	Array subscript operator. */
	inline Twist &operator [] (U32 index) {
		return j[index];
	}
};

//------------------------------------------------------------------------------

class Arm;

/** Joint.
 */
class Joint {
	friend class Arm;

public:
	typedef shared_ptr<Joint> Ptr;
	typedef std::vector<Joint*> Seq;

	/** Callback interface for data synchronization */
	class Callback {
	public:
		virtual ~Callback() {}
		/** Auto synchronization of Joint bounds descriptions */
		virtual void syncJointBoundsDesc() {} //= 0;
	};

	/** Joint description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		typedef std::vector<const Desc*> ConstSeq;
		typedef std::vector<Ptr> Seq;
		
		friend class Joint;
		friend class Arm;
	
	private:
		/** Joint index
		*/
		mutable I32 index;
		
	protected:
		/** Creates Joint given the description object. 
		* @param arm	Arm interface
		* @return		pointer to the Joint, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual Joint::Ptr create(Arm& arm) const = 0;

	public:
		/** Name ASCII string */
		std::string name;
		
 		/** minimum value of generalised coordinate of a joint */
		GenCoord min;
		/** maximum value of generalised coordinate of a joint */
		GenCoord max;
		/** exponential coordinates defining joint transformation */
		ExpCoord trn;
		/** exponential coordinates defining initial frame transformation */
		ExpCoord trnInit;

		/** bounds and visualisation of the joint in local coordinates */
		Bounds::Desc::Seq bounds;
		/** collision detection */
		bool collision;
		/** collision joint offset */
		U32 collisionOffset;

		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}

		/** Resets the description to default one */
		virtual void setToDefault() {
			index = -1;

			name = "";

			min.pos = -REAL_PI;
			min.vel = -REAL_PI;
			min.acc = -REAL_PI;

			max.pos = +REAL_PI;
			max.vel = +REAL_PI;
			max.acc = +REAL_PI;

			trn.twist.set(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO);
			trn.theta = REAL_ZERO;
			
			trnInit.twist.set(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO);
			trnInit.theta = REAL_ZERO;
			
			bounds.clear();
			collision = true;
			collisionOffset = 0;
		}

		/** Checks validity */
		virtual bool isValid() const {
			if (min.pos >= max.pos || min.vel >= max.vel || min.acc >= max.acc)
				return false;

			// there can be no bounds given, but if they are - they have to be valid
			for (Bounds::Desc::Seq::const_iterator i = bounds.begin(); i != bounds.end(); i++)
				if (*i == NULL || !(*i)->isValid())
					return false;

			return true;
		}
	};

private:
	/** Index */
	U32 index;

protected:
	/** Context object */
	golem::Context& context;
	/** Arm interface */
	Arm& arm;

	/** Name ASCII string */
	std::string name;
	
	/** minimum value of generalised coordinate of a joint */
	GenCoord min;
	/** maximum value of generalised coordinate of a joint */
	GenCoord max;
	/** exponential coordinates defining joint transformation */
	ExpCoord trn;
	/** exponential coordinates defining initial frame transformation */
	ExpCoord trnInit;
	/** bounds and visualisation of the joint */
	Bounds::Desc::Seq boundsDescSeq;
	/** collision detection */
	bool collision;
	/** collision joint offset */
	U32 collisionOffset;

	/** Callback interface for data synchronization */
	Callback* pCallback;

	/** Joint data members critical section */
	mutable CriticalSection csData;
	
	/** Creates Joint from the description. 
	* @param desc	Joint description
	* @return		<code>TRUE</code> if no errors have occured;
	*				<code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	Joint(Arm& arm);

public:
	/** Destructor */
	virtual ~Joint() {}

	/** Adds the bounds description in the local coordinate frame of a given Joint
	 * @param pDesc			description of the bounds
	*/
	virtual bool addBoundsDesc(Bounds::Desc::Ptr pDesc);

	/** Removes the bounds description of a given Joint
	 * @param pDesc			the bounds to be removed
	*/
	virtual bool removeBoundsDesc(const Bounds::Desc* pDesc);
	
	/** Returns reference to the collection of bounds description of the Joint 
	 * @return				reference to the collection of bounds description
	*/
	virtual Bounds::Desc::SeqPtr getBoundsDescSeq() const;
	
	/** Returns Name ASCII string */
	const std::string& getName() const {
		return name;
	}
	
	/** Index */
	U32 getIndex() const {
		return index;
	}

	/** Returns minimum value of generalised coordinate of a joint */
	const GenCoord& getMin() const {
		return min;
	}
	
	/** Returns maximum value of generalised coordinate of a joint */
	const GenCoord& getMax() const {
		return max;
	}
	
	/** Returns exponential coordinates defining joint transformation */
	const ExpCoord& getTrn() const {
		return trn;
	}
	
	/** Returns exponential coordinates defining initial frame transformation */
	const ExpCoord& getTrnInit() const {
		return trnInit;
	}
	
	/** Callback interface for data synchronization
	 * @param pCallback	pointer to the interface
	*/
	void setCallback(Callback* pCallback) {
		this->pCallback = pCallback;
	}
	
	/** Callback interface for data synchronization
	 * @return				pointer to the interface
	*/
	Callback* getCallback() {
		return pCallback;
	}
	
	/** collision detection */
	bool hasCollision() const {
		return collision;
	}
	
	/** Returns collision joint offset */
	U32 getCollisionOffset() const {
		return collisionOffset;
	}

	/** golem::Context object */
	const golem::Context& getContext() const {
		return context;
	}
	golem::Context& getContext() {
		return context;
	}
};

//------------------------------------------------------------------------------

/** Open-chain manipulator interface.
 */
class Arm {
public:
	typedef shared_ptr<Arm> Ptr;
	friend class Desc;

	/** Arm description */
	class Desc {
		friend class Arm;

	public:
		typedef shared_ptr<Desc> Ptr;
		
		/** Name ASCII string */
		std::string name;
		/** joints */
		Joint::Desc::Seq joints;
		/** Arm global pose */
		Mat34 globalPose;
		/** Local reference pose in the tool frame */
		Mat34 referencePose;
		/** Arm joint rest configuration */
		ConfigspaceCoord restConfig;
		/** Use specialised (overloaded) forward/inverse/jacobian functions */
		bool customKinematics;
		
		Desc() {
			Desc::setToDefault();
		}
		
		virtual ~Desc() {
		}

		virtual void setToDefault() {
			name = "";
			joints.clear();	
			globalPose.setId();
			referencePose.setId();
			restConfig.set(REAL_ZERO);
			customKinematics = false;
		}

		virtual bool isValid() const {
			if (joints.size() <= 0 || joints.size() > CONFIG_SPACE_DIM)
				return false;

			for (Joint::Desc::Seq::const_iterator i = joints.begin(); i != joints.end(); i++)
				if (*i == NULL || !(*i)->isValid())
					return false;
			
			if (!globalPose.isFinite() || !referencePose.isFinite())
				return false;
			
			return true;
		}
		
		/** Creates Arm given the description object. 
		* @param context	golem::Context object
		* @return			pointer to the Arm interface if no errors have occured, throws otherwise
		*/
		virtual Arm::Ptr create(Context& context) const = 0;

		/** Loads description from dynamic library.
		* @param path		library path
		* @return			pointer to the Arm description if no errors have occured, throws otherwise
		*/
		static Arm::Desc::Ptr load(Context& context, const std::string& path);
	};

protected:
	typedef golem::queue<GenConfigspaceState> Queue;

	/** golem::Context object. */
	golem::Context& context;

	/** Arm data members critical section */
	mutable CriticalSection csData;
	
	/** Joints collection */
	typedef std::list<Joint::Ptr> JointList;
	
	/** Name ASCII string */
	std::string name;
	/** Use specialised (overloaded) forward/inverse/jacobian functions */
	bool customKinematics;

	/** Joints collection */
	JointList jointList;
	/** Joints pointers */
	Joint::Seq joints;

	/** Arm global pose */
	Mat34 globalPose;
	/** Trajectory reference pose */
	Mat34 referencePose;
	/** Arm joint rest configuration */
	ConfigspaceCoord restConfig;

	/** Arm state queue critical section */
	mutable CriticalSection csState;
	/** Arm state queue */
	Queue qState;
	/** Arm command queue critical section */
	mutable CriticalSection csCommand;
	/** Arm command queue */
	Queue qCommand, qSent;

	/* Inserts items such that: i[0] <= cBegin, i[1] > cBegin, ..., i[n-1] <= cEnd, i[n] > cEnd
	 * but no more than in the specified pointer range begin-end
	 */
	template <typename _Items, typename _Ptr, typename _Cmp> _Ptr lookup(const _Items& items, _Ptr begin, _Ptr end, _Cmp cBegin, _Cmp cEnd) const {
		if (items.empty())
			return begin;

		typename _Items::const_iterator pos = std::upper_bound(items.begin(), items.end(), cBegin);
		if (pos == items.end())
			--pos;
		if (pos != items.begin())
			--pos;

		for (size_t n = 0; begin != end && pos != items.end();) {
			*begin++ = *pos;
			if (++n > 1 && !(*pos <= cEnd))
				break;
			++pos;
		}

		return begin;
	}
	
	/** Interpolates the arm controller state at time t.
	*/
	bool interpolate(CriticalSection& cs, const Queue& queue, GenConfigspaceState& state, SecTmReal t) const;

	/** Creates Arm from the description. 
	* @param desc	Arm description
	* @return		<code>TRUE</code> if no errors have occured;
	*				<code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	Arm(Context& context);

public:
	/** Each derived class should have virtual destructor releasing resources
	*	to avoid calling virtual functions of non-existing objects
	*/
	virtual ~Arm();

	/** Receives the latest state of the motor system (blocking call).
	 * 
	 * @param state			received state
	 * @param timeWait		waiting time in milliseconds
	 * @return				<code>true</code> no errors;
	 * 						<code>false</code> otherwise
	 */
	virtual bool recv(GenConfigspaceState& state, MSecTmU32 timeWait = MSEC_TM_U32_INF) = 0;
	
	/** Sends motor command (blocking call).
	 * 
	 * The motor system tries to achieve the target state at the time 
	 * <code>next.t</code> (see <code>State</code>). 
	 * By multiple calling of this function, by internal queuing up the target 
	 * state sequence, one can create or plan a target trajectory for a system. 
	 * The minimal time distance between two consecutive target 
	 * states is specified by <code>Desc::timeDelta</code>.
	 * 
	 * <p>
	 * 
	 * Because of the internal buffering of a system itself,
	 * in case of asynchronous calls of this function (e.g. as a reaction to 
	 * some external stimuli) it may take up to three times more time the 
	 * <code>Desc::timeDelta</code> for a system to achieve the 
	 * target state comparing to the current system time. 
	 * Internally, to avoid a potential damage of a system, Controller (should) 
	 * never allows for sending to a system, two consecutive target states 
	 * with time distance between them shorter than the minimal one.
	 * Therefore, for asynchronous calls, to avoid time mismatch between 
	 * the target state and the real state of a physical system, 
	 * as well as to keep the reaction time fixed,
	 * it is advised to send target states with the time stamp <code>next.t</code>
	 * set to at least <code>Arm#getTimeDeltaAsync()</code> 
	 * later than the current system time.
	 * 
	 * <p>
	 * 
	 * By default, the function clears up the internal state queue up to the new 
	 * target state basing on the time stamp <code>next.t</code>. 
	 * If the internal state queue is full, the function waits until no longer 
	 * than <code>timeWait</code> milliseconds.
	 * 
	 * 
	 * @param next			target state
	 * @param timeWait		waiting time in milliseconds
	 * @return				<code>true</code> if the new target state has been
	 * 						successfully updated; <code>false</code> otherwise
	 *
	 * @see	Time delta for asynchronous calls
	 * 		<code>Arm#getTimeDeltaAsync()</code>
	 * @see	System time
	 * 		<code>Arm#context->getTimer()->elapsed()</code>
	 */
	virtual bool send(const GenConfigspaceState& command, MSecTmU32 timeWait = MSEC_TM_U32_INF) = 0;

	/* Inserts arm controller states such that: state[0] <= tBegin, state[1] > tBegin, ..., state[n-1] <= tEnd, state[n] > tEnd
	 * but no more than in the specified pointer range begin-end
	 */
	template <typename _Ptr> _Ptr lookupState(_Ptr begin, _Ptr end, SecTmReal tBegin, SecTmReal tEnd) const {
		CriticalSectionWrapper csw(csState);
		return lookup(qState, begin, end, tBegin, tEnd);
	}
	/** Interpolates the arm controller state at time t.
	* @param state		state of the arm controller
	* @param t			query time t in seconds (t usually refers to the past)
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool lookupState(GenConfigspaceState &state, SecTmReal t) const {
		return interpolate(csState, qState, state, t);
	}

	/* Inserts arm controller commands such that: cmd[0] <= tBegin, cmd[1] > tBegin, ..., cmd[n-1] <= tEnd, cmd[n] > tEnd
	 * but no more than in the specified pointer range begin-end
	 */
	template <typename _Ptr> _Ptr lookupCommand(_Ptr begin, _Ptr end, SecTmReal tBegin, SecTmReal tEnd) const {
		CriticalSectionWrapper csw(csCommand);

		const bool bQCommandEmpty = qCommand.size() < 2;
		if (bQCommandEmpty || !(qCommand.front() <= tBegin))
			begin = lookup(qSent, begin, end, tBegin, tEnd);
		if (!bQCommandEmpty && qCommand.front() <= tEnd)
			begin = lookup(qCommand, begin, end, tBegin, tEnd);

		return begin;
	}
	/** Interpolates the arm controller command at time t.
	* @param command		command of the arm controller
	* @param t				query time t in seconds (t usually refers to the future)
	* @return				<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool lookupCommand(GenConfigspaceState &command, SecTmReal t) const {
		return interpolate(csCommand, qCommand.front() <= t && qCommand.size() > 1 ? qCommand : qSent, command, t);
	}

	/** Time quant is a minimum time increment.
	 *
	 * @return				time period in seconds
	 */
	virtual SecTmReal getTimeQuant() const = 0;

	/** Returns the minimum effective time period between consecutive motor commands.
	 * 
	 * The minimal time period is calculated dynamically during calibration,
	 * and is limited by communication bandwidth with a system.
	 * 
	 * @return				time period in seconds
	 */
	virtual SecTmReal getTimeDelta() const = 0;

	/** Returns the minimum effective time period between consecutive asynchronous motor commands.
	 * 
	 * The minimal time period is calculated dynamically during calibration,
	 * and is limited by communication bandwidth with a system.
	 * 
	 * @return				time period in seconds
	 * @see	Sending target states 
	 * 		<code>Controller#send(const Send& next, MSecTmU32 timeWait)</code>
	 */
	virtual SecTmReal getTimeDeltaAsync() const = 0;


	/** Forward transformation for tool frame
	 * @param trn	SE(3) transformation matrix:
	 *				tool frame pose -> base frame pose (includes the arm global pose)
	 * @param cc	configuration space coordinates
	 */
	virtual void forwardTransform(Mat34& trn, const ConfigspaceCoord& cc) const;

	/** (Extended) forward transformation for all joints
	 * @param trn	array of SE(3) transformation matrices for each joint:
	 *				joint frame poses -> base frame pose (includes the arm global pose)
	 * @param cc	configuration space coordinates
	 */
	virtual void forwardTransformEx(Mat34 trn [], const ConfigspaceCoord& cc) const;

	/** End-effector spatial velocity
	 * @param v		velocity
	 * @param cc	configuration space coordinates
	 * @param dcc	delta
	 */
	virtual void velocitySpatial(Twist& vs, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc) const;

	/** End-effector body velocity
	 * @param v		velocity
	 * @param cc	configuration space coordinates
	 * @param dcc	delta
	 */
	virtual void velocityBody(Twist& vb, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc) const;

	/** End-effector velocity
	 * @param v		velocity
	 * @param cc	configuration space coordinates
	 * @param dcc	delta
	 */
	virtual void velocity(Twist& v, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc) const;

	/** End-effector velocity
	 * @param v		velocity
	 * @param jac	manipulator Jacobian
	 * @param dcc	delta
	 */
	virtual void velocityFromJacobian(Twist& v, const Jacobian& jac, const ConfigspaceCoord& dcc) const;

	/** End-effector velocity
	 * @param v		velocity
	 * @param vs	spatial velocity
	 * @param trn	forward transformation
	 */
	virtual void velocityFromSpatial(Twist& v, const Twist& vs, const Mat34& trn) const;

	/** End-effector velocity
	 * @param v		velocity
	 * @param vb	body velocity
	 * @param trn	forward transformation
	 */
	virtual void velocityFromBody(Twist& v, const Twist& vb, const Mat34& trn) const;

	/** Spatial manipulator Jacobian
	 * @param jac	spatial manipulator Jacobian
	 * @param j		configuration space coordinates
	 */
	virtual void jacobianSpatial(Jacobian& jac, const ConfigspaceCoord& cc) const;

	/** Body manipulator Jacobian
	 * @param jac	body manipulator Jacobian
	 * @param j		configuration space coordinates
	 */
	virtual void jacobianBody(Jacobian& jac, const ConfigspaceCoord& cc) const;

	/** Manipulator Jacobian
	 * @param jac	manipulator Jacobian
	 * @param j		configuration space coordinates
	 */
	virtual void jacobian(Jacobian& jac, const ConfigspaceCoord& cc) const;

	/** Manipulator Jacobian
	 * @param jac	manipulator Jacobian
	 * @param jacs	spatial manipulator Jacobian
	 * @param trn	forward transformation
	 */
	virtual void jacobianFromSpatial(Jacobian& jac, const Jacobian& jacs, const Mat34& trn) const;

	/** Manipulator Jacobian
	 * @param jac	manipulator Jacobian
	 * @param jacb	body manipulator Jacobian
	 * @param trn	forward transformation
	 */
	virtual void jacobianFromBody(Jacobian& jac, const Jacobian& jacb, const Mat34& trn) const;

	/** Returns Name ASCII string */
	const std::string& getName() const {
		return name;
	}	

	/** Access to Joints
	 * @return		reference to Joint container
	 */
	const Joint::Seq& getJoints() const {
		return joints;
	}

	/** Returns Arm global pose
	 * @return				Arm global pose
	 */
	virtual Mat34 getGlobalPose() const;

	/** Sets Arm global pose
	 * @param globalPose	Arm global pose
	 */
	virtual void setGlobalPose(const Mat34 &globalPose);
	
	/** Returns Trajectory reference pose
	 * @return				Trajectory reference pose
	 */
	virtual Mat34 getReferencePose() const;

	/** Sets Trajectory reference pose
	 * @param referencePose	Trajectory reference pose
	 */
	virtual void setReferencePose(const Mat34 &referencePose);

	/** Returns Arm rest joint position
	 * @return				arm rest joint position
	*/
	virtual ConfigspaceCoord getRestConfig() const;
	
	/** Sets arm joint rest configuration
	 * @param restConfig	arm rest joint position
	*/
	virtual void setRestConfig(const ConfigspaceCoord &restConfig);

	/** golem::Context object */
	const golem::Context& getContext() const {
		return context;
	}
	golem::Context& getContext() {
		return context;
	}
};

//------------------------------------------------------------------------------

/** Loads arm driver description.
*	The library function must be implemented in each arm device library
*/
typedef void (*LoadArmDesc)(void* pContext, const std::string& path, void* pArmDesc);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_ARM_H_*/
