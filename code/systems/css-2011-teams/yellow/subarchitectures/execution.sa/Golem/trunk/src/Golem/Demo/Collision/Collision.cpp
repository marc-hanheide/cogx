/** @file Collision.cpp
 * 
 * Program for demonstrating and testing collision detection algorithms.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Phys/Universe.h>
#include <Golem/Phys/Renderer.h>
#include <Golem/Phys/Msg.h>
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

class Collision : public Object {
public:
	typedef shared_ptr<Collision> Ptr;
	
	/** Object description */
	class Desc : public Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Collision, Object::Ptr, Scene&)

	public:
		/** Bounds descriptions */
		std::vector<golem::Bounds::Desc::Ptr> descriptions;
		/** Bounds descriptions indices */
		U32 n[2];
		/** Collision poses */
		Mat34 pose[2];
		/** Appearance */
		Actor::Appearance grayed, selected;
		/** Rotation and translation steps */
		Real rotation, translation;
		
		/** Constructs Collision description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			Object::Desc::setToDefault();

			// setup bounds
			descriptions.clear();
			
			BoundingSphere::Desc sphereDesc;
			sphereDesc.radius = Real(0.7);
			descriptions.push_back(sphereDesc.clone());
			
			BoundingBox::Desc boxDesc;
			boxDesc.dimensions.set(Real(0.5), Real(0.7), Real(1.5));
			descriptions.push_back(boxDesc.clone());
			boxDesc.dimensions.set(Real(0.1), Real(0.1), Real(0.3));
			descriptions.push_back(boxDesc.clone());
			
			BoundingCylinder::Desc cylinderDesc;
			cylinderDesc.length = Real(1.0);
			cylinderDesc.radius = Real(0.7);
			descriptions.push_back(cylinderDesc.clone());
			cylinderDesc.length = Real(0.7);
			cylinderDesc.radius = Real(0.2);
			descriptions.push_back(cylinderDesc.clone());
			
			n[0] = 1;
			n[1] = 2;

			pose[0].setId();
			pose[0].p.set(Real(+1.00), Real(0.0), Real(1.0));
			pose[1].setId();
			pose[1].p.set(Real(-1.00), Real(0.0), Real(1.0));
			
			selected.solidColour.set(192, 192, 0, 75); // set yellow
			selected.wireColour.set(127, 127, 127, 75); // set grey

			rotation = Real(0.1);
			translation = Real(0.01);
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!Object::Desc::isValid())
				return false;
			
			if (descriptions.empty())
				return false;
			if (n[0] >= descriptions.size() || n[1] >= descriptions.size())
				return false;
			if (!pose[0].isFinite() || !pose[1].isFinite())
				return false;
			if (!grayed.isValid() || !selected.isValid())
				return false;

			return true;
		}
	};

protected:
	/** Bounds descriptions */
	std::vector<golem::Bounds::Desc::Ptr> descriptions;
	/** Bounds descriptions indices */
	U32 n[2];
	/** Collision poses */
	Mat34 pose[2];
	/** Appearance */
	Actor::Appearance grayed, selected;
	/** Rotation and translation steps */
	Real rotation, translation;
	
	/** Bounds actors */
	Actor *actors[2];
	/** Selected bounds */
	U32 index;

	/** Generator of pseudo random numbers */
	Rand rand;
	
	int mouseButton;
	int mouseX;
	int mouseY;

	U32 getSelected() {
		return index;
	}
	U32 getGrayed() {
		return (index + 1)%2;
	}
	void next() {
		index = (index + 1)%2;
	}
	
	Mat34 getActorPose(U32 index) const {
		Mat34 pose;
		NxMat34 nxPose;
		
		nxPose = actors[index]->getNxActor()->getGlobalPose(); // secure - always called in the same thread as PhysX

		nxPose.M.getRowMajor(&pose.R.m11);
		nxPose.t.get(&pose.p.v1);
		
		return pose;
	}
	void setActorPose(const Mat34 &pose, U32 index) {
		NxMat34 nxPose;
		
		nxPose.M.setRowMajor(&pose.R.m11);
		nxPose.t.set(&pose.p.v1);
		actors[index]->getNxActor()->moveGlobalPose(nxPose); // secure - always called in the same thread as PhysX
	}
	
	void mouseHandler(int button, int state, int x, int y) {
		mouseButton = button;
		mouseX = x;
		mouseY = y;

		if (button == 3 || button == 4) {
			Mat34 pose = getActorPose(getSelected());
			
			Mat33 rotZ;
			rotZ.rotZ((REAL_PI*(button == 3 ? -1.0 : +1.0)*50.0/180.0)*rotation);
			pose.R.multiply(rotZ, pose.R);
			
			setActorPose(pose, getSelected());

			collision();
		}
	}

	void motionHandler(int x, int y) {
		if (mouseButton == 2) {
			int dx = mouseX - x;
			int dy = mouseY - y;
			
			Mat34 pose = getActorPose(getSelected());
			
			Mat33 rotX;
			rotX.rotX((REAL_PI*dx/180.0)*rotation);
			pose.R.multiply(rotX, pose.R);
			
			Mat33 rotY;
			rotY.rotY((REAL_PI*dy/180.0)*rotation);
			pose.R.multiply(rotY, pose.R);
			
			setActorPose(pose, getSelected());
			
			mouseX = x;
			mouseY = y;		

			collision();
		}
	}

	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y) {
		Mat34 newPose = getActorPose(getSelected());
		Vec3 axis;
		axis.subtract(pose[getGrayed()].p, pose[getSelected()].p);
		axis.normalise();

		switch (key) {
		case '1':
			newPose.p.multiplyAdd(+translation, axis, newPose.p);
			break;
		case '2':
			newPose.p.multiplyAdd(-translation, axis, newPose.p);
			break;
		case 13:// enter
			n[getSelected()] = (n[getSelected()] + 1)%descriptions.size();
			(void)createActor(getSelected());
			return;
		case 9:// tab
			next();
			actors[getGrayed()]->setAppearance(grayed);
			actors[getSelected()]->setAppearance(selected);
			return;
		}

		setActorPose(newPose, getSelected());
			
		collision();
	}
	
	/** Check for collisions. */
	void collision() {
		const Bounds::Ptr pBounds0 = actors[0]->getGlobalBoundsSeq()->front();
		const Bounds::Ptr pBounds1 = actors[1]->getGlobalBoundsSeq()->front();

		const char* str[2] = {"NO", "YES"};
		bool collision;
		
#ifdef _BOUNDS_PERFMON
		Bounds::resetLog();
#endif
		collision = pBounds0->intersect(*pBounds1);
#ifdef _BOUNDS_PERFMON
		context.getMessageStream()->write(Message::LEVEL_INFO, "collision0: %s, {iter = %d}", str[collision], Bounds::collisionConvexIter);
#else
		context.getMessageStream()->write(Message::LEVEL_INFO, "collision: %s", str[collision]);
#endif
	}

	/** Renders the Collision. */
	virtual void render() {
	}

	/** Creates Actors. */
	virtual bool createActor(U32 index, bool bSelected = true) {
		Actor *&actor0 = actors[index];
		const golem::Bounds::Desc::Ptr &desc0 = descriptions[n[index]];
		const Mat34 &pose0 = pose[index];

		NxShapeDesc *pNxShapeDesc = scene.createNxShapeDesc(desc0); // throws
		
		Actor::Desc actorDesc;
		actorDesc.kinematic = true;
		NxBodyDesc nxBodyDesc;
		actorDesc.nxActorDesc.body = &nxBodyDesc;
		actorDesc.nxActorDesc.density = (NxReal)1.0;
		actorDesc.nxActorDesc.shapes.push_back(pNxShapeDesc);
		actorDesc.nxActorDesc.globalPose.M.setRowMajor(&pose0.R.m11);
		actorDesc.nxActorDesc.globalPose.t.set(&pose0.p.v1);
		
		if (actor0 != NULL)
			scene.releaseObject(*actor0);	

		actor0 = dynamic_cast<Actor*>(scene.createObject(actorDesc)); // throws
		
		actor0->setAppearance(bSelected ? selected : grayed);

		return true;
	}
	
	/** Creates the Collision from the Collision description. */
	bool create(const Desc& desc) {
		descriptions = desc.descriptions;
		n[0] = desc.n[0];
		n[1] = desc.n[1];
		pose[0] = desc.pose[0];
		pose[1] = desc.pose[1];
		grayed = desc.grayed;
		selected = desc.selected;
		rotation = desc.rotation;
		translation = desc.translation;

		Object::create(desc); // throws
		createActor(getSelected()); // throws
		createActor(getGrayed(), false); // throws

		return true;
	}

	/** Objects can be constructed only in the Scene context. */
	Collision(Scene &scene) :
		Object(scene), rand(context.getRandSeed())
	{
		actors[0] = actors[1] = NULL;
		index = 0;
	}
};

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	printf("Use the arrow keys to move the camera.\n");
	printf("Use the mouse to rotate the camera.\n");
	printf("Press p to pause simulations.\n");
	printf("Press pgup/pgdn/space to switch between simulations.\n");
	printf("Press v to show Actors reference frames.\n");
	printf("Use z, x, c to change randering mode.\n");
	printf("Press esc to exit.\n");
	
	try {
		// Create program context
		golem::Context::Desc contextDesc;
		golem::Context::Ptr context = contextDesc.create();

		// Do not display LEVEL_DEBUG messages (only with level at least LEVEL_INFO)
		//context->getLogger()->setMsgFilter(MessageFilter::Ptr(new LevelFilter<Message>(Message::LEVEL_INFO)));

		//-----------------------------------------------------------------------------

		// Create Universe
		Universe::Desc universeDesc;
		universeDesc.name = "Golem (Collision)";
		universeDesc.argc = argc;
		universeDesc.argv = argv;
		Universe::Ptr pUniverse = universeDesc.create(*context);

		// Create scene
		Scene::Desc sceneDesc;
		sceneDesc.name = "Collision detection demo";
		Scene *pScene = pUniverse->createScene(sceneDesc);

		// Create Collision object
		context->getMessageStream()->write(Message::LEVEL_INFO, "Initialising collision object...");
		Collision::Desc collisionDesc;
		pScene->createObject(collisionDesc);
		
		// Launch universe
		context->getMessageStream()->write(Message::LEVEL_INFO, "Launching Universe...");
		pUniverse->launch();

		while (!pUniverse->interrupted())
			PerfTimer::sleep(SecTmReal(0.01));

		context->getMessageStream()->write(Message::LEVEL_INFO, "Good bye!");
	}
	catch (const Message& msg) {
		std::cerr << msg.str() << std::endl;
	}
	catch (const std::exception &ex) {
		std::cerr << Message(Message::LEVEL_CRIT, "C++ exception: %s", ex.what()).str() << std::endl;
	}

	return 0;
}
