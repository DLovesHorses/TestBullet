
#include "btBulletDynamicsCommon.h"

#include <iostream>
#include <unistd.h>
using namespace std;

btBroadphaseInterface *broadphase;
btDefaultCollisionConfiguration *collisionConfiguration;
btCollisionDispatcher *dispatcher;
btConstraintSolver *solver;
btDiscreteDynamicsWorld *dynamicsWorld;

struct GodotDeepPenetrationContactResultCallback : public btManifoldResult {
	btVector3 m_pointNormalWorld;
	btVector3 m_pointWorld;
	btScalar m_penetration_distance;
	int m_other_compound_shape_index;

	GodotDeepPenetrationContactResultCallback(const btCollisionObjectWrapper *body0Wrap, const btCollisionObjectWrapper *body1Wrap) :
			btManifoldResult(body0Wrap, body1Wrap),
			m_penetration_distance(0),
			m_other_compound_shape_index(0) {}

	void reset() {
		m_penetration_distance = 0;
	}

	bool hasHit() {
		return m_penetration_distance < 0;
	}

	virtual void addContactPoint(const btVector3 &normalOnBInWorld, const btVector3 &pointInWorldOnB, btScalar depth) {

		bool ser = false;
		if (0 < depth) {
			cout << "depth is positive" << endl;
		}

		if (m_penetration_distance > depth) { // Has penetration?

			bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
			m_penetration_distance = depth;
			m_other_compound_shape_index = isSwapped ? m_index0 : m_index1;
			m_pointNormalWorld = isSwapped ? normalOnBInWorld * -1 : normalOnBInWorld;
			m_pointWorld = isSwapped ? (pointInWorldOnB + (normalOnBInWorld * depth)) : pointInWorldOnB;
		}
	}
};

bool RFP_convex_world_test(
	 const btConvexShape *p_shapeA,
	 const btCollisionShape *p_shapeB,
	 btCollisionObject *p_objectA,
	 btCollisionObject *p_objectB,
	 int p_shapeId_A,
	 int p_shapeId_B,
	 const btTransform &p_transformA,
	 const btTransform &p_transformB) {

	/// Contact test

	btCollisionObjectWrapper obA(NULL, p_shapeA, p_objectA, p_transformA, -1, p_shapeId_A);
	btCollisionObjectWrapper obB(NULL, p_shapeB, p_objectB, p_transformB, -1, p_shapeId_B);

	btCollisionAlgorithm *algorithm = dispatcher->findAlgorithm(&obA, &obB, NULL, BT_CONTACT_POINT_ALGORITHMS);
	if (algorithm) {
		GodotDeepPenetrationContactResultCallback contactPointResult(&obA, &obB);
		//discrete collision detection query
		algorithm->processCollision(&obA, &obB, dynamicsWorld->getDispatchInfo(), &contactPointResult);

		algorithm->~btCollisionAlgorithm();
		dispatcher->freeCollisionAlgorithm(algorithm);

		if (contactPointResult.hasHit()) {
			return true;
		}
	}
	return false;
}

int main(){

	// Create world
	collisionConfiguration = new btDefaultCollisionConfiguration(btDefaultCollisionConstructionInfo());
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	broadphase = new btDbvtBroadphase;
	solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	btVector3 localInertia(0, 0, 0);

	// Load static trimesh mesh
	btRigidBody::btRigidBodyConstructionInfo cInfo(0, NULL, shape, localInertia);
	btRigidBody* sphereBody = new btRigidBody(cInfo);
	sphereBody->setCollisionFlags(sphereBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	sphereBody->forceActivationState(DISABLE_SIMULATION);
	sphereBody->setMassProps(0, localInertia);
	sphereBody->updateInertiaTensor();

	while(true){
		float delta = 1/60;
		dynamicsWorld->stepSimulation(delta, 0, 0);
		usleep(16666.666666666667879);
		cout << "Step" << '\n';
	}
	// Step world
	cout << "Executed" << '\n';

	return 1;
}
