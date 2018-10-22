// CapsuleDistanceHull.h: Class to wrap robot links in capsule hulls for fast distance computation

#ifndef CAPSULE_DISTANCE_HULL_H
#define CAPSULE_DISTANCE_HULL_H

#include <stdexcept>
#include <string>
#include <map>
#include <chai3d.h>
#include "graphics/ChaiGraphics.h"
#include "model/ModelInterface.h"

#include <Eigen/Core>

// Distance info
struct CapsuleDistanceInfo {
	// closest point on the capsule with respect to which distance was calculated
	Eigen::Vector3d _closest_point_self;
	// closest point on the other object with respect to which distance was calculated
	Eigen::Vector3d _closest_point_other;
	// distance: greather than equal to 0. initialized to -1. TODO: << probably a bad idea for init
	double _distance;
	// local link name
	std::string _link_name;

	//ctor
	CapsuleDistanceInfo()
	: _distance(-1.0), _link_name("")
	{ }
};

// A capsule consists of all points at a fixed distance from a line segment
class Capsule {
public:
	// ctor
	Capsule(const Eigen::Vector3d& pointA,
			const Eigen::Vector3d& pointB,
			double radius,
			chai3d::cGenericObject* chai_link)
	:_pointA(pointA), _pointB(pointB), _radius(radius)
	{
		double ABnorm = (pointB - pointA).norm();
		if (ABnorm < 1e-5) {
			throw(std::runtime_error("Capsule too small"));
		}
		_AtoB = (pointB - pointA)/ABnorm;
		_dist_AtoB = ABnorm;

		// create chai object
		double cylinder_len = ABnorm + 2.0*radius;
		_chai_shape = new chai3d::cShapeCylinder(radius, radius, cylinder_len);
		chai_link->addChild(_chai_shape);
		Eigen::Matrix3d local_rot;
		Eigen::Vector3d xn = _AtoB.cross(Eigen::Vector3d(0,0,1)).normalized();
		local_rot.col(0) = xn;
		local_rot.col(1) = _AtoB.cross(xn);
		local_rot.col(2) = _AtoB;
		_chai_shape->setLocalRot(chai3d::cMatrix3d(local_rot));
		_chai_shape->setLocalPos(chai3d::cVector3d(pointA - _AtoB*radius));
		_chai_shape->setWireMode(true);
	}

	// ctor empty
	Capsule()
	: _radius(0.0), _chai_shape(NULL)
	{ }

	// dtor
	~Capsule() {
		delete _chai_shape;
	}

	// compute distance to a sphere
	// Note: center assumed to be calculated with respect to my link frame
	// TODO: mark as const once ModelInterface is marked const
	CapsuleDistanceInfo computeDistanceSphere (const Eigen::Vector3d& center, double radius) {
		Eigen::Vector3d Atocenter = center - _pointA;
		double projection = _AtoB.dot(Atocenter);
		Eigen::Vector3d pointOnCapsuleAxis;
		if (projection >= 0 && projection <= _dist_AtoB) {
			pointOnCapsuleAxis = _pointA + _AtoB*(projection);
		} else if (projection < 0) {
			pointOnCapsuleAxis = _pointA;
		} else { // projection > _dist_AtoB
			pointOnCapsuleAxis = _pointB;
		}
		double dist = fmax(0.0, (pointOnCapsuleAxis - center).norm() - radius - _radius);
		CapsuleDistanceInfo ret_info;
		ret_info._distance = dist;
		ret_info._closest_point_self = pointOnCapsuleAxis + (center - pointOnCapsuleAxis).normalized() * _radius;
		ret_info._closest_point_other = center + (pointOnCapsuleAxis - center).normalized() * radius;
		return ret_info;
	}

	// compute distance to another capsule: TODO

public:
	// point A: in local frame
	Eigen::Vector3d _pointA;
	// point B: in local frame
	Eigen::Vector3d _pointB;
	// direction vector from A to B. TODO: this should change when points change
	Eigen::Vector3d _AtoB;
	// distance between points A and B
	double _dist_AtoB;
	// radius
	double _radius;
	// chai representation
	chai3d::cShapeCylinder* _chai_shape;
};

class CapsuleDistanceHull {
public:
	//ctor
	CapsuleDistanceHull(const std::string& robot_name)
	: _robot_name(robot_name), _display_enabled(true)
	{ }

	//add link
	void addLink(
		const std::string& link_name,
		const Eigen::Vector3d& pointA,
		const Eigen::Vector3d& pointB,
		double radius,
		chai3d::cGenericObject* chai_robot
		) {
		if(_link_capsule_map.find(link_name) != _link_capsule_map.end()) {
			throw(std::runtime_error("Duplicate capsule for same robot."));
		}
		// find chai link
		chai3d::cRobotLink* first_link = NULL;
		chai3d::cRobotLink* link = NULL;
		for (unsigned int i = 0; i < chai_robot->getNumChildren(); ++i) {
			first_link = dynamic_cast<chai3d::cRobotLink*>(chai_robot->getChild(i));
			if (first_link != NULL) {
				link = getRobotLink(link_name, first_link);
				break;
			}
		}
		if (link == NULL) {
			throw(std::runtime_error("Graphics object for link not found."));
		}
		_link_capsule_map[link_name] = new Capsule(pointA, pointB, radius, link);
	}

	// compute distance to another geometry: sphere
	// TODO: mark as const once ModelInterface is marked const
	CapsuleDistanceInfo computeDistanceSphere (const Eigen::Vector3d& center_world, double radius, Model::ModelInterface* robot) {
		CapsuleDistanceInfo ret_info;
		std::string closest_link;
		Eigen::Affine3d link_transform;
		for(const auto link_capsule_map_iter: _link_capsule_map) {
			// compute center of sphere in link frame.
			// Assumes robot base is colocated with world frame.
			robot->transform(link_transform, link_capsule_map_iter.first);
			// compute distance for this link
			CapsuleDistanceInfo info = link_capsule_map_iter.second->computeDistanceSphere(
													link_transform.inverse()*center_world,
													radius
												);
			// check if smaller than current
			if (ret_info._distance < 0 || info._distance < ret_info._distance) {
				ret_info = info;
				ret_info._link_name = link_capsule_map_iter.first;
			}
		}
		return ret_info;
	}

	// compute distance to another capsule. TODO

	// update graphics
	void setDisplayEnabled(bool enable) {
		if (enable == _display_enabled) {
			return;
		}
		// TODO: propagate to capsules
		_display_enabled = enable;
	}

public:
	std::string _robot_name;
	std::map<std::string, Capsule*> _link_capsule_map;
	bool _display_enabled;

private:
	// internal function to recursively find cRobotLink in link hierarchy
	// TODO: move to cChaiGraphics or even better, just build a flat list in cRobotBase
	chai3d::cRobotLink* getRobotLink(const std::string& link_name, chai3d::cRobotLink* link) {
		if (link->m_name == link_name) {
			return link;
		}
		chai3d::cRobotLink* ret_link = NULL;
		for (unsigned int i = 0; i < link->getNumChildren(); ++i) {
			chai3d::cRobotLink* child_link = dynamic_cast<chai3d::cRobotLink*>(link->getChild(i));
			if (child_link != NULL) {
				ret_link = getRobotLink(link_name, child_link);
				if (ret_link != NULL) {
					return ret_link;
				}
			}
		}
		return ret_link;
	}
};

#endif// CAPSULE_DISTANCE_HULL_H
