#ifndef DATA_H
#define DATA_H
#include <osgSim/DOFTransform>
#include "TransformAccumulator.h"

class tank:public osg::Referenced
{
public:
	osgSim::DOFTransform* tankTurretNode;
	osgSim::DOFTransform* tankGunNode;
	osg::PositionAttitudeTransform* tanktransform;
	tank(osg::Node*n, osg::PositionAttitudeTransform*tanktransform);

	double t_rotation;   
	double turret_rotation;
	double turret_elevation;  
	
	bool t_forward;
	bool t_backward;
	bool t_left;
	bool t_right;
	bool tu_left;
	bool tu_right;
	bool tu_up;
	bool tu_down;
	
	void update_move();
	void update_turret();
	void update_gun();
	
	void set_forward(bool b);
	void set_backward(bool b);
	void set_left(bool b);
	void set_right(bool b);
	void set_turretleft(bool b);
	void set_turretright(bool b);
	void set_gunup(bool b);
	void set_gundown(bool b);

};

class Manipulator:public osgGA::CameraManipulator
{
public:
	osg::Matrixd m;
	
	virtual void setByMatrix(const osg::Matrixd& matrix); 
	virtual void setByInverseMatrix(const osg::Matrixd& matrix);
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;
	void updateMatrix();
	Manipulator(transformAccumulator* ta) { worldCoordinatesOfNode = ta; m = osg::Matrixd::identity(); }
	~Manipulator() {}
	transformAccumulator* worldCoordinatesOfNode;
	virtual bool handle(const osgGA::GUIEventAdapter&ea, osgGA::GUIActionAdapter&us);
};

#endif
