#ifndef KEYBOARD_HANDLER_H
#define KEYBOARD_HANDLER_H

#include <iostream>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgUtil/IntersectVisitor>

class keyboardEventHandler : public osgGA::GUIEventHandler
{
public:
/*	keyboardEventHandler(void);
	~keyboardEventHandler(void);

	static keyboardEventHandler * TravelToScence(osg::ref_ptr<osgViewer::Viewer>viewer); 

	osg::ref_ptr<osgViewer::Viewer> m_pHostViewer; 

	float t_speed;   //�ٶ�

	osg::Vec3 t_position;   //��ǰλ��

	osg::Vec3 t_rotation;   //��ת�Ƕ�

	bool t_collision;   //��ײ���

	float set_speed;   //�����ٶ�

	void getspeed(float &);

	virtual void setByMatrix(const osg::Matrix &matrix);    //���þ���

	virtual void setByInverseMatrix(const osg::Matrix &matrix);   //���������

	virtual osg::Matrixd getMatrix() const;  //��ȡ����

	virtual osg::Matrixd getInverseMatrix() const;  //��ȡ�����

//	void getposition(osg::Vec3 &position);

//	osg::Vec3 setposition();   //���ó�ʼλ��*/

	typedef void (*functionType) ();

	enum keyStatusType
	{
		KEY_UP, KEY_DOWN 
	};

	struct functionStatusType
	{
		functionStatusType() {keyState = KEY_UP; keyFunction = NULL;}
		functionType keyFunction;
		keyStatusType keyState;
	};

	bool addFunction(int whatKey, functionType newFunction);

	bool addFunction(int whatKey, keyStatusType keyPressStatus, functionType newFunction);

	virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

	void ChangePosition(osg::Vec3 &delta);

	//   virtual void accept(osgGA::GUIEventHandlerVisitor& v)   { v.visit(*this); };

protected:

	typedef std::map<int, functionStatusType > keyFunctionMap;
	keyFunctionMap keyFuncMap;
	keyFunctionMap keyUPFuncMap;

};

#endif
