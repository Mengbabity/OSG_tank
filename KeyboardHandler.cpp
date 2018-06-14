#include "KeyboardHandler.h"
#include <osg/LineSegment>
#include <osgUtil/IntersectVisitor>

/*
//构造函数
keyboardEventHandler::keyboardEventHandler():t_speed(1.0f),
	t_collision(true)
{
//	t_position=osg::Vec3(-22.0f, -274.0f, 100.0f);
//	t_rotation=osg::Vec3(osg::PI_2, 0.0f, 0.0f);
}

keyboardEventHandler::~keyboardEventHandler(void)
{

}*/
/*
// 设置矩阵  
void keyboardEventHandler::setByMatrix(const osg::Matrix &matrix)  
{  

}  

// 设置逆矩阵  
void keyboardEventHandler::setByInverseMatrix(const osg::Matrix &matrix)  
{  

}  

// 得到矩阵  
osg::Matrixd keyboardEventHandler::getMatrix(void)const  
{  
	osg::Matrixd mat;  

	mat.makeRotate(t_rotation._v[0], osg::Vec3(1.0f, 0.0f, 0.0f),  
		t_rotation._v[1], osg::Vec3(0.0f, 1.0f, 0.0f),  
		t_rotation._v[2], osg::Vec3(0.0f, 0.0f, 1.0f));  

	return mat * osg::Matrixd::translate(t_position);  
}  

// 得到逆矩阵  
osg::Matrixd keyboardEventHandler::getInverseMatrix(void) const  
{  
	osg::Matrixd mat;  

	mat.makeRotate(t_rotation._v[0], osg::Vec3(1.0f, 0.0f, 0.0f),  
		t_rotation._v[1], osg::Vec3(0.0f, 1.0f, 0.0f),  
		t_rotation._v[2], osg::Vec3(0.0f, 0.0f, 1.0f));  

	return osg::Matrixd::inverse(mat * osg::Matrixd::translate(t_position));  
}  */

bool keyboardEventHandler::addFunction(int whatKey, functionType newFunction)
{
	if ( keyFuncMap.end() != keyFuncMap.find( whatKey ))
	{
		std::cout << "duplicate key '" << whatKey << "' ignored." << std::endl;
		return false;
	}
	else
	{
		keyFuncMap[whatKey].keyFunction = newFunction;
		return true;
	}
}

bool keyboardEventHandler::addFunction (int whatKey, keyStatusType keyPressStatus, functionType newFunction)
{
	if (keyPressStatus == KEY_DOWN)
	{
		return addFunction(whatKey,newFunction);
	}
	else
	{
		if ( keyUPFuncMap.end() != keyUPFuncMap.find( whatKey )) 
		{
			std::cout << "duplicate key '" << whatKey << "' ignored." << std::endl;
			return false;
		}
		else
		{
			keyUPFuncMap[whatKey].keyFunction = newFunction;
			return true;
		}
	} // KEY_UP
}

bool keyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
{
	bool newKeyDownEvent = false;
	bool newKeyUpEvent   = false;


//		osg::MatrixTransform* mt=dynamic_cast<osg::MatrixTransform *>(vw->getSceneData());
			switch(ea.getEventType())
			{
			case(osgGA::GUIEventAdapter::KEYDOWN):
				{

/*					if(ea.getKey()==0x20)   //空格键
					{
						aa.requestRedraw();
						aa.requestContinuousUpdate(false);

						return true;
					}

					if(ea.getKey()=='+')   //加速
					{
						t_speed+=1.0f;

						return true;
					}

					if(ea.getKey()=='-')   //减速
					{
						t_speed-=1.0f;

						if(t_speed<1.0f)
							t_speed=1.0f;

						return true;
					}

					if(ea.getKey()=='w')   //前进W键
					{
						ChangePosition(osg::Vec3(0,t_speed*sinf(osg::PI_2 + t_rotation._v[2]),0));
						ChangePosition(osg::Vec3(t_speed*cosf(osg::PI_2 + t_rotation._v[2]),0,0));

						return true;
					}

					if(ea.getKey()=='s')   //后退S键
					{
						ChangePosition(osg::Vec3(0,-t_speed*sinf(osg::PI_2 + t_rotation._v[2]),0));
						ChangePosition(osg::Vec3(-t_speed*cosf(osg::PI_2 + t_rotation._v[2]),0,0));

						return true;
					}

					if(ea.getKey()=='a')   //向左A键
					{
						ChangePosition(osg::Vec3(0,t_speed * cosf(osg::PI_2 + t_rotation._v[2]),0));  
						ChangePosition(osg::Vec3(-t_speed * sinf(osg::PI_2 + t_rotation._v[2]),0,0));  

						return true;  
					}

					if(ea.getKey()=='d')   //向右D键
					{
						ChangePosition(osg::Vec3(0,-t_speed * sinf(osg::PI_2 + t_rotation._v[2]),0));  
						ChangePosition(osg::Vec3(t_speed * cosf(osg::PI_2 + t_rotation._v[2]),0,0));  

						return true;  
					}
					return false;*/
					keyFunctionMap::iterator itr = keyFuncMap.find(ea.getKey());
					if (itr != keyFuncMap.end())
					{
						if ( (*itr).second.keyState == KEY_UP )
						{
							(*itr).second.keyState = KEY_DOWN;
							newKeyDownEvent = true;
						}
						if (newKeyDownEvent)
						{
							(*itr).second.keyFunction();
							newKeyDownEvent = false;
						}
						return true;
					}
					return false;
				}
			case(osgGA::GUIEventAdapter::KEYUP):
				{
					keyFunctionMap::iterator itr = keyFuncMap.find(ea.getKey());
					if (itr != keyFuncMap.end() )
					{
						(*itr).second.keyState = KEY_UP;
					}
					itr = keyUPFuncMap.find(ea.getKey());
					if (itr != keyUPFuncMap.end())
					{
						(*itr).second.keyFunction();
						return true;
					}
					return false; 
				}
			default:
				return false;
			}

//		}
}
/*
void keyboardEventHandler::ChangePosition(osg::Vec3 &delta)
{
	if (t_collision)  
	{  
		// 得到新的位置  
		osg::Vec3 newPos1 = t_position + delta;  

		osgUtil::IntersectVisitor ivXY;  

		// 根据新的位置得到两条线段检测  

		osg::ref_ptr<osg::LineSegment>lineXY = new osg::LineSegment(newPos1,t_position);  

//		osg::ref_ptr<osg::LineSegment>lineZ = new osg::LineSegment(newPos1 + osg::Vec3(0.0f, 0.0f, 10.0f), newPos1 - osg::Vec3(0.0f, 0.0f, -10.0f));  

//		ivXY.addLineSegment(lineZ.get());  
		ivXY.addLineSegment(lineXY.get());  

		// 结构交集检测  
		m_pHostViewer->getSceneData()->accept(ivXY);  

		// 如果没有碰撞  
		if (!ivXY.hits())  
		{  
			t_position += delta;  
		}  

	}  
	else  
	{  
		t_position += delta;  
//	}  
}*/
