#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/Billboard>
#include <osg/BlendFunc>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osg/LineSegment>
#include <osgUtil/IntersectVisitor>
#include <osg/AlphaFunc>
#include <ctime>    
#include <cstdlib> 
#include <conio.h>
#include <osg/Math>
#include <osg/NodeCallback>
#include <osg/MatrixTransform>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/TrackballManipulator>
#include <osgParticle/Particle>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/ModularEmitter>
#include <osgParticle/ModularProgram>
#include <osgParticle/RandomRateCounter>
#include <osgParticle/SectorPlacer>
#include <osgParticle/RadialShooter>
#include <osgParticle/AccelOperator>
#include <osgParticle/FluidFrictionOperator>
#include <osgParticle/AccelOperator>
#include <osgParticle/MultiSegmentPlacer>
#include <osgParticle/RadialShooter>
#include <osgParticle/Interpolator>
#include <osgParticle/FireEffect>
#include <osgParticle/ExplosionDebrisEffect>
#include <osgParticle/ExplosionEffect>
#include <osgParticle/SmokeEffect>
#include <algorithm>

#include <osgText/Font>
#include <osgText/Text>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/Projection>
#include <osg/ShapeDrawable>
#include <string.h>

#include "KeyboardHandler.h"
#include "data.h"
#include "TransformAccumulator.h"
#include "findNodeVisitor.h"

using namespace std;

#define NUMBER_O_SHRUBS 1250

osg::Group* rootNode = new osg::Group();
osg::Node* tankNode1 = NULL;
osg::Node* tankNode2 = NULL;
osg::Node* terrainNode = NULL;
osg::PositionAttitudeTransform* tanktransform1 = new osg::PositionAttitudeTransform();
osg::PositionAttitudeTransform* tanktransform2 = new osg::PositionAttitudeTransform();
osg::Vec3 tank1_position,tank2_position;

osg::Geode* HUDGeode = new osg::Geode();
osgText::Text* textOne1 = new osgText::Text();
osgText::Text* tankLabel1 = new osgText::Text();
osgText::Text* textOne2 = new osgText::Text();
osgText::Text* tankLabel2 = new osgText::Text();
osg::Projection* HUDProjectionMatrix = new osg::Projection;

//坦克运动的回调函数
class tankNodeCallback : public osg::NodeCallback
{
public:
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::ref_ptr<tank> tankData = dynamic_cast<tank*> (node->getUserData());
		if (tankData != NULL)
		{
			tankData->update_turret();
			tankData->update_gun();
			tankData->update_move();
		}
		traverse(node, nv);
	}
};

//求交
void set_on(double tankXPosition,double tankYPosition,osg::Node *terrainNode,osg::PositionAttitudeTransform* tanktransform)
{
	osg::LineSegment* tankLocationSegment = new osg::LineSegment();
	tankLocationSegment->set(
		osg::Vec3(tankXPosition, tankYPosition, 999) ,
		osg::Vec3(tankXPosition, tankYPosition, -999) );

	osgUtil::IntersectVisitor findTankElevationVisitor;
	findTankElevationVisitor.addLineSegment(tankLocationSegment);
	terrainNode->accept(findTankElevationVisitor);

	osgUtil::IntersectVisitor::HitList tankElevationLocatorHits;
	tankElevationLocatorHits = findTankElevationVisitor.getHitList(tankLocationSegment);
	osgUtil::Hit heightTestResults;
	if ( tankElevationLocatorHits.empty() )
	{
		std::cout << " couldn't place tank on terrain" << std::endl;
		return;
	}
	heightTestResults = tankElevationLocatorHits.front();
	osg::Vec3 terrainHeight = heightTestResults.getWorldIntersectPoint();

	tanktransform->setPosition( terrainHeight );
}

//灌木丛
osg::Drawable* createShrub(const float & scale, osg::StateSet* bbState)
{
   float width = 1.5f;
   float height = 3.0f;

   width *= scale;
   height *= scale;

   osg::Geometry* shrubQuad = new osg::Geometry;

   osg::Vec3Array* shrubVerts = new osg::Vec3Array(4);
   (*shrubVerts)[0] = osg::Vec3(-width/2.0f, 0, 0);
   (*shrubVerts)[1] = osg::Vec3( width/2.0f, 0, 0);
   (*shrubVerts)[2] = osg::Vec3( width/2.0f, 0, height);
   (*shrubVerts)[3] = osg::Vec3(-width/2.0f, 0, height);

   shrubQuad->setVertexArray(shrubVerts);

   osg::Vec2Array* shrubTexCoords = new osg::Vec2Array(4);
   (*shrubTexCoords)[0].set(0.0f,0.0f);
   (*shrubTexCoords)[1].set(1.0f,0.0f);
   (*shrubTexCoords)[2].set(1.0f,1.0f);
   (*shrubTexCoords)[3].set(0.0f,1.0f);
   shrubQuad->setTexCoordArray(0,shrubTexCoords);

   shrubQuad->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

   shrubQuad->setStateSet(bbState); 

   return shrubQuad;
}

void plant()
{
	osg::Billboard* shrubBillBoard = new osg::Billboard();
	rootNode->addChild(shrubBillBoard);

	shrubBillBoard->setMode(osg::Billboard::AXIAL_ROT);
	shrubBillBoard->setAxis(osg::Vec3(0.0f,0.0f,1.0f));
	shrubBillBoard->setNormal(osg::Vec3(0.0f,-1.0f,0.0f));

	osg::Texture2D *ocotilloTexture = new osg::Texture2D;
	ocotilloTexture->setImage(osgDB::readImageFile("../NPS_Data/Textures/ocotillo.png"));

	osg::StateSet* billBoardStateSet = new osg::StateSet;

	billBoardStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	billBoardStateSet->setTextureAttributeAndModes(0, ocotilloTexture, osg::StateAttribute::ON );
	billBoardStateSet->setAttributeAndModes( new osg::BlendFunc, osg::StateAttribute::ON );

	osg::AlphaFunc* alphaFunction = new osg::AlphaFunc;
	alphaFunction->setFunction(osg::AlphaFunc::GEQUAL,0.05f);
	billBoardStateSet->setAttributeAndModes( alphaFunction, osg::StateAttribute::ON );

	srand(time(0));  // Initialize random number generator.

	osgUtil::IntersectVisitor isectVisitor;
	osg::LineSegment* terrainIsect[NUMBER_O_SHRUBS];

	int randomX, randomY;

	for (int i=0; i< NUMBER_O_SHRUBS; i++ )
	{
		randomX = (rand() % 100) + 1;
		randomY = (rand() % 100) + 1;
		std::cout << randomX <<", " << randomY << std::endl;
		terrainIsect[i] = new osg::LineSegment(
			osg::Vec3(randomX, randomY, 999) ,
			osg::Vec3(randomX, randomY, -999) );
		isectVisitor.addLineSegment(terrainIsect[i]);
	}
	terrainNode->accept(isectVisitor);

	osg::Drawable* shrubDrawable[NUMBER_O_SHRUBS];
	for (int j = 0 ; j < NUMBER_O_SHRUBS; j ++)
	{
		float randomScale = ((rand() % 15) + 1 ) / 10.0;
		shrubDrawable[j] = createShrub( randomScale, billBoardStateSet);
		osgUtil::IntersectVisitor::HitList hitList = isectVisitor.getHitList(terrainIsect[j]);
		if (! hitList.empty() )
		{    
			osgUtil::Hit firstHit = hitList.front();
			osg::Vec3 shrubPosition = firstHit.getWorldIntersectPoint();

			// osg::Vec3d shrubPosition = isectVisitor.getHitList(terrainIsect[j]).front().getWorldIntersectPoint();
			shrubBillBoard->addDrawable( shrubDrawable[j] , shrubPosition );
		}
	}
}

//粒子系统（扬尘）
void dust(osg::PositionAttitudeTransform* tanktransform)
{
	//粒子系统加入
	// Create and initialize a particle system
	osgParticle::ParticleSystem *dustParticleSystem = new osgParticle::ParticleSystem;

	// Set the attributes 'texture', 'emmisive' and 'lighting'
	dustParticleSystem->setDefaultAttributes("../NPS_Data/Textures/dust2.rgb", false, false);

	// Since the particle system is derived from the class Drawable, we can create
	// add it to the scene as a child of a geode 
	osg::Geode *geode = new osg::Geode; 
	rootNode->addChild(geode);
	geode->addDrawable(dustParticleSystem);

	// Add an 'updater' to help per-frame management
	osgParticle::ParticleSystemUpdater *dustSystemUpdater = new osgParticle::ParticleSystemUpdater;
	// Associate this updater with our particle system 
	dustSystemUpdater->addParticleSystem(dustParticleSystem);
	// add the updater node to the scene graph
	rootNode->addChild(dustSystemUpdater);

	// Create a partical to be used by our particle system and define a few
	// of its properties
	osgParticle::Particle smokeParticle; 
	smokeParticle.setSizeRange(osgParticle::rangef(0.01,8.0)); // meters
	smokeParticle.setLifeTime(3); // seconds
	smokeParticle.setMass(0.01); // in kilograms
	// Make this our particle system's default particle 
	dustParticleSystem->setDefaultParticleTemplate(smokeParticle);


	// Create a modular emitter (this contains default counter, placer and shooter.)
	osgParticle::ModularEmitter *emitter = new osgParticle::ModularEmitter;
	// Associate this emitter with the particle system 
	emitter->setParticleSystem(dustParticleSystem);

	// Get a handle to the emitter's counter and adjust the number of new particles
	// that will be added each frame
	osgParticle::RandomRateCounter *dustRate = 
		static_cast<osgParticle::RandomRateCounter *>(emitter->getCounter());
	dustRate->setRateRange(3, 8); // generate 5 to 10 particles per second

	// To customize the placer, create and initialize a multi-segment placer 
	osgParticle::MultiSegmentPlacer* lineSegment = new osgParticle::MultiSegmentPlacer();
	// Add vertices to our placer. This defines line seqments along which our particles will
	// originate. (If we co-locate a tank and this emitter, this will result in dust particles
	// originating from a line extending below and behind the tank model.) 
	lineSegment->addVertex(0,0,-2);
	lineSegment->addVertex(0,-2,-2);
	lineSegment->addVertex(0,-16,0);
	// Use this placer for our emitter 
	emitter->setPlacer(lineSegment);

	// To customize the shooter, create and initialize a radial shooter 
	osgParticle::RadialShooter* smokeShooter = new osgParticle::RadialShooter();
	// Set properties of this shooter 
	smokeShooter->setThetaRange(0.0, 3.14159/2); // radians, relative to Z axis.
	smokeShooter->setInitialSpeedRange(50,100); // meters/second
	// Use this shooter for our emitter
	emitter->setShooter(smokeShooter);

	// Add the emitter and the tank as children of this transform 

   tanktransform->addChild(emitter);   // Create a modular program and attach it to our particle system
   osgParticle::ModularProgram *moveDustInAir = new osgParticle::ModularProgram;
   moveDustInAir->setParticleSystem(dustParticleSystem);

   // Create an operator that simulates gravity, adjust it and add it to our program
   osgParticle::AccelOperator *accelUp = new osgParticle::AccelOperator;
   accelUp->setToGravity(-1); // scale factor for normal acceleration due to gravity. 
   moveDustInAir->addOperator(accelUp);

   // Add an operator to our program to calculate the friction of air.
   osgParticle::FluidFrictionOperator *airFriction = new osgParticle::FluidFrictionOperator;
   airFriction->setFluidToAir();
   //airFriction->setFluidDensity(1.2929/*air*5.0f);
   moveDustInAir->addOperator(airFriction);

   // Finally, add the program to the scene 
   rootNode->addChild(moveDustInAir);

}

//炮塔运动
void tank::update_turret()
{
	if(tu_left)
		turret_rotation+= 0.01;
	
	if(tu_right)
		turret_rotation-= 0.01;

	tankTurretNode->setCurrentHPR(osg::Vec3(turret_rotation, 0, 0));
}

//炮管运动
void tank::update_gun()
{
	if (tu_up && turret_elevation<0.5)
		turret_elevation += 0.01;

	if (tu_down && turret_elevation>0)
		turret_elevation -= 0.01;

	tankGunNode->setCurrentHPR(osg::Vec3(0, turret_elevation, 0));
}

//坦克运动函数
void  tank::update_move()   
{	
	if(t_forward)
	{
		osg::Quat old_attitude = tanktransform->getAttitude();
		osg::Vec3 old_position = tanktransform->getPosition();

		osg::Vec3 new_position;
		osg::Vec3 dis;
		osg::Matrix m;
		m.makeRotate(old_attitude);
		dis = osg::Vec3(0.0f, 0.05f, 0.0f)*m;   //前进距离
		new_position = old_position + dis;
		tanktransform->setPosition(new_position);
	}
	
	if(t_backward)
	{
		osg::Quat old_attitude = tanktransform->getAttitude();
		osg::Vec3 old_position = tanktransform->getPosition();

		osg::Vec3 new_position;
		osg::Vec3 dis;
		osg::Matrix m;
		m.makeRotate(old_attitude);
		dis = osg::Vec3(0.0f, -0.05f, 0.0f)*m;   //后退距离
		new_position = old_position + dis;
		tanktransform->setPosition(new_position);
	}

	if(t_left)
	{
		t_rotation += osg::PI/(180.0*5.0);
		tanktransform->setAttitude(osg::Quat(t_rotation, osg::Vec3(0, 0, 1)));
	}

	if(t_right)
	{
		t_rotation -= osg::PI/(180.0*5.0);
		tanktransform->setAttitude(osg::Quat(t_rotation, osg::Vec3(0, 0, 1)));
	}

	osg::Vec3 old_position = tanktransform->getPosition();
	double t_xposition = old_position.x();
	double t_yposition = old_position.y();
	set_on(t_xposition,t_yposition,terrainNode,tanktransform);   //每次运动都调用求交函数
}

//每种运动状态的赋值（是/否）
void tank::set_forward(bool b)
{
	t_forward=b;
}

void tank::set_backward(bool b)
{
	t_backward=b;
}

void tank::set_left(bool b)
{
	t_left=b;
}

void tank::set_right(bool b)
{
	t_right=b;
}

void tank::set_turretleft(bool b)
{
	tu_left=b;
}

void tank::set_turretright(bool b)
{
	tu_right=b;
}

void tank::set_gunup(bool b)
{
	tu_up=b;
}

void tank::set_gundown(bool b)
{
	tu_down=b;
}

//坦克对象
tank::tank(osg::Node* n, osg::PositionAttitudeTransform*tanktransform)
{
	turret_rotation=0.0;
	turret_elevation=0.0;
	t_rotation=0.0;
	t_forward=false;
	t_backward=false;
	t_left=false;
	t_right=false;
	tu_left=false;
	tu_right=false;
	tu_up=false;
	tu_down=false;

	this->tanktransform=tanktransform;

	findNodeVisitor findNode("turret");
	n->accept(findNode);
	tankTurretNode = dynamic_cast <osgSim::DOFTransform*> (findNode.getFirst());

	findNodeVisitor findGun("gun"); 
	n->accept(findGun);
	tankGunNode = dynamic_cast< osgSim::DOFTransform*> (findGun.getFirst());
}

//两个坦克对象
tank *t1 = NULL;
tank *t2 = NULL;

//当前控制坦克#1（true）还是坦克#2（false）
bool t1_or_t2=true;

//改变当前控制的坦克
void change()
{
    if(t1_or_t2)
		t1_or_t2=false;
	else
		t1_or_t2=true;
}

//改变坦克当前的运动状态
void tankforward()
{
	if(t1_or_t2)
	    t1->set_forward(true);
	else
		t2->set_forward(true);
}

void tankforward_stop()
{
	if(t1_or_t2)
	    t1->set_forward(false);
	else
		t2->set_forward(false);
}

void tankbackward()
{
	if(t1_or_t2)
	    t1->set_backward(true);
	else
		t2->set_backward(true);
}

void tankbackward_stop()
{
	if(t1_or_t2)
	    t1->set_backward(false);
	else
		t2->set_backward(false);
}

void tankleft()
{
	if(t1_or_t2)
	    t1->set_left(true);
	else
		t2->set_left(true);
}

void tankleft_stop()
{
	if(t1_or_t2)
	    t1->set_left(false);
	else
		t2->set_left(false);
}

void tankright()
{
	if(t1_or_t2)
	    t1->set_right(true);
	else
		t2->set_right(true);
}

void tankright_stop()
{
	if(t1_or_t2)
	    t1->set_right(false);
	else
		t2->set_right(false);
}

void turretleft()
{
	if(t1_or_t2)
		t1->set_turretleft(true);
	else
		t2->set_turretleft(true);
}

void turretleft_stop()
{
	if(t1_or_t2)
		t1->set_turretleft(false);
	else
		t2->set_turretleft(false);
}

void turretright()
{
	if(t1_or_t2)
		t1->set_turretright(true);
	else
		t2->set_turretright(true);
}

void turretright_stop()
{
	if(t1_or_t2)
		t1->set_turretright(false);
	else
		t2->set_turretright(false);
}

void gunup()
{
	if(t1_or_t2)
		t1->set_gunup(true);
	else
		t2->set_gunup(true);
}

void gunup_stop()
{
	if(t1_or_t2)
		t1->set_gunup(false);
	else
		t2->set_gunup(false);
}

void gundown()
{
	if(t1_or_t2)
		t1->set_gundown(true);
	else
		t2->set_gundown(true);
}

void gundown_stop()
{
	if(t1_or_t2)
		t1->set_gundown(false);
	else
		t2->set_gundown(false);
}

//爆炸效果
osg::Node* explode(osg::Vec3 a)
{
	osg::Group* gp = new osg::Group();

	osgParticle::ExplosionEffect* ee = new osgParticle::ExplosionEffect(a, 3.0f, 1);
	osgParticle::ExplosionDebrisEffect* ede = new osgParticle::ExplosionDebrisEffect(a, 3.0f, 1);

	gp->addChild(ee);
	gp->addChild(ede);

	return gp;
}

//坦克1向坦克2瞄准开火
void shoot1()
{
	osg::Vec3 tp1=t1->tanktransform->getPosition();
	osg::Vec3 tp2=t2->tanktransform->getPosition();
	osg::Quat tr1=t1->tanktransform->getAttitude();

	osg::Vec3 delta1=tp1-tp2;

	double r=tr1.asVec4().w()+t1->turret_rotation;

	osg::Vec3 delta2=osg::Vec3(cos(t1->turret_elevation)*cos(r),cos(t1->turret_elevation)*sin(r),sin(t1->turret_elevation));

	double delta=(delta1*delta2)/(delta1.length()*delta2.length());

	if(delta<0.5 && delta>0.0)
	   rootNode->addChild(explode(t2->tanktransform->getPosition()));
}

//坦克2向坦克1瞄准开火
void shoot2()
{
	osg::Vec3 tp1=t1->tanktransform->getPosition();
	osg::Vec3 tp2=t2->tanktransform->getPosition();
	osg::Quat tr2=t2->tanktransform->getAttitude();

	osg::Vec3 delta1=tp1-tp2;

	double r=tr2.asVec4().w()+t2->turret_rotation;

	osg::Vec3 delta2=osg::Vec3(cos(t2->turret_elevation)*cos(r),cos(t2->turret_elevation)*sin(r),sin(t2->turret_elevation));

	double delta=(delta1*delta2)/(delta1.length()*delta2.length());

	if(delta<0.5 && delta>0.0)
		rootNode->addChild(explode(t1->tanktransform->getPosition()));
}

//HUD
void simpleHUD(osg::PositionAttitudeTransform* tanktransform,string str)
{
	HUDProjectionMatrix->setMatrix(osg::Matrix::ortho2D(0,1024,0,768));//1024

	osg::MatrixTransform* HUDModelViewMatrix = new osg::MatrixTransform;
	HUDModelViewMatrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	HUDModelViewMatrix->setMatrix(osg::Matrix::identity());

	rootNode->addChild(HUDProjectionMatrix);
	HUDProjectionMatrix->addChild(HUDModelViewMatrix);
	HUDModelViewMatrix->addChild( HUDGeode );
	if(str=="1")
	{
		HUDGeode->addDrawable( textOne1 );
		textOne1->setCharacterSize(25);
		textOne1->setFont("C:/windows/Fonts/arial.ttf");
		textOne1->setText("Not so good");
		textOne1->setAxisAlignment(osgText::Text::SCREEN);
		textOne1->setPosition( osg::Vec3(242,165,1) );
		textOne1->setColor( osg::Vec4(1,0,0,1.0f) );

		tankLabel1->setCharacterSize(2);
		tankLabel1->setFont("/fonts/arial.ttf");
		tankLabel1->setText("Tank #1");	
		tankLabel1->setAxisAlignment(osgText::Text::SCREEN);
		tankLabel1->setDrawMode(osgText::Text::TEXT |
			osgText::Text::ALIGNMENT | 
			osgText::Text::BOUNDINGBOX);

		tankLabel1->setAlignment(osgText::Text::CENTER_TOP);
		tankLabel1->setPosition( osg::Vec3(0,0,4) );
		tankLabel1->setColor( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

		osg::Geode* tankLabelGeode = new osg::Geode();
		tankLabelGeode->addDrawable(tankLabel1);
		tanktransform->addChild(tankLabelGeode);
	}
	else
	{
		HUDGeode->addDrawable( textOne2 );
		textOne2->setCharacterSize(25);
		textOne2->setFont("C:/windows/Fonts/arial.ttf");
		textOne2->setText("Not so good");
		textOne2->setAxisAlignment(osgText::Text::SCREEN);
		textOne2->setPosition( osg::Vec3(242,165,1) );
		textOne2->setColor( osg::Vec4(1,0,0,1.0f) );

		tankLabel2->setCharacterSize(2);
		tankLabel2->setFont("/fonts/arial.ttf");
		tankLabel2->setText("Tank #2");	
		tankLabel2->setAxisAlignment(osgText::Text::SCREEN);
		tankLabel2->setDrawMode(osgText::Text::TEXT |
			osgText::Text::ALIGNMENT | 
			osgText::Text::BOUNDINGBOX);

		tankLabel2->setAlignment(osgText::Text::CENTER_TOP);
		tankLabel2->setPosition( osg::Vec3(0,0,4) );
		tankLabel2->setColor( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

		osg::Geode* tankLabelGeode = new osg::Geode();
		tankLabelGeode->addDrawable(tankLabel2);
		tanktransform->addChild(tankLabelGeode);
	}

}

//视点追踪的矩阵设置
void Manipulator::setByMatrix(const osg::Matrixd& mat)
{
	m=mat;
}

void Manipulator::setByInverseMatrix(const osg::Matrixd&mat)
{
	m=mat.inverse(mat);
}

osg::Matrix Manipulator::getMatrix() const
{
	return m;
}

osg::Matrix Manipulator::getInverseMatrix() const
{
	osg::Matrixd ma;
	ma = m * osg::Matrixd::rotate(-osg::PI / 2.0, osg::Vec3(1, 0, 0));
	return ma;
}

void Manipulator::updateMatrix()
{
	m = worldCoordinatesOfNode->getMatrix();
}

//事件处理
bool Manipulator::handle(const osgGA::GUIEventAdapter&ea, osgGA::GUIActionAdapter&us)
{
	switch (ea.getEventType())
	{
	case (osgGA::GUIEventAdapter::FRAME):
		{
			updateMatrix();
			return false;
		}
	}
	return false;
}

int main( int argc, char **argv )
{
   osgViewer::Viewer viewer;

   tankNode1 = osgDB::readNodeFile("../NPS_Data/Models/t72-Tank/t72-tank_des.flt");
   tankNode2 = osgDB::readNodeFile("../NPS_Data/Models/t72-Tank/t72-tank_des.flt");
   terrainNode = osgDB::readNodeFile("../NPS_Data/Models/JoeDirt/JoeDirt.flt");
   if (! (tankNode1 && tankNode2 && terrainNode))
   {
      std::cout << "Couldn't load models, quitting." << std::endl;
      return -1;
   }

   tank1_position.set(0,0,0);
   tank2_position.set(5,-5,5);

   tanktransform1->setPosition(tank1_position);
   tanktransform1->addChild(tankNode1);
   tanktransform2->setPosition(tank2_position);
   tanktransform2->addChild(tankNode2);
   rootNode->addChild(tanktransform1);
   rootNode->addChild(tanktransform2);

   rootNode->addChild(terrainNode);

   //实例化坦克对象
   t1=new tank(tankNode1,tanktransform1);
   t2=new tank(tankNode2,tanktransform2);

   tankNode1->setUserData(t1);
   tankNode2->setUserData(t2);

   //回调函数，使得坦克运动
   tankNode1->setUpdateCallback(new tankNodeCallback);
   tankNode2->setUpdateCallback(new tankNodeCallback);

   //调用粒子系统的函数，使得坦克体后有扬尘效果
   dust(tanktransform1);
   dust(tanktransform2);

   //地面随机生成植被
   plant();

   //坦克上方有显示坦克标号的渲染信息
   simpleHUD(tanktransform1,"1");
   simpleHUD(tanktransform2,"2");

   tanktransform1->addChild(tankNode1);
   rootNode->addChild(tanktransform1);
   tanktransform2->addChild(tankNode2);
   rootNode->addChild(tanktransform2);

   viewer.setSceneData(rootNode);
   viewer.realize();

   //加入键盘控制
   keyboardEventHandler* keh = new keyboardEventHandler();
   viewer.addEventHandler(keh);

   keh->addFunction('c', keyboardEventHandler::KEY_DOWN, change);//改变控制的坦克

   keh->addFunction('w', keyboardEventHandler::KEY_DOWN, tankforward);//前进
   keh->addFunction('w', keyboardEventHandler::KEY_UP, tankforward_stop);//停止前进

   keh->addFunction('s', keyboardEventHandler::KEY_DOWN, tankbackward);//后退
   keh->addFunction('s', keyboardEventHandler::KEY_UP, tankbackward_stop);//停止后退

   keh->addFunction('a', keyboardEventHandler::KEY_DOWN, tankleft);//向左转
   keh->addFunction('a', keyboardEventHandler::KEY_UP, tankleft_stop);

   keh->addFunction('d', keyboardEventHandler::KEY_DOWN, tankright);//向右转
   keh->addFunction('d', keyboardEventHandler::KEY_UP, tankright_stop);

   keh->addFunction('q', keyboardEventHandler::KEY_DOWN, turretleft);//炮塔逆时针转
   keh->addFunction('q', keyboardEventHandler::KEY_UP, turretleft_stop);

   keh->addFunction('e', keyboardEventHandler::KEY_DOWN, turretright);//炮塔顺时针转
   keh->addFunction('e', keyboardEventHandler::KEY_UP, turretright_stop);

   keh->addFunction('z', keyboardEventHandler::KEY_DOWN, gunup);   //炮管抬起
   keh->addFunction('z', keyboardEventHandler::KEY_UP, gunup_stop);

   keh->addFunction('x', keyboardEventHandler::KEY_DOWN, gundown);   //炮管下降
   keh->addFunction('x', keyboardEventHandler::KEY_UP, gundown_stop);

   keh->addFunction('k', keyboardEventHandler::KEY_DOWN, shoot1);   //坦克1开火
   keh->addFunction('l', keyboardEventHandler::KEY_DOWN, shoot2);   //坦克2开火

   //视点追踪
   //坦克#1与坦克#2的视点追踪
   osg::PositionAttitudeTransform *tank1_position = new osg::PositionAttitudeTransform();
   tank1_position->setPosition(osg::Vec3(5, -5, 5));
   tank1_position->setAttitude(osg::Quat(osg::DegreesToRadians(0.0f), osg::Vec3(0, 0, 0)));
   
   osg::PositionAttitudeTransform *tank2_position = new osg::PositionAttitudeTransform();
   tank2_position->setPosition(osg::Vec3(5, -5, 5));
   tank2_position->setAttitude(osg::Quat(osg::DegreesToRadians(-10.0f), osg::Vec3(0, 0, 0)));
   
   //全景
   osg::PositionAttitudeTransform *whole_position = new osg::PositionAttitudeTransform();
   whole_position->setPosition(osg::Vec3(150, 0, 30));
   whole_position->setAttitude(osg::Quat(osg::DegreesToRadians(90.0f), osg::Vec3(0, 0, 1)));

   tanktransform1->addChild(tank1_position);
   tanktransform2->addChild(tank2_position);
   rootNode->addChild(whole_position);

   transformAccumulator* tank1_WorldCoords = new transformAccumulator();
   transformAccumulator* tank2_WorldCoords = new transformAccumulator();
   tank1_WorldCoords->attachToGroup(tank1_position);
   tank2_WorldCoords->attachToGroup(tank2_position);
   Manipulator* track_tank1 = new Manipulator(tank1_WorldCoords);
   Manipulator* track_tank2 = new Manipulator(tank2_WorldCoords);
   
   transformAccumulator* whole_WorldCoords = new transformAccumulator();
   whole_WorldCoords->attachToGroup(whole_position);
   Manipulator* track_whole = new Manipulator(whole_WorldCoords);
   
   osgGA::KeySwitchMatrixManipulator *k = new osgGA::KeySwitchMatrixManipulator();
   if (!k)
	   return -1;
   viewer.setCameraManipulator(k);

   //不同按键可选择不同视点
   osgGA::TrackballManipulator *trackball = new osgGA::TrackballManipulator();
   k->addMatrixManipulator('0', "Trackball", trackball);
   k->addMatrixManipulator('n', "track tank", track_tank1);
   k->addMatrixManipulator('m', "track tank", track_tank2);   
   k->addMatrixManipulator('v', "track tank", track_whole);

   k->selectMatrixManipulator(0);

   viewer.setSceneData(rootNode);
   viewer.realize();

   //while( !viewer.done() )
   //{
   //   // wait for all cull and draw threads to complete.
   //   viewer.sync();

   //   // update the scene by traversing it with the update visitor which will
   //   // call all node update callbacks and animations.
   //   viewer.update();

   //   // fire off the cull and draw traversals of the scene.
   //   viewer.frame();

   //}

   // wait for all cull and draw threads to complete before exit.
   viewer.run();

   return 0;
}
