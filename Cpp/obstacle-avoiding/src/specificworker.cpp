/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx, bool startup_check) : GenericWorker(mprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    QRectF dimensions(-5000,-2500,10000, 5000);
    viewer = new AbstractGraphicViewer(this, dimensions);

    this->resize(900,450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

	this->Period = 100;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(bState.alpha * 180 / M_PI);
        robot_polygon->setPos(bState.x, bState.z);

    }
    catch (const Ice::Exception &ex) { std::cout << ex << std::endl; }
    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        draw_laser(ldata, Eigen::Vector2f(bState.x, bState.z), bState.alpha);
    }
    catch (const Ice::Exception &ex) { std::cout << ex << std::endl; }

    if (target.active)
    {
        float adv, rot;
        // convertir el punto al sistema de referencia del robot -> pr
        Eigen::Vector2f pr = world_to_robot(Eigen::Vector2f(bState.x, bState.z), bState.alpha,
                                            Eigen::Vector2f(target.pos.x(), target.pos.y()));
        // convertir al SR del robot
        float beta = atan2(pr.x(), pr.y());
        // distacia al targe: módulo de pr
        float dist = pr.norm();

        if (dist < 100)
        {
            adv = 0;
            rot = 0;
            target.active = false;
        } else // mormal
        {
            // comprobar distancias laterales
            // si es menor que un umbral, generar un delta sobre beta
            rot = beta;
            adv = MAX_ADV_SPEED * break_rotation(beta) * break_target(dist);
        }
        try { differentialrobot_proxy->setSpeedBase(adv, rot); }
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }
}
/////////////////////////////////////////////////////////////////////////
float SpecificWorker::min_dist_laser_segement(const RoboCompLaser::TLaserData &ldata, int a, int b)
{
    // comprobar que A y B
    auto min = std::min_element(ldata.begin()+a, ldata.begin()+b, [](auto a, auto b){return a.dist < b.dist;});
    return (*min).dist;
}
float SpecificWorker::break_rotation(float beta)
{
    return exp(-beta*beta);
}
float SpecificWorker::break_target(float dist)
{
    if(dist > 1000)
        return 1;
    else
        return (1/1000.f)*dist;
}
Eigen::Vector2f SpecificWorker::world_to_robot(const Eigen::Vector2f &robot, float ra, const Eigen::Vector2f &target)
{
    Eigen::Matrix2f rot;
    rot << cos(ra), -sin(ra), sin(ra), cos(ra);
    return rot.transpose() * (target - robot);
}
Eigen::Vector2f SpecificWorker::robot_to_world(const Eigen::Vector2f &robot, float ra, const Eigen::Vector2f &x)
{
    Eigen::Matrix2f rot;
    rot << cos(ra), -sin(ra), sin(ra), cos(ra);
    return rot * x + robot;
}
void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata, Eigen::Vector2f robot, float ra)
{
    // borrar el laser antiguo
    static QGraphicsItem *polygon;
    if(polygon != nullptr)
        viewer->scene.removeItem(polygon);

    // pintar el actual
    QPolygonF poly;
    auto origen = robot_to_world(robot, ra, Eigen::Vector2f(0,0));
    poly << QPointF(origen.x(), origen.y());
    for(const auto &l : ldata)
    {
        auto lw = robot_to_world(robot, ra, Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle)));
        poly << QPointF(lw.x(), lw.y());
    }
    QColor color("LightGreen");
    color.setAlpha(40);
    polygon = viewer->scene.addPolygon(poly, QPen(color),QBrush(color));
    polygon->setZValue(3);
}
/////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::new_target_slot(QPointF p)
{
    qInfo() << p;
    target.pos = p;
    target.active = true;
}


/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData




//    try
//    {
//        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
//        limit = ldata.size()/3;
//        ldata.erase(ldata.begin(), ldata.begin()+limit);
//        ldata.erase(ldata.end()-limit, ldata.end());
//        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
//
//	    if( ldata.front().dist < threshold)
//	    {
//            differentialrobot_proxy->setSpeedBase(0, rot);
//            usleep(1250000);
//            std::cout << ldata.front().dist << std::endl;
//            differentialrobot_proxy->setSpeedBase(300, 0);
//            usleep(500000);
//	    }
//        else
//        {
//            differentialrobot_proxy->setSpeedBase(300, 0);
//            usleep(500000);
//            std::cout << ldata.front().dist << std::endl;
//        }
//    }
//    catch(const Ice::Exception &ex)
//    {
//        std::cout << ex << std::endl;
//    }