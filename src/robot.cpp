/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "robot.h"

Robot::Wheel::Wheel (Robot* robot, int _id, dReal ang, dReal ang2, int wheeltexid )
{
    id = _id;
    rob = robot;
    dReal rad = robotSetting_RobotRadius - robotSetting_WheelThickness / 2.0;
    ang *= M_PI/180.0f;
    ang2 *= M_PI/180.0f;
    dReal x = rob->m_x;
    dReal y = rob->m_y;
    dReal z = rob->m_z;
    dReal centerx = x + rad * cos( ang2 );
    dReal centery = y + rad * sin( ang2 );
    dReal centerz = z - robotSetting_RobotHeight * 0.5 + robotSetting_WheelRadius - robotSetting_BottomHeight;
    cyl = new PCylinder( centerx, centery, centerz, robotSetting_WheelRadius, robotSetting_WheelThickness, robotSetting_WheelMass, wheeltexid );
    cyl->setRotation( -sin( ang ), cos( ang ), 0, M_PI * 0.5 );
    cyl->setBodyRotation( -sin( ang ), cos( ang ), 0, M_PI * 0.5, true );   //set local rotation matrix
    cyl->setBodyPosition( centerx - x, centery - y, centerz - z, true );   //set local position vector
    cyl->space = rob->space;

    rob->w->addObject( cyl );

    joint = dJointCreateHinge( rob->w->world, 0 );

    dJointAttach( joint, rob->chassis->body,cyl->body );
    const dReal *a = dBodyGetPosition( cyl->body );
    dJointSetHingeAxis( joint, cos( ang ), sin( ang ), 0 );
    dJointSetHingeAnchor( joint, a[0], a[1], a[2] );

    motor = dJointCreateAMotor( rob->w->world, 0 );
    dJointAttach( motor, rob->chassis->body, cyl->body );
    dJointSetAMotorNumAxes( motor, 1 );
    dJointSetAMotorAxis( motor, 0, 1, cos( ang ), sin( ang ), 0 );
    dJointSetAMotorParam( motor, dParamFMax, robotSetting_Wheel_Motor_FMax );
    speed = 0;
}

void Robot::Wheel::step()
{
    dJointSetAMotorParam( motor, dParamVel, speed );
    dJointSetAMotorParam(motor , dParamFMax, robotSetting_Wheel_Motor_FMax );
}

Robot::Kicker::Kicker( Robot* robot ) : holdingBall( false )
{
    rob = robot;

    dReal x = rob->m_x;
    dReal y = rob->m_y;
    dReal z = rob->m_z;
    dReal centerx = x + (robotSetting_RobotCenterFromKicker + robotSetting_KickerThickness );
    dReal centery = y;
    dReal centerz = z - (robotSetting_RobotHeight) * 0.5f + robotSetting_WheelRadius - robotSetting_BottomHeight + robotSetting_KickerZ;
    box = new PBox( centerx, centery, centerz, robotSetting_KickerThickness, robotSetting_KickerWidth, robotSetting_KickerHeight, robotSetting_KickerMass );
    box->setBodyPosition( centerx - x, centery - y, centerz - z, true );
    box->space = rob->space;

    rob->w->addObject( box );

    joint = dJointCreateHinge( rob->w->world, 0 );
    dJointAttach( joint, rob->chassis->body,box->body );
    const dReal *aa = dBodyGetPosition( box->body );
    dJointSetHingeAnchor( joint, aa[0], aa[1], aa[2] );
    dJointSetHingeAxis( joint, 0, -1, 0 );

    dJointSetHingeParam( joint, dParamVel, 0);
    dJointSetHingeParam( joint, dParamLoStop, 0);
    dJointSetHingeParam( joint, dParamHiStop, 0);

    rolling = 0;
    kicking = NO_KICK;
}

void Robot::Kicker::step()
{
    if ( !isTouchingBall() or rolling == 0 ) unholdBall();
    if ( kicking != NO_KICK )
    {
        kickstate--;
        if ( kickstate <= 0 ) kicking = NO_KICK;
    }
    else if ( rolling != 0 )
    {
        if ( isTouchingBall() )
        {
            holdBall();
        }
    }
}

bool Robot::Kicker::isTouchingBall()
{
    dReal vx, vy, vz;
    dReal bx, by, bz;
    dReal kx, ky, kz;
    rob->chassis->getBodyDirection( vx, vy, vz );
    rob->getBall()->getBodyPosition( bx, by, bz );
    box->getBodyPosition( kx, ky, kz );
    kx += vx * robotSetting_KickerThickness * 0.5f;
    ky += vy * robotSetting_KickerThickness * 0.5f;
    dReal xx = fabs( ( kx - bx ) * vx + ( ky - by ) * vy );
    dReal yy = fabs( - ( kx - bx ) * vy + ( ky - by ) * vx );
    dReal zz = fabs( kz - bz );
    return ( ( xx < robotSetting_KickerThickness * 2.0f + ballSetting_BallRadius ) and ( yy < robotSetting_KickerWidth * 0.5f ) and ( zz < robotSetting_KickerHeight * 0.5f ) );
}

KickStatus Robot::Kicker::isKicking()
{
    return kicking;
}

void Robot::Kicker::setRoller( int roller )
{
    rolling = roller;
}

int Robot::Kicker::getRoller()
{
    return rolling;
}

void Robot::Kicker::toggleRoller()
{
    if ( rolling == 0 )
        rolling = 1;
    else rolling = 0;
}

void Robot::Kicker::kick( dReal kickspeedx, dReal kickspeedz )
{    
    dReal dx, dy, dz;
    dReal vx, vy, vz;
    rob->chassis->getBodyDirection( dx, dy, dz ); dz = 0;
    dReal zf = kickspeedz;
    unholdBall();
    if ( isTouchingBall() )
    {
        dReal dlen = dx * dx + dy * dy + dz * dz;
        dlen = sqrt( dlen );
        vx = dx * kickspeedx / dlen;
        vy = dy * kickspeedx / dlen;
        vz = zf;
        const dReal* vball = dBodyGetLinearVel( rob->getBall()->body );
        dReal vn = - ( vball[0] * dx + vball[1] * dy ) * robotSetting_KickerDampFactor;
        dReal vt = - ( vball[0] * dy - vball[1] * dx);
        vx += vn * dx - vt * dy;
        vy += vn * dy + vt * dx;
        dBodySetLinearVel( rob->getBall()->body, vx, vy, vz );
        if ( kickspeedz >= 1 )
            kicking = CHIP_KICK;
        else
            kicking = FLAT_KICK;
        kickstate = 10;
    }
}

void Robot::Kicker::holdBall(){
    dReal vx, vy, vz;
    dReal bx, by, bz;
    dReal kx, ky, kz;
    rob->chassis->getBodyDirection( vx, vy, vz );
    rob->getBall()->getBodyPosition( bx, by, bz );
    box->getBodyPosition( kx, ky, kz );
    kx += vx * robotSetting_KickerThickness * 0.5f;
    ky += vy * robotSetting_KickerThickness * 0.5f;
    dReal xx = fabs( ( kx - bx ) * vx + ( ky - by ) * vy );
    dReal yy = fabs( - ( kx - bx ) * vy + ( ky - by ) * vx );
    if( holdingBall or xx - ballSetting_BallRadius < 0 ) return;
    dBodySetLinearVel( rob->getBall()->body, 0, 0, 0 );
    robot_to_ball = dJointCreateHinge( rob->getWorld()->world, 0 );
    dJointAttach( robot_to_ball, box->body, rob->getBall()->body );
    holdingBall = true;
}

void Robot::Kicker::unholdBall(){
    if( holdingBall ) 
    {
        dJointDestroy( robot_to_ball );
        holdingBall = false;
    }
}

Robot::Robot( PWorld* world, PBall *ball, dReal x, dReal y, dReal z, int rob_id, int wheeltexid, int dir )
{      
    m_x = x;
    m_y = y;
    m_z = z;
    w = world;
    m_ball = ball;
    m_dir = dir;
    m_rob_id = rob_id;

    space = w->space;

    chassis = new PCylinder( x, y, z, robotSetting_RobotRadius, robotSetting_RobotHeight, robotSetting_BodyMass * 0.99f, rob_id, true );
    chassis->space = space;
    w->addObject( chassis );

    dummy = new PBall( x, y, z, robotSetting_RobotCenterFromKicker, robotSetting_BodyMass * 0.01f );
    dummy->space = space;
    w->addObject( dummy );

    dummy_to_chassis = dJointCreateFixed( world->world, 0 );
    dJointAttach( dummy_to_chassis, chassis->body, dummy->body );

    kicker = new Kicker( this );

    wheels[0] = new Wheel( this, 0, robotSetting_Wheel1Angle, robotSetting_Wheel1Angle, wheeltexid );
    wheels[1] = new Wheel( this, 1, robotSetting_Wheel2Angle, robotSetting_Wheel2Angle, wheeltexid );
    wheels[2] = new Wheel( this, 2, robotSetting_Wheel3Angle, robotSetting_Wheel3Angle, wheeltexid );
    wheels[3] = new Wheel( this, 3, robotSetting_Wheel4Angle, robotSetting_Wheel4Angle, wheeltexid );
    firsttime = true;
    on = true;
}

Robot::~Robot()
{
}

PBall* Robot::getBall()
{
    return m_ball;
}

PWorld* Robot::getWorld()
{
    return w;
}

int Robot::getID()
{
    return m_rob_id - 1;
}

void normalizeVector( dReal& x, dReal& y, dReal& z )
{
    dReal d = sqrt( x * x + y * y + z * z );
    x /= d;
    y /= d;
    z /= d;
}

void Robot::step()
{    
    if ( on )
    {
        if ( firsttime )
        {
            if ( m_dir == -1 ) setDir( 180 );
            firsttime = false;
        }
        wheels[0]->step();
        wheels[1]->step();
        wheels[2]->step();
        wheels[3]->step();
        kicker->step();
    }
    else 
    {
        if ( last_state )
        {
            wheels[0]->speed = wheels[1]->speed = wheels[2]->speed = wheels[3]->speed = 0;
            kicker->setRoller( 0 );
            wheels[0]->step();
            wheels[1]->step();
            wheels[2]->step();
            wheels[3]->step();
            kicker->step();
        }
    }
    last_state = on;
}

void Robot::resetSpeeds()
{
    wheels[0]->speed = wheels[1]->speed = wheels[2]->speed = wheels[3]->speed = 0;
}


void Robot::resetRobot()
{
    resetSpeeds();
    dBodySetLinearVel( chassis->body, 0, 0, 0 );
    dBodySetAngularVel( chassis->body, 0, 0, 0 );
    dBodySetLinearVel( dummy->body, 0, 0, 0 );
    dBodySetAngularVel( dummy->body, 0, 0, 0 );
    dBodySetLinearVel( kicker->box->body, 0, 0, 0 );
    dBodySetAngularVel( kicker->box->body, 0, 0, 0 );
    for ( int i = 0; i < 4; i++ )
    {
        dBodySetLinearVel( wheels[i]->cyl->body, 0, 0, 0 );
        dBodySetAngularVel( wheels[i]->cyl->body, 0, 0, 0 );
    }
    dReal x,y;
    getXY( x, y );
    setXY( x, y );
    if ( m_dir == -1 ) setDir( 180 );
    else setDir( 0 );
}

void Robot::getXY( dReal& x, dReal &y )
{
    dReal xx, yy, zz;
    chassis->getBodyPosition( xx, yy, zz );
    x = xx;
    y = yy;
}

dReal Robot::getDir()
{
    dReal x, y, z;
    chassis->getBodyDirection( x, y, z );
    dReal dot = x;//zarb dar (1.0,0.0,0.0)
    dReal length = sqrt( x * x + y * y );
    dReal absAng = ( dReal )( acos ( ( dReal )( dot / length ) ) * ( 180.0f / M_PI ) );
    return ( y > 0 ) ? absAng : -absAng;
}

dReal Robot::getDir( dReal &k )
{
    dReal x, y, z;
    chassis->getBodyDirection( x, y, z, k );
    dReal dot = x;//zarb dar (1.0,0.0,0.0)
    dReal length = sqrt( x * x + y * y );
    dReal absAng = ( dReal )( acos ( ( dReal )( dot / length ) ) * ( 180.0f / M_PI ) );
    return ( y > 0 ) ? absAng : -absAng;
}

void Robot::setXY( dReal x, dReal y )
{
    dReal xx, yy, zz, kx, ky, kz;
    dReal height = ROBOT_START_Z();
    chassis->getBodyPosition( xx, yy, zz );
    chassis->setBodyPosition( x, y, height );
    dummy->setBodyPosition( x, y, height );
    kicker->box->getBodyPosition( kx, ky, kz );
    kicker->box->setBodyPosition( kx - xx + x, ky - yy + y, kz - zz + height );
    for ( int i = 0; i < 4; i++ )
    {
        wheels[i]->cyl->getBodyPosition( kx, ky, kz );
        wheels[i]->cyl->setBodyPosition( kx - xx + x, ky - yy + y, kz - zz + height );
    }
}

void Robot::setDir( dReal ang )
{
    ang *= M_PI/180.0f;
    chassis->setBodyRotation( 0, 0, 1, ang );
    kicker->box->setBodyRotation( 0, 0, 1, ang );
    dummy->setBodyRotation( 0, 0, 1, ang );
    dMatrix3 wLocalRot, wRot, cRot;
    dVector3 localPos, finalPos, cPos;
    chassis->getBodyPosition( cPos[0], cPos[1], cPos[2], false );
    chassis->getBodyRotation( cRot, false );
    kicker->box->getBodyPosition( localPos[0], localPos[1], localPos[2], true );
    dMultiply0( finalPos, cRot, localPos, 4, 3, 1);
    finalPos[0] += cPos[0];
    finalPos[1] += cPos[1];
    finalPos[2] += cPos[2];
    kicker->box->setBodyPosition( finalPos[0], finalPos[1], finalPos[2], false );
    for ( int i = 0; i < 4; i++ )
    {
        wheels[i]->cyl->getBodyRotation( wLocalRot, true );
        dMultiply0( wRot, cRot, wLocalRot, 3, 3, 3 );
        dBodySetRotation( wheels[i]->cyl->body, wRot );
        wheels[i]->cyl->getBodyPosition( localPos[0], localPos[1], localPos[2], true );
        dMultiply0( finalPos, cRot, localPos, 4, 3, 1 );
        finalPos[0] += cPos[0];
        finalPos[1] += cPos[1];
        finalPos[2] += cPos[2];
        wheels[i]->cyl->setBodyPosition( finalPos[0], finalPos[1], finalPos[2], false );
    }
}

void Robot::setSpeed( int i, dReal s )
{
    if ( ! ( ( i >= 4 ) or ( i < 0 ) ) )
        wheels[i]->speed = s;
}

void Robot::setSpeed( dReal vx, dReal vy, dReal vw )
{
    // Calculate Motor Speeds
    dReal _DEG2RAD = M_PI / 180.0;
    dReal motorAlpha[4] = {robotSetting_Wheel1Angle * _DEG2RAD, robotSetting_Wheel2Angle * _DEG2RAD, robotSetting_Wheel3Angle * _DEG2RAD, robotSetting_Wheel4Angle * _DEG2RAD};

    dReal dw1 = ( 1.0 / robotSetting_WheelRadius ) * ( ( ( robotSetting_RobotRadius * vw ) - ( vx * sin( motorAlpha[0] ) ) + ( vy * cos( motorAlpha[0] ) ) ) );
    dReal dw2 = ( 1.0 / robotSetting_WheelRadius ) * ( ( ( robotSetting_RobotRadius * vw ) - ( vx * sin( motorAlpha[1] ) ) + ( vy * cos( motorAlpha[1] ) ) ) );
    dReal dw3 = ( 1.0 / robotSetting_WheelRadius ) * ( ( ( robotSetting_RobotRadius * vw ) - ( vx * sin( motorAlpha[2] ) ) + ( vy * cos( motorAlpha[2] ) ) ) );
    dReal dw4 = ( 1.0 / robotSetting_WheelRadius ) * ( ( ( robotSetting_RobotRadius * vw ) - ( vx * sin( motorAlpha[3] ) ) + ( vy * cos( motorAlpha[3] ) ) ) );

    setSpeed( 0 , dw1 );
    setSpeed( 1 , dw2 );
    setSpeed( 2 , dw3 );
    setSpeed( 3 , dw4 );
}

dReal Robot::getSpeed( int i )
{
    if ( ( i >= 4 ) or ( i < 0 ) ) return -1;
    return wheels[i]->speed;
}

void Robot::incSpeed( int i, dReal v )
{
    if ( ! ( ( i >= 4 ) or ( i < 0 ) ) )
        wheels[i]->speed += v;
}