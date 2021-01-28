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

#include "sslworld.h"
#include <QtGlobal>

SSLWorld* _w;
RobotsFomation *form1 = new RobotsFomation( 1 );
RobotsFomation *form2 = new RobotsFomation( 1 );

dReal fric( dReal f )
{
    if ( f == -1 ) return dInfinity;
    return f;
}

bool wheelCallBack( dGeomID o1, dGeomID o2, PSurface* s, int /*robots_count*/ )
{
    //s->id2 is ground
    const dReal* r; //wheels rotation matrix
    if ( ( o1 == s->id1 ) and ( o2 == s->id2 ) )
    {
        r = dBodyGetRotation( dGeomGetBody( o1 ) );
    } else if ( ( o1 == s->id2 ) and ( o2 == s->id1 ) )
    {
        r = dBodyGetRotation( dGeomGetBody ( o2 ) );
    } else 
    {
        //XXX: in this case we dont have the rotation
        //     matrix, thus we must return
        return false;
    }

    s->surface.mode = dContactFDir1 | dContactMu2  | dContactApprox1 | dContactSoftCFM;
    s->surface.mu = fric( robotSetting_WheelPerpendicularFriction );
    s->surface.mu2 = fric( robotSetting_WheelTangentFriction );
    s->surface.soft_cfm = 0.002;

    dVector3 v = { 0, 0, 1, 1 };
    dVector3 axis;
    dMultiply0( axis, r, v, 4, 3, 1 );
    dReal l = sqrt( axis[0] * axis[0] + axis[1] * axis[1] );
    s->fdir1[0] = axis[0] / l;
    s->fdir1[1] = axis[1] / l;
    s->fdir1[2] = 0;
    s->fdir1[3] = 0;
    s->usefdir1 = true;
    return true;
}

bool rayCallback( dGeomID o1, dGeomID o2, PSurface* s, int robots_count )
{
    return false;
}

bool ballCallBack( dGeomID o1, dGeomID o2, PSurface* s, int /*robots_count*/ )
{
    if ( _w->ball->tag != -1 ) //spinner adjusting
    {
        dReal x, y, z;
        _w->robots[_w->ball->tag]->chassis->getBodyDirection( x, y, z );
        s->fdir1[0] = x;
        s->fdir1[1] = y;
        s->fdir1[2] = 0;
        s->fdir1[3] = 0;
        s->usefdir1 = true;
        s->surface.mode = dContactMu2 | dContactFDir1 | dContactSoftCFM;
        s->surface.mu = ballSetting_BallFriction;
        s->surface.mu2 = 0.5;
        s->surface.soft_cfm = 0.002;
    }
    return true;
}

SSLWorld::SSLWorld( QObject *parent ) :
    QObject( parent )
{
    customDT = -1;    
    _w = this;
    framenum = 0;
    last_dt = -1;

    observationSocket = new QUdpSocket();
    commandsSocket = new QUdpSocket();
    commandsSocket->bind( QHostAddress::Any, worldSetting_commands_port );
    QObject::connect( commandsSocket, SIGNAL( readyRead() ), this, SLOT( receivePacket() ) );

    p = new PWorld( worldSetting_DeltaTime, 9.81f, Robots_Count );
    ball = new PBall ( 0, 0, 0.5, ballSetting_BallRadius, ballSetting_BallMass );

    ground = new PGround( fieldSetting_Field_Rad, fieldSetting_Field_Length, fieldSetting_Field_Width, fieldSetting_Field_Penalty_Depth, fieldSetting_Field_Penalty_Width, fieldSetting_Field_Penalty_Point, fieldSetting_Field_Line_Width, 0 );
    ray = new PRay( 50 );
    
    // Bounding walls
    const double thick = fieldSetting_Wall_Thickness;
    const double increment = fieldSetting_Field_Margin + fieldSetting_Field_Referee_Margin + thick / 2;
    const double pos_x = fieldSetting_Field_Length / 2.0 + increment;
    const double pos_y = fieldSetting_Field_Width / 2.0 + increment;
    const double pos_z = 0.0;
    const double siz_x = 2.0 * pos_x;
    const double siz_y = 2.0 * pos_y;
    const double siz_z = 0.4;
    const double tone = 1.0;
    
    walls[0] = new PFixedBox( thick / 2, pos_y, pos_z,
                             siz_x, thick, siz_z);

    walls[1] = new PFixedBox( -thick / 2, -pos_y, pos_z,
                             siz_x, thick, siz_z);
    
    walls[2] = new PFixedBox( pos_x, -thick / 2, pos_z,
                             thick, siz_y, siz_z);

    walls[3] = new PFixedBox( -pos_x, thick / 2, pos_z,
                             thick, siz_y, siz_z);
    
    // Goal walls
    const double gthick = fieldSetting_Goal_Thickness;
    const double gpos_x = ( fieldSetting_Field_Length + gthick ) / 2.0 + fieldSetting_Goal_Depth;
    const double gpos_y = ( fieldSetting_Goal_Width + gthick ) / 2.0;
    const double gpos_z = fieldSetting_Goal_Height / 2.0;
    const double gsiz_x = fieldSetting_Goal_Depth + gthick;
    const double gsiz_y = fieldSetting_Goal_Width;
    const double gsiz_z = fieldSetting_Goal_Height;
    const double gpos2_x = ( fieldSetting_Field_Length + gsiz_x ) / 2.0;

    walls[4] = new PFixedBox( gpos_x, 0.0, gpos_z,
                             gthick, gsiz_y, gsiz_z);
    
    walls[5] = new PFixedBox( gpos2_x, -gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);
    
    walls[6] = new PFixedBox( gpos2_x, gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);

    walls[7] = new PFixedBox( -gpos_x, 0.0, gpos_z,
                             gthick, gsiz_y, gsiz_z);
    
    walls[8] = new PFixedBox( -gpos2_x, -gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);
    
    walls[9] = new PFixedBox( -gpos2_x, gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z);
    
    p->addObject( ground );
    p->addObject( ball );
    p->addObject( ray );
    for ( auto & wall : walls ) p->addObject( wall );
    const int wheeltexid = 4 * Robots_Count + 12 + 1; //37 for 6 robots


    for ( int k = 0; k < Robots_Count; k++ )
    {
        float a1 = -form1->x[k];
        float a2 = form1->y[k];
        float a3 = ROBOT_START_Z();
        robots[k] = new Robot(p,
                              ball,
                              -form1->x[k],
                              form1->y[k],
                              ROBOT_START_Z(),
                              k + 1,
                              wheeltexid,
                              1);
    }

    if ( TEAM_COUNT == 2 )
    {
        for ( int k = 0; k < Robots_Count; k++ )
            robots[k + Robots_Count] = new Robot(p,
                                                ball,
                                                form2->x[k],
                                                form2->y[k],
                                                ROBOT_START_Z(),
                                                k + Robots_Count+1,
                                                wheeltexid,
                                                -1);
    }

    p->initAllObjects();

    //Surfaces
    p->createSurface( ray, ground )->callback = rayCallback;
    p->createSurface( ray, ball )->callback = rayCallback;
    for ( int k=0; k < Robots_Count * TEAM_COUNT; k++ )
    {
        p->createSurface( ray, robots[k]->chassis )->callback = rayCallback;
        p->createSurface( ray, robots[k]->dummy )->callback = rayCallback;
    }
    PSurface ballwithwall;
    ballwithwall.surface.mode = dContactBounce | dContactApprox1;
    ballwithwall.surface.mu = 1;
    ballwithwall.surface.bounce = ballSetting_BallBounce;
    ballwithwall.surface.bounce_vel = ballSetting_BallBounceVel;
    ballwithwall.surface.slip1 = 0;

    PSurface wheelswithground;
    PSurface* ball_ground = p->createSurface( ball, ground );
    ball_ground->surface = ballwithwall.surface;
    ball_ground->callback = ballCallBack;

    PSurface ballwithkicker;
    ballwithkicker.surface.mode = dContactApprox1;
    ballwithkicker.surface.mu = fric( robotSetting_Kicker_Friction );
    ballwithkicker.surface.slip1 = 5;
    
    for ( auto & wall : walls ) p->createSurface( ball, wall )->surface = ballwithwall.surface;
    
    for ( int k = 0; k < Robots_Count * TEAM_COUNT; k++ )
    {
        p->createSurface( robots[k]->chassis, ground );
        for ( auto & wall : walls ) p->createSurface( robots[k]->chassis, wall );
        p->createSurface( robots[k]->dummy, ball );
        p->createSurface( robots[k]->kicker->box, ball )->surface = ballwithkicker.surface;
        for ( auto & wheel : robots[k]->wheels )
        {
            p->createSurface( wheel->cyl, ball );
            PSurface* w_g = p->createSurface( wheel->cyl, ground );
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = wheelCallBack;
        }
        for ( int j = k + 1; j < Robots_Count * TEAM_COUNT; j++ )
        {            
            if (k != j)
            {
                p->createSurface( robots[k]->dummy, robots[j]->dummy ); //seams ode doesn't understand cylinder-cylinder contacts, so I used spheres
                p->createSurface( robots[k]->chassis, robots[j]->kicker->box );
            }
        }
    }

    // initialize robot state
    for ( int team = 0; team < TEAM_COUNT; ++team )
    {
        for ( int i = 0; i < MAX_ROBOT_COUNT; ++i )
        {
            lastInfraredState[team][i] = false;
            lastKickState[team][i] = NO_KICK; 
        }
    }
}

int SSLWorld::robotIndex( int robot, int team )
{
    if ( robot >= Robots_Count ) return -1;
    return robot + team * Robots_Count;
}

SSLWorld::~SSLWorld()
{
    delete p;
}

void SSLWorld::step( dReal dt )
{
    if ( customDT > 0 ) dt = customDT;

    int ballCollisionTry = 5;
    for ( int kk=0; kk < ballCollisionTry; kk++ )
    {
        const dReal* ballvel = dBodyGetLinearVel( ball->body );
        dReal ballspeed = ballvel[0] * ballvel[0] + ballvel[1] * ballvel[1] + ballvel[2] * ballvel[2];
        ballspeed = sqrt( ballspeed );
        dReal ballfx=0, ballfy=0, ballfz = 0;
        dReal balltx=0, ballty=0, balltz=0;
        if ( ballspeed > 0.01 ) {
            dReal fk = ballSetting_BallFriction * ballSetting_BallMass * worldSetting_Gravity;
            ballfx = -fk * ballvel[0] / ballspeed;
            ballfy = -fk * ballvel[1] / ballspeed;
            ballfz = -fk * ballvel[2] / ballspeed;
            balltx = -ballfy * ballSetting_BallRadius;
            ballty = ballfx * ballSetting_BallRadius;
            balltz = 0;
            dBodyAddTorque( ball->body, balltx, ballty, balltz );
        }
        dBodyAddForce( ball->body, ballfx, ballfy, ballfz );
        if ( dt == 0 ) dt = last_dt;
        else last_dt = dt;

        selected = -1;
        p->step( dt / ballCollisionTry );
    }

    ball->tag = -1;
    for (int k = 0; k < Robots_Count * TEAM_COUNT; k++ )
    {
        robots[k]->step();
        robots[k]->selected = false;
    }

    dMatrix3 R;

    framenum ++;
}

void SSLWorld::sendPacket()
{
    Observation* packet = new Observation;
    packet->set_frame_number( framenum );

    dReal x, y, z, dir, k, vx, vy, vw;
    ball->getBodyPosition( x, y, z );
    const auto vel_vec = dBodyGetLinearVel( ball->body );
    Ball* ba = packet->mutable_ball()->Add();
    ba->set_vx( vel_vec[0] );
    ba->set_vy( vel_vec[1] );
    ba->set_x( x );
    ba->set_y( y );

    for ( int i = 0; i < Robots_Count; i++ )
    {
        robots[i]->getXY( x, y );
        dir = robots[i]->getDir( k );

        const dReal* vv = dBodyGetLinearVel( robots[i]->chassis->body );
        const dReal* vv2 = dBodyGetAngularVel( robots[i]->chassis->body );

        KickStatus kicking = robots[i]->kicker->isKicking();
        // Reset Turn Over
        if ( k < 0.9 )
            robots[i]->resetRobot();

        Robots* ro = packet->mutable_robots()->Add();
        ro->set_id( i );
        ro->set_yellow_team( false );
        ro->set_x( x*1000.0f );
        ro->set_y( y*1000.0f );
        ro->set_orientation( ( dir ) * M_PI / 180.0f );
        ro->set_vx( vv[0] );
        ro->set_vy( vv[1] );
        ro->set_vw( vv2[2] );
        ro->set_touch_ball( robots[i]->kicker->isTouchingBall() );
        if ( kicking == NO_KICK )
        {
            ro->set_chip_kick( false );
            ro->set_flat_kick( false );
        }
        if ( kicking == FLAT_KICK )
        {
            ro->set_chip_kick( false );
            ro->set_flat_kick( true );
        }
        if ( kicking == CHIP_KICK )
        {
            ro->set_chip_kick( true );
            ro->set_flat_kick( false );
        }
    }

    // For yellow robots
    if ( TEAM_COUNT == 2 )
    {
        for ( int i = Robots_Count; i < 2 * Robots_Count; i++ )
        {
            robots[i]->getXY( x, y );
            dir = robots[i]->getDir( k );

            const dReal* vv = dBodyGetLinearVel( robots[i]->chassis->body );
            const dReal* vv2 = dBodyGetAngularVel( robots[i]->chassis->body );

            KickStatus kicking = robots[i]->kicker->isKicking();
            // Reset Turn Over
            if ( k < 0.9 )
                robots[i]->resetRobot();

            Robots* ro = packet->mutable_robots()->Add();
            ro->set_id( i - Robots_Count );
            ro->set_yellow_team( true );
            ro->set_x( x*1000.0f );
            ro->set_y( y*1000.0f );
            ro->set_orientation( ( dir ) * M_PI / 180.0f );
            ro->set_vx( vv[0] );
            ro->set_vy( vv[1] );
            ro->set_vw( vv2[2] );
            ro->set_touch_ball( robots[i]->kicker->isTouchingBall() );
            if ( kicking == NO_KICK )
            {
                ro->set_chip_kick( false );
                ro->set_flat_kick( false );
            }
            if ( kicking == FLAT_KICK )
            {
                ro->set_chip_kick( false );
                ro->set_flat_kick( true );
            }
            if ( kicking == CHIP_KICK )
            {
                ro->set_chip_kick( true );
                ro->set_flat_kick( false );
            }
        }   
    }

    int size = packet->ByteSize();
    QByteArray buffer( size, 0 );
    packet->SerializeToArray( buffer.data(), buffer.size() );
    observationSocket->writeDatagram( buffer.data(), buffer.size(), QHostAddress::LocalHost, worldSetting_observation_port );
}

void SSLWorld::receivePacket()
{
    QHostAddress sender;
    quint16 port;
    while ( commandsSocket->hasPendingDatagrams() )
    {
        int size = commandsSocket->readDatagram( in_buffer, 65536, &sender, &port );
        if ( size > 0 )
        {
            Commands packet;
            packet.ParseFromArray( in_buffer, size );

            if ( packet.has_reset_frame_number() )
            {
                if ( packet.reset_frame_number() )
                    framenum = 0;
            }

            if( packet.has_replacement() )
            {
                if( packet.replacement().has_ball_replacement()) 
                {
                    dReal x = 0, y = 0, vx = 0, vy = 0;
                    if ( packet.replacement().ball_replacement().has_x() )  x  = packet.replacement().ball_replacement().x();
                    if ( packet.replacement().ball_replacement().has_y() )  y  = packet.replacement().ball_replacement().y();
                    if ( packet.replacement().ball_replacement().has_vx() ) vx = packet.replacement().ball_replacement().vx();
                    if ( packet.replacement().ball_replacement().has_vy() ) vy = packet.replacement().ball_replacement().vy();

                    ball->setBodyPosition( x, y, ballSetting_BallRadius * 1.2 );
                    dBodySetLinearVel( ball->body, vx, vy, 0 );
                    dBodySetAngularVel( ball->body, 0, 0, 0 );
                }

                for( int i = 0; i < packet.replacement().robot_replacement_size(); i++ )
                {
                    int team = 0;
                    if( packet.replacement().robot_replacement(i).has_yellow_team() )
                    {
                        if( packet.replacement().robot_replacement(i).yellow_team() ) team = 1;
                    }
                    if( !packet.replacement().robot_replacement(i).has_id() ) continue;
                    int k = packet.replacement().robot_replacement(i).id();
                    dReal x = 0, y = 0, dir = 0, vx = 0, vy = 0, vw = 0;
                    if( packet.replacement().robot_replacement(i).has_x() ) x = packet.replacement().robot_replacement(i).x();
                    if( packet.replacement().robot_replacement(i).has_y() ) y = packet.replacement().robot_replacement(i).y();
                    if( packet.replacement().robot_replacement(i).has_orientation() ) dir = packet.replacement().robot_replacement(i).orientation();
                    if( packet.replacement().robot_replacement(i).has_vx() ) vx = packet.replacement().robot_replacement(i).vx();
                    if( packet.replacement().robot_replacement(i).has_vy() ) vy = packet.replacement().robot_replacement(i).vy();
                    if( packet.replacement().robot_replacement(i).has_vw() ) vw = packet.replacement().robot_replacement(i).vw();
                    int id = robotIndex( k, team );
                    if ( ( id < 0 ) or ( id >= Robots_Count * 2 ) ) continue;
                    robots[id]->setXY( x, y );
                    robots[id]->resetRobot();
                    robots[id]->setDir( dir );
                    robots[id]->on = true;
                    robots[id]->setSpeed( vx, vy, vw );
                }
            }

            if( packet.robot_command_size() > 0 )
            {
                for( int i = 0; i < packet.robot_command_size(); i++ )
                {
                    int team = 0;
                    if( packet.robot_command(i).has_yellow_team() )
                    {
                        if( packet.robot_command(i).yellow_team() ) team = 1;
                    }
                    if( !packet.robot_command(i).has_id() ) continue;
                    int k = packet.robot_command(i).id();
                    dReal spin = 0, vx = 0, vy = 0, vw = 0;
                    if( packet.robot_command(i).has_vx() ) vx = packet.robot_command(i).vx();
                    if( packet.robot_command(i).has_vy() ) vy = packet.robot_command(i).vy();
                    if( packet.robot_command(i).has_vw() ) vw = packet.robot_command(i).vw();
                    int id = robotIndex( k, team );
                    if ( ( id < 0 ) or ( id >= Robots_Count * 2 ) ) continue;
                    robots[id]->setSpeed( vx, vy, vw );

                    dReal kickx = 0 , kickz = 0;
                    bool kick = false;
                    if( packet.robot_command(i).has_kickspeedx() )
                    {
                        kickx = packet.robot_command(i).kickspeedx();
                        kick = true;
                    }
                    if( packet.robot_command(i).has_kickspeedz() )
                    {
                        kickz = packet.robot_command(i).kickspeedz();
                        kick = true;
                    }
                    if ( kick and ( ( kickx > 0.0001 ) or ( kickz > 0.0001 ) ) )
                        robots[id]->kicker->kick( kickx, kickz );
                    
                    int rolling = 0;
                    if( packet.robot_command(i).has_spinner() )
                    { 
                        if( packet.robot_command(i).spinner() ) rolling = 1;
                    }
                    robots[id]->kicker->setRoller( rolling );
                }
            }
            this->step(worldSetting_DeltaTime);
        }
    }
}

dReal normalizeAngle( dReal a )
{
    if ( a > 180 ) return -360 + a;
    if ( a < -180 ) return 360 + a;
    return a;
}

void RobotsFomation::setAll( dReal* xx, dReal *yy )
{
    for ( int i = 0; i < MAX_ROBOT_COUNT; i++ )
    {
        x[i] = xx[i];
        y[i] = yy[i];
    }
}

RobotsFomation::RobotsFomation( int type )
{
    if ( type == 0 )
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = { 2.20,  1.00,  1.00,  1.00,  0.33,  1.22,
                                            3.00,  3.20,  3.40,  3.60,  3.80,  4.00,
                                            0.40,  0.80,  1.20,  1.60};
        dReal teamPosY[MAX_ROBOT_COUNT] = { 0.00, -0.75,  0.00,  0.75,  0.25,  0.00,
                                            1.00,  1.00,  1.00,  1.00,  1.00,  1.00,
                                           -3.50, -3.50, -3.50, -3.50};
        setAll(teamPosX,teamPosY);
    }
    if (type==1) // formation 1
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = { 1.50,  1.50,  1.50,  0.55,  2.50,  3.60,
                                            3.20,  3.20,  3.20,  3.20,  3.20,  3.20,
                                            0.40,  0.80,  1.20,  1.60};
        dReal teamPosY[MAX_ROBOT_COUNT] = { 1.12,  0.0,  -1.12,  0.00,  0.00,  0.00,
                                            0.75, -0.75,  1.50, -1.50,  2.25, -2.25,
                                           -3.50, -3.50, -3.50, -3.50};
        setAll(teamPosX,teamPosY);
    }
}

void RobotsFomation::resetRobots( Robot** r, int team )
{
    dReal dir = -1;
    if ( team == 1 ) dir = 1;
    for ( int k = 0; k < Robots_Count; k++ )
    {
        r[k + team*Robots_Count]->setXY( x[k] * dir, y[k] );
        r[k + team*Robots_Count]->resetRobot();
    }
}