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

#ifndef SSLWORLD_H
#define SSLWORLD_H

#include "physics/pworld.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "physics/pfixedbox.h"
#include "physics/pray.h"

#include "observation.pb.h"
#include "commands.pb.h"

#include <QtNetwork>
#include <QObject>

#include "robot.h"

#define WALL_COUNT 10

class RobotsFomation;

class SSLWorld : public QObject
{
    Q_OBJECT
private:
    int framenum;
    dReal last_dt;
    bool lastInfraredState[TEAM_COUNT][MAX_ROBOT_COUNT];
    KickStatus lastKickState[TEAM_COUNT][MAX_ROBOT_COUNT];    
public:    
    dReal customDT;
    SSLWorld( QObject *parent = 0 );

    virtual ~SSLWorld();
    void step( dReal dt = -1 );
    int  robotIndex( int robot, int team );

    void sendPacket();

    PWorld* p;
    PBall* ball;
    PGround* ground;
    PRay* ray;
    QUdpSocket* observationSocket;
    QUdpSocket* commandsSocket;
    PFixedBox* walls[WALL_COUNT];
    int selected;
    Robot* robots[MAX_ROBOT_COUNT];
    char *in_buffer;

public slots:
    void receivePacket();
};

class RobotsFomation {
    public:
        dReal x[MAX_ROBOT_COUNT];
        dReal y[MAX_ROBOT_COUNT];
        RobotsFomation( int type );
        void setAll( dReal *xx, dReal *yy );
        void resetRobots( Robot** r, int team );
};

dReal fric( dReal f );
int robotIndex( int robot, int team );

#endif // SSLWORLD_H