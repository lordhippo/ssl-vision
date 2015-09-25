//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    robocup_ssl_server.cpp
  \brief   C++ Implementation: robocup_ssl_server
  \author  Stefan Zickler, 2009
*/
//========================================================================
#include "robocup_ssl_server.h"
#include <string.h>

RoboCupSSLServer::RoboCupSSLServer(short port,
                     string net_address,
                     string net_interface)
{
  _port=port;
  _net_address=net_address;
  _net_interface=net_interface;

  socket = nullptr;

}


RoboCupSSLServer::~RoboCupSSLServer()
{
}

void RoboCupSSLServer::close() {
  if (socket != nullptr) {
    delete socket;
    socket = nullptr;
  }
}

bool RoboCupSSLServer::open() {
  close();

  try {
    socket = new UDPSocket();
  }
  catch(...) {
    fprintf(stderr,"Unable to open UDP network port: %d\n",_port);
    fflush(stderr);
    return(false);
  }

  return(true);
}

bool RoboCupSSLServer::send(const SSL_WrapperPacket & packet) {
  string buffer;
  packet.SerializeToString(&buffer);
  mutex.lock();
  try {
    socket -> sendTo ( buffer.c_str() , buffer.length() , _net_address , _port );
  }
  catch (...)
  {
    fprintf(stderr,"Sending UDP datagram failed (maybe too large?). Size was: %zu byte(s)\n",buffer.length());
  }

  mutex.unlock();
  return(true);
}

bool RoboCupSSLServer::send(const SSL_DetectionFrame & frame) {
  SSL_WrapperPacket pkt;
  SSL_DetectionFrame * nframe = pkt.mutable_detection();
  nframe->CopyFrom(frame);
  return send(pkt);
}

bool RoboCupSSLServer::send(const SSL_GeometryData & geometry) {
  SSL_WrapperPacket pkt;
  SSL_GeometryData * gdata = pkt.mutable_geometry();
  gdata->CopyFrom(geometry);
  return send(pkt);
}

bool RoboCupSSLServer::send(const LHP_Frame & frame) {
  SSL_WrapperPacket pkt;
  LHP_Frame * gdata = pkt.mutable_lframe();
  gdata->CopyFrom(frame);
  return send(pkt);
}

