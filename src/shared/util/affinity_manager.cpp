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
  \file    affinity_manager.cpp
  \brief   C++ Implementation: affinity_manager
  \author  Stefan Zickler, 2009
*/
//========================================================================
#include "affinity_manager.h"

AffinityManager::AffinityManager()
{
  _mutex=new pthread_mutex_t;
  pthread_mutex_init((pthread_mutex_t*)_mutex, NULL);
  max_cpu_id=0;
  parseCpuInfo();
}

AffinityManager::~AffinityManager()
{
  pthread_mutex_destroy((pthread_mutex_t*)_mutex);
  delete _mutex;
}

void AffinityManager::demandCore(int core) {
}

int AffinityManager::parseFileUpTo(FILE * f, char * output, int len, char end) {
  return 0;
}

void AffinityManager::parseCpuInfo() {
}
