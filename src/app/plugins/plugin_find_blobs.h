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
  \file    plugin_find_blobs.h
  \brief   C++ Interface: plugin_find_blobs
  \author  Author Name, 2008
*/
//========================================================================
#ifndef PLUGIN_FIND_BLOBS_H
#define PLUGIN_FIND_BLOBS_H

#include <visionplugin.h>
#include "lut3d.h"
#include "cmvision_region.h"
#include "camera_calibration.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "netraw.h"
#include "field_filter.h"

/**
	@author Stefan Zickler
*/
class PluginFindBlobs : public VisionPlugin
{
protected:
  YUVLUT * lut;
  int max_regions;

  VarList * _settings;
      VarInt * _v_min_blob_area;
      VarBool * _v_enable;
      VarBool * _v_send_net;
      VarString * _v_net_ip;
      VarInt * _v_net_port;
      VarDouble * _v_obstacle_height;
      VarList   * _filter;
        VarDouble * _v_max_aspect;
        VarInt * _v_min_w;
        VarInt * _v_min_h;
        VarBool   * _blob_on_field_filter;
        VarDouble * _blob_on_field_filter_threshold;
        VarBool   * _blob_on_field_filter_visualize;

      const CameraParameters& camera_parameters;
      const RoboCupField& field;

      FieldFilter field_filter;

      LHP_Frame lframe;
      Net::UDP mc; // multicast server
      bool mc_open();
      void mc_close();
      bool mc_send();

      void setFrame ( CMVision::ColorRegionList * colorlist );

    public:
        PluginFindBlobs(FrameBuffer * _buffer, YUVLUT * _lut,const CameraParameters& camera_params, const RoboCupField& _field, int _max_regions);

        ~PluginFindBlobs();

        virtual ProcessResult process(FrameData * data, RenderOptions * options);

        virtual VarList * getSettings();

        virtual string getName();
    };

    #endif
