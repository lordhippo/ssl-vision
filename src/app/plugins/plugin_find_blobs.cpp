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
  \file    plugin_find_blobs.cpp
  \brief   C++ Implementation: plugin_find_blobs
  \author  Stefan Zickler, 2008
*/
//========================================================================
#include "plugin_find_blobs.h"

PluginFindBlobs::PluginFindBlobs(FrameBuffer * _buffer, YUVLUT * _lut,const CameraParameters& camera_params, const RoboCupField& _field, int _max_regions)
 : VisionPlugin(_buffer),camera_parameters ( camera_params ),field(_field)
{
  lut=_lut;
  max_regions=_max_regions;

  _settings=new VarList("Blob Finding");
  _settings->addChild(_v_min_blob_area=new VarInt("min_blob_area", 5));
  _settings->addChild(_v_enable=new VarBool("enable", true));
  _settings->addChild(_v_send_net=new VarBool("Send Over Network", false));
  _settings->addChild(_v_net_ip=new VarString("Network IP", "224.5.33.35"));
  _settings->addChild(_v_net_port=new VarInt("Network Port", 60022));
  _settings->addChild(_v_obstacle_height=new VarDouble("Obstacle Height (mm)", 100.0));
  _settings->addChild(_filter = new VarList("Filters"));
    _filter->addChild(_blob_on_field_filter = new VarBool("In-Field Filter",true));
    _filter->addChild(_blob_on_field_filter_threshold = new VarDouble("In-Field Extra Space (mm)",30.0));
    _filter->addChild(_v_max_aspect=new VarDouble("Max Aspect", 3.0));
    _filter->addChild(_v_min_w=new VarInt("Min Width (pixel)", 2));
    _filter->addChild(_v_min_h=new VarInt("Min Height (pixel)", 2));
    _filter->addChild(_blob_on_field_filter_visualize = new VarBool("In-Field Filter Display",false));


  field_filter.update ( field );

  mc_open();
}


PluginFindBlobs::~PluginFindBlobs()
{
}


void PluginFindBlobs::mc_close() {
  mc.close();
}

bool PluginFindBlobs::mc_open() {
  mc_close();

  if(!mc.open(_v_net_port->getInt(),true,true)) {
    fprintf(stderr,"Unable to open UDP network port: %d\n",_v_net_port->getInt());
    fflush(stderr);
    return(false);
  }

  Net::Address multiaddr,interface;
  multiaddr.setHost(_v_net_ip->getString().c_str(),_v_net_port->getInt());
  interface.setAny();

  if(!mc.addMulticast(multiaddr,interface)) {
    fprintf(stderr,"Unable to setup UDP multicast\n");
    fflush(stderr);
    return(false);
  }

  return(true);
}

bool PluginFindBlobs::mc_send() {
  string buffer;
  lframe.SerializeToString(&buffer);
  Net::Address multiaddr;
  multiaddr.setHost(_v_net_ip->getString().c_str(),_v_net_port->getInt());
  bool result;
  result=mc.send(buffer.c_str(),buffer.length(),multiaddr);
  if (result==false) {
    fprintf(stderr,"Sending UDP datagram failed (maybe too large?). Size was: %zu byte(s)\n",buffer.length());
  }
  return(result);
}



ProcessResult PluginFindBlobs::process(FrameData * data, RenderOptions * options) {
  (void)options;


  CMVision::RegionList * reglist;
  if ((reglist=(CMVision::RegionList *)data->map.get("cmv_reglist")) == 0) {
    reglist=(CMVision::RegionList *)data->map.insert("cmv_reglist",new CMVision::RegionList(max_regions));
  }

  CMVision::ColorRegionList * colorlist;
  if ((colorlist=(CMVision::ColorRegionList *)data->map.get("cmv_colorlist")) == 0) {
    colorlist=(CMVision::ColorRegionList *)data->map.insert("cmv_colorlist",new CMVision::ColorRegionList(lut->getChannelCount()));
  }

  CMVision::RunList * runlist;
  if ((runlist=(CMVision::RunList *)data->map.get("cmv_runlist")) == 0) {
    printf("Blob finder: no runlength-encoded input list was found!\n");
    return ProcessingFailed;
  }

  if (_v_enable->getBool()==true) {
    //Connect the components of the runlength map:
    CMVision::RegionProcessing::connectComponents(runlist);
  
    //Extract Regions from runlength map:
    CMVision::RegionProcessing::extractRegions(reglist, runlist);
  
    if (reglist->getUsedRegions() == reglist->getMaxRegions()) {
      printf("Warning: extract regions exceeded maximum number of %d regions\n",reglist->getMaxRegions());
    }
  
    //Separate Regions by colors:
    int max_area = CMVision::RegionProcessing::separateRegions(colorlist, reglist, _v_min_blob_area->getInt());
  
    //Sort Regions:
    CMVision::RegionProcessing::sortRegions(colorlist,max_area);
  } else {
    //detect nothing.
    reglist->setUsedRegions(0);
    int num_colors=colorlist->getNumColorRegions();
    CMVision::RegionLinkedList * color=colorlist->getColorRegionArrayPointer();
  
    // clear out the region list head table
    for(int i=0; i<num_colors; i++){
      color[i].reset();
    }
  }

  if ( _v_send_net->getBool() )
  {
    field_filter.update ( field );
    lframe.Clear();
    lframe.set_frame_number(data->number);
    lframe.set_camera_id(data->cam_id);
    setFrame(colorlist);
    mc_send();
  }

  return ProcessingOk;

}

void DeleteBlob ( CMVision::Region* rgn )
{
    rgn->x1 = rgn->x2 = rgn->cen_x;
    rgn->y1 = rgn->y2 = rgn->cen_y;
    rgn->area=0;
}

void PluginFindBlobs::setFrame ( CMVision::ColorRegionList * colorlist )
{
    for ( int i = 0 ; i < colorlist->getNumColorRegions() ; i++ )
    {
        int blobCount = 0;
        CMVision::RegionLinkedList * newRegList = colorlist->getColorRegionArrayPointer() + i;
        for ( CMVision::Region* rgn = newRegList->getInitialElement() ; rgn != NULL ; rgn =rgn->next )
        {
            if ( rgn->width() < _v_min_w->getInt() || rgn->height() < _v_min_h->getInt() )
            {
                if ( _blob_on_field_filter_visualize -> getBool() )
                    DeleteBlob ( rgn );
                continue;
            }
            if ( rgn->width() / rgn->height() > _v_max_aspect->getDouble() )
            {
                if ( _blob_on_field_filter_visualize -> getBool() )
                    DeleteBlob ( rgn );
                continue;
            }
            if ( rgn->height() / rgn->width() > _v_max_aspect->getDouble() )
            {
                if ( _blob_on_field_filter_visualize -> getBool() )
                    DeleteBlob ( rgn );
                continue;
            }

            {
                //convert from image to field coordinates:
                vector2d pixel_pos ( rgn->cen_x,rgn->cen_y );
                vector3d field_pos_3d;
                camera_parameters.image2field ( field_pos_3d,pixel_pos,_v_obstacle_height->getDouble() );
                vector2d field_pos ( field_pos_3d.x,field_pos_3d.y );
                //filter points that are outside of the field:
                if ( _blob_on_field_filter->getBool()==true &&
                     ( ( field_filter.isInFieldPlusThreshold ( field_pos, max(0.0,_blob_on_field_filter_threshold->getDouble()) ) ==false ) || ( field_pos.x < -max(0.0,_blob_on_field_filter_threshold->getDouble() ) ) ) )
                {
                    if ( _blob_on_field_filter_visualize -> getBool() )
                        DeleteBlob ( rgn );
                    continue;
                }
            }

            blobCount ++;
            LHP_Blob* newBlob = lframe.add_blob();
            //newBlob->set_color(rgn->color);
            newBlob->set_color(i);
            {
                //convert from image to field coordinates:
                vector2d pixel_pos ( rgn->cen_x,rgn->cen_y );
                vector3d field_pos_3d;
                camera_parameters.image2field ( field_pos_3d,pixel_pos,_v_obstacle_height->getDouble() );
                newBlob->set_cen_x(field_pos_3d.x);
                newBlob->set_cen_y(field_pos_3d.y);
            }
            {
                //convert from image to field coordinates:
                vector2d pixel_pos ( rgn->x1,rgn->y1 );
                vector3d field_pos_3d;
                camera_parameters.image2field ( field_pos_3d,pixel_pos,_v_obstacle_height->getDouble() );
                newBlob->set_x1(field_pos_3d.x);
                newBlob->set_y1(field_pos_3d.y);
            }
            {
                //convert from image to field coordinates:
                vector2d pixel_pos ( rgn->x2,rgn->y2 );
                vector3d field_pos_3d;
                camera_parameters.image2field ( field_pos_3d,pixel_pos,_v_obstacle_height->getDouble() );
                newBlob->set_x2(field_pos_3d.x);
                newBlob->set_y2(field_pos_3d.y);
            }
            newBlob->set_bb_area(fabs((rgn->x2-rgn->x1)*(rgn->y2-rgn->y1)));
            newBlob->set_area(rgn->area);
            //if ( blobCount > 20 )
            //    break;
        }
    }
}

VarList * PluginFindBlobs::getSettings() {
  return _settings;
}

string PluginFindBlobs::getName() {
  return "FindBlobs";
}
