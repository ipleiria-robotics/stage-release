/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2004, 2005 Richard Vaughan
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Desc: A plugin driver for Player that gives access to Stage devices.
 * Author: Richard Vaughan
 * Date: 10 December 2004
 * CVS: $Id$
 */

#include "p_driver.h"

//#include "playerclient.h" // for the dumb pioneer forklift command defines

/** @addtogroup player
@par Forklift interface
- PLAYER_FORKLIFT_DATA_STATE
- PLAYER_FORKLIFT_CMD_STATE
- PLAYER_FORKLIFT_REQ_GET_GEOM
*/

#include "p_driver.h"
using namespace Stg;

InterfaceForklift::InterfaceForklift(player_devaddr_t addr, StgDriver *driver, ConfigFile *cf,
                                     int section)
    : InterfaceModel(addr, driver, cf, section, "forklift")
{
  // nothing to do
}

void InterfaceForklift::Publish(void)
{
  ModelForklift *fmod = reinterpret_cast<ModelForklift *>(this->mod);

  player_forklift_data_t pdata;
  memset(&pdata, 0, sizeof(pdata));

  // set the proper bits
  pdata.beams = 0;
  pdata.beams |= fmod->GetConfig().beam[0] ? 0x04 : 0x00;
  pdata.beams |= fmod->GetConfig().beam[1] ? 0x08 : 0x00;

  switch (fmod->GetConfig().lift) {
  case ModelForklift::LIFT_UP: pdata.state = PLAYER_FORKLIFT_STATE_UP; break;
  case ModelForklift::LIFT_DOWN: pdata.state = PLAYER_FORKLIFT_STATE_DOWN; break;
  case ModelForklift::DOWNING:
  case ModelForklift::LIFT_UPPING: pdata.state = PLAYER_FORKLIFT_STATE_MOVING; break;
  default: pdata.state = PLAYER_FORKLIFT_STATE_ERROR;
  }

  // Write data
  this->driver->Publish(this->addr, PLAYER_MSGTYPE_DATA, PLAYER_GRIPPER_DATA_STATE, (void *)&pdata);
}

int InterfaceForklift::ProcessMessage(QueuePointer &resp_queue, player_msghdr_t *hdr, void *data)
{
  ModelForklift *fmod = reinterpret_cast<ModelForklift *>(this->mod);

  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_FORKLIFT_CMD_UP, this->addr)) {
    fmod->CommandUp();
  }
    return 0;
  } else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_FORKLIFT_CMD_DOWN, this->addr)) {
    fmod->CommandDown();
    return 0;
  }

  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_FORKLIFT_REQ_GET_GEOM, this->addr)) {
    Geom geom = this->mod->GetGeom();
    Pose pose = this->mod->GetPose();

    player_gripper_geom_t pgeom;
    memset(&pgeom, 0, sizeof(pgeom));

    pgeom.pose.px = pose.x;
    pgeom.pose.py = pose.y;
    pgeom.pose.pz = pose.z;
    pgeom.pose.pyaw = pose.a;

    pgeom.outer_size.sl = geom.size.x;
    pgeom.outer_size.sw = geom.size.y;
    pgeom.outer_size.sh = geom.size.z;

    pgeom.num_beams = 2;

    this->driver->Publish(this->addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK,
                          PLAYER_FORKLIFT_REQ_GET_GEOM, (void *)&pgeom);
    return (0);
  }

  PRINT_WARN2("stage fork_lift doesn't support message id:%d/%d", hdr->type, hdr->subtype);
  return -1;
}
