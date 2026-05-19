///////////////////////////////////////////////////////////////////////////
//
// File: model_forklift.cc
// Authors: Hugo Costelha <hugo.costelha@ipleiria.pt>, based don the
//   Model_gripper Richard Vaughn and Doug Blank
// Date: 06 May 2026
//
//  $Revision$
//
///////////////////////////////////////////////////////////////////////////

/**
   @ingroup model
   @defgroup model_forklift Forklift model

   The forkligt model simulates a simple two-fingered forkliftr with two
   internal break-beams, similar to the the Pioneer gripper, but just with
   vertical motion linear joint.

   <h2>Worldfile properties</h2>

   @par Summary and default values

   @verbatim
   forklift
   (
   # forklift properties
   paddle_size [ 0.66 0.1 0.4 ]
   paddle_state [ "down" ]

   # model properties
   size [ 0.2 0.3 0.2 ]
   )
   @endverbatim

   @par Notes

   @par Details

   - paddle_size [ <float x> <float y < float z> ]\n
   Forklift paddle size as a proportion of the model's body size (0.0 to 1.0)
   - paddle_state [ <string open/close> <string up/down> ]\n
   Forklift lift state, either "up" or "down"
*/

#include "stage.hh"
#include "worldfile.hh"
#include <sys/time.h>
using namespace Stg;

#include "option.hh"
Option ModelForklift::showData("Forklift data", "show_forklift_data", "", true, NULL);

// TODO - simulate energy use when moving the forklifts

ModelForklift::ModelForklift(World *world, Model *parent, const std::string &type)
    : Model(world, parent, type), cfg(), // configured below
      cmd(CMD_NOOP)
{
  // set up a forklift-specific default config structure
  // (these can be overridden by worldfiles)
  cfg.paddle_size.x = 0.20; // Paddles length [m]
  cfg.paddle_size.y = 0.04; // Paddles width [m]
  cfg.paddle_size.z = 0.02; // Paddles height [m]

  cfg.lift = LIFT_DOWN;
  lift_position = 0.0;
  gripped = NULL;
  left_beam = NULL; // Beam sensor not implementeed yet, so set to null
  right_beam = NULL; // Beam sensor not implementeed yet, so set to null

  // Set to 0 initially.
  part_zoffset = 0.0;
  SetColor(Color(0.3, 0.3, 0.3, 1.0));

  // Update() is not reentrant
  thread_safe = false;

  // set default size
  SetGeom(Geom(Pose(0, 0, 0, 0), Size(0.2, 0.3, 0.2)));

  RegisterOption(&showData);
}

ModelForklift::~ModelForklift()
{ /* do nothing */
}

void ModelForklift::Load()
{
  Size tmp_size;
  wf->ReadTuple(wf_entity, "size", 0, 3, "lll", &tmp_size.x, &tmp_size.y, &tmp_size.z);
  SetGeom(Geom(Pose(0, 0, 0, 0), tmp_size));

  wf->ReadTuple(wf_entity, "paddle_size", 0, 3, "lll", &cfg.paddle_size.x, &cfg.paddle_size.y,
                &cfg.paddle_size.z);

  char *lift = NULL;
  wf->ReadTuple(wf_entity, "paddle_state", 0, 1, "s", &lift);

  if (lift && strcmp(lift, "up") == 0) {
    lift_position = 1.0;
    cfg.lift = LIFT_UP;
  } else if (lift && strcmp(lift, "down") == 0) {
    lift_position = 0.0;
    cfg.lift = LIFT_DOWN;
  }

  // Redraw paddles
  FixBlocks();
  // Position the forklift correctly
  PositionForklift();

  // do this at the end to ensure that the blocks are resize correctly
  Model::Load();

  // Store initial local pose
  initial_pose = this->GetPose();

  // Store beam location in local coordinates for raytracing
  // @TODO: check why this 0.12 offset to ensure the mean is placed correctly.
  left_beam_pose.x = tmp_size.x - cfg.paddle_size.x - 0.12;
  right_beam_pose.x = tmp_size.x - cfg.paddle_size.x - 0.12;
  left_beam_pose.y = tmp_size.y / 2.0 - cfg.paddle_size.y / 2.0;
  right_beam_pose.y = - tmp_size.y / 2.0 + cfg.paddle_size.y / 2.0;
  left_beam_pose.z = cfg.paddle_size.z;
  right_beam_pose.z = cfg.paddle_size.z;
  left_beam_pose.a = 0.0;
  right_beam_pose.a = 0.0;
  beam_range = cfg.paddle_size.x;
}

void ModelForklift::Save()
{
  Model::Save();

  wf->WriteTuple(wf_entity, "paddle_size", 0, 3, "lll", cfg.paddle_size.x, cfg.paddle_size.y,
                 cfg.paddle_size.z);

  wf->WriteTuple(wf_entity, "paddle_state", 0, 2, "ss",
                 (cfg.lift == LIFT_UP) ? "up" : "down");
}

void ModelForklift::FixBlocks()
{
  // get rid of the default cube
  ClearBlocks();

  // add three blocks that make the forklift
  // base
  AddBlockRect(0, 0, 1.0 - cfg.paddle_size.x/geom.size.x, 1.0, 1.0);
  // left paddle
  AddBlockRect(1.0 - cfg.paddle_size.x/geom.size.x, 0.0 ,
               cfg.paddle_size.x/geom.size.x, cfg.paddle_size.y/geom.size.y,
               cfg.paddle_size.z/geom.size.z);
  // right paddle
  AddBlockRect(1.0 - cfg.paddle_size.x/geom.size.x, 1.0 - cfg.paddle_size.y/geom.size.y,
               cfg.paddle_size.x/geom.size.x, cfg.paddle_size.y/geom.size.y,
               cfg.paddle_size.z/geom.size.z);
  
  // forklift base
  paddle_base = &blockgroup.GetBlockMutable(0);
  // left (top) paddle
  paddle_left = &blockgroup.GetBlockMutable(1);
  // right (bottom) paddle
  paddle_right = &blockgroup.GetBlockMutable(2);
}

// Update the blocks that are the forklift's body
void ModelForklift::PositionForklift()
{
  Pose curr_pose = initial_pose;
  curr_pose.z = initial_pose.z + lift_position * geom.size.z;
  this->SetPose(curr_pose);
}

static bool forklift_raytrace_match(Model *hit, const Model *finder, const void *dummy)
{
  (void)dummy; // avoid warning about unused var

  return ((hit != finder) && hit->vis.forklift_return);
  // can't use the normal relation check, because we may pick things
  // up and we must still see them.
}

void ModelForklift::Update()
{
  float start_lift_position = lift_position;

  switch (cmd) {
  case CMD_NOOP: break;

  case CMD_UP:
    if (cfg.lift != LIFT_UP) {
      //puts( "raising forklift" );
      cfg.lift = LIFT_UPPING;
    }
    break;

  case CMD_DOWN:
    if (cfg.lift != LIFT_DOWN) {
      //puts( "lowering forklift" );
      cfg.lift = LIFT_DOWNING;
    }
    break;

  default: printf("unknown forklift command %d\n", cmd);
  }

  switch (cfg.lift) {
  case LIFT_DOWNING:
    lift_position -= 0.05;

    if (lift_position < 0.0) // if we're fully down
    {
      lift_position = 0.0;
      cfg.lift = LIFT_DOWN; // change state
    }
    break;

  case LIFT_UPPING:
    lift_position += 0.05;

    if (lift_position > 1.0) // if we're fully open
    {
      lift_position = 1.0;
      cfg.lift = LIFT_UP; // change state
    }
    break;

  case LIFT_DOWN: // nothing to do for these cases
  case LIFT_UP:
  default: break;
  }

  // if the paddles or lift have changed position
  if (start_lift_position != lift_position)
  {
    // figure out where the paddles should be
    PositionForklift();
    // check for contacts with the break beam
    UpdateBreakBeamContactsPart();
  } else // @TODO: Remove this part
  {
      // store the model (possibly NULL) hit by the breakbeam
    left_beam = Raytrace(left_beam_pose, beam_range, forklift_raytrace_match,
                        NULL, true).mod;
    right_beam = Raytrace(right_beam_pose, beam_range, forklift_raytrace_match,
                        NULL, true).mod;
  }


  Model::Update();
}

void ModelForklift::UpdateBreakBeamContactsPart()
{
  if (gripped != NULL) { // If we alread have a gripped part
    // Update the part z position
    Pose world_pose = gripped->GetGlobalPose();

    bool drop_part = false;
    // if we're already carrying something, make sure it moves with the forklift
    //double zpos = cfg.lift_position * (1.0 - cfg.paddle_size.z) * geom.size.z;
    if (world_pose.z - part_zoffset <= 0.0) { // if the gripped object is below the top of the forklift
      if (cfg.lift == LIFT_DOWNING)
        drop_part = true;
      else
        printf("Warning: gripped object global z coordinate has an unexpected value: %.2f.\n",
          gripped->GetGlobalPose().z);      
    }

     if (drop_part) {
        // drop the gripped object
        gripped->SetParent(NULL);
        Pose world_pose = gripped->GetGlobalPose();
        world_pose.z = part_zoffset;
        gripped->SetGlobalPose(world_pose);
        gripped = NULL;
     }
  } else // We are not yet holding any part
  {
    // store the model (possibly NULL) hit by the breakbeam
    left_beam = Raytrace(left_beam_pose, beam_range, forklift_raytrace_match,
                        NULL, true).mod;
    right_beam = Raytrace(right_beam_pose, beam_range, forklift_raytrace_match,
                        NULL, true).mod;

    if (left_beam && right_beam && (cfg.lift == LIFT_UPPING)) {
      // Get the object and parents' pose
      Pose pose_world = left_beam->GetGlobalPose();
      // Attach it to our forklift
      left_beam->SetParent(this);
      // Make sure the part moves correctly with the forklift
      Pose local_pose = left_beam->GetPose();
      part_zoffset = left_beam->GetGlobalPose().z - pose_world.z;
      local_pose.z -= part_zoffset;
      left_beam->SetPose(local_pose);
      part_zoffset = pose_world.z; // For later part placement
      gripped = left_beam;
    }
  }
}

void ModelForklift::DataVisualize(Camera *cam)
{
  (void)cam; // avoid warning about unused var

  // only draw if someone is using the forklift
  if (subs < 1)
    return;

  // if( ! showData )
  // return;

  // outline the sensor lights in black
  PushColor(0, 0, 0, 1.0); // black
  glTranslatef(0, 0, left_beam_pose.z); // move up to the break beam height
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // size of the paddle indicator lights
  double led_dx = cfg.paddle_size.x;
  double led_dy = cfg.paddle_size.y;

  // if the forklift detects anything, fill the lights in with yellow
  if (left_beam && right_beam) {
    PushColor(1, 1, 0, 1.0); // yellow
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  // paddle break beams
  Gl::draw_centered_rect(right_beam_pose.x+led_dx/2.0, right_beam_pose.y , led_dx, led_dy);
  Gl::draw_centered_rect(left_beam_pose.x+led_dx/2.0, left_beam_pose.y , led_dx, led_dy);

  if (left_beam && right_beam)
    PopColor(); // yellow

  PopColor(); // black
}
