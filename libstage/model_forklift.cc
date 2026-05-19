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
  cfg.lift_position = 0.0;
  cfg.gripped = NULL;
  cfg.beam = 0; // Beam sensor not implementeed yet, so set to null

  // Set to 0 initially.
  part_zoffset = 0.0;

  SetColor(Color(0.3, 0.3, 0.3, 1.0));

  //FixBlocks();

  // Update() is not reentrant
  thread_safe = false;

  // set default size
  SetGeom(Geom(Pose(0, 0, 0, 0), Size(0.2, 0.3, 0.2)));

  //PositionPaddles();

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
    cfg.lift_position = 1.0;
    cfg.lift = LIFT_UP;
  } else if (lift && strcmp(lift, "down") == 0) {
    cfg.lift_position = 0.0;
    cfg.lift = LIFT_DOWN;
  }

  FixBlocks();

  PositionForklift();

  // do this at the end to ensure that the blocks are resize correctly
  Model::Load();
  // printf("Forklift loaded with paddle size %.2f, %.2f, %.2f, state %s\n", cfg.paddle_size.x,
  //        cfg.paddle_size.y, cfg.paddle_size.z,
  //        (cfg.lift == LIFT_UP) ? "up" : "down");

  // Store initial local pose
  initial_pose = this->GetPose();

  // Store beam location in local coordinates for raytracing
  beam_pose.x = 0.5 * cfg.paddle_size.x;
  beam_pose.y = tmp_size.y / 2.0 - cfg.paddle_size.y;
  beam_pose.z = cfg.paddle_size.z;
  beam_pose.a = -M_PI / 2.0;
  cfg.beam_range = tmp_size.y - 2.0 * cfg.paddle_size.y;

  // Check for contacts
  //UpdateBreakBeamContactsPart();

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
double ModelForklift::PositionForklift()
{
  //unsigned int layer = world->GetUpdateCount() % 2;
  //UnMap(layer);
  Pose curr_pose = initial_pose;
  curr_pose.z = initial_pose.z + cfg.lift_position * geom.size.z;
  this->SetPose(curr_pose);
  // Update Z position of the paddles based on the lift position.
  /*double paddle_bottom = cfg.lift_position;// * geom.size.z;
  double paddle_top = paddle_bottom + cfg.paddle_size.z/geom.size.z;
  paddle_left->SetZ(paddle_bottom, paddle_top);
  paddle_right->SetZ(paddle_bottom, paddle_top);
  printf("lift position %.2f, paddle bottom %.2f, paddle top %.2f\n", cfg.lift_position, paddle_bottom, paddle_top);
  */
  //Map(layer);

  return 0;//paddle_bottom;
}

void ModelForklift::Update()
{
  float start_lift_position = cfg.lift_position;

  switch (cmd) {
  case CMD_NOOP: break;

  case CMD_UP:
    if (cfg.lift != LIFT_UP) {
      puts( "raising forklift" );
      cfg.lift = LIFT_UPPING;
    }
    break;

  case CMD_DOWN:
    if (cfg.lift != LIFT_DOWN) {
      puts( "lowering forklift" );
      cfg.lift = LIFT_DOWNING;
    }
    break;

  default: printf("unknown forklift command %d\n", cmd);
  }

  switch (cfg.lift) {
  case LIFT_DOWNING:
    cfg.lift_position -= 0.05;

    if (cfg.lift_position < 0.0) // if we're fully down
    {
      cfg.lift_position = 0.0;
      cfg.lift = LIFT_DOWN; // change state
    }
    break;

  case LIFT_UPPING:
    cfg.lift_position += 0.05;

    if (cfg.lift_position > 1.0) // if we're fully open
    {
      cfg.lift_position = 1.0;
      cfg.lift = LIFT_UP; // change state
    }
    break;

  case LIFT_DOWN: // nothing to do for these cases
  case LIFT_UP:
  default: break;
  }

  // if the paddles or lift have changed position
  if (start_lift_position != cfg.lift_position)
  {
    // figure out where the paddles should be
    PositionForklift();
    // check for contacts with the break beam
    UpdateBreakBeamContactsPart();
  }

  Model::Update();
}

static bool forklift_raytrace_match(Model *hit, const Model *finder, const void *dummy)
{
  (void)dummy; // avoid warning about unused var

  return ((hit != finder) && hit->vis.forklift_return);
  // can't use the normal relation check, because we may pick things
  // up and we must still see them.
}

void ModelForklift::UpdateBreakBeamContactsPart()
{
  if (cfg.gripped != NULL) { // If we alread have a gripped part
    // Update the part z position
    Pose world_pose = cfg.gripped->GetGlobalPose();
    //local_pose.z = this->geom.size.z; // - world_pose.z; // - this->GetPose().z;
    //cfg.gripped->SetPose(local_pose);

    printf("gripped global z: %.2f, gripped local z: %.2f\nforklift global z: %.2f, forklift local z: %.2f",
      cfg.gripped->GetGlobalPose().z, cfg.beam->GetPose().z, this->GetGlobalPose().z, this->GetPose().z);
    bool drop_part = false;
    // if we're already carrying something, make sure it moves with the forklift
    //double zpos = cfg.lift_position * (1.0 - cfg.paddle_size.z) * geom.size.z;
    if (world_pose.z - part_zoffset <= 0.0) { // if the gripped object is below the top of the forklift
      if (cfg.lift == LIFT_DOWNING)
        drop_part = true;
      else
        printf("Warning: gripped object global z coordinate has an unexpected value: %.2f.\n",
          cfg.gripped->GetGlobalPose().z);      
    }

     if (drop_part) {
        // drop the gripped object
        cfg.gripped->SetParent(NULL);
        Pose world_pose = cfg.gripped->GetGlobalPose();
        world_pose.z = part_zoffset;
        cfg.gripped->SetGlobalPose(world_pose);
        cfg.gripped = NULL;
     }
  } else // We are not yet holding any part
  {
    // store the model (possibly NULL) hit by the breakbeam
    cfg.beam = Raytrace(beam_pose, cfg.beam_range, forklift_raytrace_match,
                        NULL, true).mod;

    if (cfg.beam && (cfg.lift == LIFT_UPPING)) {
      Model *hit = cfg.beam;
      // Get the object and parents' pose
      Pose pose_world = hit->GetGlobalPose();
      pose_world.Print("World pose: ");
      hit->GetPose().Print("Local pose: ");
      this->GetGlobalPose().Print("Parent world pose: ");
      // Attach it to our forklift
      hit->SetParent(this);
      // Make sure the part moves correctly with the forklift
      Pose local_pose = hit->GetPose();
      local_pose.Print("New local pose: ");
      hit->GetGlobalPose().Print("New world pose: ");
      part_zoffset = hit->GetGlobalPose().z - pose_world.z;
      local_pose.z -= part_zoffset;
      hit->SetPose(local_pose);
      hit->GetPose().Print("New local pose after z offset: ");
      hit->GetGlobalPose().Print("New world pose after z offset: ");
      part_zoffset = pose_world.z; // For later part placement
      cfg.gripped = hit;
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
  glTranslatef(0, 0, beam_pose.z); // move up to the break beam height
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // size of the paddle indicator lights
  double led_dx = cfg.paddle_size.y * 0.5;

  // if the forklift detects anything, fill the lights in with yellow
  if (cfg.beam) {
    PushColor(1, 1, 0, 1.0); // yellow
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  // paddle break beams
  Gl::draw_centered_rect(beam_pose.x, beam_pose.y + led_dx, led_dx, led_dx);
  Gl::draw_centered_rect(beam_pose.x, -beam_pose.y - led_dx, led_dx, led_dx);

  if (cfg.beam)
    PopColor(); // yellow

  PopColor(); // black
}
