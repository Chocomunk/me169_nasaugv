/*
**   depthtolaserscan.cpp
**
**   Convert an depth image into a laser scan.  This assumes the camera
**   is mounted horizontally!
**
**   Node:       /depthtolaserscan
**
**   Params:     ~ground_frame_id    Frame at ground level, with Z vertical up
**               ~laser_frame_id     Frame for laser scan
**                                   (x forward, Z vertical up, Y left)
**               ~min_height         Min height for object to "be detected"
**               ~max_height         Max height for object to "be detected"
**               ~horz_samples       Number of laser scan samples
**               ~vert_samples       Number of vert samples per laser sample
**               ~correction         Quadratic depth correction factor
**               ~minimum_contacts   Number of contacts/object to be valid
**               ~similar_fraction   Percentage +/- of range to be same object
**
**   Subscribe:  /camera/depth/...
**                 .../camera_info       sensor_msgs/CameraInfo
**                 .../image_rect_raw    sensor_msgs/Image
**
**   Publish:    /scan                   sensor_msgs/LaserScan
**
**   TF Required:  ground_frame -> camera_frame
**
**   TF Provided:  camera_frame -> laser_frame (static)
*/
#include <math.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

using namespace ros;
using namespace geometry_msgs;
using namespace sensor_msgs;


/*
**   Constants
*/
#define TIMEOUT     (60.0)	// Timeout for the initialization

#define INFO_TOPIC  "/camera/depth/camera_info"
#define IMAGE_TOPIC "/camera/depth/image_rect_raw"
#define SCAN_TOPIC  "/scan"

// Reporting a distance below the minimum implies the scanner was not
// able to get any information, equivalent to an error.  Reporting a
// distance above the maximum implies there are no contacts (between
// the minimum to maximum distance), everything is clear.
#define RANGE_BAD        (0.000)	// Range considered zero/unknown
#define RANGE_MINIMUM    (0.050)	// Minimum valid contact distance
#define RANGE_MAXIMUM    (4.000)	// Maximum valid contact distance
#define RANGE_NOCONTACT  (5.000)	// Valid range is clear

// Default Parameter Values
#define MINIMUM_CONTACTS (5.0)		// Minimum number of contacts/object
#define SIMILAR_FRACTION (0.030)	// Range within 3% is the same object


/*
**   Global Variables
*/
// Frames
std::string groundframe;	// At ground level, Z vertical up
std::string imageframe;		// At camera, X image right, Z image out
std::string laserframe;		// At camera, X horiz forward, Z vert up

// Image parameters.
unsigned int Nu, Nv;		// Width/Height (number of pixels)

double cu, cv;			// Image Center (in pixels)
double fu, fv;			// Focal length (in pixels)

// Pose parameters.
double pitch;			// Camera pitched up angle
double spitch, cpitch;
double hcam;			// Height of camera
double hmin;			// Min height to consider contacts
double hmax;			// Max height to consider contacts

// Sampling info.
int horzsamples;		// Number of angles (-1 = default)
int vertsamples;		// Number of lines (-1 = default)

double correction;		// Depth quadratic correction factor

double min_contacts;		// Min number of contacts/object
double sim_fraction;		// Similarity percentage (fraction)
double sim_below;	        // Similar lower range = 1/(1+fraction)
double sim_above;		// Similar upper range = (1+fraction)

unsigned int Nangles;		// Number of angles to sample
unsigned int Nlines;		// Number of image lines to sample per angle
int          dlines;		// Step size in lines between samples

double angle_min;		// Scan minimum angle
double angle_max;		// Scan maximum angle
double angle_inc;		// Scan increment between sample angles

// Sampling specification data (pre-computed).
typedef struct _pointspec	// Specifications of a point sample
{
  int          index;		// Index to flattened depth data[row][col]
  uint16_t     depthfreespace;	// Depth (mm) showing freespace
  uint16_t     depthoutofband;	// Depth (mm) showing out of vertical band
  double       depthtorange;	// Depth to range factor mm to m!
} pointspec_t;

typedef struct _anglespec	// Specifications of an angle sample
{
  double       angle;		// Angle of this sample
  int          points;		// Number of points (for this angle)
  pointspec_t  *pointspecs;	// Points to sample
} anglespec_t;

anglespec_t    *anglespecs;	// Pre-computed sample info

// Scan message and publisher
LaserScan scanmsg;
Publisher scanpub;



/*********************************************************************
**		       Mathematical Definitions			    **
**********************************************************************
**
**   Before we derive the algorithm, we define two coordinate frames
**   in the same location, but with different axes.  The Image frame
**   is horizontal, but pitched up.  The Laser frame is aligned with
**   the ground (Z vertical).
**
**     Image Frame:  X image right horizontal, Y image down, Z into image
**     Laser Frame:  X forward horizontal, Y left horizontal, Z vertical up
**
**   A point in space can be decribed in image frame (xi, yi, zi) or
**   laser frame (xl, yl, zl) and converted with:
**
**     xl = cos(pitch) zi + sin(pitch) yi  = cp * zi + sp * yi
**     yl = - xi                           = -xi
**     zl = sin(pitch) zi - cos(pitch) yi  = sp * zi - cp * yi
**
**     xi = yl                             = yl
**     yi = sin(pitch) xl - cos(pitch) zl  = sp * xl - cp * zl
**     zi = cos(pitch) xl + sin(pitch) zl  = cp * xl + sp * zl
**
**   The point (given in image frame) can also be mapped to the image
**   coordinates (u,v) and depth (d):
**
**     u = cu + fu * (xi/zi)         ubar = xi/zi
**     v = cv + fv * (yi/zi)         vbar = yi/zi
**     d = zi
**
**   where cu/cv are the center of the image, and fu/fv are the focal
**   lengths expressed in pixel size.  This again can be inverted as:
**
**     xi = d * (u-cu)/fu  = d * ubar
**     yi = d * (v-cv)/fv  = d * vbar
**     zi = d              = d
**
**   And then the image coordinates can be directly mapped to the
**   laser frame (vertical up):
**
**     xl = d * (cp + sp * vbar)     = r * cos(alpha)
**     yl = d * (    -ubar     )     = r * sin(alpha)
**     zl = d * (sp - cp * vbar)     = r * tan(gamma)
**
**   where  r     = range in horz plane (for laser scan)
**          alpha = angle in horz plane (for laser scan)
**          gamma = angle above horz plane (to be projected down)
**
**    r = sqrt(xl^2 + yl^2) = d * sqrt(ubar^2 + (cp + sp*vbar)^2)
**    tan(alpha) = yl/xl = -ubar/(cp+sp*vbar)
**    tan(gamma) = zl/r = cos(alpha) * (sp - cp*vbar) / (cp + sp*vbar)
**
**  or in reverse direction
**
**     vbar = (calpha * sp - tgamma * cp) / (calpha * cp + tgamma * sp)
**     vbar_horizon =  sp/cp
**     vbar_vanish  = -cp/sp
**
**     ubar         = - tan(alpha) * (cp + sp*vbar)
**     ubar_horizon = - tan(alpha) / cp
*/


/*********************************************************************
**		       General Setup Utilities			    **
*********************************************************************/
/*
**   Grab the (private) ROS parameters
*/
void grabParameters()
{
  // Create a handle to the private namespace.
  ros::NodeHandle node_private("~");

  // Pull out the private parameters.
  node_private.param<std::string>("ground_frame_id", groundframe, "base");
  node_private.param<std::string>("laser_frame_id",  laserframe,  "laser");

  node_private.param("min_height", hmin, 0.030);
  node_private.param("max_height", hmax, 0.250);

  node_private.param("horz_samples", horzsamples, -1);
  node_private.param("vert_samples", vertsamples, -1);

  node_private.param("correction", correction, 0.000056);

  node_private.param("minimum_contacts", min_contacts, MINIMUM_CONTACTS);
  node_private.param("similar_fraction", sim_fraction, SIMILAR_FRACTION);
  sim_above = 1.0 + sim_fraction;
  sim_below = 1.0 / sim_above;
}

/*
**   Grab the Camera Info
*/
int grabCameraInfo()
{
  CameraInfo::ConstPtr msgp;

  // Report.
  ROS_INFO("Getting depth camera info...");

  // Grab a single message of the camera_info topic.
  Time tbefore = Time::now();
  msgp = topic::waitForMessage<CameraInfo>(INFO_TOPIC, Duration(TIMEOUT));
  Time tafter  = Time::now();
  if (msgp == NULL)
    {
      ROS_ERROR("Unable to read the depth camera's info in %5.1fsec!",
		TIMEOUT);
      return -1;
    }
  else
    {
      ROS_INFO("Received depth camera info after %5.1fsec",
	       (tafter - tbefore).toSec());
    }

  // Save the image frame ID.
  imageframe = msgp->header.frame_id;

  // Save the width/height.
  Nu = msgp->width;
  Nv = msgp->height;

  // Save the image resolution and center numbers from the projection
  // matrix.  note this maps an (x,y,z) point to (u=column,v=row) as:
  //    u = cu + fu * (x/z)
  //    v = cv + fv * (y/z)
  fu = msgp->P[0];
  cu = msgp->P[2];
  fv = msgp->P[5];
  cv = msgp->P[6];

  // Report.
  ROS_INFO("Projection:  u = %7.3f + %7.3f * (x/z)", cu, fu);
  ROS_INFO("             v = %7.3f + %7.3f * (y/z)", cv, fv);
  ROS_INFO("Image %dx%d = %.0fx%.0f deg FOV", Nu, Nv,
	   2.0 * atan((double)Nu/2.0/fu) * 180.0/M_PI,
	   2.0 * atan((double)Nv/2.0/fv) * 180.0/M_PI);
  return 0;
}

/*
**   Grab the Camera Pose (Optical/Image Frame)
**
**   The comprises the Height (above ground) and Pitch angle (upward)
*/
int grabCameraPose()
{
  TransformStamped msg;

  // Report.
  ROS_INFO("Getting depth camera pose...");

  // Create a TF buffer and listener.
  tf2_ros::Buffer            buffer;
  tf2_ros::TransformListener listener(buffer);

  // Try to grab the ground to image transform.  Use ros::Time(0) to
  // grab the latest transform, as opposed to the transform at a
  // particular time.  Use a timeout, in case no transforms appear.
  try
    {
      msg = buffer.lookupTransform(groundframe, imageframe,
				   Time(0), Duration(TIMEOUT));
    }
  catch (tf2::TransformException &e)
    {
      ROS_ERROR("Unable to get the '%s' to '%s' transform!",
		groundframe.c_str(), imageframe.c_str());
      ROS_ERROR("Have you started the robot_state_publisher?");
      return -1;
    }

  // Extract the rotation and the vertical components of the X/Y/Z
  // axes, knowing the ground frame's Z axis is vertical.
  Quaternion r = msg.transform.rotation;
  double xvertical =       2.0 * (r.x*r.z - r.w*r.y);	// X = Image right
  double yvertical =       2.0 * (r.y*r.z + r.w*r.x);	// Y = Image down
  double zvertical = 1.0 - 2.0 * (r.x*r.x + r.y*r.y);	// Z = Image forward

  // Check the camera (Image-right) being horizontal.
  if (fabs(xvertical) > 1e-9)
    {
      ROS_ERROR("The depth camera is not mounted horizontally!");
      return -1;
    }

  // Grab the pitch angle.
  spitch =  zvertical;		// Vertical component of Z axis
  cpitch = -yvertical;		// Vertical component of -Y axis
  pitch  = atan2(spitch, cpitch);

  // Also grab the height.
  hcam = msg.transform.translation.z;

  // Report.
  ROS_INFO("The depth camera is located %.2fmm above ground",
	   hcam*1000.0);
  ROS_INFO("The depth camera is pitched up %6.3fdeg (%4.3frad)",
	   pitch * 180.0/M_PI, pitch);

  ROS_INFO("Quadratic correction factor %9.6f", correction);
  ROS_INFO("Required at least %d contacts per object", (int) min_contacts);
  ROS_INFO("Same object if range is +/- %4.1f%%", sim_fraction * 100.0);

  return 0;
}

/*
**   Broadcast the Laser Frame, rotated down by Pitch angle
*/
void broadcastLaserFrame(tf2_ros::StaticTransformBroadcaster &broadcaster)
{
  // Precompute the sin/cos for the quaternion.
  double s = sin(pitch/2.0);
  double c = cos(pitch/2.0);

  // Prepare the transform message.  Rotate such that the laser frame
  // is X horizontal along image axis, Y left, Z up (vertical).
  TransformStamped msg;
  msg.header.stamp    = Time::now();
  msg.header.frame_id = imageframe;
  msg.child_frame_id  = laserframe;
  msg.transform.translation.x = 0.0;
  msg.transform.translation.y = 0.0;
  msg.transform.translation.z = 0.0;
  msg.transform.rotation.x    = 0.5 * (c - s);
  msg.transform.rotation.y    = 0.5 * (s - c);
  msg.transform.rotation.z    = 0.5 * (s + c);
  msg.transform.rotation.w    = 0.5 * (s + c);

  // Broadcast.
  broadcaster.sendTransform(msg);
}

/*
**   Setup the parameters and frames.
*/
int setup()
{
  // Grab the private ROS parameters.
  grabParameters();

  // Grab the camera image info.
  if (grabCameraInfo())
    return -1;

  // Report the frames.
  ROS_INFO("Ground frame '%s'", groundframe.c_str());
  ROS_INFO("Image  frame '%s'", imageframe.c_str());
  ROS_INFO("Laser  frame '%s'", laserframe.c_str());

  // Grab the camera pose.
  if (grabCameraPose())
    return -1;

  // Generate the laser frame (in which to publish the scan data).
  // Do this from the main() loop so the broadcaster stays in scope
  // and thus alive.
  // broadcastLaserFrame();
  return 0;
}


/*********************************************************************
**			 Prepare the Sampling			    **
*********************************************************************/
/*
**   Depth to Range Factor
**
**   Multiply a depth (in image) to get matching range for laser scan.
*/
double depthToRange(int u, int v)
{
  // Precompute the normalized image coordinates.
  double ubar = ((double) u - cu) / fu;
  double vbar = ((double) v - cv) / fv;

  return sqrt((cpitch + spitch*vbar)*(cpitch + spitch*vbar) + ubar*ubar);
}

/*
**   Depth Thresholds
**
**   We consider two distinct thresholds:
**
**   1) If the reported depth is greater than depthForOutOfBand(), the
**      possible contact lies above hmax or below hmin (vertically
**      outside the range of interest).  That means we do not report
**      the contact, but we also do not use this sample to declare
**      free space.  Basically, we ignore it.
**
**   2) If the reported depth is less than depthForOutOfBand(), i.e.
**      still in the vertical range of interest, but greater than
**      depthForFreespace(), it is past the maximum we want for the
**      laser range and we report freespace or no contact.
*/
double depthForOutOfBand(int u, int v)
{
  // Precompute the normalized image coordinate.
  double vbar = ((double) v - cv) / fv;
  
  // Consider the depth at which contacts fall vertically outside the
  // hmin-to-hmax range and hence shold be ignored.  Assuming hcam
  // lies in range, check looking up or down seperately.  The depth
  // can (in theory) report up to 65535mm, so set infinity to 66m.
  double a = spitch - cpitch * vbar;
  if      (a > 0.0)  return (hmax-hcam)/a;	// Looking up
  else if (a < 0.0)  return (hmin-hcam)/a;	// Looking down
  else               return 66.0;		// Looking flat (infinity)
}

double depthForFreespace(int u, int v)
{
  // Precompute the normalized image coordinate.
  double vbar = ((double) v - cv) / fv;
    
  // Depth corresponding to max contact range (in given direction).
  return RANGE_MAXIMUM / depthToRange(u,v);
}

/*
**   Set (pre-compute) the Sample Information
*/
int setSampleSpecs()
{
  // Allocate space for the overall sampling specifications.
  anglespecs = (anglespec_t *) malloc(Nangles * sizeof(anglespec_t));
  if (anglespecs == NULL)
    {
      ROS_ERROR("Unable to malloc space for the angle sample specs!");
      return -1;
    }

  // Iterate over all angles.
  for (int iangle = 0 ; iangle < Nangles ; iangle++)
    {
      // Set the angle.
      double angle  = angle_min + (double) iangle * angle_inc;
      double tangle = tan(angle);
      anglespecs[iangle].angle = angle;

      // Allocate space for the angle sampling specifications.
      anglespecs[iangle].pointspecs =
	(pointspec_t *) malloc(Nlines * sizeof(pointspec_t));
      if (anglespecs[iangle].pointspecs == NULL)
	{
	  ROS_ERROR("Unable to malloc space for the point sample specs!");
	  return -1;
	}

      // Interate over all lines.
      for (int iline = 0 ; iline < Nlines ; iline++)
	{
	  // Determine the pixels = image coordinates.
	  int v = iline * dlines;
	  double vbar = ((double) v - cv) / fv;
	  double ubar = - tangle * (cpitch + spitch * vbar);
	  int u = (int) round(cu + fu * ubar);
	  if ((u < 0) || (u >= Nu))
	    break;

	  // Set the specifications.
	  pointspec_t *ptspecp    = &anglespecs[iangle].pointspecs[iline];
	  ptspecp->index          = v*Nu + u;
	  ptspecp->depthoutofband = (uint16_t)(1000.0*depthForOutOfBand(u,v));
	  ptspecp->depthfreespace = (uint16_t)(1000.0*depthForFreespace(u,v));
	  ptspecp->depthtorange   = depthToRange(u,v) / 1000.0;
	  anglespecs[iangle].points = iline+1;
	}
    }

  return 0;
}

/*
**   Prepare the Sample Information
*/
int prepareSampling()
{
  // Determine the number of angles to sample.  And the image line
  // sampling step size.
  if (horzsamples > 0)   Nangles = horzsamples;
  else                   Nangles = 1 + Nu/2;
  if (vertsamples > 0)   dlines = Nv / vertsamples;
  else                   dlines = 2;
  Nlines = Nv/dlines;
  ROS_INFO("Sampling %d angles, each on %d image lines", Nangles, Nlines);

  // Possibly expand the min/max height to include the camera height.
  if (hmin > hcam)  hmin = hcam;
  if (hmax < hcam)  hmax = hcam;
  ROS_INFO("Scanning for objects %.2fmm to %.2fmm above ground",
	   hmin * 1000.0, hmax * 1000.0);

  // For reporting, compute the horizon line.
  //  int vhorizon = (int) round(cv + fv * spitch/cpitch);
  //  ROS_INFO("The horizon is on row v = %d", vhorizon);

  // Determine the angle samples.
  angle_min = atan(-cpitch * ((double) (Nu-1) - cu)/fu);
  angle_max = atan(-cpitch * (                - cu)/fu);
  angle_inc = (angle_max - angle_min) / ((double) (Nangles - 1));

  // Pre-compute the sample specs/info.
  if (setSampleSpecs())
    return -1;
  
  // Pre-populate the scan message.
  scanmsg.header.frame_id = laserframe;
  scanmsg.angle_min       = angle_min;
  scanmsg.angle_max       = angle_max;
  scanmsg.angle_increment = angle_inc;
  scanmsg.time_increment  = 0.0;
  scanmsg.scan_time       = 1.0 / 30.0;
  scanmsg.range_min       = RANGE_MINIMUM;	// Minimum range value [m]
  scanmsg.range_max       = RANGE_MAXIMUM;	// Maximum range value [m]
  scanmsg.ranges.clear();
  for (int iangle = 0 ; iangle < Nangles ; iangle++)
    scanmsg.ranges.push_back(RANGE_BAD);

  return 0;
}


/*********************************************************************
**		    Callback - Sampling Algorithm		    **
*********************************************************************/
/*
**   Depth Image Callback
*/
void callback_image(const Image::ConstPtr& imagemsgp)
{
  // // Report the timing (for debugging).
  // Time now = Time::now();
  // ROS_INFO("Image #%4d at %10d.%09d secs. Now %10d.%09d, diff = %5.3fs",
  //          imagemsgp->header.seq,
  // 	   imagemsgp->header.stamp.sec,
  // 	   imagemsgp->header.stamp.nsec,
  // 	   now.sec,
  // 	   now.nsec,
  // 	   (now - imagemsgp->header.stamp).toSec());

  // Check the image width/height.
  if ((imagemsgp->width != Nu) || (imagemsgp->height != Nv))
    {
      ROS_ERROR("Depth image has changed size (%dx%d) vs (%dx%d)!",
		imagemsgp->width, imagemsgp->height, Nu, Nv);
      return;
    }

  // Check the data encoding.
  if (imagemsgp->encoding.compare("16UC1") != 0)
    {
      ROS_ERROR("Depth image type encoding '%s' different from '16UC1'!",
		imagemsgp->encoding.c_str());
      return;
    }

  // Grab a pointer to the depth data (appropriately cast).
  const uint16_t *depthdata = (const uint16_t *) &imagemsgp->data[0];

  // Testing...
  // int u = Nu/2;
  // int v = Nv/2;
  // ROS_INFO("Distance at (row %d, col %d) = %dmm", v, u, dp[v*Nu+u]);

  // Compute the ranges at each angle, as the minimum range of all
  // sample points per angle.
  for (int iangle = Nangles-1 ; iangle >= 0 ; iangle--)
    {
      // Initialize the minimum range bigger than anything to come.
      // Set the count of samples (at this range) to zero.
      double range = 2.0 * RANGE_MAXIMUM * sim_above;
      double count = 0.0;

      // Also note no contacts past the full range.
      int contact_past_range = 0;
      int contact_in_range   = 0;

      // Examine each sample point for this angle.
      for (int ipoint = anglespecs[iangle].points-1 ; ipoint >= 0 ; ipoint--)
	{
	  // Pointer to the sample point details.
	  pointspec_t *ptspecp = &anglespecs[iangle].pointspecs[ipoint];

	  // Grab the depth at the point of interest.
	  uint16_t  depth = depthdata[ptspecp->index];

	  // Check whether the depth places the contact (a) vertically
	  // out of band of interest (ignore), (b) beyond the maximum
	  // range (freespace), (c) in range (report), of (d) at zero
	  // being an error (ignore).
	  if (depth > ptspecp->depthoutofband)
	    {
	      // Ignore.
	      ;
	    }
	  else if (depth > ptspecp->depthfreespace)
	    {
	      // Mark the contact.
	      contact_past_range = 1;
	    }
	  else if (depth > 0)
	    {
	      // Grab the sample range.
	      double d, samplerange;
	      d = (double) depth;
	      d = d * (1.0 + correction * d);
	      samplerange = d * ptspecp->depthtorange;

	      // Mark the contact and update the range.
	      contact_in_range = 1;
	      if (samplerange < sim_below * range)
		{
		  // This is a new, closer contact.
		  count = 1.0;
		  range = samplerange;
		}
	      else if (samplerange < sim_above * range)
		{
		  // This is the same (close) range -> Average.
		  count += 1.0;
		  range += (samplerange - range) / count;
		}
	    }
	}

      // A valid object will have at least 5 contact points!
      if (contact_in_range)
	{
          if (count >= min_contacts)  scanmsg.ranges[iangle] = range;
          else                        scanmsg.ranges[iangle] = RANGE_BAD;
        }
      else if (contact_past_range)    scanmsg.ranges[iangle] = RANGE_NOCONTACT;
      else                            scanmsg.ranges[iangle] = RANGE_BAD;
    }

  // Update the mssage and publish.  Use the depth camera time!
  scanmsg.header.stamp = imagemsgp->header.stamp;
  scanpub.publish(scanmsg);
}



/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize ROS and grab regular/private node handles.
  ros::init(argc, argv, "depthtolaserscan2");
  ros::NodeHandle node;

  // Setup the parameters and frames.
  if (setup())
    return -1;

  // Prepare the sampling specifications/data structures.
  if (prepareSampling())
    return -1;

  // Create a publisher for the laser scan (queue size of 10).
  scanpub = node.advertise<LaserScan>(SCAN_TOPIC, 10);

  // Create a subscriber to listen to the depth images.  Limiting the
  // queue size would prevent too many messages from building up, if
  // this subscriber can not keep up.
  Subscriber imagesub = node.subscribe(IMAGE_TOPIC, 6, callback_image);

  // Finally broadcast the static image to laser frame transform.
  tf2_ros::StaticTransformBroadcaster broadcaster;
  broadcastLaserFrame(broadcaster);

  // And spin indefinitely.
  spin();
  ROS_INFO("End convert depth image to laser scan.");
  return 0;
}
