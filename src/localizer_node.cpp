/* vim: set expandtab tabstop=2 softtabstop=2 shiftwidth=2 number relativenumber: */
//Localizer given initial position

#include <cmath>
#include <iostream>
#include <random>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#define METRE_TO_PIXEL_SCALE 50
#define NUM_PARTICLES 100
#define FOCAL_HALF_WIDTH 10
#define FORWARD_SWIM_SPEED_SCALING 0.07
#define YAW_RATE 0.0000
#define POSITION_GRAPHIC_RADIUS 20.0
#define HEADING_GRAPHIC_LENGTH 50.0
#define REEF_Z (-7.0)
#define ROBOT_Z (-5.0)
#define VARIANCE_MOTION_YAW 0.1
#define VARIANCE_MOTION_FORWARD .05
#define VARIANCE_PIXEL 10.
#define SIZE_MATCH_CACHE (3*255*255+1)

#define SQ(x) ((x) * (x))


// To bring the angle in radians into the interval [0, 2pi)
void intoZeroTwoPI(double* s) {
  while (*s < 0.0) (*s)+= 2*M_PI;
  while (*s >= 2*M_PI) (*s)-= 2*M_PI;
}

// To compute the result of incrementing by the given amount from a current
//angle to a target angle, in the closer direction.
double incrementedAngleFromCurrToTarget(double curr,double target,double inc) {
  double targetCopy = target;
  intoZeroTwoPI(&curr);
  intoZeroTwoPI(&target);

  if (std::abs(target - curr) <= inc || 2*M_PI - inc <= std::abs(target - curr))
    return targetCopy;

  else if ((curr < target && target - curr <= M_PI) ||
            (target < curr && curr - target >= M_PI))
    return curr + inc;
  else
    return curr - inc;
}

// To compute the closer of the two mean values of two angles in radians
double radianMean(double s, double t) {
  //Bring s and t into the interval [0, 2pi)
  intoZeroTwoPI(&s);
  intoZeroTwoPI(&t);
  double mid = (s+t)/2;
  if (-M_PI <= s-t && s-t <= M_PI)
    return mid;
  else if (mid >= M_PI)
    return mid - M_PI;
  else
    return mid + M_PI;
}

// Random value according to uniform distribution
double uniformRand(double low, double high) {
  std::mt19937 generator(std::random_device{}());
  std::uniform_real_distribution<double> dist(low, high);
  return dist(generator);
}

// Random value according to normal distribution
double normalRand(double mean, double variance) {
  std::mt19937 generator(std::random_device{}());
  std::normal_distribution<double> distribution(mean, variance);
  return distribution(generator);
}

// Function to return a yaw according to some chosen distribution, given mean
double probableYaw(double mean) {
  return normalRand(mean, VARIANCE_MOTION_YAW);
}

// Function to return a probable translation value
double probableTranslation(double mean) {
  return normalRand(mean, VARIANCE_MOTION_FORWARD);
}



class Particle
{
public:
  double x, y, z, yaw, weight;
  double lastTargetYaw;
  ros::Time yawStartTime;
};


// Class Localizer is a sample stub that you can build upon for your implementation
// (advised but optional: starting from scratch is also fine)
//
class Localizer
{
private:
  static double matchCache[SIZE_MATCH_CACHE];  // experimental
  Particle particles[NUM_PARTICLES];
  double cumulativeWeights[NUM_PARTICLES + 1];
  const double K[3][3] = {{238.3515418007097, 0.0, 200.5},
                          {0.0, 238.3515418007097, 200.5},
                          {0.0, 0.0, 1.0}};    //intrinsic matrix of camera
  const double camR[3] = {-0.32, 0.0, -0.06};  //camera's origin in robot frame

public:
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  image_transport::Subscriber gt_img_sub;
  image_transport::Subscriber robot_img_sub;

  ros::Subscriber motion_command_sub;

  geometry_msgs::PoseStamped estimated_location;

  cv::Mat map_image;
  cv::Mat ground_truth_image;
  cv::Mat localization_result_image;

  Localizer( int argc, char** argv )
  {
    std::fill_n(matchCache, SIZE_MATCH_CACHE, -1.0);  // experimental
    image_transport::ImageTransport it(nh);
    pub = it.advertise("/assign2/localization_result_image", 1);
    map_image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    localization_result_image = map_image.clone();
    ground_truth_image = map_image.clone();

    gt_img_sub = it.subscribe("/assign2/ground_truth_image", 1,
              &Localizer::groundTruthImageCallback, this);
    robot_img_sub = it.subscribe("/aqua/back_down/image_raw", 1,
              &Localizer::robotImageCallback, this);
    motion_command_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aqua/target_pose", 1,
              &Localizer::motionCommandCallback, this);

    ROS_INFO( "Localizer node constructed and subscribed." );

    //Acquire initial position.
    estimated_location.pose.position.x = 0;
    estimated_location.pose.position.y = 0;

    //Initialize particles and cumulative weights.
    initParticles();
    ROS_INFO("Particles initialized.");
  }

  //To initialize particles and cumulative weights. Starting location (0,0)
  void initParticles() {
    ROS_INFO("Initialializing particles.");
    ros::Time nowTime = ros::Time::now();
    cumulativeWeights[0] = 0.0;
    for (int i = 0; i < NUM_PARTICLES; i++) {
      double radius = normalRand(0.0, 0.7);
      double theta = uniformRand(0.0, 2*M_PI);
      double yaw = normalRand(0.0, VARIANCE_MOTION_YAW);
      particles[i].x = radius * cos(theta);
      particles[i].y = radius * sin(theta);
      //particles[i].x = 0.0;                   // experimental
      //particles[i].y = 0.0;                   // experimental
      particles[i].z = ROBOT_Z;
      particles[i].yaw = yaw;
      particles[i].yaw = 0.0;
      particles[i].weight = 1.0 / NUM_PARTICLES;
      particles[i].lastTargetYaw = particles[i].yaw;
      particles[i].yawStartTime = nowTime;
      cumulativeWeights[i + 1] = cumulativeWeights[i] + particles[i].weight;
    }
  }

  // To choose a particle index according to weights of the particles
  int pickIndex() {
    int low = 0, high = NUM_PARTICLES, mid;
    double random = uniformRand(0.0, cumulativeWeights[NUM_PARTICLES]);

    while (low + 1 < high) {
      mid = (low + high) / 2;   //certainly >= low + 1, and <= high - 1
      if (random < cumulativeWeights[mid])
        high = mid;
      else
        low = mid;
    }

    return low;
  }

  // Percentage by which a pixel matches a given pixel value.
  // (I.e., probability of observing pixelToMatch given that
  //pixelGiven is the expected actual pixel value.)
  // In fact num-th root of that percentage is returned, making it
  //convenient for computing geometric mean of many such percentages.
  double match(cv::Vec3b pixelGiven, cv::Vec3b pixelToMatch, int num) {
    int key;
    key = (pixelToMatch[0]-pixelGiven[0])*(pixelToMatch[0]-pixelGiven[0]) +
          (pixelToMatch[1]-pixelGiven[1])*(pixelToMatch[1]-pixelGiven[1]) +
          (pixelToMatch[2]-pixelGiven[2])*(pixelToMatch[2]-pixelGiven[2]);
    if (matchCache[key] < 0.0) {
      matchCache[key] = exp(-((double) key) / (2 * VARIANCE_PIXEL * num));
    }
    return matchCache[key];
    /* return exp( -(double) (
              (pixelToMatch[0]-pixelGiven[0])*(pixelToMatch[0]-pixelGiven[0]) +
              (pixelToMatch[1]-pixelGiven[1])*(pixelToMatch[1]-pixelGiven[1]) +
              (pixelToMatch[2]-pixelGiven[2])*(pixelToMatch[2]-pixelGiven[2])) /
              (2. * VARIANCE_PIXEL * num)); */
  }

  // Pixel on the map corresponding to the point in the camera image.
  cv::Vec3b imagePointToMapPixel(int xI, int yI, Particle particle) {
    double pointM[2];//point in map frame corresponding to image point
    double pointW[3];//point in world frame corresponding to image point
    double pointC[3];//point in camera frame corresponding to image point
    double camW[3];//camera's origin, in world frame
    double yaw = particle.yaw;

    camW[0] = camR[0] * cos(yaw) - camR[1] * sin(yaw) + particle.x;
    camW[1] = camR[0] * sin(yaw) + camR[1] * cos(yaw) + particle.y;
    camW[2] = camR[2] + particle.z;

    pointC[2] = camW[2] - REEF_Z;           //assuming planar motion
    pointC[0] = (pointC[2] * xI - pointC[2] * K[0][2]) / K[0][0];
    pointC[1] = (pointC[2] * yI - pointC[2] * K[1][2]) / K[1][1];

    pointW[0] = pointC[0] * sin(yaw) - pointC[1] * cos(yaw) + camW[0];
    pointW[1] = -pointC[0] * cos(yaw) - pointC[1] * sin(yaw) + camW[1];
    pointW[2] = REEF_Z;   //equal to -pointC[2] + camW[2]

    pointM[0] = map_image.size().width/2 + METRE_TO_PIXEL_SCALE * pointW[0];
    pointM[1] = map_image.size().height/2 - METRE_TO_PIXEL_SCALE * pointW[1];

    if (pointM[0] < 0 || pointM[0] >= map_image.size().width ||
        pointM[1] < 0 || pointM[1] >= map_image.size().height) {
      return cv::Vec3b(0,0,0);
    }
    else {
      return map_image.at<cv::Vec3b>(pointM[1], pointM[0]);
    }
  }

  // To propagate the motion model.
  void updateParticles(double forward, double target_yaw) {
    ros::Time nowTime = ros::Time::now();
    double probYaw, probTrans;
    for (int i = 0; i < NUM_PARTICLES; i++) {
      double yawIncrement = YAW_RATE *
                          (nowTime - particles[i].yawStartTime).toSec();
      double meanYaw = incrementedAngleFromCurrToTarget(
                            particles[i].lastTargetYaw, target_yaw, yawIncrement);
 
      if (meanYaw == target_yaw) {  //if target yaw reached by now
        particles[i].yawStartTime = nowTime;
        particles[i].lastTargetYaw = target_yaw;
      }
     
      //Yaw actually achieved, according to distribution
      probYaw = probableYaw(meanYaw);
      //Forward translation actually achieved, according to distribution
      probTrans = probableTranslation(FORWARD_SWIM_SPEED_SCALING * forward);

      //probYaw = meanYaw;                                // experimental
      //probTrans = FORWARD_SWIM_SPEED_SCALING * forward; // experimental

      particles[i].yaw = probYaw;
      particles[i].x+= probTrans * cos(probYaw);
      particles[i].y+= probTrans * sin(probYaw);
    }
  }

  // Function to draw particle of given index on result image
  void drawParticleOnMap(int i) {
    int estimated_robo_image_x = localization_result_image.size().width/2 +
            METRE_TO_PIXEL_SCALE * particles[i].x;
    int estimated_robo_image_y = localization_result_image.size().height/2 -
            METRE_TO_PIXEL_SCALE * particles[i].y;

    int estimated_heading_image_x = estimated_robo_image_x +
            HEADING_GRAPHIC_LENGTH * cos(-particles[i].yaw);
    int estimated_heading_image_y = estimated_robo_image_y +
            HEADING_GRAPHIC_LENGTH * sin(-particles[i].yaw);

    cv::circle( localization_result_image,
        cv::Point(estimated_robo_image_x, estimated_robo_image_y),
        4 /* POSITION_GRAPHIC_RADIUS *
          particles[i].weight / cumulativeWeights[NUM_PARTICLES] */,
        CV_RGB(250,0,0), -1);
    /* cv::line( localization_result_image,
      cv::Point(estimated_robo_image_x, estimated_robo_image_y),
      cv::Point(estimated_heading_image_x, estimated_heading_image_y),
      CV_RGB(250,0,0), 10); */
  }

  // Camera image callback
  void robotImageCallback( const sensor_msgs::ImageConstPtr& robot_img )
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cameraImage;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(robot_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cameraImage = cv::Mat(cv_ptr->image);
    int xMid = cameraImage.size().width/2;
    int xStart = xMid - FOCAL_HALF_WIDTH;
    int xEnd = xMid + FOCAL_HALF_WIDTH;
    int yMid = cameraImage.size().height/2; 
    int yStart = yMid - FOCAL_HALF_WIDTH;
    int yEnd = yMid + FOCAL_HALF_WIDTH;

    //Propagate observation model: resample, and update weights.
    Particle newParticles[NUM_PARTICLES];
    for (int i = 0; i < NUM_PARTICLES; i++) {
      newParticles[i] = particles[pickIndex()];
    }
    for (int i = 0; i < NUM_PARTICLES; i++) {
      particles[i] = newParticles[i];
      double newWeight = 1.0;
      for (int xI = xStart; xI < xEnd; xI++) {
        for (int yI = yStart; yI < yEnd; yI++) {
          newWeight*= match(
            imagePointToMapPixel(xI, yI, particles[i]),
            cameraImage.at<cv::Vec3b>(yI, xI),
            2*FOCAL_HALF_WIDTH*(1+sqrt(SQ(xI-xMid)+SQ(yI-yMid))));
        }
      }
      particles[i].weight = newWeight;
      cumulativeWeights[i+1] = cumulativeWeights[i] + particles[i].weight;
    }

    /* localization_result_image = cv::Mat(cv_ptr->image);
    buildEstimatesCameraImage(&localization_result_image, particles[0]); */
  }

  // Helper to rebuild camera image given estimated pose
  // Used to test whether imagePointToMapPixel is working correctly
  void buildEstimatesCameraImage(cv::Mat* image_ptr, Particle particle) {
    for (int xI = 0; xI < image_ptr->size().width; xI++)
      for (int yI = 0; yI < image_ptr->size().height; yI++) {
        cv::Vec3b pixel = imagePointToMapPixel(xI, yI, particle);
        image_ptr->at<cv::Vec3b>(yI, xI) = pixel;
      }

  }

  // Motion command callback
  void motionCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& motion_command )
  {

    geometry_msgs::PoseStamped command = *motion_command;
    double target_roll, target_pitch, target_yaw;
    tf::Quaternion target_orientation;
    tf::quaternionMsgToTF(command.pose.orientation, target_orientation);
    tf::Matrix3x3(target_orientation).getEulerYPR( target_yaw, target_pitch, target_roll );

    //Propagate the motion model.
    updateParticles(command.pose.position.x, target_yaw);

    // Comment the one following line to plot your whole trajectory without ground truth
    localization_result_image = ground_truth_image.clone();

    //Draw particles on result image.
    for (int i = 0; i < NUM_PARTICLES; i++) {
      drawParticleOnMap(i);
    }
  }

  // This is a provided convenience function that allows you to compare your localization result to a ground truth path
  void groundTruthImageCallback( const sensor_msgs::ImageConstPtr& gt_img )
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(gt_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ground_truth_image = cv::Mat(cv_ptr->image);
  }

  // This function publishes your localization result image and spins ROS to execute its callbacks
  void spin()
  {
    ros::Rate loop_rate(30);
    while (nh.ok()) {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", localization_result_image).toImageMsg();
      pub.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }


};

double Localizer::matchCache[];

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localizer");
  Localizer my_loc(argc, argv);
  my_loc.spin();
}
