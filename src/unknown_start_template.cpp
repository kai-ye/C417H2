#define NUM_HORIZ 10
#define NUM_VERT 20
#define NUM_PARTICLES (NUM_HORIZ * NUM_VERT)


  //To initialize particles and cumulative weights. Starting location (0,0)
  void initParticles() {
    ROS_INFO("Initialializing particles.");
    ros::Time nowTime = ros::Time::now();
    double width = (double)map_image.size().width / METRE_TO_PIXEL_SCALE;
    double height = (double)map_image.size().height / METRE_TO_PIXEL_SCALE;
    //ROS_INFO("map width: %f, height: %f", width, height);
    cumulativeWeights[0] = 0.0;
    for (int i = 0; i < NUM_HORIZ; i++) {
      for (int j = 0; j < NUM_VERT; j++) {
        double xMean = (width/NUM_HORIZ) * i + width * (.5/NUM_HORIZ - .5);
        double yMean = (height/NUM_VERT) * j + height * (.5/NUM_VERT - .5);
        double radius = normalRand(0.0, 0.2);
        double theta = uniformRand(0.0, 2*M_PI);
        double yaw = uniformRand(0.0, 2 * M_PI);
        int k = i * NUM_VERT + j;
        //ROS_INFO("cell %d centered at %f, %f", k, xMean, yMean);
        particles[k].x = xMean + radius * cos(theta);
        particles[k].y = yMean + radius * sin(theta);
        particles[k].z = ROBOT_Z;
        particles[k].yaw = yaw;
        particles[k].penultimateTargetYaw = particles[k].yaw;
        particles[k].lastTargetYaw = particles[k].yaw;
        particles[k].yawStartTime = nowTime;
        particles[k].weight = 1.0 / NUM_PARTICLES;
        cumulativeWeights[k + 1] = cumulativeWeights[k] + particles[k].weight;
        drawParticleOnMap(k);
      }
    }
  }

