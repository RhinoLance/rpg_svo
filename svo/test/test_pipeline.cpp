// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include <dirent.h>
#include "test_utils.h"

namespace svo {

std::vector <std::string> read_directory( const std::string& path = std::string() )
  {
  std::vector <std::string> result;
  dirent* de;
  DIR* dp;
  errno = 0;
  dp = opendir( path.empty() ? "." : path.c_str() );
  if (dp)
    {
    while (true)
      {
      errno = 0;
      de = readdir( dp );
      if (de == NULL) break;
      result.push_back( std::string( de->d_name ) );
      }
    closedir( dp );
    std::sort( result.begin(), result.end() );
    }
  return result;
  }


class BenchmarkNode
{
  vk::AbstractCamera* cam_;
  svo::FrameHandlerMono* vo_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void runFromFolder( std::string folderPath, int skip);
};

BenchmarkNode::BenchmarkNode()
{
  /*
  //Test office data
  cam_ = new vk::PinholeCamera(
      752, 480,         //image width, height
      315.5, 315.5,     //focal length
      376.0, 240.0      //principal point
    );

  //UniFarm
  cam_ = new vk::PinholeCamera(
      639, 480,       	//image width, height
      315.5, 315.5,     //focal length
      376.0, 240.0      //principal point
    );

	*/
  //cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0); //Grass
  cam_ = new vk::PinholeCamera(640, 480, 819.5, 819.5, 328.8, 233.0); //chameleon

  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void BenchmarkNode::runFromFolder(std::string folderPath, int skip)
{
	//get the files in the directory
	std::vector <std::string> fileList;	
	fileList = read_directory( folderPath );
	
  
	int cI, nonValidKeyPts;
	cI=0;
	nonValidKeyPts = 0;
	bool empty_key_pts;

  for(std::vector<std::string>::iterator it = fileList.begin(); it != fileList.end(); ++it)
  {
	cI++;

	//Skip rows to start later in the dataset if required
	if( cI < skip){
		continue;
	}
  
	// load image
	cv::Mat img(cv::imread(folderPath + "/" + string(*it), 0));
    if( img.empty()) {
		std::cerr << "[WARN]\t" << *it << " empty, skipping\n";		
		continue;
	}

	// process frame
    vo_->addImage(img, 0.01*cI);

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
    	std::cout << *it << ","
    			  << vo_->lastFrame()->id_ << ","
                  << vo_->lastNumObservations() << ","
                  << vo_->lastFrame()->isKeyframe() << ","
                  << vo_->lastProcessingTime()*1000 << "\n";

    	// access the pose of the camera via vo_->lastFrame()->T_f_w_.
    }	

  }	
	
}
} // namespace svo

int main(int argc, char** argv)
{
  {
	
	if( argc < 2 ) {
		std::cout << "Usage: " << argv[0] << " <image_path> [skip]\n";
		return -1;
	}

	//Get the skipper command line argument.
	std::istringstream ssn(argv[2]);
	int skip;
	if (!(ssn >> skip)) {
    	skip = 0;
    }
    
	std::cout << "Image,Frame-Id,#Features,KeyFrame,Proc. Time\n";

	svo::BenchmarkNode benchmark;
    benchmark.runFromFolder(argv[1], skip);
  }
  //printf("BenchmarkNode finished.\n");
  return 0;
}

