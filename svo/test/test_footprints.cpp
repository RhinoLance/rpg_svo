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
  void runFromFolder( std::string folderPath);
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
  */

  //UniFarm
  cam_ = new vk::PinholeCamera(
      639, 480,       	//image width, height
      315.5, 315.5,     //focal length
      376.0, 240.0      //principal point
    );
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void BenchmarkNode::runFromFolder(std::string folderPath)
{
	std::cout << "{ \"type\": \"FeatureCollection\",\n"
    			<< "\t\"features\": [\n";  

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
  
	// load image
	//std::cout << "Loading " << cI << " " << *it;    
	
	cv::Mat img(cv::imread(folderPath + "/" + string(*it), 0));
    if( img.empty()) {
		//std::cout << "... empty, skipping\n";		
		continue;
	}

	std::cout << "\n";   

    // process frame
    vo_->addImage(img, 0.01*cI);

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
    	//Try three times to get good keypoints.
		vo_->lastFrame()->setKeyPoints();
		vo_->lastFrame()->setKeyPoints();
		vo_->lastFrame()->setKeyPoints();
		
		empty_key_pts = false;
		
		for(std::vector<Feature*>::iterator it = vo_->lastFrame()->key_pts_.begin(); it != vo_->lastFrame()->key_pts_.end(); ++it){
		//for( size_t keyPtInx=1; keyPtInx<vo_->lastFrame()->key_pts_.size(); keyPtInx++ ) {	
			Feature* feat = *it;			
			//std::cout << feat->point->pos_.x();			
			empty_key_pts = ( feat == NULL );
			if( empty_key_pts )break;

			//empty_key_pts = ( vo_->lastFrame()->key_pts_[keyPtInx] == NULL);
		}

		if( empty_key_pts ) {
			nonValidKeyPts++;
			continue;
		}
			
		std::cout << "\t\t{ \"type\": \"Feature\",\n"
			<< "\t\t\t\"properties\":{\n"
			<< "\t\t\t\t\"frameId\": \"" << vo_->lastFrame()->id_ << "\",\n"
			<< "\t\t\t\t\"isKeyframe\": \"" << vo_->lastFrame()->isKeyframe() << "\"\n"
			<< "\t\t\t},\n"
			<< "\t\t\t\"geometry\":{\n"
			<< "\t\t\t\t\"type\": \"Polygon\",\n"
			<< "\t\t\t\t\"coordinates\": [[\n";
//						
		for( size_t keyPtInx=1; keyPtInx<vo_->lastFrame()->key_pts_.size(); keyPtInx++ ) {
			
			if(vo_->lastFrame()->key_pts_[keyPtInx] == NULL){
				std::cout << "\t\t\t\t\t[-0, -0],\n";			
				continue;
			}
/*
			std::cout << keyPtInx << " size " << vo_->lastFrame()->key_pts_.size() << "\n";
			std::cout << keyPtInx << " index " << vo_->lastFrame()->key_pts_[keyPtInx] << "\n";
			std::cout << keyPtInx << " point " << vo_->lastFrame()->key_pts_[keyPtInx]->point << "\n";
			std::cout << keyPtInx << ".x " << vo_->lastFrame()->key_pts_[keyPtInx]->point->pos_.x() << "\n";
			std::cout << keyPtInx << ".y " << vo_->lastFrame()->key_pts_[keyPtInx]->point->pos_.y() << "\n";
*/
			std::cout << "\t\t\t\t\t[" 
				<< vo_->lastFrame()->key_pts_[keyPtInx]->point->pos_.x() << ", "
				<< vo_->lastFrame()->key_pts_[keyPtInx]->point->pos_.y() 
				<< "],\n";
		}
//
		std::cout << "\t\t\t\t\t[" 
			<< vo_->lastFrame()->key_pts_[0]->point->pos_.x() << ", "
			<< vo_->lastFrame()->key_pts_[0]->point->pos_.y() 
			<< "]\n"
			<< "\t\t\t\t]]\n"
			<< "\t\t\t}\n"
			<< "\t\t},\n";
				
		

    }
  }
	
	std::cout << "\t]\n}"; 

	std::cout << nonValidKeyPts << " frames with invalid key points.\n";
}
} // namespace svo

int main(int argc, char** argv)
{
  {
	
	if( argc < 2 ) {
		std::cout << "Usage: " << argv[0] << " <image_path>\n";
		return -1;
	}
    
	svo::BenchmarkNode benchmark;
    benchmark.runFromFolder(argv[1]);
  }
  //printf("BenchmarkNode finished.\n");
  return 0;
}

