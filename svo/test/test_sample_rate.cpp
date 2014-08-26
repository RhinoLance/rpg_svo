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
// along with this program.	If not, see <http://www.gnu.org/licenses/>.

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
#include <sstream>
#include <dirent.h>
#include "test_utils.h"

namespace svo {

	std::vector <std::string> read_directory( const std::string& path = std::string() ) {
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

	} //read_directory


	class BenchmarkNode
	{
		vk::AbstractCamera* cam_;
		svo::FrameHandlerMono* vo_;

		public:
			BenchmarkNode();
			~BenchmarkNode();
			void runFromFolder( std::string folderPath);
			void processImagesInFolder( std::string folderPath, int nth);
	};

	BenchmarkNode::BenchmarkNode()
	{
		cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
		vo_ = new svo::FrameHandlerMono(cam_);
		vo_->start();
	}

	BenchmarkNode::~BenchmarkNode()
	{
		delete vo_;
		delete cam_;
	}

	void BenchmarkNode::processImagesInFolder(std::string folderPath, int nth)
	{
		//get the files in the directory
		std::vector <std::string> fileList;	
		fileList = read_directory( folderPath );
		
		int totalImageCount, candidateImageCount, processedImageCount, nonValidKeyPts, skipper;
		totalImageCount=0;
		candidateImageCount=0;
		processedImageCount=0;

		skipper=nth;
		nonValidKeyPts = 0;
		bool empty_key_pts;

		std::cout << "Frame ID,# Features Tracked, Processing Time\n"; 

		for(std::vector<std::string>::iterator it = fileList.begin(); it != fileList.end(); ++it) {

			totalImageCount++;

			//Manage the skipper so that we are only processing every n'th image
			if( skipper > 1 ) {
				skipper--;
				continue;
			}
			else{
				skipper = nth;
			}

			candidateImageCount++;

			// load image
			cv::Mat img(cv::imread(folderPath + "/" + string(*it), 0));
			
			//Skip empty images
			if( img.empty()) {
				std::cout << string(*it) << " empty, skipping\n";		
				continue;
			}

			// process frame
			vo_->addImage(img, 0.01*processedImageCount);

			// display tracking quality
			if(vo_->lastFrame() != NULL) {
				
				//increment the succesfull count
				if( vo_->lastNumObservations() > 4 ){
					processedImageCount++;
				}

				//Output as <FileName>, <FrameId>, <Num features tracked>, <Processing time (ms)>
				/*
				std::cout << string(*it) << ","
					<< vo_->lastFrame()->id_ << ","
	                << vo_->lastNumObservations() << "/" << vo_->lastFrame()->fts_.size() << "/" << vo_->lastFrame()->nObs() << ","
	                << vo_->lastProcessingTime()*1000 << ","
	                << "[" << vo_->lastFrame()->pos().x() << ", " << vo_->lastFrame()->pos().y() << ", " << vo_->lastFrame()->pos().z() << "]\n";
	            */
			}
		}

		std::cout << "Processed <total> <candidates> <succesfully processed> <success rate> " 
			<< totalImageCount << ", " 
			<< candidateImageCount << ", " 
			<< processedImageCount << ", " 
			<< ((float)processedImageCount / (float)candidateImageCount )*100 << "%\n";
		
		
	}// BenchmarkNode::processImagesInFolder

} // namespace svo

int main(int argc, char** argv)
{
	if( argc < 3 ) {
		std::cerr << "Usage: " << argv[0] << " <image_path> <nth_image_to_process> [cycles]\n";
		return -1;
	}
	
	//Get the n'th identifier command line argument.
	std::istringstream ssn(argv[2]);
	int nth;
	if (!(ssn >> nth)) {
    	std::cerr << "Usage: " << argv[0] << " <image_path> [nth_image_to_process] [cycles]\n";
    	return -1;
    }

	//Get the number of cycles to run.
	int cycles;
	if( argc < 4 ){
    	cycles = 1;
    }
    std::istringstream ssc(argv[3]);
	if (!(ssc >> cycles)) {
    	cycles = 1;
    }

	svo::BenchmarkNode benchmark;
	for( int runs=0; runs<cycles; runs++){
		benchmark.processImagesInFolder(argv[1], nth);
	}

	return 0;
}

