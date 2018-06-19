#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iterator>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <stdexcept>
#include <stdlib.h>
#include <string>
#include <vector>
#include "macros.hpp"
#include "object_perception_node.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

class ObjectPerceiver
{
    std::string openCvWindow;

    vector<Mat> trainData_;
    int trainImgNum_;
    int objNum_;
    std::vector<std::string> objName_;
    ros::Publisher pub_;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;

public:
    ObjectPerceiver()
        : it_(nh_)
    {
        // TODO: define topics!
        std::string pub_topic = "/hearts/object_perception/recognised_image";
        std::string sub_topic = "/xtion/rgb/image_raw";

        pub_ = nh_.advertise<std_msgs::String>(pub_topic, 1000);

        // load ros params

        std::string trainPath;

        if (nh_.getParam("trainPath", trainPath))
        {
            ROS_INFO("Got param: %s", trainPath.c_str());
        }
        else
        {
            ROS_ERROR("Failed to get param 'trainParam'");
            throw std::runtime_error("Failed to get param 'trainParam'");
        }
    
        std::string objects;

        if (nh_.getParam("objects", objects))
        {
            ROS_INFO("Got param: %s", objects.c_str());
        }
        else
        {
            ROS_ERROR("Failed to get param 'objects'");
            throw std::runtime_error("Failed to get param 'objects'");
        }

        objName_ = split(objects, ',');
        objNum_ = objName_.size();

        if (nh_.getParam("trainImgNum", trainImgNum_))
        {
            ROS_INFO("Got param: %d", trainImgNum_);
        }
        else
        {
            ROS_ERROR("Failed to get param 'trainImgNum'");
            throw std::runtime_error("Failed to get param 'trainImgNum'");
        }

        // load training data

        cout << "objects: " << objNum_ << ", training images: " << trainImgNum_ << endl;
        
        // loading all training image data
        if ((objNum_ > 0) && (trainImgNum_ > 0))
        {
            for (int i = 0; i < objNum_; i++)
            {
                for (int j = 1; j <= trainImgNum_; j++)
                {
                    stringstream ss;
                    ss << objName_[i] << "_" << j << ".png"; // jpg
                    string imName = ss.str();
                    cout << "loading " << imName << "..." << endl;
                    Mat imgTemp = imread(trainPath + imName, IMREAD_GRAYSCALE);
                    trainData_.push_back(imgTemp);
                    cout << "loaded " << imName << endl;
                }
            }
        }

        // TODO: camera rostopic?
        sub_ = it_.subscribe(sub_topic, 1, &ObjectPerceiver::imageCallback, this);

        openCvWindow = "Image window";
        cv::namedWindow(openCvWindow);
    }

    ~ObjectPerceiver() 
    {
        destroyWindow(openCvWindow); 
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        int objectNo = 0;
        int totalTrainNum = objNum_ * trainImgNum_; //total number of training images

        Point posText(50, 50);

        int minHessian;
        nh_.param("minHessian", minHessian, 500);

        double mindist_thresh;
        nh_.param("mindist_thresh", mindist_thresh, 0.1);

        double maxRangeScale;
        nh_.param("maxRangeScale", maxRangeScale, 1.2);

        double dist_thresh;
        nh_.param("dist_thresh", dist_thresh, 30.0);

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        Mat query = cv_ptr->image;

        ROS_INFO_STREAM("Parameters: " << endl << "Hessian - " << minHessian << endl << "Min distance - " << mindist_thresh << endl << "MinDist scale - " << maxRangeScale << endl << "Total dist thresh - " << dist_thresh << endl << endl);

        if (query.empty())
        {
            ROS_INFO(" --(!) No captured frame -- Break!");
        }
        else
        {
            cvtColor(query, query, COLOR_BGR2GRAY); // to grayscale

		//imshow("test", query);
		//waitKey(2);

            //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
            SurfFeatureDetector detector(minHessian);
SurfDescriptorExtractor surfDesc;	

            std::vector<KeyPoint> keypoints_1, keypoint_query;
            Mat descriptors_1, descriptors_query;

            //detector.detectAndCompute(query, Mat(), keypoint_query, descriptors_query);
            detector.detect(query, keypoint_query);
		surfDesc.compute(query, keypoint_query, descriptors_query);

            vector<double> distVec;

            for (int trainNum = 0; trainNum < totalTrainNum; trainNum++) // in this loop, compare the query image with each image in the training dataset
            {

                double sumOfDist = 0;

                Mat img_1 = trainData_.at(trainNum);
		

                detector.detect(img_1, keypoints_1); // detect keypoints Mat(), 
		surfDesc.compute(img_1, keypoints_1, descriptors_1);
                //-- Step 2: Matching descriptor vectors using FLANN matcher
                FlannBasedMatcher matcher;
                std::vector< DMatch > matches;
                matcher.match(descriptors_1, descriptors_query, matches); // match keypoints

                objectNo = 1;
                //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
                //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
                //-- small)
                //-- PS.- radiusMatch can also be used here.
                std::vector< DMatch > good_matches;

		ROS_INFO_STREAM("NUM ROWS ALL: " << descriptors_1.rows);

                for (int i = 0; i < descriptors_1.rows; i++)
                {
                    if (matches[i].distance <= max(maxRangeScale*mindist_thresh, 0.02))
                    {
                        sumOfDist = 1 / matches[i].distance + sumOfDist;
                        good_matches.push_back(matches[i]);
                    }
                }

                distVec.push_back(sumOfDist);
            }
            //Plotting on the image that's found to be the best matching one in the database

            std::vector<double>::iterator best_match_num = std::max_element(distVec.begin(), distVec.end());
            int largestIndex = std::distance(distVec.begin(), best_match_num); // find the best match

            Mat bestImg = trainData_.at(largestIndex);
            std::vector<KeyPoint> keypoints_best;
            Mat descriptors_best;

            detector.detect(bestImg, keypoints_best);
		surfDesc.compute(bestImg, keypoints_best, descriptors_best);

            //-- Step 2: Matching descriptor vectors using FLANN matcher
            FlannBasedMatcher finalMatcher;
            std::vector< DMatch > finalMatches;
            finalMatcher.match(descriptors_best, descriptors_query, finalMatches);

            std::vector< DMatch > best_matches;
            for (int i = 0; i < descriptors_best.rows; i++)
            {
                if (finalMatches[i].distance <= max(maxRangeScale * mindist_thresh, 0.02))
                {
                    best_matches.push_back(finalMatches[i]);
                }
            }
            
            if (!bestImg.empty())
            {            
                //-- Draw only "good" matches
                Mat img_matches;
                drawMatches(bestImg, keypoints_best, query, keypoint_query,
                    best_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), 0); // default drawing

                cout << "3" << endl;
                
                //DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS

                if (img_matches.cols > 1920) // resize if image too big
                {
                    double resizeRatio = 1920.0 / img_matches.cols;
                    cv::resize(img_matches, img_matches, Size(), resizeRatio, resizeRatio);
                }

                double max_dist = 0; double min_dist = 100;

		    ROS_INFO_STREAM("NUM ROWS: " << descriptors_best.rows);
                //-- Quick calculation of max and min distances between keypoints
                for (int i = 0; i < descriptors_best.rows; i++)
                {
                    double dist = finalMatches[i].distance;
     		if (dist < min_dist) min_dist = dist;
                    
		    //min_dist = dist;
                    if (dist > max_dist) max_dist = dist;
                }

                ROS_INFO_STREAM("-- Min dist: " << min_dist << endl);
                ROS_INFO_STREAM("-- Total distance (inversed): " << distVec.at(largestIndex) << endl);

                if (distVec.at(largestIndex) > dist_thresh)
                {
                    int whichObjInx = floor(largestIndex) / trainImgNum_;
                    string whichObjStr = objName_.at(whichObjInx);

                    ROS_INFO_STREAM(whichObjStr << " Recognised!" << endl);
                    putText(img_matches, whichObjStr + " Recognised!", posText, FONT_HERSHEY_COMPLEX, 1.2, BLUE, 2, 8); // displaying recognised names
                    imshow(openCvWindow, img_matches);

                    std_msgs::String msg;
                    msg.data = whichObjStr;
                    pub_.publish(msg);

    ROS_INFO("Published to topic!");
                }
                else
                {
                    imshow(openCvWindow, query);
                }
            }
        }
    }

    template<typename Out>
    void split(const std::string &s, char delim, Out result) 
    {
        std::stringstream ss;
        ss.str(s);
        std::string item;
        while (std::getline(ss, item, delim)) 
        {
            *(result++) = item;
        }
    }

    std::vector<std::string> split(const std::string &s, char delim) 
    {
        std::vector<std::string> elems;
        split(s, delim, std::back_inserter(elems));
        return elems;
    }
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "object_perception_node");  
  
    ObjectPerceiver op;

    ROS_INFO("perceiving objects new!");

    ros::spin();
    return 0;
}

