//Jai shri shyam

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;


class KeyGrabber
{
public:
    KeyGrabber(){};
    void GrabKeysMat(const sensor_msgs::ImageConstPtr& imu_msg);

    std::vector<cv::KeyPoint> Get_Key(const sensor_msgs::ImageConstPtr& imu_msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat m_keys;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(imu_msg,"");
        cv_ptr->image.copyTo(m_keys);
    }
    catch(cv_bridge::Exception& e){
    }
    std::vector<cv::Point2f> key_2f_vec = m_keys;
    std::vector<cv::KeyPoint> mvKeys;
    cv::KeyPoint::convert(key_2f_vec, mvKeys);
    return mvKeys;
    }

    /*

    void tr(){
      cout<<"Hi"<<endl;
    }
    */

    queue<sensor_msgs::ImageConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, KeyGrabber *pImuGb): mpSLAM(pSLAM), mpImuGb(pImuGb){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
   
    ORB_SLAM3::System* mpSLAM;
    KeyGrabber *mpImuGb;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono");
  ros::start();
  if(argc != 3)
  {
      cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
      ros::shutdown();
      return 1;
  }  

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
  
  KeyGrabber KeysObj;
  ImageGrabber igb(&SLAM, &KeysObj);

  ros::NodeHandle nodeHandler;
  ros::Subscriber sub_img_left = nodeHandler.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImageLeft,&igb);
  ros::Subscriber sub_img_right = nodeHandler.subscribe("/image/descriptor", 100, &ImageGrabber::GrabImageRight,&igb);
  ros::Subscriber key_sub = nodeHandler.subscribe("/image/key", 10, &KeyGrabber::GrabKeysMat,&KeysObj);


  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();

  return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexLeft.lock();
  //cout<<"t1"<<endl;
  if (!imgLeftBuf.empty())
  {
    cout<<"t1"<<endl;
    imgLeftBuf.pop();
  }
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRight.lock();
  cout<<"t2"<<endl;
  if (!imgRightBuf.empty())
  {
    cout<<"t2"<<endl;
    imgRightBuf.pop();
  }
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, "");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void KeyGrabber::GrabKeysMat(const sensor_msgs::ImageConstPtr& imu_msg)
{
  cout<<"t3"<<endl;
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

void ImageGrabber::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while(1)
  {
    //cout<<"in here"<<endl;
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      /*
      cout<<"t5"<<endl;
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      cout<<"tImLeft"<<tImLeft<<endl;

      tImRight = imgRightBuf.front()->header.stamp.toSec();
      cout<<"tImRight"<<tImRight<<endl;

      this->mBufMutexRight.lock();
      while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
      {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
      {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
      */
      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      cout<<imLeft.type()<<endl;
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      std::vector<cv::KeyPoint> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
        {
          vImuMeas = mpImuGb->Get_Key(mpImuGb->imuBuf.front());
          //vImuMeas = mpImuGb->Get_Key(NULL);
          //mpImuGb->tr();

          //double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          //cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          //cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          //vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      //mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
//cv::Mat System::TrackMonocular(const cv::Mat &im, const cv::Mat &descriptors, const std::vector<cv::KeyPoint> &mvKeys, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
      cout<<imLeft.type()<<endl;

      cout<<imRight.type()<<endl;

      cout<<vImuMeas.size()<<endl;
      //mpSLAM->TrackMonocular(imLeft,imRight,vImuMeas,tImLeft);
      mpSLAM->TrackMonocular(imLeft,imRight,tImLeft);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}



