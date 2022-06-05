/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <libusb-1.0/libusb.h>
#include <iostream>

namespace usb_cam {

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, autoexposure_, auto_white_balance_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;



  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }


  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  UsbCamNode() :
      node_("~")
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
    node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
    node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);
    node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    node_.param("autoexposure", autoexposure_, true);
    node_.param("exposure", exposure_, 100);
    node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance_, true);
    node_.param("white_balance", white_balance_, 4000);

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

    // check for default camera info
    if (!cinfo_->isCalibrated())
    {
      cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_;
      camera_info.height = image_height_;
      cinfo_->setCameraInfo(camera_info);
    }


    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
		     image_height_, framerate_);

    // set camera parameters
    if (brightness_ >= 0)
    {
      cam_.set_v4l_parameter("brightness", brightness_);
    }

    if (contrast_ >= 0)
    {
      cam_.set_v4l_parameter("contrast", contrast_);
    }

    if (saturation_ >= 0)
    {
      cam_.set_v4l_parameter("saturation", saturation_);
    }

    if (sharpness_ >= 0)
    {
      cam_.set_v4l_parameter("sharpness", sharpness_);
    }

    if (gain_ >= 0)
    {
      cam_.set_v4l_parameter("gain", gain_);
    }

    // check auto white balance
    if (auto_white_balance_)
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }

    // check auto exposure
    if (!autoexposure_)
    {
      // turn down exposure control (from max of 3)
      cam_.set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      cam_.set_v4l_parameter("exposure_absolute", exposure_);
    }

    // check auto focus
    if (autofocus_)
    {
      cam_.set_auto_focus(1);
      cam_.set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("focus_auto", 0);
      if (focus_ >= 0)
      {
        cam_.set_v4l_parameter("focus_absolute", focus_);
      }
    }
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(&img_);

    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, *ci);

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (cam_.is_capturing()) {
        if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
      }
      ros::spinOnce();
      loop_rate.sleep();

    }
    return true;
  }


  int initLibUsb() {
    using namespace std;
    libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
    libusb_context *ctx = NULL; //a libusb session
    int r; //for return values
    ssize_t cnt; //holding number of devices in list
    r = libusb_init(&ctx); //initialize a library session
    if(r < 0) {
      cout<<"Init Error "<<r<<endl; //there was an error
          return 1;
    }
    libusb_set_debug(ctx, 3); //set verbosity level to 3, as suggested in the documentation
    cnt = libusb_get_device_list(ctx, &devs); //get the list of devices
    if(cnt < 0) {
      cout<<"Get Device Error"<<endl; //there was an error
    }
    cout<<cnt<<" Devices in list."<<endl; //print total number of usb devices
      ssize_t i; //for iterating through the list
    uint8_t *dev_path;
    uint8_t dev_path_length;

    for(i = 0; i < cnt; i++) {
      libusb_device_descriptor desc;
      int r = libusb_get_device_descriptor(devs[i], &desc);
      if (r < 0) {
        cout<<"failed to get device descriptor"<<endl;
        return 0;
      }
      printdev(devs[i]); //print specs of this device
      if(desc.idVendor == 11427) {
        libusb_get_port_path(ctx, devs[i], dev_path, dev_path_length);
        cout << "Device path: " << dev_path << " length: " << dev_path_length << endl;
      }
    }
    libusb_free_device_list(devs, 1); //free the list, unref the devices in it
    libusb_exit(ctx); //close the session
    return 0;
  }

  void printdev(libusb_device *dev) {
    using namespace std;
    libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0) {
      cout<<"failed to get device descriptor"<<endl;
      return;
    }
    cout<<"Number of possible configurations: "<<(int)desc.bNumConfigurations<<"  ";
    cout<<"Device Class: "<<(int)desc.bDeviceClass<<"  ";
    cout<<"VendorID: "<<desc.idVendor<<"  ";
    cout<<"ProductID: "<<desc.idProduct<<endl;
    libusb_config_descriptor *config;
    libusb_get_config_descriptor(dev, 0, &config);
    cout<<"Interfaces: "<<(int)config->bNumInterfaces<<" ||| ";
    const libusb_interface *inter;
    const libusb_interface_descriptor *interdesc;
    const libusb_endpoint_descriptor *epdesc;
    for(int i=0; i<(int)config->bNumInterfaces; i++) {
      inter = &config->interface[i];
      cout<<"Number of alternate settings: "<<inter->num_altsetting<<" | ";
      for(int j=0; j<inter->num_altsetting; j++) {
        interdesc = &inter->altsetting[j];
        cout<<"Interface Number: "<<(int)interdesc->bInterfaceNumber<<" | ";
        cout<<"Number of endpoints: "<<(int)interdesc->bNumEndpoints<<" | ";
        for(int k=0; k<(int)interdesc->bNumEndpoints; k++) {
          epdesc = &interdesc->endpoint[k];
          cout<<"Descriptor Type: "<<(int)epdesc->bDescriptorType<<" | ";
          cout<<"EP Address: "<<(int)epdesc->bEndpointAddress<<" | ";
        }
      }
    }
    cout<<endl<<endl<<endl;
    libusb_free_config_descriptor(config);
}
/*
Index 5: 11427 31
    idVendor: 11427,
    idProduct: 31,

    todo
    see voc-poc
*/



};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  a.initLibUsb();
  a.spin();
  return EXIT_SUCCESS;
}
