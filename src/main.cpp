// User Headers
#include "cmdline/cmdline.h"
#include "funcLib.hpp"

// Intel Realsense Headers
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API

// PCL Headers
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// Prototype
void get_devices_info(const rs2::device& selected_device);
rs2::device_list get_rs_devices(void);
string get_device_name(const rs2::device& dev);
void Load_PCDFile(void);

// Global Variables
string cloudFile;  // .pcd file name
int i = 1;         // Index for incremental file name

int main(int argc, char* argv[]) try {

  FrameCfg fConfig = config_frame_from_cmd(argc, argv);
  //====================
  // Object Declaration
  //====================
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> openCloud;
  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;

  // when no -i arguments, will get serial number of device
  if (fConfig.serialNumOrBagPath.empty()) {
    // use a connected device
    /* useBagPath = false; */
    // Get serial number
    rs2::device_list devices = get_rs_devices();
    // 设备在列表中的序号
    int n = 0;
    if (devices.size() > 1) {
      cout << "Which device you should get?(must be a number)" << endl;
      cin >> n;
    }
    rs2::device dev = devices[n];
    get_devices_info(dev);
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
      fConfig.serialNumOrBagPath = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  }

  //======================
  // Stream configuration with parameters resolved internally. See enable_stream() overloads for extended usage
  //======================
  if (fConfig.serialNumOrBagPath.rfind(".bag") ==
      fConfig.serialNumOrBagPath.length() - 4) {
    cfg.enable_device_from_file(fConfig.serialNumOrBagPath, true);
  } else {
    if (!fConfig.serialNumOrBagPath.empty()) {
      cfg.enable_device(fConfig.serialNumOrBagPath);
      cfg.enable_stream(RS2_STREAM_COLOR, fConfig.width, fConfig.height,
                        RS2_FORMAT_RGB8, fConfig.frameRate);
      cfg.enable_stream(RS2_STREAM_INFRARED, fConfig.width, fConfig.height,
                        RS2_FORMAT_Y8, fConfig.frameRate);
      cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    }
  }
  rs2::pipeline_profile prof = pipe.start(cfg);

  direction dir = direction::to_color;  // Alignment direction
  /* rs2::colorizer c;                     // Helper to colorize depth images */

  while (true) {
    // Wait for frames from the camera to settle
    for (int i = 0; i < 30; i++) {
      pipe.wait_for_frames();
    }
    // Using the align object, we block the application until a frameset is
    // available
    rs2::frameset frames = pipe.wait_for_frames();

    frames = frames_align_to(frames, dir);

    // With the aligned frameset we proceed as usual
    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::video_frame color = frames.get_color_frame();
    /* auto colorized_depth = c.colorize(depth); */

    // Map color texture to each point
    pc.map_to(color);

    //Generate point cloud
    rs2::points points = pc.calculate(depth);

    // Convert generated Point Cloud to PCL Formatting
    cloud_pointer cloud = PCL_Conversion(points, color);

    //==============================
    // Write PC to .pcd File Format
    //==============================
    cloudFile = "Captured_Frame" + to_string(i) + ".pcd";

    // Take Cloud Data and write to .PCD File Format
    cout << "Generating PCD Point Cloud File... " << endl;
    pcl::io::savePCDFileASCII(cloudFile,
                              *cloud);  // Input cloud to be saved to .pcd
    cout << cloudFile << " successfully generated. " << endl;

    //Load generated PCD file for viewing
    Load_PCDFile();
    i++;  // Increment File Name
  }       //End-while
  cout << "Exiting Program... " << endl;
  return EXIT_SUCCESS;
} catch (const rs2::error& e) {
  cerr << "RealSense error calling " << e.get_failed_function() << "("
       << e.get_failed_args() << "):\n    " << e.what() << endl;
  return EXIT_FAILURE;
} catch (const exception& e) {
  cerr << e.what() << endl;
  return EXIT_FAILURE;
}

void get_devices_info(const rs2::device& selected_device) {
  for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++) {
    rs2_camera_info info_type = static_cast<rs2_camera_info>(i);

    // 可以以流的方式处理SDK枚举类型，以获取表示它们的字符串
    cout << "  " << left << setw(20) << info_type << " : ";

    // 设备可能不支持所有类型的RS2_CAMERA_INFO。
    // 为了防止从“get_info”方法引发异常，我们首先检查设备是否支持这种类型的信息
    if (selected_device.supports(info_type))
      cout << selected_device.get_info(info_type) << endl;
    else
      cout << "N/A" << endl;
  }
  cout << endl;
}

rs2::device_list get_rs_devices() {
  // First, create a rs2::context.
  // The context represents the current platform with respect to connected
  // devices
  rs2::context ctx;

  // Using the context we can get all connected devices in a device list
  rs2::device_list devices = ctx.query_devices();

  /* rs2::device selected_device; */
  if (devices.size() == 0) {
    cerr << "No device connected, please connect a RealSense device" << endl;

    // To help with the boilerplate code of waiting for a device to connect
    // The SDK provides the rs2::device_hub class
    rs2::device_hub device_hub(ctx);

    // Using the device_hub we can block the program until a device connects
    device_hub.wait_for_device();
  } else {
    cout << "Found the following devices:\n" << endl;

    // device_list is a "lazy" container of devices which allows
    // The device list provides 2 ways of iterating it
    // The first way is using an iterator (in this case hidden in the
    // Range-based for loop)
    int index = 0;
    for (rs2::device device : devices) {
      cout << "  " << index++ << " : " << get_device_name(device) << endl;
    }
  }
  return devices;
}

string get_device_name(const rs2::device& dev) {
  // Each device provides some information on itself, such as name:
  string name = "Unknown Device";
  if (dev.supports(RS2_CAMERA_INFO_NAME))
    name = dev.get_info(RS2_CAMERA_INFO_NAME);

  // and the serial number of the device:
  string sn = "########";
  if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
    sn = string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

  return name + " " + sn;
}

void Load_PCDFile(void) {
  string openFileName;

  // Generate object to store cloud in .pcd file
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudView(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  openFileName = "Captured_Frame" + to_string(i) + ".pcd";
  pcl::io::loadPCDFile(openFileName, *cloudView);  // Load .pcd File

  //==========================
  // Pointcloud Visualization
  //==========================
  // Create viewer object titled "Captured Frame"
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("Captured Frame"));

  // Set background of viewer to black
  viewer->setBackgroundColor(0, 0, 0);
  // Add generated point cloud and identify with string "Cloud"
  viewer->addPointCloud<pcl::PointXYZRGB>(cloudView, "Cloud");
  // Default size for rendered points
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
  // Viewer Properties
  viewer->initCameraParameters();  // Camera Parameters for ease of viewing

  cout << endl;
  cout << "Press [Q] in viewer to continue. " << endl;

  viewer->spin();  // Allow user to rotate point cloud and view it

  // Note: No method to close PC visualizer, pressing Q to continue software
  // flow only solution.
}
