#include "funcLib.hpp"
#include "cmdline/cmdline.h"
//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values.
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
tuple<int, int, int> RGB_Texture(rs2::video_frame texture,
                                 rs2::texture_coordinate Texture_XY) {
  // Get Width and Height coordinates of texture
  int width = texture.get_width();    // Frame width in pixels
  int height = texture.get_height();  // Frame height in pixels

  // Normals to Texture Coordinates conversion
  int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
  int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

  int bytes =
      x_value * texture.get_bytes_per_pixel();  // Get # of bytes per pixel
  int strides =
      y_value * texture.get_stride_in_bytes();  // Get line width in bytes
  int Text_Index = (bytes + strides);

  const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

  // RGB components to save in tuple
  int NT1 = New_Texture[Text_Index];
  int NT2 = New_Texture[Text_Index + 1];
  int NT3 = New_Texture[Text_Index + 2];

  return tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//===================================================
cloud_pointer PCL_Conversion(const rs2::points& points,
                             const rs2::video_frame& color) {

  // Object Declaration (Point Cloud)
  cloud_pointer cloud(new point_cloud);

  // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
  tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

  //================================
  // PCL Cloud Object Configuration
  //================================
  // Convert data captured from Realsense camera to Point Cloud
  auto sp = points.get_profile().as<rs2::video_stream_profile>();

  cloud->width = static_cast<uint32_t>(sp.width());
  cloud->height = static_cast<uint32_t>(sp.height());
  cloud->is_dense = false;
  cloud->points.resize(points.size());

  auto Texture_Coord = points.get_texture_coordinates();
  auto Vertex = points.get_vertices();

  // Iterating through all points and setting XYZ coordinates
  // and RGB values
  for (int i = 0; i < points.size(); i++) {
    //===================================
    // Mapping Depth Coordinates
    // - Depth data stored as XYZ values
    //===================================
    cloud->points[i].x = Vertex[i].x;
    cloud->points[i].y = Vertex[i].y;
    cloud->points[i].z = Vertex[i].z;

    // Obtain color texture for specific point
    RGB_Color = RGB_Texture(color, Texture_Coord[i]);

    // Mapping Color (BGR due to Camera Model)
    cloud->points[i].r = get<2>(RGB_Color);  // Reference tuple<2>
    cloud->points[i].g = get<1>(RGB_Color);  // Reference tuple<1>
    cloud->points[i].b = get<0>(RGB_Color);  // Reference tuple<0>
  }

  return cloud;  // PCL RGB Point Cloud generated
}

FrameCfg config_frame_from_cmd(int argc, char* argv[]) {
  //======================
  // Variable Declaration
  //======================
  // create a command line parser
  cmdline::parser argParser;

  FrameCfg cfg = {"", 640, 480, 30};

  // add input arguments
  argParser.add<string>(
      "input", 'i', "The path of .bag file or none to use camera.", true, "");
  argParser.add<int>("width", 'w', "The stream width.", false, 640);
  argParser.add<int>("height", 'l', "The stream height.", false, 480);
  argParser.add<int>("fps", 'f', "Frames per second.", false, 30);

  // Run parser
  argParser.parse_check(argc, argv);
  cfg.serialNumOrBagPath = argParser.get<string>("input");
  cfg.width = argParser.get<int>("width");
  cfg.height = argParser.get<int>("height");
  cfg.frameRate = argParser.get<int>("fps");

  return cfg;
}

// align
rs2::frameset frames_align_to(rs2::frameset frames, direction dir) {

  // Define two align objects. One will be used to align
  // to depth viewport and the other to color.
  // Creating align object is an expensive operation
  // that should not be performed in the main loop
  rs2::align align_to_depth(RS2_STREAM_DEPTH);
  rs2::align align_to_color(RS2_STREAM_COLOR);

  if (dir == direction::to_depth) {
    // Align all frames to depth viewport
    frames = align_to_depth.process(frames);
  } else {
    // Align all frames to color viewport
    frames = align_to_color.process(frames);
  }

  return frames;
}

//========================================
// userInput
// - Prompts user for a char to
// test for decision making.
// [y|Y] - Capture frame and save as .pcd
// [n|N] - Exit program
//========================================
bool userInput(void) {

  bool setLoopFlag;
  bool inputCheck = false;
  char takeFrame;  // Utilize to trigger frame capture from key press ('t')
  do {

    // Prompt User to execute frame capture algorithm
    cout << endl;
    cout << "Generate a Point Cloud? [y/n] ";
    cin >> takeFrame;
    cout << endl;
    // Condition [Y] - Capture frame, store in PCL object and display
    if (takeFrame == 'y' || takeFrame == 'Y') {
      setLoopFlag = true;
      inputCheck = true;
      takeFrame = 0;
    }
    // Condition [N] - Exit Loop and close program
    else if (takeFrame == 'n' || takeFrame == 'N') {
      setLoopFlag = false;
      inputCheck = true;
      takeFrame = 0;
    }
    // Invalid Input, prompt user again.
    else {
      inputCheck = false;
      cout << "Invalid Input." << endl;
      takeFrame = 0;
    }
  } while (inputCheck == false);

  return setLoopFlag;
}
