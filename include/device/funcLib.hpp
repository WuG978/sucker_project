#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>

using namespace std;

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef point_cloud::Ptr prevCloud;

struct FrameCfg {
  string serialNumOrBagPath;
  int width;
  int height;
  int frameRate;
};

// This assumes camera with depth and color
// streams, and direction lets you define the target stream
enum class direction { to_depth, to_color };

tuple<int, int, int> RGB_Texture(rs2::video_frame texture,
                                 rs2::texture_coordinate Texture_XY);
cloud_pointer PCL_Conversion(const rs2::points& points,
                             const rs2::video_frame& color);
FrameCfg config_frame_from_cmd(int argc, char* argv[]);

// frame align
rs2::frameset frames_align_to(rs2::frameset frames, direction dir);

// user input
bool userInput(void);
