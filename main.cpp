#include "cmdline/cmdline.h"
#include <pcl/io/real_sense_2_grabber.h>

using namespace std;

int main(int argc, char *argv[]) {
  // create a command line parser
  cmdline::parser argParser;

  // add input arguments
  argParser.add<string>("input", 'i',
                        "input the path of .bag file or none to use camera.");

  // Run parser
  argParser.parse_check(argc, argv);
  basic_string<char> input = argParser.get<string>("input");
  if (!input.empty()) {
    // Get serial number
  }

  /* deal with arguments */

  pcl::RealSense2Grabber grabber;
}
