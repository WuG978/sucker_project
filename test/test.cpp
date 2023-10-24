#include <iostream>
#include <string>
#include "../3rdparty/cmdline/cmdline.h"

using namespace std;

int main(int argc, char* argv[]) {
  //======================
  // Variable Declaration
  //======================
  // create a command line parser
  cmdline::parser argParser;

  // add input arguments
  argParser.add<string>(
      "input", 'i', "The path of .bag file or none to use camera.", true, "");
  argParser.add<int>("width", 'w', "The stream width.", false, 640);
  argParser.add<int>("height", 'l', "The stream height.", false, 480);
  argParser.add<int>("fps", 'f', "Frames per second.", false, 30);

  // Run parser
  argParser.parse_check(argc, argv);
  basic_string<char> serialNumOrBagPath = argParser.get<string>("input");
  int width = argParser.get<int>("width");
  int height = argParser.get<int>("height");
  int frameRate = argParser.get<int>("fps");
  cout << "serialNumOrBagPath: " << serialNumOrBagPath << endl;
  cout << "width: " << width << endl;
  cout << "height: " << height << endl;
  cout << "frameRate: " << frameRate << endl;
}
