#include "LateralControl.h"
#include "Vehicle.h"
#include <getopt.h>
#include <fstream>
#include <sstream>

struct ControlInfo {
    std::string controlMethod;
    int pathLength;
    std::string modelName;
    State init_state;
    std::vector<aiforce::decision::SinglePoint> pathPoints;
};


int main(int argc, char *argv[]) {
    const char* help;  // = "Usage: Vechicle-Simulator [options] -i file\n"
                       //"Options:\n"
                       //"-h, --help          prints this help message and exit\n"
                       //"-t                  treat input values as cartesic coordinates instead of\n"
                       //"                    using mercator projection on geographic coordinates\n"
                       //"-i, --input         input file\n"
                       //"-o, --output        output file\n";
    bool true_input = 0;    // -t
    std::ifstream inputFile;     // -i, --input
    FILE* output = stdout;  // -o, --output
    // parse arguments
    const char* short_options = "hti:o:";
    const struct option long_options[] = {
        {"help", no_argument, NULL, 'h'},
        {"input", required_argument, NULL, 'i'},
        {"output", required_argument, NULL, 'o'},
        {NULL, 0, NULL, 0}
    };
    int c, option_index;
    while ((c = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1) {
        switch (c) {
            case 'h':
                printf(help);
                return 0;
            case 't':
                true_input = true;
                break;
            case 'i':
                if (optarg != NULL) {
                    inputFile.open(optarg);
                    if (!inputFile.is_open()) {
                    std::cerr << "Unable to open file." << std::endl;
                    std::cerr<< "Name is " << optarg <<endl;
                    return 1;
                }
                }
                break;
            case 'o':
                output = fopen(optarg, "w");
                if (output == NULL) {
                    fprintf(stderr, "output file \"%s\" could not be created.\n", optarg);
                    return 1;
                }
                break;
            default:
                return 1;
        }
    }

    if (!inputFile.is_open()) {
        std::cerr << "Unable to open file." << std::endl;
        return 1;
    }

    ControlInfo controlInfo;

    std::string line;
    int lineCount = 0;

    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);

        switch (lineCount) {
            case 0: // 控制方法名称
                if (line.find("Policy: ") == 0) {
                    controlInfo.controlMethod = line.substr(8);
                }
                break;
            case 1: // 输入路径长度
                if (line.find("Path length: ") == 0) {
                    controlInfo.pathLength = std::stoi(line.substr(13));
                }
                break;
            case 2: // 使用模型名称
                if (line.find("Model: ") == 0) {
                    controlInfo.modelName = line.substr(7);
                }
                break;
            case 3: // vehicle init pose
                if (!line.empty()) {
                    std::istringstream ss(line);
                    ss >> controlInfo.init_state.x >> controlInfo.init_state.y >> controlInfo.init_state.psi;
                }
                break;
            default: // 路径点坐标
                if (!line.empty()) {
                    aiforce::decision::SinglePoint point;
                    std::istringstream ss(line);
                    ss >> point.x >> point.y;
                    controlInfo.pathPoints.push_back(point);
                }
                break;
        }

        lineCount++;
    }

    // 显示读取的信息
    std::cout << "Control Method: " << controlInfo.controlMethod << std::endl;
    std::cout << "Path Length: " << controlInfo.pathLength << std::endl;
    std::cout << "Model Name: " << controlInfo.modelName << std::endl;
    std::cout << "Initial Pose: " << controlInfo.init_state.x <<", " << controlInfo.init_state.y <<", "<< controlInfo.init_state.psi << std::endl;

    std::cout << "Path Points:" << std::endl;
    for (const auto& point : controlInfo.pathPoints) {
        std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
    }
    inputFile.close();

    // simulate
}