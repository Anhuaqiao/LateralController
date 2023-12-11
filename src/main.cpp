#include "Vehicle.h"
#include <getopt.h>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>

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
    std::ofstream outputFile; // Open a file for writing
    string intputfilename, outputfilename;
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
                    intputfilename = optarg;
                    if (!inputFile.is_open()) {
                    std::cerr << "Unable to open file." << std::endl;
                    std::cerr<< "Name is " << optarg <<endl;
                    return 1;
                }
                }
                break;
            case 'o':
                if (optarg != NULL) {
                    outputFile.open(optarg);
                    outputfilename = optarg;
                    if (!outputFile.is_open()) {
                    std::cerr << "Unable to open output file." << std::endl;
                    std::cerr<< "Name is " << optarg <<endl;
                    return 1;
                    }
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
    double time_length;

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
                if (line.find("Seen length: ") == 0) {
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
            case 4: // vehicle init pose
                if (!line.empty()) {
                    std::istringstream ss(line);
                    ss >> controlInfo.speed;
                }
                break;
            case 5: // sampling time
                if (!line.empty()) {
                    std::istringstream ss(line);
                    ss >> controlInfo.dt;
                }
                break;
            case 6: // time length
                if (!line.empty()) {
                    std::istringstream ss(line);
                    ss >> time_length;
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
    std::cout << "Sampling time " << controlInfo.dt <<", Time legnth: " << time_length << std::endl;
    std::cout << "Path Points:" << std::endl;

    for (const auto& point : controlInfo.pathPoints) {
        std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
    }
    inputFile.close();

    //start simulator
    Vehicle tractor(controlInfo.init_state.x,
                    controlInfo.init_state.y,
                    controlInfo.init_state.psi,
                    controlInfo.speed,
                    2,
                    controlInfo.dt);
    
    tractor.Simulator(time_length, controlInfo);


    // Check if the file is open
    if (outputFile.is_open()) {
        std::cout << tractor.xout.size() << std::endl;

        // Write vector elements into the file
        for (auto i=0;i<tractor.xout.size();i++) {
            std::cout << "(" << tractor.xout[i] << "," << tractor.yout[i] << ")"<< std::endl;
            outputFile << tractor.xout[i] << " " << tractor.yout[i]<<"\n";
        }

        // Close the file
        outputFile.close();
        std::cout << "Vector elements written to output.txt successfully." << std::endl;
    } else {
        // Display an error message if the file couldn't be opened
        std::cerr << "Unable to open the file." << std::endl;
    }

  // 生成一些示例数据并保存到文件中
    std::ofstream dataFile("data.txt");
    if (!dataFile.is_open()) {
        std::cerr << "Error: Unable to create data file." << std::endl;
        return 1;
    }
    // 写入示例数据到文件中
    dataFile << "1 2\n";
    dataFile << "2 3\n";
    dataFile << "3 5\n";
    dataFile << "4 7\n";
    dataFile << "5 11\n";
    dataFile.close();

    // 创建GNUplot指令
    std::ostringstream gnuplotCmd;
    gnuplotCmd << "set terminal epscairo enhanced color font 'Arial,12' size 5in,3in\n"; // 设置输出为EPS，指定输出文件尺寸和字体
    gnuplotCmd << "set output 'output.eps'\n";
    gnuplotCmd << "plot '"<<outputfilename<<"' with linespoints title '"<<"vehicle path"<<"', "
               << "'"<<intputfilename<<"'every ::7 with linespoints title '"<<"refer path"<<"'\n";// 使用文件中的数据绘制图表

    // 调用GNUplot绘制图表
    FILE *gnuplotPipe = popen("gnuplot -persistent", "w");
    if (gnuplotPipe) {
        fprintf(gnuplotPipe, "%s", gnuplotCmd.str().c_str()); // 发送GNUplot指令
        fprintf(gnuplotPipe, "exit\n"); // 退出GNUplot
        pclose(gnuplotPipe);
    } else {
        std::cerr << "Error: Could not open GNUplot." << std::endl;
    }
    return 0;
}