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
            case 4: // vehicle speed
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
                    4,
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

    std::ofstream steer_file;
    string steer_file_name = "steer.txt";
    steer_file.open(steer_file_name);
    if (steer_file.is_open()) {
        double index_=0.0;
        // Write vector elements into the file
        for (const auto & i:tractor.steerout){
            steer_file << index_ << " "<< i <<"\n";
            index_ += 0.01;
        }

        // Close the file
        steer_file.close();
        std::cout << "Vector elements written to steer.txt successfully." << std::endl;
    } else {
        // Display an error message if the file couldn't be opened
        std::cerr << "Unable to open the file." << std::endl;
    }

    // 创建GNUplot指令
    std::ostringstream gnuplotCmd_steer;
    gnuplotCmd_steer << "set terminal epscairo enhanced color font 'Arial,12' size 5in,3in\n"; // 设置输出为EPS，指定输出文件尺寸和字体
    gnuplotCmd_steer << "set output 'steer.eps'\n";
    gnuplotCmd_steer << "plot '"<<steer_file_name<<"' with linespoints title '"<<"steer angle(radius)"<<"\n";// 使用文件中的数据绘制图表

    // 调用GNUplot绘制图表
    FILE *gnuplotPipe_steer = popen("gnuplot -persistent", "w");
    if (gnuplotPipe_steer) {
        fprintf(gnuplotPipe_steer, "%s", gnuplotCmd_steer.str().c_str()); // 发送GNUplot指令
        fprintf(gnuplotPipe_steer, "exit\n"); // 退出GNUplot
        pclose(gnuplotPipe_steer);
    } else {
        std::cerr << "Error: Could not open GNUplot." << std::endl;
    }

    std::ofstream crs_track_err;
    string cross_track_error_file_name = "cross_track_error.txt";
    crs_track_err.open(cross_track_error_file_name);
    if (crs_track_err.is_open()) {
        double index = 0;
        // Write vector elements into the file
        for (const auto & i:tractor.crs_track_err){
            crs_track_err << index << " " << i <<"\n";
            index += 0.01;
        }

        // Close the file
        crs_track_err.close();
        std::cout << "Vector elements written to crs_track_err.txt successfully." << std::endl;
    } else {
        // Display an error message if the file couldn't be opened
        std::cerr << "Unable to open the file." << std::endl;
    }
    double mean_error = calculateMean(tractor.crs_track_err);
    string mean_error_str = doubleToStringWithPrecisionLimit(mean_error, 3);

    // 创建GNUplot指令
    std::ostringstream gnuplotCmd_crs_track_err;
    gnuplotCmd_crs_track_err << "set title \"mean error = "<<mean_error_str<<"\"\n";
    //gnuplotCmd_crs_track_err << "set title \"{/:Bold Title in Bold} {/:Italic Italic Title}\"\n";
    gnuplotCmd_crs_track_err << "set title font \"Helvetica,18\" textcolor rgb \"blue\"\n";
    gnuplotCmd_crs_track_err << "set terminal epscairo enhanced color font 'Arial,12' size 5in,3in\n"; // 设置输出为EPS，指定输出文件尺寸和字体
    gnuplotCmd_crs_track_err << "set output 'cross_track_error.eps'\n";
    gnuplotCmd_crs_track_err << "plot '"<<cross_track_error_file_name<<"' with linespoints title '"<<"cross track error"<<"\n";// 使用文件中的数据绘制图表

    // 调用GNUplot绘制图表
    FILE *gnuplotPipe_crs_track_err = popen("gnuplot -persistent", "w");
    if (gnuplotPipe_crs_track_err) {
        fprintf(gnuplotPipe_crs_track_err, "%s", gnuplotCmd_crs_track_err.str().c_str()); // 发送GNUplot指令
        fprintf(gnuplotPipe_crs_track_err, "exit\n"); // 退出GNUplot
        pclose(gnuplotPipe_crs_track_err);
    } else {
        std::cerr << "Error: Could not open GNUplot." << std::endl;
    }

    std::cout << "total controller running time: " << tractor.accumulation_time<<std::endl; 
    return 0;
}