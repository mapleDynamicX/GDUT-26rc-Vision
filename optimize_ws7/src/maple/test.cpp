// 包含输入输出流库，用于控制台输入输出
#include <iostream>
// 包含文件流库，用于文件操作
#include <fstream>
// 包含字符串流库，用于字符串与其他类型的转换
#include <sstream>
// 包含map容器库，用于存储键值对数据
#include <map>
// 包含vector容器库，用于动态数组
#include <vector>
// 包含string库，用于字符串处理
#include <string>

// 主函数，程序入口
int main() {
    // 创建一个map容器，键为string类型(编号)，值为vector<int>(数字集合)
    // 用于存储文件中读取的编号及其对应的数字
    std::map<std::string, std::vector<int>> data;

    // 创建输入文件流对象，并尝试打开名为"data.txt"的文件
    std::ifstream file("D:\\code.entrance\\daily life\\read_txt_result\\placements_and_paths_blue_sorted.txt");

    // 检查文件是否成功打开
    if (!file.is_open()) {
        // 如果文件无法打开，输出错误信息到标准错误流
        std::cerr << "无法打开文件" << std::endl;
        // 返回1表示程序异常退出
        return 1;
    }

    // 定义一个字符串变量line，用于存储从文件中读取的每一行
    std::string line;

    // 循环读取文件中的每一行，直到文件末尾
    while (std::getline(file, line)) {
        // 创建字符串流对象iss，并将当前行内容传入
        std::istringstream iss(line);
        // 定义字符串变量id，用于存储编号
        std::string id;
        // 从字符串流中读取第一个字符串作为编号
        iss >> id;

        // 创建vector<int>容器nums，用于存储当前编号对应的数字
        std::vector<int> nums;
        // 定义整数变量num，用于临时存储读取的数字
        int num;

        // 从字符串流中继续读取整数，直到行尾
        while (iss >> num) {
            // 将读取到的数字添加到vector容器中
            nums.push_back(num);
        }

        // 将编号及其对应的数字集合存入map容器
        data[id] = nums;
    }

    // 关闭文件流
    file.close();
    // 定义字符串变量inputId，用于存储用户输入的编号
    std::string inputId("242144342121");
    // 创建一个包含12个整数的数组arr，并初始化为0
    int arr[12] = { 0 };

    // 在map中查找用户输入的编号
    auto it = data.find(inputId);

    // 如果找到了对应的编号
    if (it != data.end()) {
        // 将找到的数字填充到数组arr中，最多12个
        for (size_t i = 0; i < it->second.size() && i < 12; ++i) {
            arr[i] = it->second[i];
        }

        // 输出找到的编号及其对应的数组
        std::cout << "找到编号 " << inputId << "，对应的数组为: ";
        for (int i = 0; i < 12; ++i) {
            std::cout << arr[i] << " ";
        }
        std::cout << std::endl;
    }
    else {
        // 如果未找到编号，输出提示信息
        std::cout << "未找到编号 " << inputId << std::endl;
    }
    // 程序正常结束
    return 0;
}