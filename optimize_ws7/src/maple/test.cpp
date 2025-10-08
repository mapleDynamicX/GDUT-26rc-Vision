// ��������������⣬���ڿ���̨�������
#include <iostream>
// �����ļ����⣬�����ļ�����
#include <fstream>
// �����ַ������⣬�����ַ������������͵�ת��
#include <sstream>
// ����map�����⣬���ڴ洢��ֵ������
#include <map>
// ����vector�����⣬���ڶ�̬����
#include <vector>
// ����string�⣬�����ַ�������
#include <string>

// ���������������
int main() {
    // ����һ��map��������Ϊstring����(���)��ֵΪvector<int>(���ּ���)
    // ���ڴ洢�ļ��ж�ȡ�ı�ż����Ӧ������
    std::map<std::string, std::vector<int>> data;

    // ���������ļ������󣬲����Դ���Ϊ"data.txt"���ļ�
    std::ifstream file("D:\\code.entrance\\daily life\\read_txt_result\\placements_and_paths_blue_sorted.txt");

    // ����ļ��Ƿ�ɹ���
    if (!file.is_open()) {
        // ����ļ��޷��򿪣����������Ϣ����׼������
        std::cerr << "�޷����ļ�" << std::endl;
        // ����1��ʾ�����쳣�˳�
        return 1;
    }

    // ����һ���ַ�������line�����ڴ洢���ļ��ж�ȡ��ÿһ��
    std::string line;

    // ѭ����ȡ�ļ��е�ÿһ�У�ֱ���ļ�ĩβ
    while (std::getline(file, line)) {
        // �����ַ���������iss��������ǰ�����ݴ���
        std::istringstream iss(line);
        // �����ַ�������id�����ڴ洢���
        std::string id;
        // ���ַ������ж�ȡ��һ���ַ�����Ϊ���
        iss >> id;

        // ����vector<int>����nums�����ڴ洢��ǰ��Ŷ�Ӧ������
        std::vector<int> nums;
        // ������������num��������ʱ�洢��ȡ������
        int num;

        // ���ַ������м�����ȡ������ֱ����β
        while (iss >> num) {
            // ����ȡ����������ӵ�vector������
            nums.push_back(num);
        }

        // ����ż����Ӧ�����ּ��ϴ���map����
        data[id] = nums;
    }

    // �ر��ļ���
    file.close();
    // �����ַ�������inputId�����ڴ洢�û�����ı��
    std::string inputId("242144342121");
    // ����һ������12������������arr������ʼ��Ϊ0
    int arr[12] = { 0 };

    // ��map�в����û�����ı��
    auto it = data.find(inputId);

    // ����ҵ��˶�Ӧ�ı��
    if (it != data.end()) {
        // ���ҵ���������䵽����arr�У����12��
        for (size_t i = 0; i < it->second.size() && i < 12; ++i) {
            arr[i] = it->second[i];
        }

        // ����ҵ��ı�ż����Ӧ������
        std::cout << "�ҵ���� " << inputId << "����Ӧ������Ϊ: ";
        for (int i = 0; i < 12; ++i) {
            std::cout << arr[i] << " ";
        }
        std::cout << std::endl;
    }
    else {
        // ���δ�ҵ���ţ������ʾ��Ϣ
        std::cout << "δ�ҵ���� " << inputId << std::endl;
    }
    // ������������
    return 0;
}