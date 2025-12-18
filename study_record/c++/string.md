# string

```cpp
#include <iostream>
#include <string>
using namespace std;

void printCString(const char* str) {
    cout << str << endl;
}

int main() {
    string s = "Hello World";
    printCString(s.c_str()); // 正确：调用 c_str() 转换为 const char*
    return 0;
}
```
