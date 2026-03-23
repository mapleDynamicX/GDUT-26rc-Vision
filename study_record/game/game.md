# game

### 一、SFML 库的极简安装（核心：一行命令搞定）

```bash
sudo apt update && sudo apt install libsfml-dev
```

## 编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```


