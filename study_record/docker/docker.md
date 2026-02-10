# docker

## 配置

```bash
sudo rm -f /usr/share/keyrings/docker-archive-keyring.gpg
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release
curl -fsSL http://mirrors.aliyun.com/docker-ce/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] http://mirrors.aliyun.com/docker-ce/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
# 更新源索引（此时用的是阿里云源）
sudo apt-get update
# 安装Docker核心组件
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
# 启动Docker服务
sudo systemctl start docker
# 设置开机自启
sudo systemctl enable docker
# 验证安装（无需sudo也可先测试）
sudo docker run hello-world




sudo mkdir -p /etc/docker
sudo vim /etc/docker/daemon.json
```

```yaml
{
"registry-mirrors": [
     "https://docker.m.daocloud.io",
     "https://dockerproxy.com",
     "https://docker.mirrors.ustc.edu.cn",
     "https://docker.nju.edu.cn"
 ]

}
```


