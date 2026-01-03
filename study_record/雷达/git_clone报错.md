git clone 报错

主要是：

```bash
正克隆到 'pmc-src'...
fatal: 无法访问 'https://github.com/jingnanshi/pmc.git/'：Could not resolve proxy: proxy.example.com
正克隆到 'pmc-src'...
fatal: 无法访问 'https://github.com/jingnanshi/pmc.git/'：Could not resolve proxy: proxy.example.com
正克隆到 'pmc-src'...
fatal: 无法访问 'https://github.com/jingnanshi/pmc.git/'：Could not resolve proxy: proxy.example.com‵
```

```bash
Could not resolve proxy: proxy.example.com

```



解决： 

```bash
# 1. 清空当前终端所有代理环境变量（大小写全清，彻底无残留）
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ftp_proxy FTP_PROXY socks_proxy SOCKS_PROXY all_proxy ALL_PROXY

# 2. 清空Git【本地/全局/系统】三级的所有http/https代理配置（核心！）
git config --local  --unset-all http.proxy 2>/dev/null
git config --local  --unset-all https.proxy 2>/dev/null
git config --global --unset-all http.proxy 2>/dev/null
git config --global --unset-all https.proxy 2>/dev/null
git config --system --unset-all http.proxy 2>/dev/null
git config --system --unset-all https.proxy 2>/dev/null

# 3. 强制Git全局关闭代理，写入「无代理」配置（关键新增，强制覆盖）
git config --global --unset http.proxy
git config --global --unset https.proxy
git config --global http.proxy ""
git config --global https.proxy ""

# 4. 验证Git是否彻底无代理（执行后无任何proxy相关输出=成功）
echo -e "Git代理配置检查:\n$(git config --list | grep -i proxy)"


# 彻底删除整个build目录（所有缓存、失败记录全部清除）
rm -rf build

# 重新创建干净的build目录，并进入
mkdir build && cd build

cmake ..
```


