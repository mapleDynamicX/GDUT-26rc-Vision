# git使用文档

### 克隆远程仓库

git clone git@github.com:Han0301/GDUT-26rc-Vision.git

### 进入本地仓库

cd GDUT-26rc-Vision （将个人文件复制到当前文件夹中）

**注释：先看最后面再进行后续步骤**

### 创建自己的分支

git branch -M main（main: 分支名--一定不要与已有的分支同名）

### 提交代码到本地

git add .

git commit -m "本次提交的说明没有的话可以填日期"

### 提交代码到远程

#### 第一次提交该分支

git push -u origin main（main: 你的分支名）

#### 非第一次提交该分支

git push 

```bash
枚举对象中: 9, 完成.
对象计数中: 100% (9/9), 完成.
使用 16 个线程进行压缩
压缩对象中: 100% (4/4), 完成.
写入对象中: 100% (5/5), 437 字节 | 437.00 KiB/s, 完成.
总共 5 （差异 2），复用 0 （差异 0）
remote: Resolving deltas: 100% (2/2), completed with 2 local objects.
To github.com:Han0301/GDUT-26rc-Vision.git
   b8ec348..bf57af2  main -> main
```

出现这样就可以了

--------------------------------------------------------------------------------

**注意：1.只有加入仓库才可提交，不然没有权限，没加的找ZH**

**2.每次提交前先 git pull把自己的仓库更新到最新，不然提交会有报错**

**3.每次提交记得清理工作空间，不要提交视屏，rosbag等大型文件**

### 一些常用指令

git status #可以查看状态

git branch #可以查看当前在那个分支

git checkout main #可以切换到你想要的分支（这里是main）
