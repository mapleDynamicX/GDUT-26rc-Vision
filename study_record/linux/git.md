# git

## git使用

git add .  #

git commit -m "提交说明"

git push #推送到远程

git status #查看状态

## 与远程建立联系

git init #初始化git

git add .

git status

git commit -m  "xxx"

git remote add origin git@github.com:Han0301/GDUT-26rc-Vision.git

git branch -M main

git push -u origin main

## 分支管理

```bash
git branch #查看本地分支
```

```bash
git branch -r #查看远程分支
```

```bash
git branch -d feature/login #删除本地分支
```

```bash
git checkout <目标分支名> #本地切换分支
```

echo "# test" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin git@github.com:mapleDynamicX/test.git
git push -u origin main

```bash
git push -u origin feature-branch#第一次推送这个分支
```

git push --mirror ## 强制同步（慎用）
