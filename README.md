# GIT操作步骤

## github账号配置

- 设置自动保存github账号密码：`git config --global credential.helper store`

- 本地设置链接github账户（引号内修改为自己的用户名和账号）:

  ```shell
  git config --global user.name "name"
  git config --global user.email "mail"
  ssh-keygen -t rsa -C "mail"
  ```

  - 默认配置按回车键，最后生成的密钥文件在~/.ssh
  - 复制id_rsa.pub文件里所有内容，放到 github账户的 setting/SSH and GPG keys里的 SSH kyes里

- 测试链接：`ssh -T git@github.com`,如果链接成功，则有**You've successfully authenticated, but GitHub does not provide shell access**输出

## git仓库拉取

```shell
git clone https://github.com/JmcAuto/JMC_Auto
```

若要切换到lshi分支，进入到仓库文件夹后输入：`git checkout lsh`
ssh登录时会提示JMC_AUTO最近三天是否有更新,请根据提示判断是否需要拉取最新代码

## 创建自己的分支

```shell
cd jmcauto #一定要进入该git文件夹
git checkout <模块名> #带-b参数时为新建分支
```
## 提交修改

```shell
git pull #拉取远程服务器上的更新，有冲突的文件部分需要手动合并
#若有新增文件，则需输入git add，若只是修改文件，可以直接git commit -a
git add <代码文件夹> <或者文件> ... #多个文件夹或文件之间以空格分开
git commit -a #添加注释和版本号
```
### git pull原理

`git pull`这个命令其实相当于`git fetch + git merge`，先获取服务器上的数据，然后将数据合并

#### git fetch

从远程服务器拉取最新内容，将保存在git仓库中的origin/<分支名>中

#### git merge

在merge的时候，git会比对同一个文件的base版本、合并用版本和被合并版本，相对于base版本有改动的文件会保留，如果合并版本和被合并版本都比较与base版本有改动，就需要手动解决冲突，git会在文件中将冲突行标注出来。

- base版本并非基于时间戳查找，而是寻找合并版本与被合并版本的公共源头版本
- git rebase变基命令：将当前分支的新提交放在被变基的分支的提交之后，并创建新提交来替代之前的提交，好处是可以保持分支的线性，坏处是用得不当会影响其他人员的提交

## 切换分支或者某分支中的文件

- 若要切换到某分支，在保证`git status`中当前分支为干净的分支时，直接输入`git checkout <分支名/commit代号>`切换分支
- 若要切换某分支（或者某个commit）的文件到当前分支，输入`git checkout <分支名/commit代号> <文件名或路径>`

## 合并代码

需要合并某分支代码到当前分支时输入`git merge <目标分支名>`，之后根据合并状况commit合并或者先解决冲突

- 需要合并某分支中部分文件到当前分支时，分以下两种

  - 直接覆盖当前分支的文件，见##切换分支或者某分支中的文件
  - 不确定是否有代码冲突时，按照下列步骤执行：

  ```shell
  git status #查看当前分支状态，确保为干净的分支
  git checkout -b <新命名一个分支> #新建一个分支
  git checkout <目标分支名> <文件名或路劲> #将目标分支中需要合并的文件覆盖到新分支
  git commit
  git checkout <原被合并分支名> #切换回需要合并的分支
  git merge <新分支名> #将新建的分支合并到原分支
  #视情况提交代码或解决冲突
  ```

## 上传代码到远程服务器

```shell
git push --set-upstream origin <分支名> #请勿push master分支
#执行一次上方指令后,以后再上传代码不需要输入--set-upstream
#远程拉取下来的分支可直接git push
```

## 删除历史提交的文件

谨慎操作，最好联系管理员执行下列步骤

- 删除文件：`git log --pretty=oneline --branches -- <文件名>`

- 删除文件的历史记录：`git filter-branch --force --index-filter 'git rm -rf --cached --ignore-unmatch <文件名>' --prune-empty --tag-name-filter cat -- --all`

- 清楚缓存：

  ```shell
  rm -rf .git/refs/original/
  git reflog expire --expire=now --all
  git gc --prune=now
  git gc --aggressive --prune=now
  git push origin master
  ```

- 清楚远程仓库缓存：`git remote prune origin`

### 删除之后的操作：

需在每一个对应的Git仓库下执行下列操作以强制覆盖本地提交：

```shell
#保证当前分支状态为干净的分支，有未提交的修改请手动备份到其他目录
git pull -f
git reset --hard origin/<对应的分支名>
```

# 工控机及环境配置步骤见[漫游指南](./漫游指南.md)
