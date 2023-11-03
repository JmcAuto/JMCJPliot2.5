# Git与Bazel['beɪzəl]食用指南

## 前菜-Git账号配置

由于开源社区Github在2021年七夕当天更改了策略，把账户验证的密码改为了token令牌，在使用`git pul`l或者`git push`操作时会出现下列问题：

```shell
remote: Support for password authentication was removed on August 13, 2021. Please use a personal access token instead.
remote: Please see https://github.blog/2020-12-15-token-authentication-requirements-for-git-operations/ for more information.
```

需要在Github的个人设置-开发者设置-个人访问令牌中选择生成令牌，它长这个样子：

`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`（一点也不开胃🤢）

![image-20211105100803778](https://tva1.sinaimg.cn/large/008i3skNgy1gw41t3ypdej30sg0a70tz.jpg)

并且这个令牌只显示一次，需要手动保存下来，并且每次拉取或上传代码时都需要输入这个令牌

### 两种解决办法

#### 1. 在个人电脑上设置保存Github的账号及令牌

- 终端中设置自动保存，输入：`git config --global credential.helper store`

- 本地设置链接Github账号：

  ```shell
  git config --global user.name "name"
  git config --global user.email "mail"
  #引号内修改为自己的账号名密码及邮箱
  ```

- 引号内输入Github账号邮箱，生成ssh密钥，：`ssh-keygen -t rsa -C "mail"`

  - 默认配置按回车键，最后生成的密钥文件在~/.ssh
  - 复制id_rsa.pub文件里所有内容，放到 Github的个人设置-SSH and GPG keys里的 SSH kyes里

- 测试链接：`ssh -T git@github.com`,如果链接成功，则有**You've successfully authenticated, but GitHub does not provide shell access**输出

- 输入一次账号及令牌后就可以放心食用😋

  > 保存的账号及令牌配置在`~/.git-credentials`中，可按需修改

#### 2. 在Git仓库文件夹中配置账号

这个方法试用于多人共用的电脑中配置

- 引号内输入Github账号邮箱，生成ssh密钥，：`ssh-keygen -t rsa -C "mail"`
  - 默认配置按回车键，最后生成的密钥文件在~/.ssh，打开复制里面对应邮箱地址的rsa
  - 复制对应内容，放到 Github的个人设置-SSH and GPG keys里的 SSH kyes里

- 把令牌加入到已有的仓库中：

  ````shell
  git remote set-url origin https://<令牌>@github.com/JmcAuto/<仓库名>.git
  ````

- 或者使用拉取新的仓库：

  ```shell
  git clone https://<令牌>@github.com/JmcAuto/<仓库名>.git
  ```

## Git仓库内操作步骤见[README.md](../README.md)

## 主菜-Bazel编译方式介绍

### WORKSPACE文件的作用

Bazel中的WORKSPACE文件主要就是命名工作空间以及声明外部依赖

目前JMC_AVP中的编译脚本在编译时使用到的是`WORKSPACE.in`这个文件，编译脚本会先拷贝`WORKSPACE.in`文件命名为`WORKSPACE`，再读取进行编译，所以想要修改外部依赖等配置需要修改`WORKSPACE.in`这个文件

WORKSPACE中可以指定外部工程的BUILD文件，也可以指定依赖其他文件系统目标或是从网络上下载

目前JMC_AVP中的外部依赖全都改为了本地其他文件系统目标，路径是`/home/tmp`，并且该文件在运行docker_into脚本时会自动链接到docker容器内，需要添加外部依赖时可以参照`WORKSPACE.in`中的写法，再将支持Bazel编译的第三方库压缩包拷贝到`/home/tmp`下

### BUILD文件的一些规则

JMC_AVP工程中用到的Bazel版本相当落后，版本是0.5.3（截止2021/11/4，Bazel的最新版本为4.2.1‼️）

不同版本的BUILD语法规则都有不同的改动，其中主要的先进改动为添加了`cc_import`这个规则

```
cc_import(
  name = "mylib",
  hdrs = ["mylib.h"],
  shared_library = "libmylib.so",
)
```

hdrs选项需要将动态库中调用的头文件都包含进去

这个规则用来调用静态库或者动态库，需要配合`cc_binary`生成动态库：

```
cc_binary(
        name="libmylib.so",
        srcs=["mylib.h","mylib.cc"],
        linkshared=True,
        )
```

其中name项一定要命名为`lib*.so`的形式，否则编译器定位会出错，linkshared表示动态库

然后再调用动态库：

```
cc_binary(
        name="test",
        srcs=["main.cc"],
        deps=["mylib"],
        )
```

- 0.5.3版本也是可以链接动态库的，但是没有cc_import规则，需要在生成执行文件的cc_binary中的src选项中添加动态库，并且在hdrs选项中添加包含的头文件。对于同一个动态库被多个执行文件链接时，需要在每一个生成执行文件的规则中都添加引用的头文件，造成大量重复工作:

  ```
  cc_binary(
          name="test",
          srcs=["main.cc","libmylib.so"],
          hdrs=["mylib.h"],
          )
  ```

#### 使用静态库或者动态库的区别

- 静态库在链接阶段，会将编译生成的目标文件.o和引用到的库一起打包到可执行文件中，运行简单，移植方便，但是浪费空间资源
- 动态库在编译时不会被链接到可执行文件中，而是程序运行时才被载入，好处是更新时只需要更新动态库即可

如果使用静态库的方式，某一个静态库更新后，所有使用到它的程序都需要重新编译，所以在测试阶段使用动态库是一个很好的方法。

### 脚本的使用方法

介绍一下jmc_auto.sh编译脚本的一些使用方法

- build、build_opt、build_opt_gpu、build_py、build_usbcam等
  - 用来编译jmcauto的模块代码，可以带上模块名做参数以单独编译某个模块的代码，不同功能的区别是编译参数不同，比如`_opt`参数带上后编译的是优化后的执行文件，体积小；`gpu`参数带上后增加了Caffe GPU的支持
  - `_py`、`_usbcam`等功能则是编译特殊模块或代码
  - 这些功能可以执行`bash jmc_auto.sh help`来查看说明
- clean：执行bazel clean的命令，清除编译文件
- release：本次新增的功能，利用脚本将Bazel编译生成的执行文件和其他必须的文件单独拷贝到特定文件夹中
  - 保存路径为`~/.cache/jmcauto_release/jmcauto`，并且docker内外已链接
  - 会在当前仓库下创建名为release的软链接到`~/.cache/jmcauto_release/jmcauto`中
  - 拷贝的文件有：`cc_binary`生成的执行文件到对应的模块路径下，感知的模型文件、各个模块的配置文件、地图文件、DV的前端文件、tools文件、脚本文件、第三方库文件以及配置信息
- 其他功能可以执行`bash jmc_auto.sh help`来查看说明，说明文档也会根据版本进行更新

![image-20211105102741032](https://tva1.sinaimg.cn/large/008i3skNgy1gw42dgqkzcj30qz0bqac7.jpg)

想要运行完整的AVP代码，就只需要将release功能生成的release文件夹放到车端，***在车端启动docker_into.sh脚本即可***

![image-20211105103149185](https://tva1.sinaimg.cn/large/008i3skNgy1gw42m4k2s1j30g806jgmk.jpg)

如果模块内有更新的话，就只需要通过编译功能编译更新的模块，然后将对应生成的可执行文件单独拷贝到车端对应的目录下即可，如果配置文件有更新也是相同的操作。

**但是**，如果更新的代码在其他模块中有引用，则也需要编译对应的模块代码并更新到车端，理由同静态库的缺点

### 未来要做的改动

更新Bazel版本的话，模块中BUILD文件需要作出相应的改动，计划是根据功能及算法生成动态库，将这些动态库链接到对应的模块的执行文件上，这些生成的动态库都将放到release文件夹中的lib文件夹下。如果有更新代码的话只需要编译生成对应的动态库，再将动态库拷贝到lib文件夹下即可

## 甜品-Docker的一些说明

Docker是一个应用容器引擎，其中需要了解的有两个东西：

- images镜像，由用户打包发布的一种存储文件，jmcauto运行所需的就是一个附带GPU驱动的Ubuntu 14操作环境（Linux也是一个文件系统），包含了jmcauto运行所需要的第三方库及一些模型文件，镜像文件保存在docker的目录下，可输入`docker images`查看
  
  ![image-20211105131333991](https://tva1.sinaimg.cn/large/008i3skNgy1gw4762msa1j30r90433zp.jpg)
  
  - 保存镜像修改，***建议联系管理员操作***：
    1. 使用`docker ps`查看当前运行的容器，记下CONTAINER ID
    
       ![image-20211105131410000](https://tva1.sinaimg.cn/large/008i3skNgy1gw476p0j0dj30t8021jrq.jpg)
    
    2. 输入`docker commit <ID号> <镜像名，一般为jmcauto>:<版本号，不输入则默认为latest>`
  - 导出镜像：`docker save -o <压缩文件名>.tar.gz <镜像名>`（不建议使用docker export命令，该命令不会导出镜像的标签信息）
  - 导入镜像：`docker load -i <压缩文件名>.tar.gz`（不建议使用docker import命令，该命令不会导入镜像的标签信息）
  
- container容器，docker创建一个用于存放镜像的空间，启动容器需要镜像文件。***不同容器之间进程不独立***，但是每个进程都有一个对应的容器的父进程ID。jmcauto中模块进程管理是通过进程名来区分而非进程ID，所以无法同时使用多个容器运行相同程序。

#### docker_into

docker_into脚本主要包含start、into及stop三个功能，默认检查是否有对应的容器正在运行，有则执行into功能，否则执行start功能

- 配置了远程图形显示的环境变量，但是由于linux的x11工具依赖用户目录下的`.Xauthority`文件验证，所以目前功能受限，当有多个ssh窗口登录同一账户时，仅最新一个使用ssh登录的窗口可以使用远程图形显示
- 在`local_volumes`中链接了docker内外的一些文件，其中jmcauto仓库这个文件夹就是通过这个函数链接进docker中的，还包括了一些配置文件，挂载的硬件、硬盘，驱动文件等。因为这一步是在启动容器时就执行的，所以在启动docker_into脚本后再挂载的硬盘，就无法链接进容器内。
  - 可以自己修改改部分代码，添加自己的文件夹链接进容器中
- 在容器中创建用户，第一次启动docker_into（检查.USER_NAME文件）会提示输入用户名，脚本会根据用户名创建容器中的用户，并以此命名容器空间
  - 如果根据之前的`docker commit`操作，保存的自己的镜像，可以在脚本中修改`DOCKER_REPO=jmc/jmcauto`这一行为对应的镜像名
