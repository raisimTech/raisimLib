# Raisim 中文使用指南

## 写在前面

相信大家都是因为自己老师的要求，过来学习Raisim，或者之前也已经尝试过一些别的项目（issac,gazebo,webots）之类的。

首先，我想说一下，为什么要使用Raisim

1. `C++` + `Python`，这个软件对`python`还是比较友好的，但是为了提高你的使用体验，你需要有一定的`C++`基础，尤其是强化学习部分，你需要自己动手修改`Environment.hpp` 如果不太理解`C++`的面对对象的概念，以及一些`STL`的基本结构（`iterator`之类的），使用起来会有一些难度
2. `Linux， Win, Mac` 友好。目前为止我只尝试了`win` 和`linux`，这两个系统的支持度还都是挺好的，尤其是`linux`，对视频的录制很好用，不需要一直盯着屏幕。
3. 强化学习很快，由于代码完全开放，相较于`Unity`而言，你的自由度更高，你可以很轻松的添加你自己的东西。

其次，我说一下本版本相较于作者的原版本的区别

1. 添加了更详细的`log`系统，在`gym`文件夹的`helper`里添加了`logging`系统，使用起来能够在对应的`saver.dir`中生成名为`train.log`的文件，方便后期查询自己的日志
2. 训练过程中自动保存最优的模型，后期会修改把之前的弱鸡模型给删除
3. 完善了`runner.py --mode test`模块，在`gymtorch`文件夹中，你的`runner`可以直接通过运行`python yourfilepath(imply it use your path)/runner.py --mode test --weight yourweightpath`在`unity`等渲染引擎中展示你的模型

## 怎么使用

### 复制项目

1. `fork`本`repo`到你个人的`github`中（右上角有一个`fork`按钮）
2. 在你自己的`fork`版本中，选择`code`按钮，复制`https`链接
3. 在你本地打开`powershell`(windows)或者是`base/zsh`(linux)，然后`git clone your-repo-http`（不清楚的自行`google`）
4. 切换到你的目标位置，通常是`raisimLib`

### 编译项目

编译是`C/C++`在执行之前都需要进行的事情，简单来说，就是把做饭需要的菜都放到桌子上，具体执行方式如下

#### 安装依赖

我下边只列出来你需要安装的软件，如果有不会的请自行`google`

1. `cmake`
2. `Eigen` （`C++`需要用到）
3. 编译器
   1. Linux: `gnu` 
   2. Windows: `Visual Studio`（注意不要安装错误了，是紫色的VS，不是蓝色的VSC）

#### 设置路径

##### Windows

自行`google`: 如何将路径添加到环境变量

##### Linux

自行`google`: 如何将路径添加到`Linux`路径

#### checklist（检查你需要添加的路径）

1. `cmake`:添加的路径下需要有`cmake.exe`文件

2. `eigen`:添加的路径需要有`Dense.hpp`文件
3. `WORKSPACE`:这个变量指向的是`RaisimLib`的位置，这个路径在windows中需要单独列出来，在环境变量中直接新建，而不是通过`path`建立
4. `LOCAL_INSTALL`：这个变量指向的是你希望在那里部署你的库文件， 建议直接在`raisimLib`下新建文件夹，命名为`raisim_build`，然后指向这个位置即可

#### 开始部署/编译

1. 让你的终端切换到`raisimlib`文件夹
2. 创建`build`文件夹，并切换进去
3. 参考[官方文档](https://raisim.com/sections/Installation.html#building-raisim-examples-and-installing-raisimpy)的建议，运行对应的命令，如果你不想编译`python`，就把`-DRAISIM_PY=ON`删掉（他的`python`好像作者自己都不喜欢用）

### 激活文件

1. [点这里申请你的激活文件](https://docs.google.com/forms/d/e/1FAIpQLScNL0vbZPDNS93L6Jv6fgR51WTsvXxfhnVOtKDVRdAmHIoG4w/viewform?usp=sf_link)
   1. 写自己的信息
   2. 通过提供的`google drive`获取软件，读取你的机器码
   3. 将机器码复制进去，然后申请`license`就可以了（他们会给你的邮箱发一个`activation.raisim`）
2. 在你的根目录(`linux`就是`~/`，`windows`是`c:users/xxx（你的账户）/`)，创建一个文件夹，命名为`.raisim`，将`activation.raisim`复制进去即可。

### 运行例子

#### Linux

生成的`example`就在你创建的`build`文件夹下，进去就可以找到类似`alingo`之类的二进制文件，你需要通过`chmod +X ./alingo` 运行即可

#### Windows

如果你是通过VS建立的项目，这个路径通常在`RasimLib/raisim/win32/bin`里，会有一大堆的`exe`，点击运行即可





# `RaisimGymTorch`

如果你是要学习`raisimgymmtorch`进行强化学习，那就需要理解以下几个基本概念

1. `setup.py`(`raisimlib/raisimgymtorch/`) 对所有的环境进行编译，生成在python中可以使用的库
2. `Environment.hpp` 这个文件定义了你的机器人与环境作用的方式
   1. 执行你的`action`
   2. 计算`reward`
   3. 更新`observation`
3. `runner.py` 这个文件是你的`python`的主框架
   1. 记录`log`
   2. 更新`observation`
   3. 记录`state-action`对
   4. `ppo`计算
   5. 生成`action`
   6. 更新`model`
4. `cfg.yaml` 定义超参数
   1. `num_envs`：子空间的数量（开多空间）
   2. `eval_per_iters`：执行多少个`iters`之后会记录并展示





## 欢迎PR

有问题发`Issue`，觉得ok的话，点一个`star`，tks!