# 环境配置

1. 参考[PSDK官方文档](https://developer.dji.com/doc/payload-sdk-tutorial/cn/quick-start/quick-guide/jetson-nano.html)的软件环境部分，部署环境依赖

2. 安装mqtt

   - 安装依赖

   ```bash
   sudo apt-get install build-essential gcc make cmake libssl-dev doxygen graphviz
   ```

   - 安装c库

   ```bash
   # 进入 https://github.com/eclipse/paho.mqtt.c 下载 v1.3.8 版的源码
   cd paho.mqtt.c-1.3.8
   cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON \
      -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON
   sudo cmake --build build/ --target install
   sudo ldconfig
   ```

   - 安装c++库

   ```bash
   # 进入 https://github.com/eclipse/paho.mqtt.cpp 下载 v1.2.0 版的源码
   cd paho.mqtt.cpp-1.2.0
   cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON \
      -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE
   sudo cmake --build build/ --target install
   sudo ldconfig
   # ldconfig命令作用：https://linux265.com/course/linux-command-ldconfig.html
   ```

3. [docker环境自启动--AI识别相关](https://blog.csdn.net/weekdawn/article/details/128713590)

# Jetson NX 板配置USB bulk方法

## 第一步：开启USB bulk 功能

1. 参考[PSDK官方文档](https://developer.dji.com/doc/payload-sdk-tutorial/cn/quick-start/quick-guide/jetson-nano.html)下载psdk-usb-configure.sh文件

2. 重命名为nv-l4t-usb-device-mode-start.sh，并提权

   ```bash
   sudo cp startup_bulk/psdk-usb-configure.sh  /opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-start.sh
   sudo chmod 777 /opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-start.sh
   ```

3. 替换原config

   ```bash
   sudo gedit /opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-start.sh
   change the "startup_bulk" path
   sudo bash /opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-start.sh
   sudo rm -rf /sys/kernel/config/usb_gadget/l4t/
   sudo bash /opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-start.sh
   ```

4. 检查startup_bulk是否开启

   ```bash
   ps -aux | grep startup_bulk
   ```

## 第二步：配置USB bulk 端口

1. 修改配置 samples/sample_c++/platform/linux/nvidia_jetson_csg/hal/hal_usb_bulk.h：
   - 把数据线插入慧享AI盒子最右侧C口，另一端连接UBUNTU主机，等待约1分钟后，`lsusb`或`lsusb -d 0955:7020 -v` 查看NVIDIA设备
   - 搜索 Interface Descriptor中 bInterfaceClass 等于 255 Vendor Specific Class的部分，对应填写如下配置文件

```c++
#define LINUX_USB_BULK1_EP_OUT_FD               "/dev/usb-ffs/bulk1/ep1"
#define LINUX_USB_BULK1_EP_IN_FD                "/dev/usb-ffs/bulk1/ep2"

#define LINUX_USB_BULK1_INTERFACE_NUM           (2)
#define LINUX_USB_BULK1_END_POINT_IN            (0x83)
#define LINUX_USB_BULK1_END_POINT_OUT           (2)

#define LINUX_USB_BULK2_EP_OUT_FD               "/dev/usb-ffs/bulk2/ep1"
#define LINUX_USB_BULK2_EP_IN_FD                "/dev/usb-ffs/bulk2/ep2"

#define LINUX_USB_BULK2_INTERFACE_NUM           (3)
#define LINUX_USB_BULK2_END_POINT_IN            (0x84)
#define LINUX_USB_BULK2_END_POINT_OUT           (3)
```

# 配置NX板操作系统的自启动项

1. 编写如下脚本文件run_sunlogin.sh，并提权

   ```sh
   #!/bin/bash

   # Run xhost +
   /usr/bin/xhost +

   # Run sunloginclient
   /usr/local/sunlogin/bin/sunloginclient
   ```
   ```bash
   sudo chmod 777 run_sunlogin.sh
   ```

2. 把上述脚本设置为开机自动启动

   ```bash
   gnome-session-properties
   ```
   然后在打开的窗口中填写 `bash /path/to/run_sunlogin.sh`

3. 如果需要把build/bin/csg_project_on_jetson_cxx配置为开机自动启动
   - 添加`sudo /path/to/csg_project_on_jetson_cxx`到自启动脚本中
   - 为了避免输入密码：
      ```bash
      sudo visudo
      ```
   - 在打开的文件中，找到以root ALL=(ALL:ALL) ALL开头的行
   - 在该行的下方添加以下内容(zxw 为用户名)
      ```
      zxw ALL=(ALL) NOPASSWD: /path/to/csg_project_on_jetson_cxx
      ```
4. 最后别忘记关掉开机输入密码的选项
   ```
   设置--用户--解锁--打开自动登录
   设置--电源--熄屏--永不
   ```