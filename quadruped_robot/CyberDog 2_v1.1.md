以下是安装和配置CyberDog 2应用的步骤，以及运行Python代码的指导：

1. **安装CyberDog 2的App：**

   - 前往应用商店（例如App Store或Google Play），搜索并下载CyberDog 2的官方应用程序。
   - 安装完成后，打开应用并按照提示完成初始设置。

2. **长按狗的后脑勺配置Wi-Fi：**

   - 启动CyberDog 2。
   - 单指触摸机器狗后脑勺上的交互区3s，直到它进入Wi-Fi配置模式。通常会有橙色指示灯或语音提示确认它已进入配置模式。
   - 使用手机连接到CyberDog 2发出的Wi-Fi网络。

3. **Wi-Fi配置完成：**

   - 在连接到CyberDog 2的Wi-Fi后，打开已安装的CyberDog 2应用程序。
   - 按照应用中的步骤输入你希望CyberDog 2连接的Wi-Fi网络名称和密码。
   - 完成配置后，CyberDog 2将连接到指定的Wi-Fi网络，且应用程序会显示连接成功的提示。

4. **进入`my controller`文件夹，找到代码，运行Python代码：**

   - 通过SSH或其他方式连接到CyberDog 2的开发环境。

   - 使用命令行导航到

     ```
     cd src/my_controller
     ```

     文件夹：

     ```
     cd /path/to/my/controller
     ```

   - 找到需要运行的Python代码。例如，如果代码文件名为

     ```
     control.py
     ```

     使用以下命令运行代码：

     ```
     bash
     复制代码
     python3 control.py
     ```

以上步骤将帮助你成功配置CyberDog 2并运行相关代码。如果在过程中遇到任何问题，可以参考CyberDog 2的用户手册或联系技术支持。



假设网络配置好，ssh mi@网络IP



如果网络已经配置好，并且你知道CyberDog 2的IP地址，你可以通过SSH连接到设备。以下是具体步骤：

1. **打开终端：**

   - 在你的计算机上打开终端（Linux或macOS）或命令提示符（Windows）。

2. **连接到CyberDog 2：**

   - 使用以下命令通过SSH连接到CyberDog 2：

     ```
     ssh mi@<CyberDog_IP>
     ```

   - 其中，

     ```
     <CyberDog_IP>
     ```

      是CyberDog 2的实际IP地址。例如，如果IP地址是

     ```
     192.168.1.10
     ```

     ，则命令为：

     ```
     ssh mi@192.168.1.10
     ```

3. **输入密码：**

   - 系统会提示你输入密码。输入与CyberDog 2关联的密码，然后按回车键。如果这是你第一次连接，系统可能会询问是否信任该主机。输入`yes`继续。

4. **成功连接：**

   - 一旦成功连接，你将进入CyberDog 2的命令行界面，现在你可以执行相应的命令和操作。

例如，要运行某个Python脚本，你可以导航到相关目录并使用`python3`命令运行脚本。以下是一个简单示例：

```
bash复制代码cd /path/to/my/controller
python3
```



CyberDog 2的SSH密码是`123`，你可以按照以下步骤通过SSH连接到设备：

1. **打开终端或命令提示符：**

   - 在你的计算机上打开终端（Linux或macOS）或命令提示符（Windows）。

2. **输入SSH命令：**

   - 使用以下命令连接到CyberDog 2：

     ```
     ssh mi@<CyberDog_IP>
     ```

   - 将

     ```
     <CyberDog_IP>
     ```

     替换为CyberDog 2的实际IP地址。例如，如果IP地址是

     ```
     192.168.1.10
     ```

     ，则命令为：

     ```
     bash
     复制代码
     ssh mi@192.168.1.10
     ```

3. **输入密码：**

   - 当提示输入密码时，输入`123`并按回车键。

4. **成功连接：**

   - 如果密码正确，你将成功连接到CyberDog 2的命令行界面。在这里，你可以执行你需要的操作，例如运行Python脚本或配置其他设置。

例如，要运行某个Python脚本，你可以执行以下命令：

```
bash复制代码cd /path/to/my/controller
python3 control.py
```

确保你的网络和IP地址设置正确，这样你可以顺利连接到CyberDog 2。



