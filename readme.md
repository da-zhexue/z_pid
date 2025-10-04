pid.py 为调pid、无UI的版本  
pid_ui.py 为调pid、有UI的版本(串口连接时就开始发送数据了，若要改数据要断开再连接)  
SelfDefine_UI.py 为可设置自定义参数的版本  
  
PID参数调试工具.exe 与 自定义参数调试工具.exe 都不需要安装依赖即可运行(以管理员模式运行)  
在Linux系统上请在python环境(python 3.12.7)中  
'''shell
pip install -r requirements.txt
'''
后运行  

TIPS：  
1、Please open these files with UTF-8  
2、所有数据传输采用小端序  
3、使用时用蓝牙模块连接(透传模式)或USB转TTL连接  