@echo off
echo ========================================
echo 打开Keil工程
echo ========================================

set KEIL_PATH=C:\Keil_v5\UV4\UV4.exe
set PROJECT_FILE=%~dp0..\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx

if not exist "%KEIL_PATH%" (
    echo 错误：找不到Keil安装路径
    echo 请确认Keil已安装在 C:\Keil_v5
    echo 或修改此脚本中的KEIL_PATH变量
    pause
    exit /b 1
)

if not exist "%PROJECT_FILE%" (
    echo 错误：找不到工程文件
    echo 文件路径：%PROJECT_FILE%
    pause
    exit /b 1
)

echo 正在打开Keil工程...
start "" "%KEIL_PATH%" "%PROJECT_FILE%"

echo 工程已打开
echo.
echo 使用说明：
echo 1. 点击Build (F7) 编译工程
echo 2. 点击Download (F8) 下载程序
echo 3. 点击Debug (Ctrl+F5) 进入调试模式
echo.
pause
