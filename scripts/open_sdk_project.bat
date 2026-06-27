@echo off
echo ========================================
echo 打开SDK示例工程
echo ========================================

set SDK_PATH=C:\ti\mspm0_sdk_2_09_00_01
set PROJECT_PATH=%SDK_PATH%\examples\nortos\LP_MSPM0G3507\driverlib\empty\keil

if not exist "%SDK_PATH%" (
    echo 错误：找不到SDK路径 %SDK_PATH%
    pause
    exit /b 1
)

if not exist "%PROJECT_PATH%\empty_LP_MSPM0G3507_nortos_keil.uvprojx" (
    echo 错误：找不到Keil工程文件
    echo 路径：%PROJECT_PATH%
    pause
    exit /b 1
)

echo 正在打开Keil工程...
start "" "%PROJECT_PATH%\empty_LP_MSPM0G3507_nortos_keil.uvprojx"

echo 工程已打开
echo.
echo 工程位置：
echo %PROJECT_PATH%
echo.
echo 使用说明：
echo 1. 点击Build (F7) 编译工程
echo 2. 点击Download (F8) 下载程序
echo 3. 点击Debug (Ctrl+F5) 进入调试模式
echo.
pause
