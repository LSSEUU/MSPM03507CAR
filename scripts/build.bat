@echo off
echo ========================================
echo MSPM0G3507 编译脚本 (Keil 命令行)
echo ========================================

set KEIL_PATH=C:\Keil_v5\UV4\UV4.exe
set PROJECT_FILE=%~dp0..\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx

if not exist "%KEIL_PATH%" (
    echo 错误：找不到Keil安装路径 C:\Keil_v5
    echo 请确认Keil已安装，或修改此脚本中的KEIL_PATH变量
    pause
    exit /b 1
)

if not exist "%PROJECT_FILE%" (
    echo 错误：找不到工程文件
    echo 文件路径：%PROJECT_FILE%
    pause
    exit /b 1
)

echo 开始编译...
"%KEIL_PATH%" -b "%PROJECT_FILE%" -j0 -o "%~dp0..\keil\build.log"

if %errorlevel% equ 0 (
    echo 编译成功！
    echo 输出文件：keil\Objects\empty_LP_MSPM0G3507_nortos_keil.axf
    echo HEX文件：keil\Objects\empty_LP_MSPM0G3507_nortos_keil.hex
) else (
    echo 编译失败！请查看 keil\build.log
)

pause
