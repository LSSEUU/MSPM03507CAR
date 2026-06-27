@echo off
echo ========================================
echo MSPM0G3507 串口测试助手
echo ========================================

echo 检查USB转串口设备...
mode com3 2>nul
if %errorlevel% equ 0 (
    echo 找到串口设备：COM3
    set COM_PORT=COM3
) else (
    mode com4 2>nul
    if %errorlevel% equ 0 (
        echo 找到串口设备：COM4
        set COM_PORT=COM4
    ) else (
        echo 未找到串口设备，请检查连接
        pause
        exit /b 1
    )
)

echo.
echo 串口测试命令：
echo 1. 打开串口调试助手
echo 2. 选择端口：%COM_PORT%
echo 3. 设置波特率：115200
echo 4. 发送测试数据
echo.

echo 常用测试命令：
echo - 发送单个字符：A, B, C...
echo - 发送字符串：Hello, Test...
echo - 发送数字：1, 2, 3...
echo.

echo 预期响应：
echo 发送：A
echo 收到：Received: A
echo.

echo 注意事项：
echo - 确保TX/RX正确连接（交叉连接）
echo - 波特率必须匹配：115200
echo - 如果无响应，检查DAPlink是否下载成功
echo.

pause
