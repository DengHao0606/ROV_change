@echo off
setlocal enabledelayedexpansion

:: ANSI 颜色代码
set "RED=[91m"
set "GREEN=[92m"
set "YELLOW=[93m"
set "BLUE=[94m"
set "RESET=[0m"

set foundPath=
set binPath=
set exeName=STM32_Programmer_CLI.exe

:: 预定义多个盘符和可能的路径
set drives=C D E F
set folderIDE=\ST\STM32CubeIDE*
set folderCLT=\ST\STM32CUbeCLT*
set folderPath=\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_*

:: 遍历所有盘符并查找 STM32_Programmer_CLI.exe
for %%d in (%drives%) do (
    for /d %%p in ("%%d:%folderCLT%") do (
        if exist "%%p\STM32CubeProgrammer\bin\%exeName%" (
            set foundPath=%%p\STM32CubeProgrammer\bin\%exeName%
            goto foundCLI
        )
    )
    for /d %%p in ("%%d:%folderIDE%") do (
        if exist "%%p%folderPath%" (
            for /d %%q in ("%%p%folderPath%") do (
                if exist "%%q\tools\bin\%exeName%" (
                    set foundPath=%%q\tools\bin\%exeName%
                    goto foundCLI
                )
            )
        )
    )
)

:: 未找到 CLI 工具，开始递归搜索
set searchFolderIDE=com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32*
set searchFolderCLT=STM32CubeProgrammer

echo %YELLOW%软件可能未安装在默认位置，开始递归查找%RESET%
echo 正在从 D 盘查找，请稍等片刻......
:: D 盘
:: 递归搜索 STM32CubeIDE 工具
for /d /r D:\ %%i in (%searchFolderIDE%) do (
    if exist "%%i\tools\bin\%exeName%" (
        echo %GREEN%找到 CLT 工具！马上开始下载%RESET%
        set foundPath=%%i\tools\bin\%exeName%
        goto foundCLI
    )
)
:: 递归搜索 CLT 工具
for /d /r D:\ %%i in (%searchFolderCLT%) do (
    if exist "%%i\bin\%exeName%" (
        echo %GREEN%找到 CLT 工具！马上开始下载%RESET%
        set foundPath=%%i\bin\%exeName%
        goto foundCLI
    )
)

echo 在 D 盘未找到
echo.
echo 正在从 C 盘查找，请稍等片刻......
:: D 盘
:: 递归搜索 STM32CubeIDE 工具
for /d /r C:\ %%i in (%searchFolderIDE%) do (
    if exist "%%i\tools\bin\%exeName%" (
        echo %GREEN%找到 CLT 工具！马上开始下载%RESET%
        set foundPath=%%i\tools\bin\%exeName%
        goto foundCLI
    )
)
:: 递归搜索 CLT 工具
for /d /r C:\ %%i in (%searchFolderCLT%) do (
    if exist "%%i\bin\%exeName%" (
        echo %GREEN%找到 CLT 工具！马上开始下载%RESET%
        set foundPath=%%i\bin\%exeName%
        goto foundCLI
    )
)

:: 仍然未找到 CLI 工具，报错并退出
echo %RED%[ERROR]%RESET% STM32_Programmer_CLI.exe not found. Please install STM32CubeProgrammer.
echo %RED%[ERROR]%RESET% 操作失败！您的电脑未安装 STM32CubeIDE 软件，必须先安装才可以下载。
echo %RED%[ERROR]%RESET% 操作失败！您的电脑未安装 STM32CubeIDE 软件，必须先安装才可以下载。
echo %RED%[ERROR]%RESET% 操作失败！您的电脑未安装 STM32CubeIDE 软件，必须先安装才可以下载。
timeout /t 29
exit /b 1

:foundCLI
echo %GREEN%Found STM32_Programmer_CLI:%RESET% %foundPath%

:: 自动查找 bin 文件
if exist ".\build\Debug\*.bin" (
    for %%f in (.\build\Debug\*.bin) do (
        set binPath=%%~ff
        goto foundBin
    )
)

if exist ".\Debug\*.elf" (
    for %%f in (.\Debug\*.elf) do (
        set binPath=%%~ff
        goto foundBin
    )
)

:: 没找到 bin 文件，报错退出
echo %RED%[ERROR]%RESET% No bin file found in expected locations!
echo.
echo %RED%[ERROR]%RESET% 操作失败！未找到可下载的文件，请检查：
echo %RED%%RESET% 1、请不要在压缩包里打开，必须先解压缩再运行
echo %RED%%RESET% 2、工程文件不完整，请重新下载本例程
echo %RED%%RESET% 3、此脚本需要放在工程根目录中运行
timeout /t 60
exit /b 1

:foundBin
echo %GREEN%Found BIN file:%RESET% %binPath%

:: 下载 bin 文件到 Flash
echo %YELLOW%Flashing STM32 firmware...%RESET%
echo %YELLOW%开始下载...%RESET%
"%foundPath%" -c port=SWD -w "%binPath%" 0x08000000
if %errorlevel% neq 0 (
    echo %RED%[ERROR]%RESET% ST工具报告下载失败
    timeout /t 60
    exit /b 1
)

:: 发送复位命令
echo %YELLOW%Flash programming successful. Resetting STM32...%RESET%
"%foundPath%" -c port=SWD -rst
if %errorlevel% neq 0 (
    echo %RED%[ERROR]%RESET% 未能复位单片机，请手动按下复位，或重新上电板子
    timeout /t 60
    exit /b 1
)

:: 高亮成功信息
echo %GREEN%STM32 successfully programmed and reset! [0m
:: 换行
echo.
echo %GREEN%下载成功！！ [0m
timeout /t 15
exit


